#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ESP32-S3 N16R8 + 2 BTS7960 + HC-SR04 + MPU6050
// Protocolo con Raspberry: $D,v_mm_s,w_mrad_s*XX, $S, $K, $Z, $G, $W, $U, $C, $I
// Telemetria: $T,ms,batt_mV,yaw_mdeg,encL,encR,blocked,faults,0*XX
// Ultrasonido: $U,dist_mm,valid,blocked*XX
// SIN encoders y SIN medicion de bateria: batt_mV=12000 es NOMINAL, NO REAL.
// Motores deben tener alimentacion externa regulada a 12V, GND comun.
// ECHO del HC-SR04 (5V) requiere divisor a 3.3V.

#define L_RPWM 38
#define L_LPWM 39
#define L_EN   40
#define R_RPWM 41
#define R_LPWM 42
#define R_EN   47
#define SDA_PIN 8
#define SCL_PIN 9
#define US_TRIG_PIN 16
#define US_ECHO_PIN 17

static constexpr int MOTOR_L_SIGN = 1;
static constexpr int MOTOR_R_SIGN = 1;
static constexpr float WHEEL_BASE_M = 0.214f;
static constexpr float V_MAX_MM_S = 300.0f;
static constexpr float MIN_DUTY = 0.18f;
static constexpr float DUTY_SLEW = 0.08f;
static constexpr uint32_t PWM_FREQ = 20000;
static constexpr uint8_t PWM_BITS = 8;
static constexpr int BATT_NOMINAL_MV = 12000;

static constexpr float US_STOP_CM = 25.0f;
static constexpr float US_RELEASE_CM = 32.0f;
static constexpr float US_MAX_CM = 300.0f;
static constexpr uint32_t US_TIMEOUT_US = 18000;
static constexpr uint32_t US_SAMPLE_MS = 80;
static constexpr uint32_t US_MAX_AGE_MS = 300;

static constexpr float GYRO_SCALE = 131.0f;  // MPU +/-250 grados/s
static constexpr float FILTRO_ALPHA = 0.25f;
static constexpr float UMBRAL_Z = 0.5f;
static constexpr uint32_t IMU_INTERVAL_US = 10000;
static constexpr uint32_t COMMAND_TIMEOUT_MS = 600;

static constexpr uint8_t F_OBSTACLE = 1;
static constexpr uint8_t F_BATT_LOW = 2;   // desactivado, sin ADC
static constexpr uint8_t F_BATT_CRIT = 4;  // desactivado, sin ADC
static constexpr uint8_t F_WATCHDOG = 8;
static constexpr uint8_t F_IMU = 16;

static uint8_t mpuAddr = 0;
static bool imuLista = false;
static float offsetX = 0, offsetY = 0, offsetZ = 0;
static float gyroZFiltrado = 0;
static float yawDeg = 0;
static uint32_t imuLastUs = 0;
static uint32_t imuLastGoodMs = 0;
static uint8_t imuErrors = 0;
static bool imuCalibrada = false;
static uint32_t imuLastRecoveryMs = 0;
static constexpr uint32_t IMU_RETRY_MS = 1000;
static uint32_t imuReadFailuresTotal = 0;
static float gyroZActual = 0;

static bool distanceValid = false;
static bool obstacleActive = true;
static float distanceCm = -1;
static uint32_t lastDistanceMs = 0;
static uint32_t lastPingMs = 0;

static float desiredV = 0, desiredW = 0;
static float dutyL = 0, dutyR = 0;
static uint32_t lastDriveMs = 0;
static uint32_t lastControlMs = 0;
static uint32_t lastTelemMs = 0;
static uint32_t lastUltraTxMs = 0;
static uint8_t faults = 0;
static bool motorEnabled = false;
static char rxBuf[100];
static size_t rxLen = 0;

static void motorsOff();

static bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool readRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, n, (uint8_t)true) != n) return false;
  for (uint8_t i = 0; i < n; ++i) buf[i] = Wire.read();
  return true;
}
static bool readGyro(float &x, float &y, float &z) {
  uint8_t b[6];
  if (!readRegs(0x43, b, 6)) return false;
  int16_t rx = (int16_t)((uint16_t)b[0] << 8 | b[1]);
  int16_t ry = (int16_t)((uint16_t)b[2] << 8 | b[3]);
  int16_t rz = (int16_t)((uint16_t)b[4] << 8 | b[5]);
  x = rx / GYRO_SCALE; y = ry / GYRO_SCALE; z = rz / GYRO_SCALE;
  return true;
}
static bool initMPU() {
  mpuAddr = 0;
  for (uint8_t addr : {uint8_t(0x68), uint8_t(0x69)}) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) { mpuAddr = addr; break; }
  }
  if (!mpuAddr) return false;
  uint8_t who = 0;
  if (!readRegs(0x75, &who, 1) || who != 0x68) return false;
  if (!writeReg(0x6B, 0x80)) return false;
  delay(150);
  if (!writeReg(0x6B, 0x01)) return false;
  if (!writeReg(0x1A, 0x03)) return false;
  if (!writeReg(0x1B, 0x00)) return false;
  if (!writeReg(0x19, 0x09)) return false;
  delay(300);
  return true;
}
static bool calibrateMPU() {
  // Durante la calibracion se deshabilitan las ruedas.
  motorEnabled = false;
  desiredV = desiredW = 0;
  double sx = 0, sy = 0, sz = 0, sz2 = 0;
  int valid = 0;
  delay(2000); // mantener robot completamente inmovil
  for (int i = 0; i < 600; ++i) {
    float x, y, z;
    if (readGyro(x, y, z)) {
      sx += x; sy += y; sz += z; sz2 += (double)z * z;
      ++valid;
    }
    delay(10);
  }
  if (valid < 550) return false;
  const float oz = (float)(sz / valid);
  double variance = sz2 / valid - (double)oz * oz;
  if (variance < 0) variance = 0;
  if (sqrt(variance) > 1.0) return false;
  offsetX = (float)(sx / valid);
  offsetY = (float)(sy / valid);
  offsetZ = oz;
  gyroZFiltrado = 0;
  yawDeg = 0;
  imuLastUs = micros();
  imuLastGoodMs = millis();
  imuErrors = 0;
  imuCalibrada = true;
  return true;
}
static void updateIMU() {
  // Nunca integrar valores viejos cuando se pierde la lectura I2C.
  // Tras 5 errores seguidos, apagar motores y reintentar iniciar el MPU.
  if (!imuLista) {
    if (millis() - imuLastRecoveryMs < IMU_RETRY_MS) return;
    imuLastRecoveryMs = millis();
    motorsOff();
    motorEnabled = false;
    desiredV = desiredW = 0;
    // La calibracion de offsets SOLO se hace en setup o por comando C
    // con el robot quieto. Nunca recalibrar automaticamente en movimiento.
    if (initMPU() && imuCalibrada) {
      imuLista = true;
      imuErrors = 0;
      gyroZFiltrado = 0;
      gyroZActual = 0;
      imuLastUs = micros();
      imuLastGoodMs = millis();
    }
    return;
  }

  uint32_t now = micros();
  uint32_t elapsed = now - imuLastUs;
  if (elapsed < IMU_INTERVAL_US) return;

  float x, y, z;
  if (!readGyro(x, y, z)) {
    imuLastUs = micros();
    ++imuReadFailuresTotal;
    if (imuErrors < 255) ++imuErrors;
    if (imuErrors >= 5) {
      imuLista = false;
      imuLastRecoveryMs = millis();
      motorsOff();
      motorEnabled = false;
      desiredV = desiredW = 0;
    }
    return;
  }

  imuErrors = 0;
  float dt = elapsed * 1.0e-6f;
  imuLastUs = now;
  imuLastGoodMs = millis();
  gyroZActual = z - offsetZ;
  gyroZFiltrado = FILTRO_ALPHA * gyroZActual +
                   (1.0f - FILTRO_ALPHA) * gyroZFiltrado;
  float wz = fabsf(gyroZFiltrado) < UMBRAL_Z ? 0.0f : gyroZFiltrado;
  if (dt > 0.0f && dt < 0.1f) yawDeg += wz * dt;
}

static void updateUltrasonic() {
  uint32_t now = millis();
  if (now - lastPingMs < US_SAMPLE_MS) return;
  lastPingMs = now;
  digitalWrite(US_TRIG_PIN, LOW); delayMicroseconds(3);
  digitalWrite(US_TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);
  unsigned long echoUs = pulseIn(US_ECHO_PIN, HIGH, US_TIMEOUT_US);
  if (echoUs == 0) {
    distanceValid = false; obstacleActive = true; distanceCm = -1;
    return;
  }
  float cm = echoUs * 0.0343f * 0.5f;
  if (cm < 2.0f || cm > US_MAX_CM) {
    distanceValid = false; obstacleActive = true; distanceCm = -1;
    return;
  }
  distanceCm = cm;
  distanceValid = true;
  lastDistanceMs = millis();
  if (obstacleActive) {
    if (cm >= US_RELEASE_CM) obstacleActive = false;
  } else if (cm <= US_STOP_CM) obstacleActive = true;
}
static bool frontBlocked() {
  return false; // SIN ULTRASONIDO: solo para control manual supervisado con paro fisico
}
static void motorWrite(int rpwm, int lpwm, float duty) {
  duty = constrain(duty, -1.0f, 1.0f);
  uint32_t pwm = (uint32_t)roundf(fabsf(duty) * 255.0f);
  if (duty > 0) { ledcWrite(rpwm, pwm); ledcWrite(lpwm, 0); }
  else if (duty < 0) { ledcWrite(rpwm, 0); ledcWrite(lpwm, pwm); }
  else { ledcWrite(rpwm, 0); ledcWrite(lpwm, 0); }
}
static void motorsOff() {
  dutyL = dutyR = 0;
  motorWrite(L_RPWM, L_LPWM, 0);
  motorWrite(R_RPWM, R_LPWM, 0);
  digitalWrite(L_EN, LOW);
  digitalWrite(R_EN, LOW);
}
static float dutyFromSpeed(float mmps) {
  float normalized = constrain(mmps / V_MAX_MM_S, -1.0f, 1.0f);
  if (fabsf(normalized) < 0.001f) return 0;
  return copysignf(MIN_DUTY + (1.0f - MIN_DUTY) * fabsf(normalized), normalized);
}
static float slew(float actual, float target) {
  return actual + constrain(target - actual, -DUTY_SLEW, DUTY_SLEW);
}
static void controlStep() {
  uint32_t now = millis();
  if (now - lastControlMs < 20) return;
  lastControlMs = now;
  faults = 0;
  // No hay sensor de distancia; conservar watchdog e IMU.
  if (now - lastDriveMs > COMMAND_TIMEOUT_MS) faults |= F_WATCHDOG;
  if (!imuLista || now - imuLastGoodMs > 200) faults |= F_IMU;
  if ((faults & (F_WATCHDOG | F_IMU)) || !motorEnabled) {
    motorsOff(); return;
  }
  float v = desiredV, w = desiredW;
  // Solo recibir comandos temporales supervisados desde Raspberry Pi.
  float halfTrackMm = WHEEL_BASE_M * 500.0f;
  float l = v - w * halfTrackMm;
  float r = v + w * halfTrackMm;
  float targetL = dutyFromSpeed(l * MOTOR_L_SIGN);
  float targetR = dutyFromSpeed(r * MOTOR_R_SIGN);
  dutyL = slew(dutyL, targetL);
  dutyR = slew(dutyR, targetR);
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);
  motorWrite(L_RPWM, L_LPWM, dutyL);
  motorWrite(R_RPWM, R_LPWM, dutyR);
}
static uint8_t checksum(const char *s) {
  uint8_t x = 0;
  while (*s) x ^= (uint8_t)*s++;
  return x;
}
static void sendFrame(const char *payload) {
  Serial.printf("$%s*%02X\n", payload, checksum(payload));
}
static void sendTelemetry() {
  char b[128];
  snprintf(b, sizeof(b), "T,%lu,%d,%ld,0,0,%u,%u,0",
           (unsigned long)millis(), BATT_NOMINAL_MV,
           (long)lroundf(yawDeg * 1000.0f),
           frontBlocked() ? 1u : 0u, (unsigned)faults);
  sendFrame(b);
}
static void sendUltrasonic() {
  char b[80];
  snprintf(b, sizeof(b), "U,%ld,%u,%u",
           distanceValid ? (long)lroundf(distanceCm * 10.0f) : -1L,
           distanceValid ? 1u : 0u, frontBlocked() ? 1u : 0u);
  sendFrame(b);
}
static void processCommand(char *line) {
  if (line[0] != '$') return;
  char *star = strchr(line, '*');
  if (!star || strlen(star + 1) != 2) return;
  *star = '\0';
  char *payload = line + 1;
  char *end = nullptr;
  unsigned long got = strtoul(star + 1, &end, 16);
  if (!end || *end != '\0' || got > 255 || checksum(payload) != (uint8_t)got) return;
  if (strcmp(payload, "S") == 0 || strcmp(payload, "K") == 0) {
    desiredV = desiredW = 0;
    motorEnabled = false;
    motorsOff();
  } else if (strcmp(payload, "Z") == 0) {
    // Rezero del angulo, NO recalibracion: la calibracion se hace en setup/C.
    yawDeg = 0;
    gyroZFiltrado = 0;
    imuLastUs = micros();
  } else if (strcmp(payload, "C") == 0) {
    motorsOff();
    imuCalibrada = false;
    imuLista = initMPU() && calibrateMPU();
    imuLastRecoveryMs = millis();
  } else if (strcmp(payload, "I") == 0) {
    // Diagnostico bajo demanda: $I,ready,erroresConsecutivos,erroresTotales,
    // gyroZCorregido_x100,gyroZFiltrado_x100,yaw_mdeg*XX
    char b[120];
    snprintf(b, sizeof(b), "I,%u,%u,%lu,%ld,%ld,%ld",
             imuLista ? 1u : 0u, (unsigned)imuErrors,
             (unsigned long)imuReadFailuresTotal,
             (long)lroundf(gyroZActual * 100.0f),
             (long)lroundf(gyroZFiltrado * 100.0f),
             (long)lroundf(yawDeg * 1000.0f));
    sendFrame(b);
  } else if (strcmp(payload, "U") == 0) {
    sendUltrasonic();
  } else if (strcmp(payload, "G") == 0 || strcmp(payload, "W") == 0) {
    sendTelemetry();
  } else if (payload[0] == 'D' && payload[1] == ',') {
    // D,v_mm_s,w_mrad_s; no reactivar motores si la IMU falla.
    char *comma = strchr(payload + 2, ',');
    if (!comma) return;
    *comma = '\0';
    char *e1 = nullptr, *e2 = nullptr;
    float v = strtof(payload + 2, &e1);
    float w = strtof(comma + 1, &e2);
    if (!e1 || *e1 || !e2 || *e2 || !isfinite(v) || !isfinite(w)) return;
    desiredV = constrain(v, -V_MAX_MM_S, V_MAX_MM_S);
    desiredW = constrain(w / 1000.0f, -1.5f, 1.5f);
    lastDriveMs = millis();
    motorEnabled = imuLista && (millis() - imuLastGoodMs <= 200);
  }
}
static void readSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen) { rxBuf[rxLen] = '\0'; processCommand(rxBuf); rxLen = 0; }
    } else if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
    else rxLen = 0;
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(L_EN, OUTPUT); pinMode(R_EN, OUTPUT);
  digitalWrite(L_EN, LOW); digitalWrite(R_EN, LOW);
  ledcAttach(L_RPWM, PWM_FREQ, PWM_BITS);
  ledcAttach(L_LPWM, PWM_FREQ, PWM_BITS);
  ledcAttach(R_RPWM, PWM_FREQ, PWM_BITS);
  ledcAttach(R_LPWM, PWM_FREQ, PWM_BITS);
  motorsOff();
  // Pines de HC-SR04 sin uso.
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  imuLista = initMPU() && calibrateMPU();
  imuLastRecoveryMs = millis();
  // No texto de depuracion por Serial: el enlace Raspberry usa tramas exclusivamente.
  if (!imuLista) imuLastGoodMs = 0;
  lastDriveMs = millis();
  lastControlMs = millis();
  lastPingMs = millis() - US_SAMPLE_MS;
}
void loop() {
  readSerialCommands();
  updateIMU();
  // HC-SR04 deshabilitado en esta variante; no llamar updateUltrasonic();
  controlStep();
  uint32_t now = millis();
  if (now - lastTelemMs >= 50) { lastTelemMs = now; sendTelemetry(); }
  if (now - lastUltraTxMs >= 200) { lastUltraTxMs = now; sendUltrasonic(); }
}