/*
  roomba_esp32.ino  --  Firmware de bajo nivel del robot aspirador casero (v2)
  ========================================================================
  ESP32 (Arduino core 2.x o 3.x). Habla con la Raspberry Pi 5 por USB-serial
  (115200) con el protocolo de pi/esp32_link.py.

  HARDWARE PARA EL QUE ESTÁ CONFIGURADO
    * 2 x BTS7960 (43 A), motores de 12 V, pack Ni-MH 14.4 V 3000 mAh
    * IMU "HW-123" (se autodetecta; se asume familia MPU6050/6500/9250 por I2C)
    * 4 IR de precipicio del Roomba serie 600 (fototransistores, lectura analógica)
    * Encoders en ambas ruedas (diámetro 75 mm, separación 214 mm)

  RESPONSABILIDADES (todo lo crítico para la seguridad vive AQUÍ, no en la Pi):
    * Motores: velocidad diferencial (v, w), rampa de aceleración,
      voltaje efectivo LIMITADO A 12 V (el pack llega a ~16 V cargado)
    * IMU: integra el giroscopio Z -> yaw
    * IR de precipicio: con un borde detectado solo se permite retroceder
    * Batería: aviso "baja"; corte de motores en "crítica"
    * Watchdog: si la Pi deja de mandar comandos 400 ms, se detiene

  PINES -- ESP32-S3-N16R8 (el predeterminado). En ese módulo NO se pueden usar:
     GPIO 26-37 (flash/PSRAM octal), 19-20 (USB nativo), 43-44 (UART0 = serial hacia la Pi),
     0, 3, 45, 46 (strapping). Se usan solo pines libres:
    BTS7960 izq : RPWM=38  LPWM=39  R_EN+L_EN unidos=40
    BTS7960 der : RPWM=41  LPWM=42  R_EN+L_EN unidos=47
    IMU I2C     : SDA=8   SCL=9
    Encoders    : IzqA=10 IzqB=11  DerA=12 DerB=13
    Batería     : 1  (ADC1) <- divisor 100k (arriba) / 15k (abajo) + 100 nF a GND
    IR cliff    : 2, 4, 5, 6 (ADC1)  [pull-up a 3.3 V, NUNCA a 5 V]
    Emisores IR : 7 (opcional, vía transistor; ver CONEXIONES.md)
  (Si usas un ESP32 clásico, define ROOMBA_CLASSIC_ESP32 para usar el mapa de pines anterior.)

  SERIAL HACIA LA PI: conecta el USB-C rotulado "UART" (puente CH340/CP210x) a la Pi y, en Arduino IDE,
  deja "USB CDC On Boot: Disabled". Aparece como /dev/ttyUSB0. (Con el USB-C nativo y CDC activado
  aparecería como /dev/ttyACM0; ajusta config.ESP32_PORT.)

  ⚠ SEGURIDAD: primera prueba con las ruedas en el aire. Tierra común entre Pi, ESP32 y
    BTS7960. Nunca alimentes la Pi desde el ESP32. VCC lógico de los BTS7960 a 5 V.
  ⚠ NO ESTÁ PROBADO EN HARDWARE: revisa cada #define de "CONFIGURACIÓN".
*/
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ======================= CONFIGURACIÓN =======================
// --- BTS7960: RPWM = avance de ese motor, LPWM = retroceso, EN = habilita ambas mitades ---
#ifndef ROOMBA_CLASSIC_ESP32
  #define L_RPWM 38
  #define L_LPWM 39
  #define L_EN 40
  #define R_RPWM 41
  #define R_LPWM 42
  #define R_EN 47
#else
  #define L_RPWM 25
  #define L_LPWM 26
  #define L_EN 27
  #define R_RPWM 18
  #define R_LPWM 19
  #define R_EN 23
#endif
#define PWM_FREQ 20000            // el BTS7960 admite hasta ~25 kHz
#define PWM_BITS 8
#define MOTOR_L_SIGN 1            // pon -1 si esa rueda gira al revés
#define MOTOR_R_SIGN 1

// --- Cinemática / motores ---
#define WHEEL_BASE_M 0.214f       // distancia entre ruedas
#define WHEEL_DIAMETER_M 0.075f
#define MOTOR_NOM_MV 12000.0f     // voltaje nominal del motor: el voltaje EFECTIVO nunca lo supera
#define V_MAX_MM_S 300.0f         // [MEDIR] velocidad de rueda a 12 V efectivos
#define MIN_DUTY 0.18f            // [MEDIR] fracción de 12 V mínima para que el motor arranque
#define DUTY_SLEW 0.08f           // máx. cambio de PWM por ciclo de 20 ms (rampa)

// --- Encoders ---
#define ENCODERS_PRESENT 1        // cuenta pulsos y los reporta (útil para calibrar)
#define ENC_CLOSED_LOOP 0         // 1 = controlar velocidad con PID. ACTÍVALO SOLO tras calibrar ENC_TICKS_PER_REV
#define ENC_QUADRATURE 1          // 1 = canales A y B; 0 = un solo canal (dirección según el mando)
#ifndef ROOMBA_CLASSIC_ESP32
  #define ENC_L_A 10
  #define ENC_L_B 11
  #define ENC_R_A 12
  #define ENC_R_B 13
#else
  #define ENC_L_A 16
  #define ENC_L_B 17
  #define ENC_R_A 4
  #define ENC_R_B 5
#endif
#define ENC_L_SIGN 1
#define ENC_R_SIGN 1
#define ENC_TICKS_PER_REV 508.8f  // [MEDIR] pulsos por vuelta de RUEDA (desconocido: ver check_hardware.py encoders)
#define TICKS_PER_M (ENC_TICKS_PER_REV / (3.14159265f * WHEEL_DIAMETER_M))
#define PID_KP 0.8f
#define PID_KI 1.5f

// --- IMU: familia MPU (6050/6500/9250). Registros de giroscopio idénticos ---
#ifndef ROOMBA_CLASSIC_ESP32
  #define SDA_PIN 8
  #define SCL_PIN 9
#else
  #define SDA_PIN 21
  #define SCL_PIN 22
#endif
#define GYRO_YAW_SIGN 1.0f        // +1: yaw crece al girar antihorario (visto desde arriba)

// --- Batería Ni-MH 12 celdas 14.4 V. Divisor 100k/15k -> 17.5 V = 2.28 V en el ADC (zona lineal) ---
#ifndef ROOMBA_CLASSIC_ESP32
  #define BATT_PIN 1
#else
  #define BATT_PIN 32
#endif
#define BATT_DIV_RATIO 7.667f     // (100k+15k)/15k
#define BATT_CAL 1.0f             // [MEDIR] ajusta con un multímetro
#define BATT_LOW_MV 13000         // aviso: la Pi manda al robot a casa. Ni-MH: curva MUY plana, ver README
#define BATT_CRIT_MV 12000        // corte (1.0 V/celda). Mide con carga y ajusta
#define BATT_CRIT_HOLD_MS 2000    // debe mantenerse así este tiempo (evita cortes por picos de corriente)

// --- IR de precipicio ---
#define NUM_CLIFF 4
#ifndef ROOMBA_CLASSIC_ESP32
static const uint8_t CLIFF_PINS[NUM_CLIFF] = {2, 4, 5, 6};
#else
static const uint8_t CLIFF_PINS[NUM_CLIFF] = {34, 35, 36, 39};
#endif
#define CLIFF_EMIT_PIN -1         // >=0 (p. ej. 7 en S3): pin que conmuta los emisores (con transistor) para cancelar luz ambiente
#define CLIFF_DELTA_RATIO 0.45f   // desvío relativo vs. lectura de calibración
#define CLIFF_MIN_DELTA 150       // desvío mínimo absoluto (cuentas ADC de 12 bits)

#define WATCHDOG_MS 400
#define DEBUG_CLIFF_RAW 0         // 1: manda $R,... (lecturas crudas) para calibrar umbrales

// ======================= FALLAS (bits de telemetría) =======================
#define F_CLIFF 1
#define F_BATT_LOW 2
#define F_BATT_CRIT 4
#define F_WATCHDOG 8
#define F_IMU 16

// ======================= PWM (compatible core 2.x y 3.x) =======================
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_BITS)
  #define PWM_WRITE(pin, ch, duty) ledcWrite(pin, duty)
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); } while (0)
  #define PWM_WRITE(pin, ch, duty) ledcWrite(ch, duty)
#endif

// ======================= ESTADO =======================
static float cmdV = 0, cmdW = 0;           // mm/s, rad/s
static uint32_t lastCmdMs = 0;
static float yawDeg = 0, gyroBias = 0;
static bool imuOk = false;
static uint8_t mpuAddr = 0x68, mpuWho = 0;

static float battMv = 0;
static uint32_t battCritSince = 0;
static bool battCritLatched = false;

static int cliffBase[NUM_CLIFF];
static bool cliffActive = false;
static uint8_t cliffMask = 0;
static uint8_t faults = 0;

static float dutyL = 0, dutyR = 0;
static float intL = 0, intR = 0;
static volatile int32_t encL = 0, encR = 0;
static volatile int8_t dirL = 1, dirR = 1; // solo para encoders de un canal
static int32_t encLPrev = 0, encRPrev = 0;

static char rxBuf[96];
static uint8_t rxLen = 0;

// ======================= MOTORES (BTS7960) =======================
static void setMotor(uint8_t rpwm, uint8_t chR, uint8_t lpwm, uint8_t chL, float u) {
  u = constrain(u, -1.0f, 1.0f);
  int d = (int)(fabsf(u) * 255.0f);
  // Nunca se activan RPWM y LPWM a la vez
  if (d == 0)      { PWM_WRITE(rpwm, chR, 0); PWM_WRITE(lpwm, chL, 0); }
  else if (u > 0)  { PWM_WRITE(lpwm, chL, 0); PWM_WRITE(rpwm, chR, d); }
  else             { PWM_WRITE(rpwm, chR, 0); PWM_WRITE(lpwm, chL, d); }
}

static void enableDrivers(bool on) {
  digitalWrite(L_EN, on ? HIGH : LOW);
  digitalWrite(R_EN, on ? HIGH : LOW);
}

static void stopMotors() {
  dutyL = dutyR = 0;
  intL = intR = 0;
  setMotor(L_RPWM, 0, L_LPWM, 1, 0);
  setMotor(R_RPWM, 2, R_LPWM, 3, 0);
}

// PWM con signo para una velocidad de rueda en mm/s. 'frac' es la fracción de MOTOR_NOM_MV:
// el voltaje efectivo es duty*batería = frac*12 V  (nunca más de 12 V aunque el pack tenga 16 V).
static float openLoopDuty(float vmm) {
  if (fabsf(vmm) < 5.0f) return 0;
  float frac = MIN_DUTY + (1.0f - MIN_DUTY) * fminf(fabsf(vmm) / V_MAX_MM_S, 1.0f);
  float duty = (battMv > 8000.0f) ? frac * MOTOR_NOM_MV / battMv : frac;
  return copysignf(fminf(duty, 1.0f), vmm);
}

static float slew(float cur, float target) {
  float d = target - cur;
  if (d > DUTY_SLEW) d = DUTY_SLEW;
  if (d < -DUTY_SLEW) d = -DUTY_SLEW;
  return cur + d;
}

// ======================= ENCODERS =======================
#if ENCODERS_PRESENT
void IRAM_ATTR isrL() {
#if ENC_QUADRATURE
  encL += ENC_L_SIGN * ((digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) ? 1 : -1);
#else
  encL += ENC_L_SIGN * dirL;
#endif
}
void IRAM_ATTR isrR() {
#if ENC_QUADRATURE
  encR += ENC_R_SIGN * ((digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) ? 1 : -1);
#else
  encR += ENC_R_SIGN * dirR;
#endif
}
#endif

// ======================= IMU =======================
static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

// Busca el chip en 0x68/0x69 y lee WHO_AM_I (0x75): 0x68=MPU6050, 0x70=MPU6500, 0x71/0x73=MPU9250/9255.
static bool mpuInit() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  const uint8_t addrs[2] = {0x68, 0x69};
  bool found = false;
  for (int i = 0; i < 2 && !found; i++) {
    uint8_t id = 0;
    if (i2cRead(addrs[i], 0x75, &id, 1)) { mpuAddr = addrs[i]; mpuWho = id; found = true; }
  }
  if (!found) return false;
  if (!mpuWrite(0x6B, 0x00)) return false;   // salir de sleep
  mpuWrite(0x1B, 0x08);                      // giroscopio ±500 °/s (65.5 LSB/°/s)
  mpuWrite(0x1A, 0x03);                      // filtro pasa-bajos ~44 Hz
  return true;
}

static bool readGyroZ(float *dps) {
  uint8_t b[2];
  if (!i2cRead(mpuAddr, 0x47, b, 2)) return false;
  *dps = (int16_t)((b[0] << 8) | b[1]) / 65.5f;
  return true;
}

// Robot QUIETO. Promedia ~1 s para estimar el sesgo del giroscopio.
static void calibrateGyro() {
  stopMotors();
  float sum = 0;
  int n = 0;
  for (int i = 0; i < 400; i++) {
    float g;
    if (readGyroZ(&g)) { sum += g; n++; }
    delay(2);
  }
  if (n > 100) { gyroBias = sum / n; imuOk = true; }
  else imuOk = false;
  yawDeg = 0;
}

static void integrateYaw(float dt) {
  float g;
  if (!readGyroZ(&g)) { imuOk = false; return; }
  imuOk = true;
  float r = g - gyroBias;
  if (fabsf(r) < 0.05f) r = 0;               // banda muerta contra ruido en reposo
  yawDeg += r * GYRO_YAW_SIGN * dt;
}

// ======================= IR DE PRECIPICIO =======================
// Lectura de los 4 sensores. Con emisor conmutado se resta la luz ambiente (sol, lámparas).
static void measureCliffs(int *out) {
  int on[NUM_CLIFF];
#if CLIFF_EMIT_PIN >= 0
  digitalWrite(CLIFF_EMIT_PIN, HIGH);
  delayMicroseconds(300);
#endif
  for (int i = 0; i < NUM_CLIFF; i++) {
    int s = 0;
    for (int k = 0; k < 4; k++) s += analogRead(CLIFF_PINS[i]);
    on[i] = s / 4;
  }
#if CLIFF_EMIT_PIN >= 0
  digitalWrite(CLIFF_EMIT_PIN, LOW);
  delayMicroseconds(300);
  for (int i = 0; i < NUM_CLIFF; i++) {
    int s = 0;
    for (int k = 0; k < 4; k++) s += analogRead(CLIFF_PINS[i]);
    out[i] = on[i] - s / 4;
  }
#else
  for (int i = 0; i < NUM_CLIFF; i++) out[i] = on[i];
#endif
}

// Robot sobre el PISO normal. Guarda la lectura de referencia de cada sensor.
static void calibrateCliffs() {
  long acc[NUM_CLIFF] = {0};
  for (int k = 0; k < 24; k++) {
    int v[NUM_CLIFF];
    measureCliffs(v);
    for (int i = 0; i < NUM_CLIFF; i++) acc[i] += v[i];
    delay(3);
  }
  for (int i = 0; i < NUM_CLIFF; i++) cliffBase[i] = (int)(acc[i] / 24);
}

static void updateCliffs() {
  int v[NUM_CLIFF];
  measureCliffs(v);
  cliffMask = 0;
  for (int i = 0; i < NUM_CLIFF; i++) {
    int delta = abs(v[i] - cliffBase[i]);
    int thr = (int)fmaxf((float)CLIFF_MIN_DELTA, fabsf((float)cliffBase[i]) * CLIFF_DELTA_RATIO);
    if (delta > thr) cliffMask |= (1 << i);
  }
  cliffActive = (cliffMask != 0);
}

// ======================= BATERÍA =======================
static void updateBattery() {
  float mv = analogReadMilliVolts(BATT_PIN) * BATT_DIV_RATIO * BATT_CAL;
  battMv = (battMv < 1.0f) ? mv : battMv + 0.1f * (mv - battMv);
  uint32_t now = millis();
  if (battMv < BATT_CRIT_MV) {
    if (battCritSince == 0) battCritSince = now;
    else if (now - battCritSince > BATT_CRIT_HOLD_MS) battCritLatched = true;
  } else {
    battCritSince = 0;
  }
}

// ======================= PROTOCOLO =======================
static void sendFramed(const char *payload) {
  uint8_t x = 0;
  for (const char *p = payload; *p; p++) x ^= (uint8_t)*p;
  Serial.printf("$%s*%02X\n", payload, x);
}

static void sendTelemetry() {
  char buf[120];
  snprintf(buf, sizeof(buf), "T,%lu,%d,%ld,%ld,%ld,%u,%u", (unsigned long)millis(), (int)battMv,
           (long)(yawDeg * 1000.0f), (long)encL, (long)encR, (unsigned)cliffMask, (unsigned)faults);
  sendFramed(buf);
}

static void sendImuInfo() {                  // $W,<direccion I2C>,<WHO_AM_I>,<ok>
  char buf[48];
  snprintf(buf, sizeof(buf), "W,%u,%u,%u", (unsigned)mpuAddr, (unsigned)mpuWho, (unsigned)imuOk);
  sendFramed(buf);
}

static void handleLine(char *line) {
  if (line[0] != '$') return;
  char *star = strrchr(line, '*');
  if (!star) return;
  *star = 0;
  uint8_t x = 0;
  for (char *p = line + 1; *p; p++) x ^= (uint8_t)*p;
  if ((uint8_t)strtol(star + 1, NULL, 16) != x) return;   // checksum inválido: se ignora

  char *p = line + 1;
  switch (p[0]) {
    case 'D': {
      long v = 0, w = 0;
      if (sscanf(p, "D,%ld,%ld", &v, &w) == 2) {
        cmdV = constrain((float)v, -V_MAX_MM_S, V_MAX_MM_S);
        cmdW = constrain(w / 1000.0f, -3.0f, 3.0f);
        lastCmdMs = millis();
      }
      break;
    }
    case 'S': cmdV = 0; cmdW = 0; lastCmdMs = millis(); break;
    case 'K': cmdV = cmdW = 0; stopMotors(); calibrateCliffs(); break;
    case 'Z': yawDeg = 0; encL = 0; encR = 0; encLPrev = 0; encRPrev = 0; break;
    case 'G': cmdV = cmdW = 0; calibrateGyro(); break;
    case 'W': sendImuInfo(); break;
    case 'C': if (battMv > BATT_CRIT_MV + 800) { battCritLatched = false; battCritSince = 0; } break;
    default: break;
  }
}

static void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) { rxBuf[rxLen] = 0; handleLine(rxBuf); }
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;                              // línea demasiado larga: descartar
    }
  }
}

// ======================= CONTROL (50 Hz) =======================
static void controlStep(float dt) {
  updateBattery();
  updateCliffs();

  float v = cmdV, w = cmdW;
  faults = 0;

  if (millis() - lastCmdMs > WATCHDOG_MS) { v = 0; w = 0; faults |= F_WATCHDOG; }
  if (battMv < BATT_LOW_MV) faults |= F_BATT_LOW;
  if (battCritLatched) { v = 0; w = 0; faults |= F_BATT_CRIT; }
  if (!imuOk) faults |= F_IMU;
  if (cliffActive) {                          // solo se permite RETROCEDER
    faults |= F_CLIFF;
    if (v > 0) v = 0;
    if (v >= 0) w = 0;
  }

  float halfBase = w * 1000.0f * WHEEL_BASE_M * 0.5f;   // mm/s
  float vl = v - halfBase;
  float vr = v + halfBase;
  float tl = openLoopDuty(vl);
  float tr = openLoopDuty(vr);

#if ENCODERS_PRESENT && ENC_CLOSED_LOOP
  int32_t el = encL, er = encR;
  float mL = (el - encLPrev) / TICKS_PER_M * 1000.0f / dt;   // mm/s medidos
  float mR = (er - encRPrev) / TICKS_PER_M * 1000.0f / dt;
  encLPrev = el; encRPrev = er;
  if (tl == 0) intL = 0;
  else { float e = (vl - mL) / V_MAX_MM_S; intL = constrain(intL + e * dt, -0.3f, 0.3f); tl += PID_KP * e + PID_KI * intL; }
  if (tr == 0) intR = 0;
  else { float e = (vr - mR) / V_MAX_MM_S; intR = constrain(intR + e * dt, -0.3f, 0.3f); tr += PID_KP * e + PID_KI * intR; }
  // el PID tampoco puede superar los 12 V efectivos
  float cap = (battMv > 8000.0f) ? fminf(1.0f, MOTOR_NOM_MV / battMv) : 1.0f;
  tl = constrain(tl, -cap, cap);
  tr = constrain(tr, -cap, cap);
#endif

  if (v == 0 && w == 0) { tl = 0; tr = 0; }
  dutyL = slew(dutyL, constrain(tl, -1.0f, 1.0f));
  dutyR = slew(dutyR, constrain(tr, -1.0f, 1.0f));
  if (cliffActive || battCritLatched) { dutyL = tl; dutyR = tr; }   // frenado inmediato, sin rampa

  enableDrivers(!battCritLatched);            // batería crítica: se corta la etapa de potencia
  dirL = (dutyL >= 0) ? 1 : -1;
  dirR = (dutyR >= 0) ? 1 : -1;
  setMotor(L_RPWM, 0, L_LPWM, 1, MOTOR_L_SIGN * dutyL);
  setMotor(R_RPWM, 2, R_LPWM, 3, MOTOR_R_SIGN * dutyR);
}

// ======================= SETUP / LOOP =======================
static uint32_t tImuUs = 0, tCtl = 0, tTel = 0, tDbg = 0;

void setup() {
  Serial.begin(115200);
  pinMode(L_EN, OUTPUT); pinMode(R_EN, OUTPUT);
  enableDrivers(false);                       // motores sin energía hasta terminar la inicialización
  PWM_ATTACH(L_RPWM, 0); PWM_ATTACH(L_LPWM, 1);
  PWM_ATTACH(R_RPWM, 2); PWM_ATTACH(R_LPWM, 3);
  stopMotors();

  analogSetAttenuation(ADC_11db);            // rango ~0-3.1 V en los ADC
  analogReadResolution(12);

#if CLIFF_EMIT_PIN >= 0
  pinMode(CLIFF_EMIT_PIN, OUTPUT);
  digitalWrite(CLIFF_EMIT_PIN, LOW);
#endif

#if ENCODERS_PRESENT
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, CHANGE);
#endif

  imuOk = mpuInit();
  delay(300);
  calibrateGyro();                            // robot quieto al encender
  calibrateCliffs();                          // robot sobre el piso al encender
  battMv = 0;
  updateBattery();

  lastCmdMs = millis();
  tImuUs = micros();
  enableDrivers(true);
}

void loop() {
  readSerial();

  uint32_t nowUs = micros();
  if (nowUs - tImuUs >= 5000) {
    float dt = (nowUs - tImuUs) * 1e-6f;
    tImuUs = nowUs;
    integrateYaw(dt);
  }

  uint32_t ms = millis();
  if (ms - tCtl >= 20) { tCtl = ms; controlStep(0.02f); }
  if (ms - tTel >= 50) { tTel = ms; sendTelemetry(); }

#if DEBUG_CLIFF_RAW
  if (ms - tDbg >= 500) {
    tDbg = ms;
    int v[NUM_CLIFF];
    measureCliffs(v);
    char buf[80];
    snprintf(buf, sizeof(buf), "R,%d,%d,%d,%d", v[0], v[1], v[2], v[3]);
    sendFramed(buf);
  }
#endif
}
