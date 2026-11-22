# Conexiones — Raspberry Pi 5 + ESP32-S3-N16R8 (dos baterías)

> ⚠️ Esquema **sin probar en tu robot**. Antes de energizar haz las verificaciones con multímetro del final.

## 1. Idea general: dos baterías separadas

| Batería | Alimenta SOLO a |
|---|---|
| **A) Pack Ni-MH 14.4 V** (la del Roomba) | Los 2 BTS7960 → motores de ruedas (y, si quieres, los emisores IR y el divisor de medición) |
| **B) Batería externa de 5 V (power bank USB-C)** | Raspberry Pi 5 → y por USB: ESP32-S3, LIDAR (vía adaptador USB-UART) y cámara |

```
 BATERÍA A  14.4 V (motores)                      BATERÍA B  power bank 5 V
 (+)──[FUSIBLE]──[INTERRUPTOR]──┬─ B+ BTS7960 izq        USB-C ──► Raspberry Pi 5
                                └─ B+ BTS7960 der                     │ USB ──► ESP32-S3 (puerto "UART")
 (−)───────────────────────────┬─ B− BTS7960 izq                      │ USB ──► adaptador USB-UART ◄─ LIDAR
                               └─ B− BTS7960 der                      └ USB ──► webcam
                                      │
                    TIERRA COMÚN: unir (−) de la batería A  ⟷  GND del ESP32-S3
                    (basta con conectar el pin GND de los BTS7960 al GND del ESP32; en el módulo
                     ese GND ya está unido a B−: compruébalo con continuidad)
```

**Qué cambia respecto a usar un solo pack**
1. **Desaparece el convertidor (buck) de 5 V.** La Pi y el ESP32 ya no cuelgan del pack de motores.
2. **La tierra SIGUE siendo común.** Aunque las baterías son independientes, las señales PWM del ESP32 necesitan la misma referencia de 0 V que los BTS7960. Sin ese cable de GND, los motores se comportan de forma errática.
3. **El 5 V lógico de los BTS7960** (pin VCC) sale ahora del **pin "5V" del ESP32-S3** (que es el 5 V del USB de la Pi; consumen solo unos mA). Antes salía del buck.
4. **Las resistencias de 10 kΩ a GND en RPWM/LPWM/EN pasan a ser aún más importantes:** si la batería B se agota o apagas la Pi, el ESP32 se apaga y esas resistencias mantienen los motores parados.
5. **La medición de batería (GPIO 1) ahora mide solo la batería A (motores)**, que es la que limita la autonomía. **La batería B no se monitorea:** si se agota, la Pi se apaga de golpe (riesgo para la tarjeta SD y el robot se detiene). Usa una de buena capacidad (≥ 20 000 mAh) y apaga la Pi con `sudo shutdown` antes de cortar la energía. Puedes revisar `vcgencmd get_throttled` para detectar bajo voltaje.
6. **Requisitos de la batería B:** salida USB-C con **al menos 5 V / 3 A** (lo ideal es PD de 5 V / 5 A para la Pi 5; con 3 A quizá necesites `usb_max_current_enable=1` en `/boot/firmware/config.txt`: verifica la documentación oficial). Evita power banks que se apagan solos con poca corriente.
7. **Dos interruptores independientes** son recomendables: uno para la batería A (motores) y la propia de la batería B. Encender la B sin la A es seguro (motores sin energía); apagar la B con la A encendida también (resistencias pull-down).
8. Los cepillos/succión (siempre encendidos) tienen su propia alimentación y fusible; únelos también a la tierra común.

## 2. Mapa de pines del ESP32-S3-N16R8

En este módulo **no se pueden usar**: GPIO **26–37** (flash/PSRAM), **19–20** (USB), **43–44** (UART0), **0, 3, 45, 46** (arranque).

| GPIO | Función | Va a | Resistencia / componente |
|---|---|---|---|
| 38 | RPWM motor izq | BTS7960 izq · RPWM | **10 kΩ a GND** |
| 39 | LPWM motor izq | BTS7960 izq · LPWM | **10 kΩ a GND** |
| 40 | EN motor izq | BTS7960 izq · R_EN y L_EN (unidos) | **10 kΩ a GND** |
| 41 | RPWM motor der | BTS7960 der · RPWM | **10 kΩ a GND** |
| 42 | LPWM motor der | BTS7960 der · LPWM | **10 kΩ a GND** |
| 47 | EN motor der | BTS7960 der · R_EN y L_EN (unidos) | **10 kΩ a GND** |
| 8 / 9 | SDA / SCL (IMU) | IMU | pull-ups 4.7–10 kΩ a 3.3 V **solo si el módulo no los trae** |
| 10 / 11 | Encoder izq A / B | encoder izq | según tipo (§7) |
| 12 / 13 | Encoder der A / B | encoder der | según tipo (§7) |
| 1 | Medición de batería A | punto medio del divisor | **100 kΩ + 15 kΩ + 100 nF** (§4) |
| 2 / 4 / 5 / 6 | IR de precipicio ×4 | colector de cada fototransistor | **10 kΩ a 3.3 V** por canal (§6) |
| 7 | Conmutar emisores IR (opcional) | base de un transistor | **1 kΩ** en serie (§6) |
| **14** | **Sensor RCON** | cable blanco/amarillo | ninguna obligatoria (§5) |
| 3V3 | Alimenta IMU, RCON y pull-ups | — | — |
| 5V | Alimenta el VCC lógico de los BTS7960 | — | — |
| GND | Tierra común | todo | — |

## 3. BTS7960 (módulo "IBT-2" de 43 A), uno por motor

```
Lado de potencia                        Lado de control (8 pines)
  B+  ← batería A (+) tras fusible        RPWM ← GPIO 38 (izq) / 41 (der)   + 10 kΩ a GND
  B−  ← batería A (−)                     LPWM ← GPIO 39 (izq) / 42 (der)   + 10 kΩ a GND
  M+  → motor (un hilo)                   R_EN ┐ unidos ← GPIO 40 (izq) / 47 (der) + 10 kΩ a GND
  M−  → motor (otro hilo)                 L_EN ┘
                                          R_IS, L_IS → sin conectar
                                          VCC  ← pin 5V del ESP32-S3
                                          GND  ← GND del ESP32-S3   (esta es la tierra común)
```
* Los BTS7960 suelen aceptar 3.3 V en sus entradas; si un motor no responde, falta un adaptador de nivel (p. ej. 74HCT245).
* Recomendado: **1000 µF / 25 V** entre B+ y B− junto a cada módulo y **100 nF cerámico** entre los bornes de cada motor.
* Si un motor gira al revés: invierte `MOTOR_L_SIGN` / `MOTOR_R_SIGN` en el `.ino`.

## 4. Medición de la batería A (motores)

```
Batería A (+) tras el fusible ──[ R1 = 100 kΩ ]──┬──────────► GPIO 1
                                                  ├──[ R2 = 15 kΩ ]──► GND
                                                  └──[ C = 100 nF ]──► GND   (pegado al ESP32)
```
16.8 V → 2.19 V en el pin. Resistencias de película metálica 1 %. Ajusta `BATT_CAL` con un multímetro.

## 5. Sensor RCON (receptor infrarrojo de 38 kHz) — conexión sencilla

**Qué es:** un receptor de infrarrojo ya demodulado (como el de un control remoto de TV). **Solo avisa "estoy recibiendo infrarrojo a 38 kHz".**
**No es** un sensor de precipicio ni mide distancia, y con un solo cable de salida **no indica la dirección**. En el Roomba sirve para ver la base o las paredes virtuales; aquí sirve para detectar una baliza IR que tú pongas.

| Cable del RCON | Se conecta a | Para qué |
|---|---|---|
| **Rojo (VCC)** | **3V3** del ESP32-S3 | Alimentación. **Usa 3.3 V, no 5 V** (ver abajo) |
| **Negro (GND)** | **GND** del ESP32-S3 | Tierra |
| **Blanco o amarillo (OUT)** | **GPIO 14** | Señal: en reposo ALTA; baja a BAJA mientras recibe pulsos IR |

* **Resistencias: ninguna es obligatoria.** El sensor trae pull-up interna (y el ESP32 activa la suya). Opcionales: **1 kΩ** en serie en el cable de señal (protección) y **100 nF** entre VCC y GND pegado al sensor.
* **Por qué 3.3 V:** la salida es de colector abierto con pull-up hacia el voltaje con que lo alimentes. A 5 V la señal subiría a 5 V y dañaría el ESP32.
* **Antes de conectarlo al GPIO:** aliméntalo a 3.3 V y mide con el multímetro el cable de señal en reposo: debe dar **≈ 3.3 V, nunca más**.
* **Cómo probarlo:** ejecuta `python3 check_hardware.py esp32` y apunta un **control remoto de TV** al sensor: el número `rcon=` debe aumentar.
* Si quisieras usarlo como detector de obstáculos, tendrías que añadir tu propio LED IR modulado a 38 kHz; no está incluido.

## 6. IR de precipicio del Roomba (4 sensores, ⚠️ verifica antes de conectar)

**Receptores** (uno por sensor):
```
3.3 V ──[ 10 kΩ ]──┬──────► GPIO 2 / 4 / 5 / 6   (uno por sensor)
                   └── colector del fototransistor
Emisor del fototransistor (cable negro común) ──► GND
```
**El pull-up va a 3.3 V, nunca a 5 V.** Opcional por canal: 1 kΩ en serie hacia el GPIO y 10 nF a GND.

**Emisores** (cadena de 4 LED infrarrojos en serie). Como ya no hay un riel de 9 V, lo más simple es alimentarlos desde la **batería A (14.4 V)** con una resistencia:
1. Mide con el multímetro en *prueba de diodos* la caída total de la cadena, **ΣVf** (≈ 4.5–5.5 V para 4 LED).
2. Calcula **`R = (14.4 − ΣVf) / 0.015`**. Ejemplo: ΣVf = 4.8 V → R ≈ 640 Ω → usa **680 Ω, ½ W** (el voltaje del pack varía de 12 a 16.8 V; la corriente queda entre ~11 y ~18 mA).
3. Conexión, con interruptor electrónico (recomendado: cancela la luz del sol y ahorra batería):
```
Batería A (+) ──[ R ≈ 680 Ω ½W ]── cadena de 4 LED (ánodo → cátodo) ──► colector de un 2N2222
GPIO 7 ──[ 1 kΩ ]──► base del 2N2222                      emisor del 2N2222 ──► GND común
```
   y en el firmware pon `#define CLIFF_EMIT_PIN 7`. Sin transistor: la cadena va de R directo a GND (siempre encendida).
4. **Nunca** alimentes los LED desde un pin del ESP32. Mide ΣVf antes: con otro valor, la resistencia cambia.

## 7. Encoders (necesito saber cómo son)

| Tu encoder | Conexión |
|---|---|
| Se alimenta a **3.3 V** y sus salidas son de 3.3 V | A/B directo a GPIO 10–13; añade **4.7–10 kΩ a 3.3 V** si son de colector abierto o los cables son largos |
| Se alimenta a **5 V** con salidas push-pull de 5 V | ⚠️ **No directo.** Divisor por canal: **1 kΩ** en serie + **2 kΩ** a GND (5 V → 3.3 V), o adaptador de nivel; o aliméntalo a 3.3 V si lo admite |
| Colector abierto con 5 V | Pull-up de **4.7–10 kΩ a 3.3 V** (no a 5 V) |

Un canal en vez de dos: conecta solo A (GPIO 10 y 12) y pon `ENC_QUADRATURE 0` en el firmware.

## 8. IMU "HW-123"

`VCC ← 3V3` · `GND ← GND` · `SDA ↔ GPIO 8` · `SCL ↔ GPIO 9`. Aliméntala a **3.3 V** (no 5 V) para que sus pull-ups I2C no lleven 5 V al ESP32.
Móntala firme, lejos de los cables de motor.

## 9. LIDAR LD19 y cámara

* **LD19 → adaptador USB-UART (CP2102) → USB de la Pi.** TX del LIDAR → RXD del adaptador; GND ↔ GND; 5 V del LIDAR ← 5 V del adaptador. Sin resistencias.
* Su pin **PWM**: según la hoja de datos de LDROBOT, para la regulación interna de velocidad se conecta a GND; **confírmalo** antes de energizar.
* Webcam: a un USB 3 (azul).

## 10. Lista de resistencias y componentes

| Cantidad | Valor | Dónde |
|---|---|---|
| 6 | 10 kΩ | Pull-down de RPWM, LPWM y EN de cada BTS7960 → GND |
| 4 | 10 kΩ | Pull-up de cada fototransistor IR → **3.3 V** |
| 1 | 100 kΩ (1 %) | R1 del divisor de batería |
| 1 | 15 kΩ (1 %) | R2 del divisor de batería |
| 1 | 100 nF | Filtro del divisor (GPIO 1) |
| 1 | ≈ 680 Ω ½ W (calcular) | Resistencia de los emisores IR desde la batería A |
| 1 + 1 | 1 kΩ + 2N2222 | Conmutación de emisores IR (opcional) |
| 0–1 | 1 kΩ y 100 nF | RCON (opcionales) |
| 0–4 | 1 kΩ + 2 kΩ | Solo si los encoders son de 5 V push-pull (por canal) |
| 2 | 1000 µF / 25 V | Junto a cada BTS7960 (B+/B−) |
| 2 | 100 nF cerámico | Entre los bornes de cada motor |

## 11. Verificaciones ANTES de energizar

1. **Sin cortocircuito** entre B+ y B− (continuidad, batería A desconectada).
2. **Tierra común:** con todo apagado, mide continuidad entre el GND del ESP32 y el borne (−) de la batería A (≈ 0 Ω).
3. Conecta SOLO la batería B: mide el pin VCC de los BTS7960 (≈ 5 V). Luego conecta la A y mide el nodo del divisor (**≤ 2.3 V**).
4. Nodo de cada IR con su pull-up: **entre 0 y 3.3 V**, nunca más. Señal del RCON en reposo: **≈ 3.3 V**.
5. Confirma que ningún pin de 5 V ni de 14.4 V llega a un GPIO.
6. Primera prueba con **ruedas en el aire**, fusible puesto y mano en el interruptor de la batería A.
