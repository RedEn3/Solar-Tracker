
#include <Wire.h>
#include <Adafruit_QMC5883P.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>

// =====================================================
// ESP32-CAM AI-THINKER
// =====================================================

// I2C
#define I2C_SDA 15
#define I2C_SCL 14

// UART do Waveshare
#define UART_TX 13

// Czujniki
Adafruit_QMC5883P qmc;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

bool compassInitialized = false;
bool accelInitialized = false;

// Offsety kalibracyjne magnetometru
const float OFFSET_X = 164.0;
const float OFFSET_Y = -108.0;
const float OFFSET_Z = 6.0;

// Interwał pomiarowy
const unsigned long SEND_INTERVAL = 50;
unsigned long lastSendTime = 0;

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== SOLAR TRACKER - ESP32-CAM ===");

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // UART: tylko TX, RX nieużywany
  Serial2.begin(115200, SERIAL_8N1, -1, UART_TX);

  Serial.println("UART TX GPIO13 -> Waveshare RX GPIO16");

  // QMC5883P
  if (!qmc.begin()) {
    Serial.println("BLAD: Brak QMC5883P!");
  } else {
    qmc.setMode(QMC5883P_MODE_NORMAL);
    qmc.setODR(QMC5883P_ODR_50HZ);
    qmc.setOSR(QMC5883P_OSR_4);
    qmc.setDSR(QMC5883P_DSR_2);
    qmc.setRange(QMC5883P_RANGE_8G);
    qmc.setSetResetMode(QMC5883P_SETRESET_ON);

    compassInitialized = true;
    Serial.println("QMC5883P OK");
  }

  // ADXL345
  if (!accel.begin()) {
    Serial.println("BLAD: Brak ADXL345!");
  } else {
    accel.setRange(ADXL345_RANGE_2_G);

    accelInitialized = true;
    Serial.println("ADXL345 OK");
  }

  if (compassInitialized && accelInitialized) {
    Serial.println("CZUJNIKI GOTOWE - WYSYLANIE DANYCH");
  } else {
    Serial.println("BLAD INICJALIZACJI CZUJNIKOW!");
  }
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  if (!(compassInitialized && accelInitialized)) {
    delay(500);
    return;
  }

  if (millis() - lastSendTime < SEND_INTERVAL)
    return;

  if (!qmc.isDataReady())
    return;

  lastSendTime = millis();

  // ---------------------------------------------------
  // 1. ODCZYT MAGNETOMETRU
  // ---------------------------------------------------

  int16_t mx_raw, my_raw, mz_raw;

  if (!qmc.getRawMagnetic(&mx_raw, &my_raw, &mz_raw))
    return;

  // ---------------------------------------------------
  // 2. ODCZYT AKCELEROMETRU
  // ---------------------------------------------------

  sensors_event_t event;
  accel.getEvent(&event);

  float ax_raw = event.acceleration.x;
  float ay_raw = event.acceleration.y;
  float az_raw = event.acceleration.z;

  // ---------------------------------------------------
  // 3. KALIBRACJA MAGNETOMETRU
  // ---------------------------------------------------

  float mx_in = (float)mx_raw - OFFSET_X;
  float my_in = (float)my_raw - OFFSET_Y;
  float mz_in = (float)mz_raw - OFFSET_Z;

  // ---------------------------------------------------
  // 4. AKCELEROMETR -> FRD
  // ---------------------------------------------------

  float G_x = ay_raw;
  float G_y = -ax_raw;
  float G_z = -az_raw;

  // ---------------------------------------------------
  // 5. MAGNETOMETR -> FRD
  // ---------------------------------------------------

  float B_x = mz_in;
  float B_y = my_in;
  float B_z = mx_in;

  // ---------------------------------------------------
  // 6. ROLL I PITCH
  // ---------------------------------------------------

  float roll = atan2(G_y, G_z);

  float pitch = atan2(
    -G_x,
    G_y * sin(roll) + G_z * cos(roll)
  );

  float cosRoll = cos(roll);
  float sinRoll = sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // ---------------------------------------------------
  // 7. KOMPENSACJA PRZECHYŁU MAGNETOMETRU
  // ---------------------------------------------------

  float B_fx =
    B_x * cosPitch +
    B_y * sinPitch * sinRoll +
    B_z * sinPitch * cosRoll;

  float B_fy =
    B_y * cosRoll +
    B_z * sinRoll;

  // ---------------------------------------------------
  // 8. HEADING
  // ---------------------------------------------------

  float heading = atan2(B_fy, B_fx) * 180.0 / PI;

  heading += 160.0;

  while (heading < 0) heading += 360.0;
  while (heading >= 360) heading -= 360.0;

  // ---------------------------------------------------
  // 9. PITCH I ROLL W STOPNIACH
  // ---------------------------------------------------

  float pitchDeg = -pitch * 180.0 / PI;
  float rollDeg  = -roll * 180.0 / PI;

  // ---------------------------------------------------
  // 10. WYSYŁANIE DO WAVESHARE
  // ---------------------------------------------------

  Serial2.print("<");
  Serial2.print(heading, 1);
  Serial2.print(",");
  Serial2.print(pitchDeg, 1);
  Serial2.print(",");
  Serial2.print(rollDeg, 1);
  Serial2.println(">");

  // ---------------------------------------------------
  // 11. DIAGNOSTYKA USB
  // ---------------------------------------------------

  Serial.print("<");
  Serial.print(heading, 1);
  Serial.print(",");
  Serial.print(pitchDeg, 1);
  Serial.print(",");
  Serial.print(rollDeg, 1);
  Serial.println(">");
}
