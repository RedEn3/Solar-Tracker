#include <Wire.h>
#include <Adafruit_QMC5883P.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_QMC5883P qmc;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

bool compassInitialized = false;
bool accelInitialized = false;

// Offsety kalibracyjne magnetometru
const float OFFSET_X = 164.0;
const float OFFSET_Y = -108.0;
const float OFFSET_Z = 6.0;

float angleError(float current, float target)
{
    float error = target - current;

    while (error > 180.0) error -= 360.0;
    while (error < -180.0) error += 360.0;

    return error;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin();

  if (!qmc.begin()) {
    Serial.println("Brak QMC5883P!");
  } else {
    qmc.setMode(QMC5883P_MODE_NORMAL);
    qmc.setODR(QMC5883P_ODR_50HZ);
    qmc.setOSR(QMC5883P_OSR_4);
    qmc.setDSR(QMC5883P_DSR_2);
    qmc.setRange(QMC5883P_RANGE_8G);
    qmc.setSetResetMode(QMC5883P_SETRESET_ON);
    compassInitialized = true;
  }

  if (!accel.begin()) {
    Serial.println("Brak ADXL345!");
  } else {
    accel.setRange(ADXL345_RANGE_2_G);
    accelInitialized = true;
  }
}

void loop() {

  if (!(compassInitialized && accelInitialized))
    return;

  if (!qmc.isDataReady())
    return;

  int16_t mx_raw, my_raw, mz_raw;

  if (!qmc.getRawMagnetic(&mx_raw, &my_raw, &mz_raw))
    return;

  sensors_event_t event;
  accel.getEvent(&event);

  float ax_raw = event.acceleration.x;
  float ay_raw = event.acceleration.y;
  float az_raw = event.acceleration.z;

  float mx_in = (float)mx_raw - OFFSET_X;
  float my_in = (float)my_raw - OFFSET_Y;
  float mz_in = (float)mz_raw - OFFSET_Z;

  // Akcelerometr -> FRD
  float G_x = ay_raw;
  float G_y = ax_raw;
  float G_z = az_raw;

  // Magnetometr -> FRD
  float B_x = mz_in;
  float B_y = -my_in;
  float B_z = -mx_in;

  float roll  = atan2(G_y, G_z);
  float pitch = atan2(-G_x, G_y * sin(roll) + G_z * cos(roll));

  float cosRoll = cos(roll);
  float sinRoll = sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  float B_fx = B_x * cosPitch +
               B_y * sinPitch * sinRoll +
               B_z * sinPitch * cosRoll;

  float B_fy = B_y * cosRoll +
               B_z * sinRoll;

  float heading = atan2(-B_fy, B_fx) * 180.0 / PI;

  while (heading < 0) heading += 360;
  while (heading >= 360) heading -= 360;

  float pitchDeg = -pitch * 180.0 / PI;
  float rollDeg  = -roll * 180.0 / PI;

 // Kierunek zadany przez ESP (na razie na sztywno)
  float targetHeading = 270.0;   // Zachód

  float headingError = angleError(heading, targetHeading);

  Serial.print("<");
  Serial.print(headingError, 1);
  Serial.print(",");
  Serial.print(pitchDeg, 1);
  Serial.print(",");
  Serial.print(rollDeg, 1);
  Serial.println(">");

  delay(50);
}