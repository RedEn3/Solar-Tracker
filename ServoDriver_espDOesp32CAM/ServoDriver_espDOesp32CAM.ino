
#include <math.h>
#include <stdio.h>

// WIFI_AP settings.
const char* AP_SSID = "ESP32_DEV";
const char* AP_PWD  = "12345678";

// WIFI_STA settings.
const char* STA_SSID = "OnePlus 8";
const char* STA_PWD  = "TU_WPISZ_SWOJE_HASLO";

// ESP-NOW
uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0x93, 0x5F, 0xA8};

typedef struct struct_message {
  int ID_send;
  int POS_send;
  int Spd_send;
} struct_message;

struct_message myData;

#define DEFAULT_ROLE 0
#define DEFAULT_WIFI_MODE 1

#define S_RXD 18
#define S_TXD 19
#define S_SCL 22
#define S_SDA 21
#define RGB_LED   23
#define NUMPIXELS 10

int MAX_ID = 20;
bool SERIAL_FORWARDING = false;

String MAC_ADDRESS;
IPAddress IP_ADDRESS;
byte SERVO_NUMBER;
byte DEV_ROLE;
byte WIFI_MODE;
int WIFI_RSSI;

#define threadingInterval 600
#define clientInterval    10

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include "RGB_CTRL.h"
#include "STSCTRL.h"
#include "CONNECT.h"
#include "BOARD_DEV.h"


// ==================================================
// PAN AUTO-POINTING
// ==================================================

#define PAN_ID 1

// 0 = Polnoc, 90 = Wschod, 180 = Poludnie, 270 = Zachod
float targetHeadingDeg = 270.0;

const float PAN_MAX_RANGE_DEG = 270.0;
const float STEPS_PER_DEG = 4095.0 / 360.0;

const int PAN_MIN_STEPS =
    (int)(2047 - (PAN_MAX_RANGE_DEG / 2.0) * STEPS_PER_DEG);

const int PAN_MAX_STEPS =
    (int)(2047 + (PAN_MAX_RANGE_DEG / 2.0) * STEPS_PER_DEG);

int panSpeed = 900;

// Filtr EMA
float filteredError = 0.0;

const float FILTER_ALPHA = 0.15;
const float DEADBAND_DEG = 1.5;

// Maksymalny akceptowany przechyl
const float TRUST_LIMIT_DEG = 65.0;


// ==================================================
// UART2 - ODBIOR DANYCH Z ESP32-CAM
// ==================================================

HardwareSerial ArduinoSerial(2);

String arduinoFrame = "";


// ==================================================
// FUNKCJE STEROWANIA SERWEM
// ==================================================

int clampSteps(int steps, int minS, int maxS) {
  if (steps < minS) return minS;
  if (steps > maxS) return maxS;

  return steps;
}


void moveServoDeg(int id, float deg, int minS, int maxS, int speed) {

  int steps = clampSteps(
    (int)(2047 + deg * STEPS_PER_DEG),
    minS,
    maxS
  );

  st.WritePosEx(id, steps, speed, ServoInitACC);
}


float angleError(float current, float target) {

  float error = target - current;

  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;

  return error;
}


// ==================================================
// PRZETWARZANIE ODEBRANEJ RAMKI
// ==================================================

void processSensorFrame(String frame) {

  float heading;
  float pitch;
  float roll;

  // Odczyt formatu: heading,pitch,roll
  if (sscanf(
        frame.c_str(),
        "%f,%f,%f",
        &heading,
        &pitch,
        &roll
      ) != 3) {

    Serial.print("Niepoprawna ramka: ");
    Serial.println(frame);

    return;
  }

  // Kontrola przechylu - oryginalna logika
  bool trustworthy =
      (abs(roll) < TRUST_LIMIT_DEG) &&
      (abs(pitch) < TRUST_LIMIT_DEG);

  if (!trustworthy) {

    Serial.print("IGNOROWANO - zbyt mocny przechyl. Pitch: ");
    Serial.print(pitch);

    Serial.print(" Roll: ");
    Serial.println(roll);

    return;
  }

  // Obliczenie bledu kierunku
  float headingError = angleError(
    heading,
    targetHeadingDeg
  );

  // Oryginalny filtr EMA
  filteredError =
      filteredError +
      FILTER_ALPHA * (headingError - filteredError);

  // Sterowanie serwem - oryginalna logika
  if (abs(filteredError) > DEADBAND_DEG) {

    moveServoDeg(
      PAN_ID,
      -filteredError,
      PAN_MIN_STEPS,
      PAN_MAX_STEPS,
      panSpeed
    );
  }

  // Dane diagnostyczne
  Serial.print("Heading: ");
  Serial.print(heading, 1);

  Serial.print("  Pitch: ");
  Serial.print(pitch, 1);

  Serial.print("  Roll: ");
  Serial.print(roll, 1);

  Serial.print("  Cel: ");
  Serial.print(targetHeadingDeg, 1);

  Serial.print("  Blad(filt): ");
  Serial.println(filteredError, 2);
}


// ==================================================
// ODBIOR RAMKI UART
// ==================================================

void readSensorUART() {

  while (ArduinoSerial.available()) {

    char c = ArduinoSerial.read();

    if (c == '<') {

      // Początek nowej ramki
      arduinoFrame = "";

    }

    else if (c == '>') {

      // Koniec ramki
      if (arduinoFrame.length() > 0) {
        processSensorFrame(arduinoFrame);
      }

      arduinoFrame = "";

    }

    else {

      // Zabezpieczenie bufora
      if (arduinoFrame.length() < 64) {

        arduinoFrame += c;

      } else {

        arduinoFrame = "";
      }
    }
  }
}


// ==================================================
// SETUP
// ==================================================

void setup() {

  // USB Serial Monitor
  Serial.begin(115200);

  // ESP32-CAM TX GPIO13 -> Waveshare RX GPIO16
  ArduinoSerial.begin(
    115200,
    SERIAL_8N1,
    16,
    17
  );

  while (!Serial) {}

  // Oryginalna inicjalizacja Waveshare
  espNowInit();

  getMAC();

  boardDevInit();

  RGBcolor(0, 64, 255);

  servoInit();

  wifiInit();

  webServerSetup();

  RGBoff();

  delay(1000);

  pingAll(true);

  threadInit();

  Serial.println();
  Serial.println("================================");
  Serial.println("WAVESHARE SERVO DRIVER READY");
  Serial.println("UART2 RX: GPIO16");
  Serial.println("UART2 TX: GPIO17");
  Serial.println("Baudrate: 115200");
  Serial.println("================================");
}


// ==================================================
// LOOP
// ==================================================

void loop() {

  // Odbieranie danych z ESP32-CAM
  readSensorUART();

}
