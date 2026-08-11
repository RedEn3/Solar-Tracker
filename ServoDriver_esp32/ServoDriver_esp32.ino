// WIFI_AP settings.
const char* AP_SSID = "ESP32_DEV";
const char* AP_PWD  = "12345678";

// WIFI_STA settings.
const char* STA_SSID = "OnePlus 8";
const char* STA_PWD  = "40963840";

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
byte   SERVO_NUMBER;
byte   DEV_ROLE;
byte   WIFI_MODE;
int    WIFI_RSSI;

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

// ================== NASZA CZĘŚĆ: PAN AUTO-POINTING ==================

#define PAN_ID  1

// >>> TU ZMIENIASZ KIERUNEK W PRZYSZŁOŚCI - JEDNA LINIJKA <
float targetHeadingDeg = 270.0;   // 0=Polnoc, 90=Wschod, 180=Poludnie, 270=Zachod

const float PAN_MAX_RANGE_DEG = 270.0;
const float STEPS_PER_DEG = 4095.0 / 360.0;

const int PAN_MIN_STEPS = (int)(2047 - (PAN_MAX_RANGE_DEG / 2.0) * STEPS_PER_DEG);
const int PAN_MAX_STEPS = (int)(2047 + (PAN_MAX_RANGE_DEG / 2.0) * STEPS_PER_DEG);

int panSpeed = 900;   // predkosc pan-a, dostrojaj tu

// filtr + strefa martwa - MUSZA byc globalne, zeby pamietac stan miedzy klatkami
float filteredError = 0.0;
const float FILTER_ALPHA = 0.15;   // 0.05=bardzo gladko/wolno, 0.3=szybciej ale mniej gladko
const float DEADBAND_DEG = 1.5;    // ponizej tego serwo sie nie rusza

// prog zaufania - powyzej tego przechylu ignorujemy odczyt (gimbal lock ryzyko)
const float TRUST_LIMIT_DEG = 65.0;

HardwareSerial ArduinoSerial(2);   // RX=16, TX=17 (od Arduino)
String arduinoFrame = "";

int clampSteps(int steps, int minS, int maxS){
  if (steps < minS) return minS;
  if (steps > maxS) return maxS;
  return steps;
}

void moveServoDeg(int id, float deg, int minS, int maxS, int speed){
  int steps = clampSteps((int)(2047 + deg * STEPS_PER_DEG), minS, maxS);
  st.WritePosEx(id, steps, speed, ServoInitACC);
}

float angleError(float current, float target){
  float error = target - current;
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;
  return error;
}

// ================== SETUP / LOOP ==================

void setup() {
  ArduinoSerial.begin(115200, SERIAL_8N1, 16, 17);
  Serial.begin(115200);
  while(!Serial) {}

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
}

void loop() {
  while (ArduinoSerial.available()) {
    char c = ArduinoSerial.read();
    if (c == '<') {
      arduinoFrame = "";
    }
    else if (c == '>') {
      float heading, pitch, roll;
      if (sscanf(arduinoFrame.c_str(), "%f,%f,%f", &heading, &pitch, &roll) == 3) {

        // strefa niezaufania - przy mocnym przechyle odczyt heading moze byc niewiarygodny
        bool trustworthy = (abs(roll) < TRUST_LIMIT_DEG) && (abs(pitch) < TRUST_LIMIT_DEG);

        if (trustworthy) {
          float headingError = angleError(heading, targetHeadingDeg);

          // filtr wygladzajacy (EMA)
          filteredError = filteredError + FILTER_ALPHA * (headingError - filteredError);

          if (abs(filteredError) > DEADBAND_DEG) {
            moveServoDeg(PAN_ID, -filteredError, PAN_MIN_STEPS, PAN_MAX_STEPS, panSpeed);
          }

          Serial.print("Heading: "); Serial.print(heading);
          Serial.print("  Cel: "); Serial.print(targetHeadingDeg);
          Serial.print("  Blad(filt): "); Serial.println(filteredError);
        }
        else {
          Serial.print("IGNOROWANO - zbyt mocny przechyl. Pitch:");
          Serial.print(pitch);
          Serial.print(" Roll:");
          Serial.println(roll);
        }
      }
    }
    else {
      arduinoFrame += c;
    }
  }
}