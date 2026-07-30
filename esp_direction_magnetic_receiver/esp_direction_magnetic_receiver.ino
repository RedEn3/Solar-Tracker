HardwareSerial ArduinoSerial(2);

String frame = "";

void setup() {

  Serial.begin(115200);

  // RX = GPIO16
  // TX = GPIO17
  ArduinoSerial.begin(115200, SERIAL_8N1, 16, 17);

  Serial.println("ESP32 gotowe");
}

void loop() {

  while (ArduinoSerial.available()) {

    char c = ArduinoSerial.read();

    if (c == '<') {
      frame = "";
    }
    else if (c == '>') {

      float heading, pitch, roll;

      if (sscanf(frame.c_str(), "%f,%f,%f",
                 &heading,
                 &pitch,
                 &roll) == 3) {

        Serial.print("Heading: ");
        Serial.print(heading);

        Serial.print("   Pitch: ");
        Serial.print(pitch);

        Serial.print("   Roll: ");
        Serial.println(roll);
      }
    }
    else {
      frame += c;
    }
  }
}