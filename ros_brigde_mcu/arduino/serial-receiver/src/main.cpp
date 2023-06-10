#include <Arduino.h>

union {
  float f;
  byte b[4];
} data;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() >= 4) {
    for (int i = 0; i < 4; i++) {
      data.b[i] = Serial.read();
    }
    Serial.print("Received: ");
    Serial.println(data.f);
  }
}