#include <Arduino.h>

union {
  float f;
  byte b[4];
} data;

float linear_x;
float linear_y;
float angular_z;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 0x36) {
      for (int i = 0; i < 4; i++) {
        data.b[i] = Serial.read();
      }
      linear_x = data.f;
      
      for (int i = 0; i < 4; i++) {
        data.b[i] = Serial.read();
      }
      linear_y = data.f;
      
      for (int i = 0; i < 4; i++) {
        data.b[i] = Serial.read();
      }
      angular_z = data.f;
      
      Serial.print("linear_x: ");
      Serial.println(linear_x);
      Serial.print("linear_y: ");
      Serial.println(linear_y);
      Serial.print("angular_z: ");
      Serial.println(angular_z);
    }
  }
}