#include <Arduino.h>


void setup() {
  Serial.begin(9600); // initialize serial communication
}
void loop() {
  if (Serial.available() > 0) { // check if data is available
    String data = Serial.readStringUntil('\n'); // read the incoming data as a string until a newline character is received
    Serial.println(data); // print the received data
  }
}
