#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  Serial.println("Ready");
  Serial.read();
}

void loop() {
  if(Serial.available() > 0)
  {
    Serial.print("IN: ");
    Serial.write(Serial.read());
  }
}
