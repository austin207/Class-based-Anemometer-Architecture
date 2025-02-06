#include <Arduino.h>
#include "pindef.h"
#include "debounce.h"
#include "motor.h"


void setup() {
  Serial.begin(115200);
  pinsetup();
  enableMotor();
}

void loop() {
  left(255);
  right(255);
  delay(1000);
  left(100);
  right(100);
  delay(1000);
  disableMotor();
}

