#include <Arduino.h>
#include "pindef.h"
#include "debounce.h"
#include "motor.h"


void setup() {
  Serial.begin(115200);
  isButtonPressed(B_START);
  pinSetup();
  enableMotor();
}

void loop() {
    left(255);
    right(255);
    delay(1000);
    left(0);
    right(0);
    delay(1000);
}