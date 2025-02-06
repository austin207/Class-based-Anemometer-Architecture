#include "debounce.h"
#include <arduino.h>


int isButtonPressed(int pin) {
  if (digitalRead(pin) == HIGH) {
    while (digitalRead(pin) == HIGH) {
      delay(25);
    }
    return 0;
  } else return 1;
  
}