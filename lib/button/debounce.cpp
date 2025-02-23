#include "Debounce.h"
#include <Arduino.h>
#include "Buzzer.h"

int Debounce::isButtonPressed(int pin) {
    if (digitalRead(pin) == HIGH) {
        while (digitalRead(pin) == HIGH) {
            delay(25);
        }
        Buzzer buzzer;
        buzzer.beepTone(100);
        return 0;
    }
    return 1;
}
