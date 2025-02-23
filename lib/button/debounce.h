#ifndef DEBOUNCE_H
#define DEBOUNCE_H

#include <Arduino.h>

class Debounce {
public:
    static int isButtonPressed(int pin);
};

#endif // DEBOUNCE_H
