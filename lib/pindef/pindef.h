#ifndef PINDEF_H
#define PINDEF_H

#include <Arduino.h>

class PinDef {
public:
    static const int VBAT = 15;
    static const int LED = 17;
    static const int B_START = 12;
    static const int B_CAL = 21;
    static const int BUZZER = 2;
    static const int R_RPWM = 23;
    static const int R_LPWM = 22;
    static const int L_RPWM = 19;
    static const int L_LPWM = 18;
    static const int LS_1 = 13;
    static const int LS_2 = 34;
    static const int LS_3 = 14;
    static const int LS_4 = 27;
    static const int IR_1 = 26;
    static const int IR_2 = 25;
    static const int IR_3 = 33;
    static const int IR_4 = 32;
    static const int IR_5 = 35;

    static void setupPins();
};

#endif // PINDEF_H
