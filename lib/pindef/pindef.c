#include "pindef.h"
#include <Arduino.h>

void pinsetup(){
    pinMode(B_START,INPUT_PULLDOWN);
    pinMode(B_CAL,INPUT_PULLDOWN);
    pinMode(VBAT,INPUT);
    pinMode(BUZZER,OUTPUT);
    pinMode(L_LPWM,OUTPUT);
    pinMode(L_RPWM,OUTPUT);
    pinMode(R_LPWM,OUTPUT);
    pinMode(R_RPWM,OUTPUT);
}