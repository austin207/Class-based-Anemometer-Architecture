#include "motor.h"
#include "pindef.h"
#include <Arduino.h>

void enableMotor(){
    digitalWrite(EN,HIGH);
}

void disableMotor(){
    digitalWrite(EN,LOW);
}

void left(int pwm){
    digitalWrite(L_LPWM,LOW);
    analogWrite(L_RPWM,pwm);
}

void right(int pwm){
    digitalWrite(R_LPWM,LOW);
    analogWrite(R_RPWM,pwm);
}

void brake(){
    digitalWrite(R_LPWM,HIGH);
    digitalWrite(L_LPWM,HIGH);
    digitalWrite(R_RPWM,HIGH);
    digitalWrite(L_RPWM,HIGH);
}