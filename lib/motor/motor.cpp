#include "motor.h"
#include "pindef.h"
#include <Arduino.h>

void enableMotor(){
    digitalWrite(EN,HIGH);
}

void disableMotor(){
    digitalWrite(EN,LOW);
    digitalWrite(R_LPWM,LOW);
    digitalWrite(L_LPWM,LOW);
    digitalWrite(R_RPWM,LOW);
    digitalWrite(L_RPWM,LOW);
}

void left(int pwm){
    if (pwm >= 0) {
        digitalWrite(L_LPWM, LOW);
        analogWrite(L_RPWM, pwm);
    } else {
        digitalWrite(L_RPWM, LOW);
        analogWrite(L_LPWM, -pwm);  // Use absolute value for PWM.
    }
}

void right(int pwm){
    if (pwm >= 0) {
        digitalWrite(R_LPWM, LOW);
        analogWrite(R_RPWM, pwm);
    } else {
        digitalWrite(R_RPWM, LOW);
        analogWrite(R_LPWM, -pwm);
    }
}

void brake(){
    digitalWrite(R_LPWM,HIGH);
    digitalWrite(L_LPWM,HIGH);
    digitalWrite(R_RPWM,HIGH);
    digitalWrite(L_RPWM,HIGH);
}