#include "motor.h"
#include "pindef.h"
#include <Arduino.h>
#include "BTS7960.h"

BTS7960 leftMotor(5, L_LPWM, L_RPWM);
BTS7960 rightMotor(5, R_LPWM, R_RPWM);

void enableMotor() {
    leftMotor.Enable();
    rightMotor.Enable();
}

void disableMotor() {
    leftMotor.Disable();
    rightMotor.Disable();
}

void left(int pwm) {
    if (pwm >= 0) {
        leftMotor.TurnLeft(pwm);
    } else {
        leftMotor.TurnRight(-pwm); // Convert negative PWM to positive
    }
}

void right(int pwm) {
    if (pwm >= 0) {
        rightMotor.TurnLeft(pwm);
    } else {
        rightMotor.TurnRight(-pwm); // Convert negative PWM to positive
    }
}

void brake(){
    analogWrite(L_LPWM,100);
    analogWrite(L_RPWM,100);
    analogWrite(R_LPWM,100);
    analogWrite(R_RPWM,100);
    delay(500);   
}