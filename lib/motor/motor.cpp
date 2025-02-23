#include "Motor.h"
#include <Arduino.h>

Motor::Motor() 
: leftMotor(5, PinDef::L_LPWM, PinDef::L_RPWM),
  rightMotor(5, PinDef::R_LPWM, PinDef::R_RPWM)
{
}

void Motor::enable() {
    leftMotor.Enable();
    rightMotor.Enable();
}

void Motor::disable() {
    leftMotor.Disable();
    rightMotor.Disable();
}

void Motor::left(int pwm) {
    if(pwm >= 0)
        leftMotor.TurnLeft(pwm);
    else
        leftMotor.TurnRight(-pwm);
}

void Motor::right(int pwm) {
    if(pwm >= 0)
        rightMotor.TurnLeft(pwm);
    else
        rightMotor.TurnRight(-pwm);
}

void Motor::brake() {
    analogWrite(PinDef::L_LPWM, 100);
    analogWrite(PinDef::L_RPWM, 100);
    analogWrite(PinDef::R_LPWM, 100);
    analogWrite(PinDef::R_RPWM, 100);
    delay(500);
}
