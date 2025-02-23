#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "PinDef.h"
#include "BTS7960.h"  // Ensure this library is available

class Motor {
public:
    Motor();
    void enable();
    void disable();
    void left(int pwm);
    void right(int pwm);
    void brake();
private:
    BTS7960 leftMotor;
    BTS7960 rightMotor;
};

#endif // MOTOR_H
