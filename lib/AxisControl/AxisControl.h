#ifndef AXISCONTROL_H
#define AXISCONTROL_H

#include <Arduino.h>
#include "Motor.h"
#include "SystemVars.h"

class AxisControl {
public:
    AxisControl(Motor* motor);
    bool axisCompletedCondition();
    void turn90Degrees(bool clockwise);
private:
    Motor* motor;
};

#endif // AXISCONTROL_H
