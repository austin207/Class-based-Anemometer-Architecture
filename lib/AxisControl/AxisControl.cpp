#include "AxisControl.h"
#include <Arduino.h>
#include "PinDef.h"

AxisControl::AxisControl(Motor* motorPtr) : motor(motorPtr) {
}

bool AxisControl::axisCompletedCondition() {
    return (SystemVars::stopCount <= 0);
}

void AxisControl::turn90Degrees(bool clockwise) {
    motor->enable();
    if (clockwise) {
        motor->left(100);
        motor->right(-100);
    } else {
        motor->left(-100);
        motor->right(100);
    }
    delay(500);
    motor->brake();
}
