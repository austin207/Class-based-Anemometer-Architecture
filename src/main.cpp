#include <Arduino.h>
#include "RobotController.h"

RobotController robot;

void setup() {
    robot.setup();
}

void loop() {
    robot.update();
}
