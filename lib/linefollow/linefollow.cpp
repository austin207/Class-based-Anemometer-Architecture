#include "LineFollow.h"
#include <Arduino.h>
#include "ROM.h"

LineFollow::LineFollow(Motor* motorPtr) : motor(motorPtr) {
}

void LineFollow::sensorInit() {
    qtr.setTypeAnalog();
    const uint8_t sensorPins[SensorCount] = {PinDef::LS_1, PinDef::LS_2, PinDef::LS_3, PinDef::LS_4};
    qtr.setSensorPins(sensorPins, SensorCount);
    qtr.calibrate(QTRReadMode::On);
    delay(100);
    for (int sensor = 0; sensor < SensorCount; sensor++) {
        qtr.calibrationOn.minimum[sensor] = SystemVars::calibrationMin[sensor];
        qtr.calibrationOn.maximum[sensor] = SystemVars::calibrationMax[sensor];
    }
}

void LineFollow::sensorCalibrate() {
  int avg = 0;
  for (uint16_t i = 0; i < 400; i++) {
      qtr.calibrate(QTRReadMode::On);
      if (i >= 50 && i <= 200) {
          motor->right(125);
          motor->left(125);
      } else {
          motor->right(0);
          motor->left(0);
      }
  }
  for (int sensor = 0; sensor < SensorCount; sensor++) {
      SystemVars::calibrationMin[sensor] = qtr.calibrationOn.minimum[sensor];
      SystemVars::calibrationMax[sensor] = qtr.calibrationOn.maximum[sensor];
      SystemVars::threshold[sensor] = (qtr.calibrationOn.minimum[sensor] + qtr.calibrationOn.maximum[sensor]) / 2;
      avg += SystemVars::threshold[sensor];
  }
  avg = avg / 4;
  for (int i = 4; i < 9; i++) {
      SystemVars::threshold[i] = avg;
  }
  // Call the ROM update function from the ROM class.
  ROM::update();
}


void LineFollow::robotControl(int error) {
    SystemVars::P = error;
    SystemVars::D = error - SystemVars::previousError;

    float PIDvalue = ((0.035 * SystemVars::P) + (1 * SystemVars::D));
    SystemVars::previousError = error;

    int lsp = SystemVars::maxpwm + PIDvalue;  // right motor speed
    int rsp = SystemVars::maxpwm - PIDvalue;    // left motor speed

    lsp = constrain(lsp, -150, 150);
    rsp = constrain(rsp, -150, 150);

    motor->right(rsp);
    motor->left(lsp);
}

void LineFollow::PID_LineFollow() {
    int position = qtr.readLineBlack(sensorValues);
    int error = position - SystemVars::ref_line;
    
    // If line is lost (all sensors above a threshold)
    if(sensorValues[0] >= 800 && sensorValues[1] >= 800 &&
       sensorValues[2] >= 800 && sensorValues[3] >= 800) {
        if(SystemVars::previousError < 0){
            motor->right(50);
            motor->left(-150);
        } else {
            motor->right(-150);
            motor->left(50);
        }
        return;
    } else {
        robotControl(error);
    }
}
