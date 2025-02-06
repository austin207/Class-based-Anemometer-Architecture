#include <Arduino.h>
#include <QTRSensors.h>
#include "linefollow.h"
#include "var.h"
#include "motor.h"
#include "pindef.h"
#include "rom.h"

uint16_t sensorValues[SensorCount];
QTRSensors qtr;

void sensorInit() {
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){LS_1,LS_2,LS_3,LS_4}, SensorCount);
    qtr.calibrate(QTRReadMode::On);
    delay(100);
     for (int sensor = 0; sensor < SensorCount; sensor++) {
        qtr.calibrationOn.minimum[sensor] = calibrationMin[sensor];
        qtr.calibrationOn.maximum[sensor] = calibrationMax[sensor];
    }
}

void sensorCalibrate() {
    int avg;
   for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate(QTRReadMode::On);
    enableMotor();
    digitalWrite(L_RPWM, 255);
    digitalWrite(R_RPWM, 255);
    digitalWrite(L_LPWM, 0);
    digitalWrite(R_LPWM, 0);

  }
    disableMotor();
  for (int sensor = 0; sensor < SensorCount; sensor++) {
        calibrationMin[sensor] = qtr.calibrationOn.minimum[sensor];
        calibrationMax[sensor] = qtr.calibrationOn.maximum[sensor];
         threshold[sensor] = (qtr.calibrationOn.minimum[sensor] + qtr.calibrationOn.maximum[sensor])/2;
         avg=avg+threshold[sensor];
    }
    avg=avg/4;
    for(int i=4;i<9;i++){
        threshold[i]=avg;
    }
    update_rom();
}

int rsp = 0;
int lsp = 0;
int position = 0;
void PID_Linefollow(){
    int error = (3000 - qtr.readLineBlack(sensorValues));
    float PIDvalue = (P*error)  + (D*(error-previousError)); // + Ivalue
    lsp = maxpwm + PIDvalue;
    rsp = maxpwm - PIDvalue;

    if (lsp > 255) {
      lsp = 255;
    }
   else if (lsp < -255) {
      lsp = -255;
    }
    if (rsp > 255) {
      rsp = 255;
    }
    else if (rsp < -255) {
      rsp = -255;
    }
    left(lsp);
    right(rsp);
    previousError = error;
}