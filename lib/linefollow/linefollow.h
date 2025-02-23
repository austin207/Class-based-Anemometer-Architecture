#ifndef LINEFOLLOW_H
#define LINEFOLLOW_H

#include <Arduino.h>
#include <QTRSensors.h>
#include "SystemVars.h"
#include "Motor.h"
#include "PinDef.h"
#include "ROM.h" // for update()

class LineFollow {
public:
    static const uint8_t SensorCount = 4;
    LineFollow(Motor* motor);
    void sensorInit();
    void sensorCalibrate();
    void PID_LineFollow();
private:
    QTRSensors qtr;
    uint16_t sensorValues[SensorCount];
    Motor* motor;
    void robotControl(int error);
};

#endif // LINEFOLLOW_H
