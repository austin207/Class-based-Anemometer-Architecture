#ifndef LINEFOLLOW_H
#define LINEFOLLOW_H
#include <QTRSensors.h>

#define SensorCount 4


extern void sensorInit();
extern void sensorCalibrate();
extern uint16_t sensorValues[SensorCount];
extern void PID_Linefollow();
#endif // LINEFOLLOW_H