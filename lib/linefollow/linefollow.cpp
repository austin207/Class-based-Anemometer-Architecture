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
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate(QTRReadMode::On);
    if(i >= 50 && i <= 200){
      right(125);
      left(125);
    } 
    else{
      right(0);
      left(0);
    }
  }
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
int the = 800;
int error = 0;

void robot_control(int error){
  P = error;
  D = error - previousError;

  float PIDvalue = ((0.035 * P) +  (1 * D));
  previousError = error;

  lsp = maxpwm + PIDvalue;  // to be applied to the right motor
  rsp = maxpwm - PIDvalue;  // to be applied to the left motor

  // Constrain speeds to the range -255 to 255.
  lsp = constrain(lsp, -150, 150);
  rsp = constrain(rsp, -150, 150);

  // Drive the motors using the custom motor functions.
  right(rsp);  // right motor
  left(lsp);   // left motor

}




void PID_Linefollow(){
  position = qtr.readLineBlack(sensorValues);
  error = position - ref_line;  // compute error relative to target
  
  // If all sensor readings are above the threshold 'the', assume the line is lost.
  if(sensorValues[0] >= 800 && sensorValues[1] >= 800 &&
     sensorValues[2] >= 800 && sensorValues[3] >= 800) {
       
      // Use the previous error to decide which way to turn.
      if(previousError < 0){
        // If the previous error was negative, the line was on the right.
        // Turn right: reverse the right motor and drive the left motor forward.
        right(50);
        left(-150);
        //SerialBT.println("Line lost: Turning Right");
      }
      else{
        // Else, turn left.
        right(-150);
        left(50);
        //SerialBT.println("Line lost: Turning Left");
      }
      return;  // Skip PID control when recovering from a lost line.
  }
  else {
    // Otherwise, perform normal PID line following.
    robot_control(error);
  }
}


