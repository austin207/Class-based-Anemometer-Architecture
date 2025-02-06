#include <Arduino.h>
#include "pindef.h"    
#include "debounce.h"  
#include "motor.h"
#include "linefollow.h"
#include "var.h"
#include "rom.h"     

// Define states for the state machine
enum SystemState {
    CALIBRATION,
    LINE_FOLLOWING,
    AT_STOP_WAIT,
    USER_CONFIRMATION,
};

SystemState currentState = CALIBRATION;


unsigned long waitStartTime = 0;
const unsigned long waitDuration = 120000; // 2 minutes in milliseconds


void setup() {
    Serial.begin(115200);
    initialize_rom();
    sensorInit();
    pinSetup();
}

void loop() {
  switch (currentState) {

    case CALIBRATION:
    if(!isButtonPressed(B_CAL)) {    // to calibrate
        //beep
        //light
        sensorCalibrate();
        //light normal
        //beep
        currentState = LINE_FOLLOWING;
        enableMotor();
        }

    else if(!isButtonPressed(B_START)) {  // to skip calibration and use previously saved calibration values
        currentState = LINE_FOLLOWING;
        enableMotor();
        }

     break;



    case LINE_FOLLOWING:
    PID_Linefollow();
    if(!isButtonPressed(B_CAL)) {
        currentState = CALIBRATION;
        }
    if(analogRead(LS_1) < threshold[5] || analogRead(LS_2) < threshold[5] || analogRead(LS_3) < threshold[5] || analogRead(LS_4) < threshold[5]) {
        currentState = AT_STOP_WAIT;
        disableMotor();
        }

      break;

    case AT_STOP_WAIT:
      delay(500);
      disableMotor();
      //beep
      //light
      if (waitStartTime == 0) {
        waitStartTime = millis();
      }
      
      if (millis() - waitStartTime >= waitDuration) {
        currentState = USER_CONFIRMATION;
        waitStartTime = 0;
      }
      //beep
      //light
      
      break;

    case USER_CONFIRMATION:

      if (!isButtonPressed(B_START)) {   // When B_START is pressed, resume operation.
          enableMotor();
          currentState = LINE_FOLLOWING;
      }
      break;

    default:
      currentState = AT_STOP_WAIT;
      break;
  }
}