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
      disableMotor();  // Make sure to implement this in motor.h
      
      // Provide feedback: turn on buzzer and change LED status
      activateBuzzer();
      updateRingLED(2); // 2 might indicate "stopped" or "waiting"

      // Record the time when the stop was first detected
      if (waitStartTime == 0) {
        waitStartTime = millis();
      }
      
      // Check if the wait period has passed
      if (millis() - waitStartTime >= waitDuration) {
        // Change state to wait for user confirmation
        currentState = USER_CONFIRMATION;
        // Reset the timer variable
        waitStartTime = 0;
      }
      
      break;

    case USER_CONFIRMATION:
      // Wait here until the user presses the continue button
      // Optionally update LED to indicate waiting for user input
      updateRingLED(3); // 3 might indicate "user input required"
      
      if (checkUserButtonInput()) {
        // If the user has pressed the continue button, resume operation.
        // Reset any necessary variables and re-enable motors.
        enableMotor();
        
        // Optionally, stop the buzzer or change the LED status back
        updateRingLED(1); // Back to active mode
        
        // Transition back to line following mode
        currentState = LINE_FOLLOWING;
      }
      // Else, remain in this state until the button is pressed.
      break;

    default:
      // Default to safe state
      currentState = AT_STOP_WAIT;
      break;
  }
  
  // Other periodic tasks can be added here if needed.
}
