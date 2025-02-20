#include <Arduino.h>
#include "pindef.h"    
#include "debounce.h"  
#include "motor.h"
#include "linefollow.h"
#include "var.h"
#include "rom.h"     
#include "buzzer.h"
#include "FastLED.h"
#include "ring.h"
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// For handling LED patterns and timings
unsigned long patternChangeTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastTheaterTime = 0;
unsigned long lastColorWaveTime = 0;
unsigned long lastCometTime = 0;
unsigned long lastBreathingTime = 0;
uint8_t startHue = 0;
uint8_t brightness = 0;
int direction = 5;
int currentPattern = 0;

// Define states for the state machine; note the new WAIT_FOR_START state.
enum SystemState {
  CALIBRATION,
  WAIT_FOR_START,   // New: Wait for the user to press Start (with calibration button released)
  LINE_FOLLOWING,
  AT_STOP_WAIT,
  USER_CONFIRMATION,
  CHANGE_AXIS,
};

SystemState currentState = CALIBRATION;

// For handling the wait timer in AT_STOP_WAIT
unsigned long waitStartTime = 0;
const unsigned long waitDuration = 120000; // 2 minutes in milliseconds
unsigned long currentMillis = 0;
unsigned long skipTime = 1750;
void setup() {
 SerialBT.begin("Anemometer");
  initialize_rom();
  sensorInit();
  pinSetup();
  buzzerStart();
  ledSetup();
}

void loop() {
  currentMillis = millis(); // Update current time at the beginning of loop

  switch (currentState) {

    case CALIBRATION:
      SerialBT.println("CALIBRATION");
      patternBlink(currentMillis, lastBlinkTime);
      
      // When the calibration button is pressed (active-low: !isButtonPressed returns true)
      if (!isButtonPressed(B_CAL)) {  
        buzzerCalibrationStart();
        //SerialBT.println("Calibration started");
        enableMotor();
        staticRed();
        sensorCalibrate();
        staticGreen();
        buzzerCalibrationClose();
        //SerialBT.println("Calibration finished");
        // After calibration, do not start motion immediately.
        // Transition to WAIT_FOR_START, where the user must press Start.
        currentState = WAIT_FOR_START;
        SerialBT.println("Changing state to START");
        delay(1000);
      } 
      // Alternatively, if the start button is pressed and calibration values exist,
      // skip calibration and go to WAIT_FOR_START.
      else if (!isButtonPressed(B_START)) {  
        //SerialBT.println("Skipping calibration; using saved calibration values");
        currentState = WAIT_FOR_START;
        SerialBT.println("Waiting to start");
        delay(1000);
      }
      break;

    case WAIT_FOR_START:
    //SerialBT.println("WAIT_FOR_START: Waiting for user to press Start (while calibration button is released)");
      // Show a waiting LED pattern
      patternTheaterChase(currentMillis, lastTheaterTime);
      // Only if the Start button is pressed AND the calibration button is NOT pressed,
      // then enable the motors and move to LINE_FOLLOWING.
      if (!isButtonPressed(B_START)) {
        //SerialBT.println("Start button pressed. Beginning movement.");
        enableMotor();
        SerialBT.println("Changing state to LINEFOLLOW");
        currentState = LINE_FOLLOWING;
        waitStartTime = currentMillis ;
        staticGreen();
        delay(1000);
      }
      break;

    case LINE_FOLLOWING:
    //SerialBT.println("LINE_FOLLOWING");
      patternBlinkRed(currentMillis, lastBlinkTime);
      PID_Linefollow();
      
      // Allow user to request a recalibration at any time by pressing the calibration button.
      // (Again, checking with active-low logic.)
      if (!isButtonPressed(B_CAL)) {
        SerialBT.println("calibration");
         currentState = CALIBRATION;
         left(0);
         right(0);
         //SerialBT.println("User requested recalibration. Switching to CALIBRATION.");
         delay(1000);
      }
      
      // Check if any line sensor detects the line (using the threshold value)
      if ((analogRead(IR_1) < threshold[5] || analogRead(IR_2) < threshold[5] ||
          analogRead(IR_3) < threshold[5] || analogRead(IR_4) < threshold[5] || analogRead(IR_5) < threshold[5]) && (currentMillis - waitStartTime >= skipTime)) {
        brake();
        disableMotor();
        stopCount++;
        SerialBT.println(stopCount);
        currentState = AT_STOP_WAIT;
        buzzerLineFound();
        buzzerTimerStart();
        //SerialBT.println("Line detected. Switching to AT_STOP_WAIT.");
        staticBlue();
        disableMotor();
        // Start the wait timer
        waitStartTime = currentMillis;
        delay(2000);
      }
      break;

    case AT_STOP_WAIT:
      // During the wait period, keep the motors disabled and run an LED pattern.
      disableMotor();
      //SerialBT.println("AT_STOP_WAIT");

      rotatingComet(currentMillis, lastCometTime);
      if (!isButtonPressed(B_CAL)) {
        currentState = USER_CONFIRMATION;
        if(stopCount == axisCount){
          SerialBT.println("EOL going to origin");
          currentState = CHANGE_AXIS;
          waitStartTime = currentMillis;
          enableMotor();
          stopCount = 0;
          break;
        }
      }

      // Non-blocking wait: once the wait duration has elapsed, transition to USER_CONFIRMATION.
      if (currentMillis - waitStartTime >= waitDuration) {
        currentState = USER_CONFIRMATION;
        waitStartTime = 0;  // Reset the timer
        buzzerTimerEnd();
        staticGreen();
        //SerialBT.println("2-minute wait complete. Switching to USER_CONFIRMATION.");
        delay(1000);
      }
      if(stopCount == axisCount){
        SerialBT.println("EOL going to origin");
        currentState = CHANGE_AXIS;
        waitStartTime = currentMillis;
        enableMotor();
        stopCount = 0;
        break;
      }
      break;

    case USER_CONFIRMATION:
    //SerialBT.println("USER_CONFIRMATION: Waiting for user confirmation to resume operation");
      // Provide a LED pattern to show the system is awaiting confirmation.
      patternTheaterChase(currentMillis, lastTheaterTime);
      // Only if the Start button is pressed AND the calibration button is not pressed, then resume.
      if (!isButtonPressed(B_START) && isButtonPressed(B_CAL)) {
        //SerialBT.println("User confirmed. Resuming operation.");
        enableMotor();
        currentState = LINE_FOLLOWING;
        waitStartTime = currentMillis;
        staticGreen();
        delay(1000);
      }
      break;

      case CHANGE_AXIS:
      patternBlinkRed(currentMillis, lastBlinkTime); // change it to binking purple
      maxpwm = 100;
      PID_Linefollow();

      if ((analogRead(IR_1) < threshold[5] || analogRead(IR_2) < threshold[5] ||
          analogRead(IR_3) < threshold[5] || analogRead(IR_4) < threshold[5] || analogRead(IR_5) < threshold[5]) && (currentMillis - waitStartTime >= 2500)) {
        brake();
        disableMotor();
        maxpwm = 50;
        currentState = WAIT_FOR_START;
        SerialBT.println("Changing state to wait to start after reaching origin");
        buzzerLineFound();
        staticBlue();
        delay(2000);
      }
      break;

    default:
      // Fallback: if for some reason an undefined state is reached, go to a safe state.
      currentState = AT_STOP_WAIT;
      break;
  }
}
