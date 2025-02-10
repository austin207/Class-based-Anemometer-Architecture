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
};

SystemState currentState = CALIBRATION;

// For handling the wait timer in AT_STOP_WAIT
unsigned long waitStartTime = 0;
const unsigned long waitDuration = 120000; // 2 minutes in milliseconds
unsigned long currentMillis = 0;

void setup() {
  Serial.begin(115200);
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
      Serial.println("CALIBRATION");
      patternBlink(currentMillis, lastBlinkTime);
      
      // When the calibration button is pressed (active-low: !isButtonPressed returns true)
      if (!isButtonPressed(B_CAL)) {  
        buzzerCalibrationStart();
        Serial.println("Calibration started");
        staticRed();
        sensorCalibrate();
        staticGreen();
        buzzerCalibrationClose();
        Serial.println("Calibration finished");
        // After calibration, do not start motion immediately.
        // Transition to WAIT_FOR_START, where the user must press Start.
        currentState = WAIT_FOR_START;
        delay(1000);
      } 
      // Alternatively, if the start button is pressed and calibration values exist,
      // skip calibration and go to WAIT_FOR_START.
      else if (!isButtonPressed(B_START)) {  
        Serial.println("Skipping calibration; using saved calibration values");
        currentState = WAIT_FOR_START;
        delay(1000);
      }
      break;

    case WAIT_FOR_START:
      Serial.println("WAIT_FOR_START: Waiting for user to press Start (while calibration button is released)");
      // Show a waiting LED pattern
      patternTheaterChase(currentMillis, lastTheaterTime);
      // Only if the Start button is pressed AND the calibration button is NOT pressed,
      // then enable the motors and move to LINE_FOLLOWING.
      if (!isButtonPressed(B_START) && isButtonPressed(B_CAL)) {
        Serial.println("Start button pressed. Beginning movement.");
        enableMotor();
        currentState = LINE_FOLLOWING;
        staticGreen();
        delay(1000);
      }
      break;

    case LINE_FOLLOWING:
      Serial.println("LINE_FOLLOWING");
      patternBlinkRed(currentMillis, lastBlinkTime);
      PID_Linefollow();
      
      // Allow user to request a recalibration at any time by pressing the calibration button.
      // (Again, checking with active-low logic.)
      if (!isButtonPressed(B_CAL)) {
         currentState = CALIBRATION;
         Serial.println("User requested recalibration. Switching to CALIBRATION.");
         delay(1000);
      }
      
      // Check if any line sensor detects the line (using the threshold value)
      if (analogRead(LS_1) < threshold[5] || analogRead(LS_2) < threshold[5] ||
          analogRead(LS_3) < threshold[5] || analogRead(LS_4) < threshold[5]) {
        currentState = AT_STOP_WAIT;
        buzzerLineFound();
        Serial.println("Line detected. Switching to AT_STOP_WAIT.");
        staticBlue();
        disableMotor();
        // Start the wait timer
        waitStartTime = currentMillis;
        delay(1000);
      }
      break;

    case AT_STOP_WAIT:
      // During the wait period, keep the motors disabled and run an LED pattern.
      disableMotor();
      Serial.println("AT_STOP_WAIT");
      buzzerTimerStart();
      rotatingComet(currentMillis, lastCometTime);
      
      // Non-blocking wait: once the wait duration has elapsed, transition to USER_CONFIRMATION.
      if (currentMillis - waitStartTime >= waitDuration) {
        currentState = USER_CONFIRMATION;
        waitStartTime = 0;  // Reset the timer
        buzzerTimerEnd();
        staticGreen();
        Serial.println("2-minute wait complete. Switching to USER_CONFIRMATION.");
        delay(1000);
      }
      break;

    case USER_CONFIRMATION:
      Serial.println("USER_CONFIRMATION: Waiting for user confirmation to resume operation");
      // Provide a LED pattern to show the system is awaiting confirmation.
      patternTheaterChase(currentMillis, lastTheaterTime);
      // Only if the Start button is pressed AND the calibration button is not pressed, then resume.
      if (!isButtonPressed(B_START) && isButtonPressed(B_CAL)) {
        Serial.println("User confirmed. Resuming operation.");
        enableMotor();
        currentState = LINE_FOLLOWING;
        staticGreen();
        delay(1000);
      }
      break;

    default:
      // Fallback: if for some reason an undefined state is reached, go to a safe state.
      currentState = AT_STOP_WAIT;
      break;
  }
}
