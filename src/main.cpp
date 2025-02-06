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


// For handling the patterns
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
  currentMillis = millis(); // Get current time
  switch (currentState) {

    case CALIBRATION:
    patternBlink(currentMillis, lastBlinkTime);
    if(!isButtonPressed(B_CAL)) {    // to calibrate
        buzzerCalibrationStart();
        staticRed(); 
        sensorCalibrate();
        staticGreen();
        buzzerCalibrationClose();
        currentState = LINE_FOLLOWING;
        enableMotor();
        delay(1000);
        }

    else if(!isButtonPressed(B_START)) {  // to skip calibration and use previously saved calibration values
        currentState = LINE_FOLLOWING;
        staticGreen();
        buzzerCalibrationClose();
        enableMotor();
        delay(1000);
        }

     break;



    case LINE_FOLLOWING:
    patternBlinkRed(currentMillis, lastBlinkTime);
    PID_Linefollow();
    if(!isButtonPressed(B_CAL)) {
        currentState = CALIBRATION;
        }
    if(analogRead(LS_1) < threshold[5] || analogRead(LS_2) < threshold[5] || analogRead(LS_3) < threshold[5] || analogRead(LS_4) < threshold[5]) {
        currentState = AT_STOP_WAIT;
        buzzerLineFound();
        staticBlue();
        disableMotor();
        delay(1000);
        }

      break;

    case AT_STOP_WAIT:
      delay(500);
      disableMotor();
      buzzerTimerStart();
      rotatingComet(currentMillis, lastCometTime);
      if (waitStartTime == 0) {
        waitStartTime = millis();
      }
      for(;;){
      if (millis() - waitStartTime >= waitDuration) {
        currentState = USER_CONFIRMATION;
        waitStartTime = 0;
        break;
      }
      }
        buzzerTimerEnd();
      staticGreen();
      delay(1000);
      
      break;

    case USER_CONFIRMATION:
      patternTheaterChase(currentMillis, lastTheaterTime);
      if (!isButtonPressed(B_START)) {   // When B_START is pressed, resume operation.
          enableMotor();
          currentState = LINE_FOLLOWING;
            staticGreen();
            delay(1000);
      }
      break;

    default:
      currentState = AT_STOP_WAIT;
      break;
  }
}