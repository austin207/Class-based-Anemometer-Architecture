#include "RobotController.h"
#include <Arduino.h>

RobotController::RobotController()
: lineFollow(&motor),
  axisControl(&motor),
  patternChangeTime(0),
  lastBlinkTime(0),
  lastTheaterTime(0),
  lastColorWaveTime(0),
  lastCometTime(0),
  lastBreathingTime(0),
  startHue(0),
  brightness(0),
  direction(5),
  currentPattern(0),
  currentState(CALIBRATION),
  previousAxis(X_AXIS),
  waitStartTime(0),
  waitDuration(120000),
  skipTime(1750)
{
}

void RobotController::setup() {
    Serial.begin(115200);
    bluetooth.initialize("Anemometer");
    ROM::initialize();
    lineFollow.sensorInit();
    PinDef::setupPins();
    buzzer.buzzerStart();
    ledRing.ledSetup();
}

void RobotController::update() {
    String appCommand = bluetooth.receiveCommand();
    Serial.println(appCommand);
    currentMillis = millis();
    
    switch (currentState) {
        case CALIBRATION:
            bluetooth.getSerial().println("CALIBRATION");
            ledRing.patternBlink(currentMillis, lastBlinkTime);
            if (!Debounce::isButtonPressed(PinDef::B_CAL) || appCommand == CMD_CALIBRATE) {
                buzzer.buzzerCalibrationStart();
                motor.enable();
                ledRing.staticRed();
                lineFollow.sensorCalibrate();
                ledRing.staticGreen();
                buzzer.buzzerCalibrationClose();
                currentState = WAIT_FOR_START;
                bluetooth.getSerial().println("Changing state to START");
                delay(1000);
            } 
            else if (!Debounce::isButtonPressed(PinDef::B_START) || appCommand == CMD_GO) {
                currentState = WAIT_FOR_START;
                bluetooth.getSerial().println("Waiting to start");
                delay(1000);
            }
            break;
            
        case WAIT_FOR_START:
            ledRing.patternTheaterChase(currentMillis, lastTheaterTime);
            if (!Debounce::isButtonPressed(PinDef::B_START) || appCommand == CMD_GO) {
                motor.enable();
                bluetooth.getSerial().println("Changing state to LINEFOLLOW");
                currentState = LINE_FOLLOWING;
                waitStartTime = currentMillis;
                ledRing.staticGreen();
                delay(1000);
            }
            break;
            
        case LINE_FOLLOWING:
            ledRing.patternBlinkRed(currentMillis, lastBlinkTime);
            lineFollow.PID_LineFollow();
            if (!Debounce::isButtonPressed(PinDef::B_CAL) || appCommand == CMD_CALIBRATE) {
                bluetooth.getSerial().println("calibration");
                currentState = CALIBRATION;
                motor.left(0);
                motor.right(0);
                delay(1000);
            }
            if ((analogRead(PinDef::IR_1) < SystemVars::threshold[5] || analogRead(PinDef::IR_2) < SystemVars::threshold[5] ||
                 analogRead(PinDef::IR_3) < SystemVars::threshold[5] || analogRead(PinDef::IR_4) < SystemVars::threshold[5] || analogRead(PinDef::IR_5) < SystemVars::threshold[5])
                && (currentMillis - waitStartTime >= skipTime)) {
                motor.brake();
                motor.disable();
                SystemVars::stopCount++;
                bluetooth.getSerial().println(String(SystemVars::stopCount));
                currentState = AT_STOP_WAIT;
                buzzer.buzzerLineFound();
                buzzer.buzzerTimerStart();
                ledRing.staticBlue();
                motor.disable();
                waitStartTime = currentMillis;
                delay(2000);
            }
            break;
            
        case AT_STOP_WAIT:
            motor.disable();
            if (appCommand == CMD_STOP) {
                motor.disable();
                currentState = WAIT_FOR_START;
            }
            ledRing.rotatingComet(currentMillis, lastCometTime);
            if (!Debounce::isButtonPressed(PinDef::B_CAL) || appCommand == CMD_CALIBRATE) {
                currentState = USER_CONFIRMATION;
                if(SystemVars::stopCount == SystemVars::axisCount) {
                    bluetooth.getSerial().println("EOL going to origin");
                    currentState = CHANGE_AXIS;
                    waitStartTime = currentMillis;
                    motor.enable();
                    SystemVars::stopCount = 0;
                    break;
                }
            }
            if (currentMillis - waitStartTime >= waitDuration) {
                currentState = USER_CONFIRMATION;
                waitStartTime = 0;
                buzzer.buzzerTimerEnd();
                ledRing.staticGreen();
                delay(1000);
            }
            if(SystemVars::stopCount == SystemVars::axisCount) {
                bluetooth.getSerial().println("EOL going to origin");
                currentState = CHANGE_AXIS;
                waitStartTime = currentMillis;
                motor.enable();
                SystemVars::stopCount = 0;
                break;
            }
            break;
            
        case USER_CONFIRMATION:
            ledRing.patternTheaterChase(currentMillis, lastTheaterTime);
            if (!Debounce::isButtonPressed(PinDef::B_START) && Debounce::isButtonPressed(PinDef::B_CAL) || appCommand == CMD_GO) {
                motor.enable();
                currentState = LINE_FOLLOWING;
                waitStartTime = currentMillis;
                ledRing.staticGreen();
                delay(1000);
            }
            break;
            
        case CHANGE_AXIS:
            ledRing.patternBlinkRed(currentMillis, lastBlinkTime);
            SystemVars::maxpwm = 100;
            lineFollow.PID_LineFollow();
            if ((analogRead(PinDef::IR_1) < SystemVars::threshold[5] || analogRead(PinDef::IR_2) < SystemVars::threshold[5] ||
                 analogRead(PinDef::IR_3) < SystemVars::threshold[5] || analogRead(PinDef::IR_4) < SystemVars::threshold[5] || analogRead(PinDef::IR_5) < SystemVars::threshold[5])
                && (currentMillis - waitStartTime >= 2500)) {
                motor.brake();
                motor.disable();
                SystemVars::maxpwm = 50;
                if (previousAxis == X_AXIS) {
                    currentState = NEGATIVE_X_AXIS;
                    previousAxis = NEGATIVE_X_AXIS;
                } else if (previousAxis == NEGATIVE_X_AXIS) {
                    currentState = Y_AXIS;
                    previousAxis = Y_AXIS;
                } else if (previousAxis == Y_AXIS) {
                    currentState = NEGATIVE_Y_AXIS;
                    previousAxis = NEGATIVE_Y_AXIS;
                } else { 
                    currentState = WAIT_FOR_START;
                }
                bluetooth.getSerial().println("Changing axis based on previous axis");
                buzzer.buzzerLineFound();
                ledRing.staticBlue();
                delay(2000);
            }
            break;
            
        case X_AXIS:
            bluetooth.getSerial().println("X_AXIS active");
            lineFollow.PID_LineFollow();
            if (axisControl.axisCompletedCondition() && (currentMillis - waitStartTime >= 2500)) {
                motor.brake();
                motor.disable();
                previousAxis = X_AXIS;
                currentState = CHANGE_AXIS;
                waitStartTime = currentMillis;
                delay(2000);
            }
            break;
            
        case NEGATIVE_X_AXIS:
            bluetooth.getSerial().println("NEGATIVE_X_AXIS active");
            lineFollow.PID_LineFollow();
            if (axisControl.axisCompletedCondition() && (currentMillis - waitStartTime >= 2500)) {
                motor.brake();
                motor.disable();
                previousAxis = NEGATIVE_X_AXIS;
                axisControl.turn90Degrees(true);
                currentState = CHANGE_AXIS;
                waitStartTime = currentMillis;
                delay(2000);
            }
            break;
            
        case Y_AXIS:
            bluetooth.getSerial().println("Y_AXIS active");
            lineFollow.PID_LineFollow();
            if (axisControl.axisCompletedCondition() && (currentMillis - waitStartTime >= 2500)) {
                motor.brake();
                motor.disable();
                previousAxis = Y_AXIS;
                currentState = CHANGE_AXIS;
                waitStartTime = currentMillis;
                delay(2000);
            }
            break;
            
        case NEGATIVE_Y_AXIS:
            bluetooth.getSerial().println("NEGATIVE_Y_AXIS active");
            lineFollow.PID_LineFollow();
            if (axisControl.axisCompletedCondition() && (currentMillis - waitStartTime >= 2500)) {
                motor.brake();
                motor.disable();
                previousAxis = NEGATIVE_Y_AXIS;
                axisControl.turn90Degrees(true);
                currentState = CHANGE_AXIS;
                waitStartTime = currentMillis;
                delay(2000);
            }
            break;
            
        default:
            currentState = AT_STOP_WAIT;
            break;
    }
}
