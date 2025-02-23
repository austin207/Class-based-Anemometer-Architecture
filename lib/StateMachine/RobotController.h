#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <Arduino.h>
#include "PinDef.h"
#include "Debounce.h"
#include "Motor.h"
#include "LineFollow.h"
#include "SystemVars.h"
#include "ROM.h"
#include "Buzzer.h"
#include "LEDRing.h"
#include "BluetoothHandler.h"
#include "BluetoothCommand.h"
#include "AxisControl.h"

enum SystemState {
    CALIBRATION,
    WAIT_FOR_START,
    LINE_FOLLOWING,
    AT_STOP_WAIT,
    USER_CONFIRMATION,
    CHANGE_AXIS,
    X_AXIS,
    NEGATIVE_X_AXIS,
    Y_AXIS,
    NEGATIVE_Y_AXIS,
};

class RobotController {
public:
    RobotController();
    void setup();
    void update();
private:
    // Components
    Motor motor;
    LineFollow lineFollow;
    Buzzer buzzer;
    LEDRing ledRing;
    BluetoothHandler bluetooth;
    AxisControl axisControl;
    
    // Timing variables
    unsigned long patternChangeTime;
    unsigned long lastBlinkTime;
    unsigned long lastTheaterTime;
    unsigned long lastColorWaveTime;
    unsigned long lastCometTime;
    unsigned long lastBreathingTime;
    
    uint8_t startHue;
    uint8_t brightness;
    int direction;
    int currentPattern;
    
    // State machine
    SystemState currentState;
    SystemState previousAxis;
    
    unsigned long waitStartTime;
    const unsigned long waitDuration;
    unsigned long currentMillis;
    unsigned long skipTime;
};

#endif // ROBOTCONTROLLER_H
