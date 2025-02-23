#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>
#include "PinDef.h"

class Buzzer {
public:
    static const int SHORT_BEEP = 100;
    static const int MEDIUM_BEEP = 300;
    static const int LONG_BEEP = 500;
    static const int GAP = 100;
    
    void beepTone(int duration);
    void buzzerStart();
    void buzzerCalibrationStart();
    void buzzerCalibrationClose();
    void buzzerLineFound();
    void buzzerTimerEnd();
    void buzzerTimerStart();
};

#endif // BUZZER_H
