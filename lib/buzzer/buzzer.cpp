#include "Buzzer.h"
#include <Arduino.h>

void Buzzer::beepTone(int duration) {
    digitalWrite(PinDef::BUZZER, HIGH);
    delay(duration);
    digitalWrite(PinDef::BUZZER, LOW);
}

void Buzzer::buzzerStart() {
    beepTone(LONG_BEEP);
    delay(GAP);
    beepTone(SHORT_BEEP);
}

void Buzzer::buzzerCalibrationStart() {
    beepTone(SHORT_BEEP);
    delay(GAP);
    beepTone(SHORT_BEEP);
    delay(GAP);
    beepTone(MEDIUM_BEEP);
}

void Buzzer::buzzerCalibrationClose() {
    beepTone(MEDIUM_BEEP);
    delay(GAP);
    beepTone(SHORT_BEEP);
    delay(GAP);
    beepTone(SHORT_BEEP);
}

void Buzzer::buzzerLineFound() {
    beepTone(SHORT_BEEP);
    delay(GAP);
    beepTone(SHORT_BEEP);
    delay(GAP);
    beepTone(SHORT_BEEP);
}

void Buzzer::buzzerTimerEnd() {
    beepTone(SHORT_BEEP);
    delay(GAP);
    beepTone(MEDIUM_BEEP);
    delay(GAP);
    beepTone(SHORT_BEEP);
}

void Buzzer::buzzerTimerStart() {
    beepTone(MEDIUM_BEEP);
    delay(GAP);
    beepTone(SHORT_BEEP);
    delay(GAP);
    beepTone(SHORT_BEEP);
}
