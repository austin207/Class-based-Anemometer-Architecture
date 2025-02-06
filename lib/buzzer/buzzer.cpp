#include <Arduino.h>
#include "pindef.h"
#include "buzzer.h"

void beepTone(int duration) {
  digitalWrite(BUZZER, HIGH);
  delay(duration);
  digitalWrite(BUZZER, LOW);
}


void buzzerStart() {
  beepTone(LONG_BEEP);  // Long beep ("beeeeeeeeeeeeep")
  delay(GAP);           // Gap (you can adjust if needed)
  beepTone(SHORT_BEEP); // Short beep ("bep")
}

// buzzer 3: "calibration start bep bep beeeep"
// This plays two short beeps followed by a medium beep.
void buzzerCalibrationStart() {
  beepTone(SHORT_BEEP); // First short beep ("bep")
  delay(GAP);
  beepTone(SHORT_BEEP); // Second short beep ("bep")
  delay(GAP);
  beepTone(MEDIUM_BEEP); // Medium beep ("beeeep")
}

// buzzer 4: "calibration close beeeep bep bep"
// This plays a medium beep followed by two short beeps.
void buzzerCalibrationClose() {
  beepTone(MEDIUM_BEEP); // Medium beep ("beeeep")
  delay(GAP);
  beepTone(SHORT_BEEP);  // First short beep ("bep")
  delay(GAP);
  beepTone(SHORT_BEEP);  // Second short beep ("bep")
}

// buzzer 5: "Line found bep bep bep"
// This plays three short beeps.
void buzzerLineFound() {
  beepTone(SHORT_BEEP); // First short beep ("bep")
  delay(GAP);
  beepTone(SHORT_BEEP); // Second short beep ("bep")
  delay(GAP);
  beepTone(SHORT_BEEP); // Third short beep ("bep")
}

// buzzer 6: "timer end bep beeeep bep"
// This plays a short beep, a medium beep, then a short beep.
void buzzerTimerEnd() {
  beepTone(SHORT_BEEP);  // First short beep ("bep")
  delay(GAP);
  beepTone(MEDIUM_BEEP); // Medium beep ("beeeep")
  delay(GAP);
  beepTone(SHORT_BEEP);  // Second short beep ("bep")
}
