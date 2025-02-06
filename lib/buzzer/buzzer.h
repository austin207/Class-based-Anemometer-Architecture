#ifndef BUZZER_H
#define BUZZER_H

#define SHORT_BEEP 100     // "bep"
#define MEDIUM_BEEP 300    // "beeeep"
#define LONG_BEEP 500      // "beeeeeeeeeeeeep"
#define GAP 100

extern void beepTone(int duration);
extern void buzzerStart();
extern void buzzerCalibrationStart();
extern void buzzerCalibrationClose();
extern void buzzerLineFound();
extern void buzzerTimerEnd();
extern void buzzerTimerStart();
#endif // BUZZER_H
