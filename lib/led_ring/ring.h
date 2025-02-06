#ifndef RING_H
#define RING_H

#define NUM_LEDS 16

extern void ledSetup();
extern void staticRed();
extern void staticGreen();
extern void staticBlue();
extern void breathingLedsBlue(unsigned long currentMillis, unsigned long &lastBreathingTime, uint8_t &brightness, int &direction);
extern void patternBlink(unsigned long currentMillis, unsigned long &lastBlinkTime);
extern void patternTheaterChase(unsigned long currentMillis, unsigned long &lastTheaterTime);
extern void patternColorWave(unsigned long currentMillis, unsigned long &lastColorWaveTime, uint8_t &startHue);
extern void rotatingComet(unsigned long currentMillis, unsigned long &lastCometTime);
extern void patternBlinkRed(unsigned long currentMillis, unsigned long &lastBlinkTime);

#endif // RING_H