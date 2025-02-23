#ifndef LEDRING_H
#define LEDRING_H

#include <Arduino.h>
#include "FastLED.h"
#include "PinDef.h" // for LED pin

#define NUM_LEDS 16

class LEDRing {
public:
    LEDRing();
    void ledSetup();
    void staticRed();
    void staticGreen();
    void staticBlue();
    void breathingLedsBlue(unsigned long currentMillis, unsigned long &lastBreathingTime, uint8_t &brightness, int &direction);
    void patternBlink(unsigned long currentMillis, unsigned long &lastBlinkTime);
    void patternTheaterChase(unsigned long currentMillis, unsigned long &lastTheaterTime);
    void patternColorWave(unsigned long currentMillis, unsigned long &lastColorWaveTime, uint8_t &startHue);
    void rotatingComet(unsigned long currentMillis, unsigned long &lastCometTime);
    void patternBlinkRed(unsigned long currentMillis, unsigned long &lastBlinkTime);
private:
    CRGB leds[NUM_LEDS];
    void patternColorWavedelay();
};

#endif // LEDRING_H
