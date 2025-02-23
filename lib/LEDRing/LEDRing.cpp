#include "LEDRing.h"
#include <Arduino.h>
#include "FastLED.h"

LEDRing::LEDRing() {
}

void LEDRing::patternBlink(unsigned long currentMillis, unsigned long &lastBlinkTime) {
    static bool state = false;
    if (currentMillis - lastBlinkTime >= 500) {
        lastBlinkTime = currentMillis;
        state = !state;
        fill_solid(leds, NUM_LEDS, state ? CRGB::Blue : CRGB::Black);
        FastLED.show();
    }
}

void LEDRing::patternBlinkRed(unsigned long currentMillis, unsigned long &lastBlinkTime) {
    static bool state = false;
    if (currentMillis - lastBlinkTime >= 500) {
        lastBlinkTime = currentMillis;
        state = !state;
        fill_solid(leds, NUM_LEDS, state ? CRGB::Red : CRGB::Black);
        FastLED.show();
    }
}

void LEDRing::patternTheaterChase(unsigned long currentMillis, unsigned long &lastTheaterTime) {
    static int offset = 0;
    if (currentMillis - lastTheaterTime >= 100) {
        lastTheaterTime = currentMillis;
        for (int i = 0; i < NUM_LEDS; i++) {
            leds[i] = ((i + offset) % 3 == 0) ? CRGB::Red : CRGB::Black;
        }
        FastLED.show();
        offset = (offset + 1) % 3;
    }
}

void LEDRing::patternColorWave(unsigned long currentMillis, unsigned long &lastColorWaveTime, uint8_t &startHue) {
    if (currentMillis - lastColorWaveTime >= 30) {
        lastColorWaveTime = currentMillis;
        for (int i = 0; i < NUM_LEDS; i++) {
            leds[i] = CHSV(startHue + (i * 10), 255, 255);
        }
        startHue += 5;
        FastLED.show();
    }
}

void LEDRing::rotatingComet(unsigned long currentMillis, unsigned long &lastCometTime) {
    static int cometPos = 0;
    if (currentMillis - lastCometTime >= 50) {
        lastCometTime = currentMillis;
        fadeToBlackBy(leds, NUM_LEDS, 40);

        int tailLength = 4;
        for (int i = 0; i < tailLength; i++) {
            int pos = (cometPos - i + NUM_LEDS) % NUM_LEDS;
            leds[pos] = CHSV(100, 255, 255 - (i * 60));
        }
        cometPos = (cometPos + 1) % NUM_LEDS;
        FastLED.show();
    }
}

void LEDRing::breathingLedsBlue(unsigned long currentMillis, unsigned long &lastBreathingTime, uint8_t &brightness, int &direction) {
    if (currentMillis - lastBreathingTime >= 30) {
        lastBreathingTime = currentMillis;
        brightness += direction;
        if (brightness > 250 || brightness < 5) direction = -direction;
        fill_solid(leds, NUM_LEDS, CHSV(180, 255, brightness));
        FastLED.show();
    }
}

void LEDRing::patternColorWavedelay() {
    static uint8_t startHue = 0;
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CHSV(startHue + (i * 10), 255, 255);
    }
    startHue += 5;
    FastLED.show();
    delay(30);
}

void LEDRing::ledSetup() {
    FastLED.addLeds<WS2811, PinDef::LED, GRB>(leds, NUM_LEDS);
    FastLED.clear();
    FastLED.show();
    FastLED.setBrightness(255);
    FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
    for (int i = 0; i < 100; i++) patternColorWavedelay();
}

void LEDRing::staticRed() {
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    FastLED.show();
}

void LEDRing::staticGreen() {
    fill_solid(leds, NUM_LEDS, CRGB::Green);
    FastLED.show();
}

void LEDRing::staticBlue() {
    fill_solid(leds, NUM_LEDS, CRGB::Blue);
    FastLED.show();
}
