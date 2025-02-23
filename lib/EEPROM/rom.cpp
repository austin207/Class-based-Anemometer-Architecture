#include "ROM.h"

Preferences ROM::preferences;

void ROM::initialize() {
    preferences.begin("rom", false);
    SystemVars::calibrationMin[0] = preferences.getInt("l3n", 10);
    SystemVars::calibrationMin[1] = preferences.getInt("l2n", 10);
    SystemVars::calibrationMin[2] = preferences.getInt("l1n", 10);
    SystemVars::calibrationMin[3] = preferences.getInt("cn", 10);
    SystemVars::calibrationMax[0] = preferences.getInt("l3m", 10);
    SystemVars::calibrationMax[1] = preferences.getInt("l2m", 10);
    SystemVars::calibrationMax[2] = preferences.getInt("l1m", 10);
    SystemVars::calibrationMax[3] = preferences.getInt("cm", 10);
    preferences.end();
    for (uint8_t i = 0; i < 7; i++) {
        SystemVars::threshold[i] = (SystemVars::calibrationMin[i] + SystemVars::calibrationMax[i]) / 2;
    }
}

void ROM::update() {
    preferences.putInt("l3n", SystemVars::calibrationMin[0]);
    preferences.putInt("l2n", SystemVars::calibrationMin[1]);
    preferences.putInt("l1n", SystemVars::calibrationMin[2]);
    preferences.putInt("cn", SystemVars::calibrationMin[3]);
    preferences.putInt("l3m", SystemVars::calibrationMax[0]);
    preferences.putInt("l2m", SystemVars::calibrationMax[1]);
    preferences.putInt("l1m", SystemVars::calibrationMax[2]);
    preferences.putInt("cm", SystemVars::calibrationMax[3]);
    preferences.end();
}
