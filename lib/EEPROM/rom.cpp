#include "rom.h"
#include "var.h"
Preferences preferences;
void initialize_rom() {
    preferences.begin("rom", false);
    calibrationMin[0] = preferences.getInt("l3n", 10);
    calibrationMin[1] = preferences.getInt("l2n", 10);
    calibrationMin[2] = preferences.getInt("l1n", 10);
    calibrationMin[3] = preferences.getInt("cn", 10);
    calibrationMax[0] = preferences.getInt("l3m", 10);
    calibrationMax[1] = preferences.getInt("l2m", 10);
    calibrationMax[2] = preferences.getInt("l1m", 10);
    calibrationMax[3] = preferences.getInt("cm", 10);
    preferences.end();
     for (uint8_t i = 0; i < 7; i++)
  {
    threshold[i] = (calibrationMin[i] + calibrationMax[i])/2;
  } 
}

void update_rom() {
      preferences.putInt("l3n", calibrationMin[0]);
         preferences.putInt("l2n", calibrationMin[1]);
         preferences.putInt("l1n", calibrationMin[2]);
         preferences.putInt("cn", calibrationMin[3]);
         preferences.putInt("l3m", calibrationMax[0]);
         preferences.putInt("l2m", calibrationMax[1]);
         preferences.putInt("l1m", calibrationMax[2]);
         preferences.putInt("cm", calibrationMax[3]);
    preferences.end();
}
