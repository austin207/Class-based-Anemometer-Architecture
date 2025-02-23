#include "SystemVars.h"

double SystemVars::P = 0.035;
double SystemVars::I = 0.0;
double SystemVars::D = 1;
int SystemVars::calibrationMin[4] = {0};
int SystemVars::calibrationMax[4] = {0};
int SystemVars::threshold[9] = {0};
int SystemVars::maxpwm = 50;
int SystemVars::previousError = 0;
int SystemVars::ref_line = 1500;
int SystemVars::stopCount = 0;
int SystemVars::axisCount = 2;
