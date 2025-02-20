#include "var.h"

double P = 0.035;
double I = 0.0;
double D = 1;
int calibrationMin[4] = {0};
int calibrationMax[4] = {0};
int threshold[9] = {0};
int maxpwm = 50;
int previousError = 0;
int ref_line = 1500;
int stopCount = 0;
int axisCount = 2;