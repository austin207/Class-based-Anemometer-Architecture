#include "var.h"

double P = 0.1;
double I = 0.0;
double D = 1.5;
int calibrationMin[4] = {0};
int calibrationMax[4] = {0};
int threshold[9] = {0};
int maxpwm = 255;
int previousError = 0;
int ref_line = 3000;