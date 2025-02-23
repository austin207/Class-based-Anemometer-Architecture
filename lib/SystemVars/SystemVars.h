#ifndef SYSTEMVARS_H
#define SYSTEMVARS_H

class SystemVars {
public:
    static double P;
    static double I;
    static double D;
    static int calibrationMin[4];
    static int calibrationMax[4];
    static int threshold[9];
    static int maxpwm;
    static int previousError;
    static int ref_line;
    static int stopCount;
    static int axisCount;
};

#endif // SYSTEMVARS_H
