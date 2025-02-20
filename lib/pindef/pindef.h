#ifndef PINDEF_H
#define PINDEF_H

#define VBAT 15
#define LED 17  // exchange with led
#define B_START 12
#define B_CAL 21 // exchange with led // 12
#define BUZZER 2 
#define R_RPWM 23
#define R_LPWM 22
#define L_RPWM 19
#define L_LPWM 18
#define LS_1 13
#define LS_2 34
#define LS_3 14
#define LS_4 27
#define IR_1 26
#define IR_2 25
#define IR_3 33
#define IR_4 32
#define IR_5 35

extern void pinSetup();

#endif // PINDEF_H