#ifndef PINDEF_H
#define PINDEF_H

#define VBAT 15
#define LED 34  // exchange with led
#define B_START 21
#define B_CAL 17 // exchange with led
#define BUZZER 2 
int EN=5;
#define R_RPWM 23
#define R_LPWM 22
#define L_RPWM 18
#define L_LPWM 19
#define LS_1 13
#define LS_2 12
#define LS_3 14
#define LS_4 27
#define IR_1 26
#define IR_2 25
#define IR_3 33
#define IR_4 32
#define IR_5 35

extern void pinsetup();

#endif // PINDEF_H