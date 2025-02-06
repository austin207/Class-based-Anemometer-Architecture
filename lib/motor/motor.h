#ifndef MOTOR_H
#define MOTOR_H


extern void enableMotor();
extern void disableMotor();
extern void left(int pwm);
extern void right(int pwm);
extern void brake();
#endif // MOTOR_H