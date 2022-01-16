#include <avr/io.h>

void PWM_setDuty_2(int  duty);
void PWM_setDuty_1(int  duty);
void PWM_setFreq  (char option);

void PWM_limit();

char calc_duty(int  pwm);
int  calc_pwm (char duty_cycle);