#include <xc.h>

#include "PWM.h"

//Funções:
#define Prescale T2CONbits.T2CKPS1
#define Set_Prescale T2CONbits.T2CKPS0
#define Timer2 T2CONbits.TMR2ON
#define PWM1_mode CCP1CONbits.CCP1M3
#define PWM1_config CCP1CONbits.CCP1M2
#define PWM2_mode CCP2CONbits.CCP2M3
#define PWM2_config CCP2CONbits.CCP2M2
#define LowByteCCP1 CCPR1L
#define selectionBit1 CCP1CONbits.DC1B1
#define selectionBit2 CCP1CONbits.DC1B0
#define LowByteCCP2 CCPR2L
#define selectionBit3 CCP2CONbits.DC2B1
#define selectionBit4 CCP2CONbits.DC2B0


void PWM_Init () {
    //periodo do pwm = (PR2 + 1) * ciclo de máquina * prescaler do timer2
    //periodo = (240+1) * 0,2us * 16 = 0,8192ms
    //frequencia = 1/periodo = 1220,70Hz = 1,29kHz
    
    PR2 = 240;

    Prescale = 1; 
    Set_Prescale = 1;   // Timer 2 Prescaler = 16 
    Timer2 = 1; 
    
    PWM1_mode = 1;
    PWM1_config = 1;     // Configura o pino CCP1 para o modo PWM
    
    PWM2_mode= 1;
    PWM2_config = 1;     // Configura o pino CCP2 para o modo PWM
    
}

void PWM1_Set_Duty (unsigned short pwm1)  // duty_ratio vai de 0 a 962
{
    
    LowByteCCP1 = pwm1 >> 2;
    selectionBit1 = pwm1 & 2;
    selectionBit2 = pwm1 & 1; 
    
}

void PWM2_Set_Duty (unsigned short pwm2)  // duty_ratio vai de 0 a 962
{
    
    LowByteCCP2 = pwm2 >> 2;
    selectionBit3 = pwm2 & 2;
    selectionBit4 = pwm2 & 1; 
    
}

