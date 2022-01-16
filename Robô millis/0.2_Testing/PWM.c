#include "PWM.h"


//PWM em fast mode de 10 bits do Timer 1 do atemga328p

void PWM_setDuty_2(int duty) //MotorB
{

    OCR1B = duty; //registrador de PWM do OC1B

} //end setDuty

void PWM_setDuty_1(int duty) //MotorA
{

    OCR1A = duty; //valores de 0 - 1023

} //end setDuty

void PWM_setFreq(char option) 
{
    /*
    TABLE:
  	//no fast Mode
        option  frequency (as frequências no timer 1 são menores do que as frequências nos timers 0 e 2)
        
          1      16    kHz
          2       2    kHz
          3     250     Hz
          4     62,5    Hz
          5     15,6    Hz
     */
    TCCR1B = option;
	
	
	
    /*
    TABLE:
  	//no phase corret PWM Mode
        option  frequency (as frequências no timer 1 são menores do que as frequências nos timers 0 e 2)
        
          1       8     kHz
          2       1     kHz
          3     125      Hz
          4     31,25    Hz
          5     7,8      Hz
     */
     
} //end setFrequency


void PWM_limit()
{
    /*Variáveis externas*/
    extern unsigned int PWMA, PWMB;
    
    //------> Limitando PWM
    static int ExcessoB = 0, ExcessoA = 0;
    
    if (PWMA > 1023)
    {
      ExcessoB = (PWMA - 1023);
      PWMA = 1023;
      PWMB -= ExcessoB;
    }

    else if (PWMB > 1023)
    {
      ExcessoA = (PWMB - 1023);
      PWMB = 1023;
      PWMA -= ExcessoA;
    }

    if (PWMA < 0)
    {
      ExcessoB = (PWMA*(-1) * 2);
         PWMA += (PWMA*(-1)*2);
         PWMB += ExcessoB;
    }

    else if (PWMB < 0)
    {
      ExcessoA = (PWMB*(-1) * 2);
         PWMB += (PWMB*(-1)*2);
         PWMA += ExcessoA;
    }        

}

int calc_pwm(char duty_cycle)
{
    int pwm;
    
	// Limita o Duty Cycle mínimo e máximo
         if(duty_cycle <   0) duty_cycle = 0;
	else if(duty_cycle > 100) duty_cycle = 100;
    
    pwm = duty_cycle*10;
			
	return pwm;
	
} /* end calc_pwm */

char calc_duty(int pwm)
{	
    char duty_cycle;
    
    // Limita o PWM mínimo e máximo
         if(pwm <    0) pwm = 0;
	else if(pwm > 1000) pwm = 1000;

    duty_cycle = pwm/10;
		
    return duty_cycle;
    
} /* end calc_duty */

/* ============================
 * Tabela calc_pwm e calc_duty
 * ......................
 * |  Duty    |     PWM |
 * ''''''''''''''''''''''
 * |   10     |     100 |
 * |   20     |     200 |
 * |   30     |     300 |
 * |   40     |     400 |
 * |   50     |     500 |
 * |   60     |     600 |
 * |   70     |     700 |
 * |   80     |     800 |
 * |   90     |     900 |
 * |  100     |    1000 |
   ''''''''''''''''''''''
===============================*/