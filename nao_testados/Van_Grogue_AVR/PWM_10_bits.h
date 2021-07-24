//PWM em fast mode de 10 bits do Timer 1 do atemga328p

void setDuty_2(int duty) //MotorB
{

    OCR1B = duty; //registrador de PWM do OC1B

} //end setDuty

void setDuty_1(int duty) //MotorA
{

    OCR1A = duty; //valores de 0 - 1023

} //end setDuty

void setFreq(char option) {
    /*
    TABLE:
  
        option  frequency (as frequências no timer 1 são menores do que as frequências nos timers 0 e 2)
        
          1      16    kHz
          2       2    kHz
          3     250     Hz
          4     62,5    Hz
          5     15,6    Hz
     */
    TCCR1B = option;

} //end setFrequency