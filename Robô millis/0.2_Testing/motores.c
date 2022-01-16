#include "motores.h"

void motores_frente() 
{

    set_bit(PORTD, AIN1); //frente direita
    clr_bit(PORTD, AIN2);
    set_bit(PORTD, BIN2); //frente esquerda
    clr_bit(PORTD, BIN1);
}

void motores_direita_frente()  //direita sentido direto
{
    set_bit(PORTD, AIN1); //frente direita
    clr_bit(PORTD, AIN2);
    clr_bit(PORTD, BIN2);
    clr_bit(PORTD, BIN1);
}

void motores_esquerda_frente()  //direita sentido direto
{
    clr_bit(PORTD, AIN1); //frente direita
    clr_bit(PORTD, AIN2);
    set_bit(PORTD, BIN2);
    clr_bit(PORTD, BIN1);
}

void motores_giro_esquerda()
{
    set_bit(PORTD, AIN1); //frente direita
    clr_bit(PORTD, AIN2);
    clr_bit(PORTD, BIN2);
    set_bit(PORTD, BIN1);
}

void motores_giro_direita()
{
    clr_bit(PORTD, AIN1); //frente direita
    set_bit(PORTD, AIN2);
    set_bit(PORTD, BIN2);
    clr_bit(PORTD, BIN1);
}

void motores_direita_tras()  //direita sentido reverso
{
    clr_bit(PORTD, AIN1); //frente direita
    set_bit(PORTD, AIN2);
    clr_bit(PORTD, BIN2);
    clr_bit(PORTD, BIN1);
}


void motores_esquerda_tras()  //esquerda sentido reverso
{
    clr_bit(PORTD, AIN1); 
    clr_bit(PORTD, AIN2);
    clr_bit(PORTD, BIN2);//frente esquerda
    set_bit(PORTD, BIN1);
}

void motores_tras() {
    clr_bit(PORTD, AIN1);
    set_bit(PORTD, AIN2); //tras direita
    clr_bit(PORTD, BIN2);
    set_bit(PORTD, BIN1); //tras esquerda

}

void motores_off() {
    clr_bit(PORTD, AIN1);
    clr_bit(PORTD, AIN2);
    clr_bit(PORTD, BIN2);
    clr_bit(PORTD, BIN1);
}

void motores_freio() {
    motores_off();        //desliga os motores para deoxar o pr�prio atrito frear o rob�
    PWM_setDuty_1(0);
    PWM_setDuty_2(0);
}
