/*
 * File:   main.c
 * Author: johan
 *
 * Created on 3 de Junho de 2021, 13:58
 */


#define F_CPU 16000000      //define a frequencia do uC para 16MHz
#include <avr/io.h>         //Biblioteca geral dos AVR
#include <avr/interrupt.h>  //Biblioteca de interrupção
#include <stdio.h>          //Bilioteca do C
#include <util/delay.h>     //Biblioteca geradora de atraso
#include "UART.h"           //Biblioteca da comunicação UART

//variáveis de comando para os registradores
#define set_bit(y,bit) (y|=(1<<bit)) //coloca em 1 o bit x da variável Y
#define clr_bit(y,bit) (y&=~(1<<bit)) //coloca em 0 o bit x da variável Y
#define cpl_bit(y,bit) (y^=(1<<bit)) //troca o estado lógico do bit x da variável Y
#define tst_bit(y,bit) (y&(1<<bit)) //retorna 0 ou 1 conforme leitura do bit


#define AIN1    PD5 //configurações das conexões do Agostinho/ Van Grogue
#define AIN2    PD6

#define BIN1    PD4
#define BIN2    PD3
//=======================//
/*
#define AIN1    PB2 //configurações das conexões do Celta Caindo
#define AIN2    PB1

#define BIN1    PB0
#define BIN2    PD5*/
//============================//



int main(void) {
    DDRB =  0b11111111;     //define o PORTB como saída
    PORTB = 0b00000000;     //Inicializa todos em 0
    DDRD =  0b11111111;     //define o PORTD como saída
    PORTD = 0b00000000;     //Inicializa todos em 0
    
    
    while (1) {
        clr_bit(PORTD, AIN1);
        set_bit(PORTD, AIN2);       //frente direita
        set_bit(PORTD, BIN1);       //frente esquerda
        clr_bit(PORTD, BIN2);
        
        _delay_ms(500);
        
        clr_bit(PORTD, AIN1);
        clr_bit(PORTD, AIN2);       //frente direita
        clr_bit(PORTD, BIN1);       //frente esquerda
        clr_bit(PORTD, BIN2);
        
        //=============================//
        //Configurações do Celta//
        /*
        clr_bit(PORTB, AIN1);
        set_bit(PORTB, AIN2);       //frente direita
        set_bit(PORTB, BIN1);       //frente esquerda
        clr_bit(PORTD, BIN2);
        
        _delay_ms(500);
        
        clr_bit(PORTB, AIN1);
        clr_bit(PORTB, AIN2);       //frente direita
        clr_bit(PORTB, BIN1);       //frente esquerda
        clr_bit(PORTD, BIN2);*/
    }
}
