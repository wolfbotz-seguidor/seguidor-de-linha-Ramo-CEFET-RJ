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
#include "ADC.h"
//#include "configbits.txt"   //configura os fusíveis

//variáveis de comando para os registradores
#define set_bit(y,bit) (y|=(1<<bit)) //coloca em 1 o bit x da variável Y
#define clr_bit(y,bit) (y&=~(1<<bit)) //coloca em 0 o bit x da variável Y
#define cpl_bit(y,bit) (y^=(1<<bit)) //troca o estado lógico do bit x da variável Y
#define tst_bit(y,bit) (y&(1<<bit)) //retorna 0 ou 1 conforme leitura do bit


#define AIN1    PD5 //configurações das conexões do Agostinho/ Van Grogue
#define AIN2    PD6

#define BIN1    PD4
#define BIN2    PD3

int Kp = 38; //prescale de 100 - prescale int
int Kd = 82; //prescale de 100 - prescale int
int Ki = 0; // Variáveis que são modificadas no PID - prescale de 100 ou mais
int PWMR = 150; // valor da força do motor em linha reta
int PWM_Curva = 350; //PWM ao entrar na curva
int erro, p, d, erroAnterior = 0, i, integral = 0, Turn = 0; //Área PID -erro também possui prescale = 100
int u = 0; //valor de retorno do PID
int u_curva = 0;
int prescale = 2000; //prescale das constantes * prescale do erro
int PWMA, PWMB; // Modulação de largura de pulso enviada pelo PID
int PWMA_C, PWMB_C; //PWM de curva com ajuste do PID
int entrou;
int erroAnterior_curva = 0, erroAnterior_traseiro = 0;
int Turn_curva, p_curva, d_curva, i_curva, integral_curva = 0;
int position_line = 0; // Método com o Read Line - sem uso por ora
int contador = 0, numParada = 4; // Borda
int curvaValor = 0, entrou = 0;
unsigned int timer2, TempoEspera = 100;
int ejetor = 0;
int tempo_atual = 0;

unsigned int delta_T = 0;
int peso [] = {-3, -2, -1, 1, 2, 3}; //utilizando um prescale de 100
int soma_direito = 0, soma_esquerdo = 0;
int denominador_direito = 6;
int denominador_esquerdo = 6;
int soma_total = 0;



unsigned int millis = 0;

ISR(TIMER0_OVF_vect) {
    TCNT0 = 240; //Recarrega o Timer 0 para que a contagem seja 1ms novamente
    millis++; //Incrementa a variável millis a cada 10ms
}

//=======================//
/*
#define AIN1    PB2 //configurações das conexões do Celta Caindo
#define AIN2    PB1

#define BIN1    PB0
#define BIN2    PD5*/
//============================//

void setDuty_1(int duty); //Seleciona o duty cycle na saída digital  3
void setFreq(char option); //Seleciona a frequência de operação do PWM
void setDuty_2(int duty);
int PID(int error);
void frente();
void tras();
void esquerda();
void direita();
void motor_off();
void freio();
void entrou_na_curva(int sensor, int sensor2, int valor_erro, int tempo_passado);
int PID_Curva(int error_curva, int tempo_curva);
int parada(int sensor_esquerdo, int sensor_direito, int value_erro, int tempo_passed);
void setup();
void loop();

int main(void) {
    setup();
    while (1) loop();
}

void setDuty_1(int duty) //MotorA
{

    OCR1B = duty;

} //end setDuty_pin3

void setDuty_2(int duty) //MotorB
{

    OCR1A = duty; //valores de 0 - 1023

} //end setDuty_pin3

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

void setup() {


    inicializa_ADC(); //Configura o ADC


    DDRB = 0b11111111; //define o PORTB como saída
    PORTB = 0b00000000; //Inicializa todos em 0
    DDRD = 0b11111111; //define o PORTD como saída
    PORTD = 0b01010000;


    TCCR0B = 0b00000101; //TC0 com prescaler de 1024
    TCNT0 = 240; //Inicia a contagem em 100 para, no final, gerar 1ms
    TIMSK0 = 0b00000001; //habilita a interrupção do TC0

    //====Configuração do PWM========================//
    TCCR1A = 0xA2; //Configura operação em fast PWM, utilizando registradores OCR1x para comparação



    setFreq(4); //Seleciona opção para frequência

    //============================//

    sei();
}

void loop() {

    tempo_atual = millis;
    delta_T = tempo_atual - timer2;
    int sensores_frontais[] = {le_ADC(3), le_ADC(2), le_ADC(1), le_ADC(0), le_ADC(7), le_ADC(6)};



    for (int j = 0; j < 3; j++) {
        soma_esquerdo += (sensores_frontais[j] * peso[j]);
        soma_direito += (sensores_frontais[5 - j] * peso[5 - j]);
    }

    soma_total = (soma_esquerdo + soma_direito) / (denominador_esquerdo + denominador_direito);

    erro = 0 - soma_total; //valor esperado(estar sempre em cima da linha) - valor medido

    soma_esquerdo = 0;
    soma_direito = 0;
    soma_total = 0;

    frente();
    setDuty_1(PWMR);
    setDuty_2(PWMR);


    //PORTD ^= (1 << AIN1) | (1 << AIN2) | (1 << BIN1) | (1 << BIN2);
    /*set_bit(PORTD, AIN1);
    clr_bit(PORTD, AIN2);
    clr_bit(PORTD, BIN1);
    set_bit(PORTD, BIN2);
    setDuty_1(400);
    setDuty_2(400);*/


    /*for (int i = 0; i < 1023; i++){
        setDuty_1(i);
        setDuty_2(i);
        _delay_ms(500);
    }*/

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

void frente() {

    set_bit(PORTD, AIN1); //frente direita
    clr_bit(PORTD, AIN2);
    set_bit(PORTD, BIN2); //frente esquerda
    clr_bit(PORTD, BIN1); 
}