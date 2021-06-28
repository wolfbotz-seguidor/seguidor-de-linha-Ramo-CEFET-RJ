/*
 * File:   main.c
 * Author: johan
 *
 * Created on 3 de Junho de 2021, 14:18
 */

#define F_CPU 16000000      //define a frequencia do uC para 16MHz
#include <avr/io.h>         //Biblioteca geral dos AVR
#include <avr/interrupt.h>  //Biblioteca de interrupção
#include <stdio.h>          //Bilioteca do C
#include <util/delay.h>     //Biblioteca geradora de atraso
#include "UART.h"           //Biblioteca da comunicação UART
#include "ADC.h"            //Biblioteca do conversor AD
//#include "configbits.txt"   //configura os fusíveis

//variáveis de comando para os registradores
#define set_bit(y,bit) (y|=(1<<bit)) //coloca em 1 o bit x da variável Y
#define clr_bit(y,bit) (y&=~(1<<bit)) //coloca em 0 o bit x da variável Y
#define cpl_bit(y,bit) (y^=(1<<bit)) //troca o estado lógico do bit x da variável Y
#define tst_bit(y,bit) (y&(1<<bit)) //retorna 0 ou 1 conforme leitura do bit


//#define sensor_de_curva PD2
//#define sensor_de_parada PD4

#define sensor_tras_esquerdo PD6 
#define sensor_tras_direito  PD7

#define sensor_de_curva PB0
#define sensor_de_parada PD7

char s [] = "Início da leitura";
char buffer[5]; //String que armazena valores de entrada para serem printadas
volatile char ch; //armazena o caractere lido
volatile char flag_com = 0; //flag que indica se houve recepção de dado
// Interrupção da UART
//======================================================

/*tempo =65536 ? Prescaler/Fosc = 65536 ? 1024/16000000 = 4, 19s
 tempo = X_bit_timer * Prescaler/Fosc
 Valor inicial de contagem = 256 ? tempo_desejado?Fosc/Prescaler = 256 ? 0,01?16000000/1024 = 98,75 ? 99
 Valor inicial de contagem = X_bit_timer - tempo_desejado*Fosc/Prescaler*/


ISR(USART_RX_vect) {
    ch = UDR0; //Faz a leitura do buffer da serial

    UART_enviaCaractere(ch); //Envia o caractere lido para o computador
    flag_com = 1; //Aciona o flag de comunicação
}
//------------------------------------------------------

void main(void) {
    DDRD =  0b00000000;       //PORTD como entrada
    PORTD = 0b10000000;       //PORTD inicia em 0, PD7 com pull up interno
    DDRB =  0b00000000;       //PORTB como entrada
    PORTB = 0b00111111;       //PB0 com pull up interno
    DDRC =  0b00000000;       //PORTC como entrada
    PORTC = 0b00001111;       //PORTC com pull up ativado

    UART_config(); //Inicializa a comunicação UART
    inicializa_ADC(); //Configura o ADC
    UART_enviaString(s); //Envia um texto para o computador
    
    while (1) {
        int sensores_laterais [] = {(tst_bit(PINB, sensor_de_curva)) >> sensor_de_curva, tst_bit(PIND, sensor_de_parada) >> sensor_de_parada};
        int sensores_frontais[] = {le_ADC(0), le_ADC(1), le_ADC(2), le_ADC(3), le_ADC(4), le_ADC(6)};
        //int sensor_borda = le_ADC(5);

        //==========Teste dos sensores Frontais====//
        for (int i = 0; i < 6; i++) {
            sprintf(buffer, "%4d", sensores_frontais[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espaço
        }
        UART_enviaCaractere(0x0A); //pula linha

        //======Sensor de borda do Agostinho====//
        /*sprintf(buffer, "%4d", sensor_borda);
        UART_enviaString(buffer); //Envia para o computador
        UART_enviaCaractere(0x0D); //pula linha*/



        //==============Robô Celta Caindo============//
        /*int sensores_frontais[] = {le_ADC(0), le_ADC(1), le_ADC(2), le_ADC(3), le_ADC(4), le_ADC(5);
        for (int i = 0; i < 7; i++) {
            sprintf(buffer, "%4d", sensores_frontais[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espaço
        }
        UART_enviaCaractere(0x0D); //pula linha
        
        //==========Sensores laterais======//
        for (int i = 0; i < 2; i++) {
            sprintf(buffer, "%4d", sensores_laterais[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espaço
        }
        UART_enviaCaractere(0x0D); //pula linha
        
        
        //=========sensores traseiros======//
        for (int i = 0; i < 2; i++) {
            sprintf(buffer, "%4d", sensores_traseiros[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espaço
        }
        UART_enviaCaractere(0x0D); //pula linha*/


        //==============Robô Van Grogue============// 

        //======Sensores frontais=====///
        /*int sensores_frontais[] = {le_ADC(3), le_ADC(2), le_ADC(1), le_ADC(0), le_ADC(7), le_ADC(6)};
        for (int i = 0; i < 6; i++) {
            sprintf(buffer, "%4d", sensores_frontais[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espaço
        }
        UART_enviaCaractere(0x0A); //pula linha*/

        //========Sensores laterais=====//
        /*for (int i = 0; i < 2; i++) {
            sprintf(buffer, "%4d", sensores_laterais[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espaço
        }
        UART_enviaCaractere(0x0A); //pula linha*/

    }
}
