/*
 * File:   main.c
 * Author: johan
 *
 * Created on 3 de Junho de 2021, 14:18
 */

#define F_CPU 16000000      //define a frequencia do uC para 16MHz
#include <avr/io.h>         //Biblioteca geral dos AVR
#include <avr/interrupt.h>  //Biblioteca de interrup��o
#include <stdio.h>          //Bilioteca do C
#include <util/delay.h>     //Biblioteca geradora de atraso
#include "UART.h"           //Biblioteca da comunica��o UART
#include "ADC.h"            //Biblioteca do conversor AD

//vari�veis de comando para os registradores
#define set_bit(y,bit) (y|=(1<<bit)) //coloca em 1 o bit x da vari�vel Y
#define clr_bit(y,bit) (y&=~(1<<bit)) //coloca em 0 o bit x da vari�vel Y
#define cpl_bit(y,bit) (y^=(1<<bit)) //troca o estado l�gico do bit x da vari�vel Y
#define tst_bit(y,bit) (y&(1<<bit)) //retorna 0 ou 1 conforme leitura do bit


#define sensor_de_curva PD2
#define sensor_de_parada PD4

#define sensor_tras_esquerdo PD6 
#define sensor_tras_direito  PD7

#define sensor_de_curva PB0
#define sensor_de_parada PD7

char s [] = "In�cio da leitura";
char buffer[5]; //String que armazena valores de entrada para serem printadas
volatile char ch; //armazena o caractere lido
volatile char flag_com = 0; //flag que indica se houve recep��o de dado
// Interrup��o da UART
//======================================================

/*tempo =65536 ? Prescaler/Fosc = 65536 ? 1024/16000000 = 4, 19s
 tempo = X_bit_timer * Prescaler/Fosc
 Valor inicial de contagem = 256 ? tempo_desejado?Fosc/Prescaler = 256 ? 0,01?16000000/1024 = 98,75 ? 99
 Valor inicial de contagem = X_bit_timer - tempo_desejado*Fosc/Prescaler*/


ISR(USART_RX_vect) {
    ch = UDR0; //Faz a leitura do buffer da serial

    UART_enviaCaractere(ch); //Envia o caractere lido para o computador
    flag_com = 1; //Aciona o flag de comunica��o
}
//------------------------------------------------------

void main(void) {
    DDRD = 0b00000000;  //PORTD como entrada
    PORTD = 0b00000000; //PORTD inicia em 0
    
    int leitura1 = 0;
    int leitura2 = 0;
    int leitura3 = 0;
    int leitura4 = 0;
    int leitura5 = 0;
    int leitura6 = 0;
    int leitura7 = 0;
    
    int sensores_traseiros [] = {sensor_tras_direito, sensor_tras_esquerdo};
    int sensores_laterais [] = {sensor_de_curva, sensor_de_parada};
    
    UART_config(); //Inicializa a comunica��o UART
    inicializa_ADC(); //Configura o ADC
    UART_enviaString(s); //Envia um texto para o computador    while (1) {
    
    while(1){
        leitura1 = le_ADC(0); //Sensores frontais
        leitura2 = le_ADC(1);
        leitura3 = le_ADC(2);
        leitura4 = le_ADC(3); // sensores forntais
        leitura5 = le_ADC(4);
        leitura6 = le_ADC(5); //sensor de borda
        leitura7 = le_ADC(6);
        int sensores_frontais[] = {leitura1, leitura2, leitura3, leitura4, leitura5, leitura7};
        int sensor_borda = leitura6;
        
        //==========Teste dos sensores Frontais====//
        for (int i = 0; i < 2; i++) {
            sprintf(buffer, "%4d", sensores_frontais[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espa�o
        }
        UART_enviaCaractere(0x0D); //pula linha
        
        //======Sensor de borda do Agostinho====//
        /*sprintf(buffer, "%4d", sensor_borda);
        UART_enviaString(buffer); //Envia para o computador
        UART_enviaCaractere(0x0D); //pula linha*/
     
        
        
    //==============Rob� Celta Caindo============//
        /*int sensores_frontais[] = {leitura1, leitura2, leitura3, leitura4, leitura5, leitura6};
        for (int i = 0; i < 7; i++) {
            sprintf(buffer, "%4d", sensores_frontais[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espa�o
        }
        UART_enviaCaractere(0x0D); //pula linha
        
        //==========Sensores laterais======//
        for (int i = 0; i < 2; i++) {
            sprintf(buffer, "%4d", sensores_laterais[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espa�o
        }
        UART_enviaCaractere(0x0D); //pula linha
        
        
        //=========sensores traseiros======//
        for (int i = 0; i < 2; i++) {
            sprintf(buffer, "%4d", sensores_traseiros[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espa�o
        }
        UART_enviaCaractere(0x0D); //pula linha*/
        
        
    //==============Rob� Van Grogue============// 
        
        //======Sensores frontais=====///
        /*int sensores_frontais[] = {leitura1, leitura2, leitura3, leitura4, leitura5, leitura6};
        for (int i = 0; i < 7; i++) {
            sprintf(buffer, "%4d\n", sensores_frontais[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espa�o
        }
        UART_enviaCaractere(0x0D); //pula linha

        //========Sensores laterais=====//
        for (int i = 0; i < 2; i++) {
            sprintf(buffer, "%4d", sensores_laterais[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espa�o
        }
        UART_enviaCaractere(0x0D); //pula linha*/
        
    }
}
