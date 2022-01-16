#include <stdio.h>            //Bilioteca do C
#include <stdbool.h>          //Biblioteca que permite utilizar vari�vel booleana
#include "UART.h"             //Biblioteca da comunicação UART
#include "ADC.h"              //Biblioteca do conversor AD
#include "PWM.h"      //Biblioteca de PWM fast mode de 10 bits
#include "motores.h"     //Biblioteca das funções de controle dos motores  //usado para ponte H tb6612fng
#include "PID.h"              //Biblioteca do controle PID
#include "sensors.h"     //lógica utilizando os sensores
#include "dados.h"            //biblioteca que contém as funções atraladas ao envio de informações via UART
#include "HAL_atmega328p.h"

/*Protótipo das funções*/
void setup();
void setup_logica();        //variáveis utilizadas na lógica
void loop();
void estrategia();          //estrategia do robô
//---------------------------------------------------------------//
void parada();              //Leitura dos sensores laterais
void fim_de_pista();        //verifica se é o fim da pista 
//---------------------------------------------------------------------//
void count_pulsesE();
void count_pulsesD();
void millis(void);
//---------------------------------------------------------------------//
void f_timers (void);       //função de temporização das rotinas
void f_timer1(void);
void f_timer2(void);
void f_timer3(void);
void f_timer4(void);
void f_timer5(void);
/*===========================================================================*/

/*Macros*/
#define NOP() __asm__ __volatile__ ("nop")
