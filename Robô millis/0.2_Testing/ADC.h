/*---------------------------------------------------------------
 * BIBLIOTECA PARA UTILIZACAO DO CONVERSOR AD DO AVR
 * Modificada por: PROF. André Costa Canella
 * 08/2021
 * -----------------------------------------------------------------*/
#define F_CPU 16000000        //define a frequencia do uC para 16MHz
#include <avr/io.h>           //Biblioteca geral dos AVR

void ADC_init (void);
void ADC_conv_ch (unsigned char canal);
unsigned char ADC_ler ();