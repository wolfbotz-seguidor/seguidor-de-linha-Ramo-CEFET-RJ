/*
 *BIBLIOTECA PARA UTILIZAÇÃO DA UART
 * Modificada por: PROF. RODRIGO RECH
 * DATA: 09/2019
 *CONFIGURAÇÃO: BAUD RATE   -   9600
 *              NBITS       -   8
 *              STOP BITS   -   1 
 */
#include <avr/io.h>           //Biblioteca geral dos AVR
void UART_config(unsigned int ubrr);
void UART_enviaCaractere(unsigned char ch);
void UART_enviaString(char *s);
void  UART_enviaHex(unsigned char ch);
