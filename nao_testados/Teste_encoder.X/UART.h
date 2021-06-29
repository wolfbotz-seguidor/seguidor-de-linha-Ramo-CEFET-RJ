/*
 *BIBLIOTECA PARA UTILIZAÇÃO DA UART
 * Modificada por: PROF. RODRIGO RECH
 * DATA: 09/2019
 *CONFIGURAÇÃO: BAUD RATE   -   9600
 *              NBITS       -   8
 *              STOP BITS   -   1 
 */

#include <avr/io.h> 	    //definições do componente especificado
#include <avr/interrupt.h>

void UART_config() 
{
	//Baud Rate de 115200bps para um cristal de 16MHz (Datasheet)
    UBRR0 = 8;    
   
    //Habilita a interrupção de recepção e os pinos TX e RX
    UCSR0B =  (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0) ;
	
	//Configura a UART com 8 bits de dados
    UCSR0C =  (1<<UCSZ01) | (1<<UCSZ00);  
}

void UART_enviaCaractere(unsigned char ch)
{
   UDR0 = ch;

   //Aguarda o buffer ser desocupado
   while (! (UCSR0A & (1<<UDRE0)) );
}

void UART_enviaString(char *s)
{
   unsigned int i=0;
   while (s[i] != '\x0') 
   {
       UART_enviaCaractere(s[i++]);
   };
}

void  UART_enviaHex(unsigned char ch)
{
    unsigned char i,temp;
     
    for (i=0; i<2; i++)
    {
        temp = (ch & 0xF0)>>4;
        if ( temp <= 9)
            UART_enviaCaractere( '0' + temp);
        else
            UART_enviaCaractere(  'A' + temp - 10);
        ch = ch << 4;    
     }   
}
