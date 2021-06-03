/*
 * File:   main.c
 * Author: johan
 *
 * Created on 30 de Maio de 2021, 16:52
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


//Lado direito
#define AIN2 PB1// Quando em HIGH, roda direita anda para frente
#define AIN1 PB2 

//Lado Esquerdo
#define BIN1 PB0 // Quando em HIGH, roda esquerda anda para frente
#define BIN2 PD5


#define sensor_de_curva PD2
#define sensor_de_parada PD4

#define sensor_tras_esquerdo PD6 
#define sensor_tras_direito  PD7

int Kp = 38; //prescale de 100 - prescale int
int Kd = 82; //prescale de 100 - prescale int
int Ki = 0; // Vari�veis que s�o modificadas no PID - prescale de 100 ou mais
int Kp_tras = 2, Kd_tras = 0, Ki_tras = 0;
int PWMR = 200, PWMAux = 0; // valor da for�a do motor em linha reta
int PWM_Curva = 150; //PWM ao entrar na curva
int erro, p, d, erroAnterior = 0, i, integral = 0, Turn = 0; //�rea PID -erro tamb�m possui prescale = 100
int pt, dt, it;
int u = 0; //valor de retorno do PID
int u_curva = 0;
int prescale = 2000; //prescale das constantes * prescale do erro
int PWMA, PWMB; // Modula��o de largura de pulso enviada pelo PID
int PWMA_C, PWMB_C; //PWM de curva com ajuste do PID
int entrou;
int erroAnterior_curva = 0, erroAnterior_traseiro = 0;
int Turn_curva, p_curva, d_curva, i_curva, integral_curva = 0;
int position_line = 0; // M�todo com o Read Line;
int contador = 0, numParada = 4; // Borda
int curvaValor = 0, entrou = 0;
unsigned int timer2, TempoEspera = 100;
int ejetor = 0;
int prescale_tras = 10, erro_tras = 0, u_tras = 0;
int integral_traseiro = 0, Turn_traseiro = 0;

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


unsigned int millis = 0;

ISR(TIMER0_OVF_vect) {
    TCNT0 = 99; //Recarrega o Timer 0 para que a contagem seja 10ms novamente
    millis++; //Incrementa a vari�vel millis a cada 10ms
}

ISR(USART_RX_vect) {
    ch = UDR0; //Faz a leitura do buffer da serial

    UART_enviaCaractere(ch); //Envia o caractere lido para o computador
    flag_com = 1; //Aciona o flag de comunica��o
}
//------------------------------------------------------

void setDuty_1(int duty); //Seleciona o duty cycle na sa�da digital  3
void setFreq(char option); //Seleciona a frequ�ncia de opera��o do PWM
void setDuty_2(int duty);
int PID(int error, int tempo);
void frente();
void tras();
void esquerda();
void direita();
void motor_off();
void freio();
void entrou_na_curva(int sensor, int sensor2, int valor_erro, int tempo_passado, int u_traseiro);
int PID_Curva(int error_curva, int tempo_curva);
int PID_traseiro(int erro_traseiro, int tempo_tras);
int parada(int sensor_esquerdo, int sensor_direito, int value_erro, int tempo_passed, int u_traseir);

int main(void) {
    unsigned int delta_T = 0;
    int peso [] = {-3, -2, -1, 1, 2, 3}; //utilizando um prescale de 100
    int soma_direito, soma_esquerdo;
    int denominador_direito = 6;
    int denominador_esquerdo = 6;
    int soma_total = 0;
    int leitura1 = 0;
    int leitura2 = 0;
    int leitura3 = 0;
    int leitura4 = 0;
    int leitura5 = 0;
    int leitura6 = 0;

    int sensores_traseiros [] = {sensor_tras_direito, sensor_tras_esquerdo};
    int soma_tras = 0;

    DDRD = 0b11110100; //PD5 e PD2 e PD4, PD6 e PD7 definidos como sa�da
    PORTD = 0b00000000; //inicializados em n�vel baixo
    DDRB = 0b00100111; //Habilita PB1 e PB2 e PB0 como sa�da
    PORTB = 0b00000000; //PORTB inicializa desligado e sa�das sem pull up


    //esquerdo pino 4 - PD2
    UART_config(); //Inicializa a comunica��o UART
    inicializa_ADC(); //Configura o ADC
    UART_enviaString(s); //Envia um texto para o computador

    TCCR0B = 0b00000101; //TC0 com prescaler de 1024
    TCNT0 = 99; //Inicia a contagem em 100 para, no final, gerar 10ms
    TIMSK0 = 0b00000001; //habilita a interrup��o do TC0

    TCCR2A = 0xA3; //Configura opera��o em fast PWM, utilizando registradores OCR1x para compara��o

    setFreq(6); //Seleciona op��o para frequ�ncia

    sei(); //Habilita as interrup��es


    //----> Calibra��o dos Sensores frontais <----\\

    for (int i = 0; i < 120; i++) {
        //qtra.calibrate();
        _delay_ms(5);
    }

    set_bit(PORTB, PB5); //subrotina de acender e apagar o LED 13
    _delay_ms(1000);
    clr_bit(PORTB, PB5);
    _delay_ms(500);
    set_bit(PORTB, PB5);
    _delay_ms(500);
    clr_bit(PORTB, PB5);


    while (1) {
        delta_T = millis;
        leitura1 = le_ADC(0); //Sensores frontais
        leitura2 = le_ADC(1);
        leitura3 = le_ADC(2);
        leitura4 = le_ADC(3); // sensores forntais
        leitura5 = le_ADC(4);
        leitura6 = le_ADC(5); //sensor de borda
        int sensores_frontais[] = {leitura1, leitura2, leitura3, leitura4, leitura5, leitura6};

        for (int i = 0; i < 2; i++) {
            sprintf(buffer, "%4d\n", sensores_frontais[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espa�o
        }
        UART_enviaCaractere(0x0D); //pula linha

        for (int i = 0; i < 3; i++) {
            soma_tras += sensores_traseiros[i];
        }

        soma_tras = (soma_tras / 2) * prescale_tras; //m�dia do valor dos sensores traseiros

        erro_tras = 0 - soma_tras;

        u_tras = PID_traseiro(erro_tras, delta_T);

        //regi�o que seta os valores nos sensores frontais ap�s a calibra��o

        //----------------------------------------------------------------//

        for (int j = 0; j < 3; j++) {
            soma_esquerdo += (sensores_frontais[j] * peso[j]);
            soma_direito += (sensores_frontais[5 - j] * peso[5 - j]);
        }

        soma_total = (soma_esquerdo + soma_direito) / (denominador_esquerdo + denominador_direito);

        erro = 0 - soma_total; //valor esperado(estar sempre em cima da linha) - valor medido

        sprintf(buffer, "%4d\n", erro); //Converte para string
        UART_enviaString(buffer); //Envia para o computador
        UART_enviaCaractere(0x0D); //pula linha

        //--------------->AREA DO PID<---------------

        u = PID(erro, delta_T);

        PWMA = PWMR - u + u_tras;
        PWMB = PWMR + u - u_tras;

        //--------------->AREA DOS SENSORES<---------------

        switch (ejetor) {
            case 0:
                if ((!(tst_bit(PORTD, sensor_de_curva))) || (!(tst_bit(PORTD, sensor_de_parada))))//verifica se sos sensores est�o em n�vel 0
                {
                    timer2 = delta_T;
                    ejetor = 1;
                }
                break;

            case 1:
                if ((delta_T - timer2) > TempoEspera) {
                    parada(sensor_de_curva, sensor_de_parada, erro, delta_T, u_tras); // Verifica se � um marcador de parada
                    ejetor = 2;
                }
                break;

            case 2:
                if ((tst_bit(PORTD, sensor_de_curva)) && (tst_bit(PORTD, sensor_de_parada))) {
                    timer2 = 0;
                    ejetor = 0;
                }
                break;
        }

        //-----> �rea do senstido de giro

        if (erro < 0) //virar para a esquerda
        {
            entrou_na_curva(sensor_de_curva, sensor_de_parada, erro, delta_T, u_tras);
            set_bit(PORTB, PB5); //liga o LED
            while (erro < 0) {
                esquerda();
            }
            clr_bit(PORTB, PB5);

        } else if (erro > 0) {
            entrou_na_curva(sensor_de_curva, sensor_de_parada, erro, delta_T, u_tras);
            set_bit(PORTB, PB5); //liga o LED
            while (erro > 0) {
                direita();
            }
            clr_bit(PORTB, PB5);
        }

        //A fun��o que fazia o rob� rodar em seu pr�prio eixo foi removida


        //------> Limitando PWM

        if (PWMA > 255) {
            PWMA = 250;
        } else if (PWMB > 255) {
            PWMB = 250;
        }


    }
}

void setDuty_2(int duty) //MotorB
{

    OCR2B = duty;

} //end setDuty

void setDuty_1(int duty) //MotorA
{

    OCR2A = duty; //valores de 0 - 1023

} //end setDuty

void setFreq(char option) {
    /*
    TABLE:
  
        option  frequency
        
          1      62.5  kHz
          2       7.81 kHz
          3       1.95 kHz
          4     976.56  Hz
          5     488.28  Hz
          6     244.14  Hz
          7      61.03  Hz   
     */
    TCCR2B = option;


} //end setFrequency

int PID(int error, int tempo) {
    p = (error * Kp) / prescale; // Proporcao

    integral += error; // Integral
    i = ((Ki * integral) / prescale) * tempo;

    d = ((Kd * (error - erroAnterior)) / prescale) / tempo; // Derivada
    erroAnterior = error;

    Turn = p + i + d;
    return Turn; //retorna os valores ap�s o PID
}

void frente() {

    clr_bit(PORTD, AIN1);
    set_bit(PORTD, AIN2); //frente direita
    clr_bit(PORTD, BIN2);
    set_bit(PORTD, BIN1); //frente esquerda
}

void tras() {
    set_bit(PORTD, AIN1);
    clr_bit(PORTD, AIN2); //frente direita
    clr_bit(PORTD, BIN2);
    set_bit(PORTD, BIN1); //frente esquerda

}

void motor_off() {
    clr_bit(PORTD, AIN1);
    clr_bit(PORTD, AIN2);
    clr_bit(PORTD, BIN2);
    clr_bit(PORTD, BIN1);
}

void freio() {
    frente();

    setDuty_1(50);
    setDuty_2(50);

    _delay_ms(500);

    tras();

    setDuty_1(10);
    setDuty_2(10);

    _delay_ms(5);

    frente();

    setDuty_1(0);
    setDuty_2(0);

    _delay_ms(2000);

    motor_off();

    _delay_ms(60000);
}

void direita() {
    set_bit(PORTD, AIN1); //tras direita
    clr_bit(PORTD, AIN2);
    clr_bit(PORTD, BIN2);
    set_bit(PORTD, BIN1); //frente esquerda

    setDuty_1(PWMA_C);
    setDuty_2(PWMB_C);

    //calibra��o dos sensores frontais - seta o valor m�dio
}

void esquerda() {
    clr_bit(PORTD, AIN1);
    set_bit(PORTD, AIN2); //direita frente
    clr_bit(PORTD, BIN2);
    set_bit(PORTD, BIN1); //esquerda tr�s

    setDuty_1(PWMA_C);
    setDuty_2(PWMB_C);


    //calibra��o dos sensores frontais - seta o valor m�dio
}

void entrou_na_curva(int sensor, int sensor2, int valor_erro, int tempo_passado, int u_traseiro) {
    if ((!tst_bit(PORTD, PD2)) && tst_bit(PORTD, PD4)) {
        switch (entrou) {
            case 0: //entrou na curva
                u_curva = PID_Curva(valor_erro, tempo_passado);
                PWMA_C = PWM_Curva - u_curva + u_tras;
                PWMB_C = PWM_Curva + u_curva - u_tras;
                setDuty_1(PWMA_C);
                setDuty_2(PWMB_C);
                entrou = 1;
                break;

            case 1:
                entrou = 0;
                setDuty_1(PWMA); //t�mino da curva
                setDuty_2(PWMB);
                break;
        }
    }
}

int PID_Curva(int error_curva, int tempo_curva) {
    p_curva = (error_curva * Kp) / prescale; // Proporcao

    integral_curva += error_curva; // Integral
    i_curva = ((Ki * integral_curva) / prescale) * tempo_curva;

    d_curva = ((Kd * (error_curva - erroAnterior_curva)) / prescale) / tempo_curva; // Derivada
    erroAnterior_curva = error_curva;

    Turn_curva = p_curva + i_curva + d_curva;
    return Turn_curva; //retorna os valores ap�s o PID
}

int PID_traseiro(int erro_traseiro, int tempo_tras) {
    pt = (erro_traseiro * Kp) / prescale; // Proporcao

    integral_traseiro += erro_traseiro; // Integral
    it = ((Ki_tras * integral_traseiro) / prescale_tras) * tempo_tras;

    dt = ((Kd_tras * (erro_traseiro - erroAnterior_traseiro)) / prescale_tras) / tempo_tras; // Derivada
    erroAnterior_traseiro = erro_traseiro;

    Turn_traseiro = pt + it + dt;
    return Turn_traseiro; //retorna os valores ap�s o PID
}



int parada(int sensor_esquerdo, int sensor_direito, int value_erro, int tempo_passed, int u_traseir) {
    if ((!tst_bit(PORTD, PD2)) && tst_bit(PORTD, PD4)) {
        contador++;
        entrou_na_curva(sensor_esquerdo, sensor_direito, value_erro, tempo_passed, u_traseir); // Verifica se � uma curva
    } 
    else if((!tst_bit(PORTD, PD2)) && (!tst_bit(PORTD, PD4)))   //verifica se � crizamento
    {
        setDuty_1(PWMA);
        setDuty_2(PWMB);
    }

    while (contador == numParada) {
        freio();
    }
}