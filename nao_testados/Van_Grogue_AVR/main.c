/*
 * File:   main.c
 * Author: johan
 *
 * Created on 30 de Maio de 2021, 16:52
 */


#define F_CPU 16000000      //define a frequencia do uC para 16MHz
#include <avr/io.h>         //Biblioteca geral dos AVR
#include <avr/interrupt.h>  //Biblioteca de interrupção
#include <stdio.h>          //Bilioteca do C
#include <util/delay.h>     //Biblioteca geradora de atraso
#include "UART.h"           //Biblioteca da comunicação UART
#include "ADC.h"            //Biblioteca do conversor AD
#include "configbits.txt"   //configura os fusíveis

//variáveis de comando para os registradores
#define set_bit(y,bit) (y|=(1<<bit)) //coloca em 1 o bit x da variável Y
#define clr_bit(y,bit) (y&=~(1<<bit)) //coloca em 0 o bit x da variável Y
#define cpl_bit(y,bit) (y^=(1<<bit)) //troca o estado lógico do bit x da variável Y
#define tst_bit(y,bit) (y&(1<<bit)) //retorna 0 ou 1 conforme leitura do bit


//Lado direito
#define AIN2 PD6// Quando em HIGH, roda direita anda para frente
#define AIN1 PD5 

//Lado Esquerdo
#define BIN1 PD4 // Quando em HIGH, roda esquerda anda para frente
#define BIN2 PD3


#define sensor_de_curva PB0
#define sensor_de_parada PD7


int Kp = 38; //prescale de 100 - prescale int
int Kd = 82; //prescale de 100 - prescale int
int Ki = 0; // Variáveis que são modificadas no PID - prescale de 100 ou mais
int PWMR = 200; // valor da força do motor em linha reta
int PWM_Curva = 150; //PWM ao entrar na curva
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

char s [] = "Início da leitura";
char buffer[5]; //String que armazena valores de entrada para serem printadas
volatile char ch; //armazena o caractere lido
volatile char flag_com = 0; //flag que indica se houve recepção de dado
// Interrupção da UART
//======================================================


/*tempo =65536 ? Prescaler/Fosc = 65536 ? 1024/16000000 = 4, 19s
 tempo = X_bit_timer * Prescaler/Fosc
 Valor inicial de contagem = 256 - tempo_desejado*Fosc/Prescaler = 256 - 0,001*16000000/1024 = 255
 Valor inicial de contagem = X_bit_timer - tempo_desejado*Fosc/Prescaler*/


unsigned int millis = 0;

ISR(TIMER0_OVF_vect) {
    TCNT0 = 255; //Recarrega o Timer 0 para que a contagem seja 1ms novamente
    millis++; //Incrementa a variável millis a cada 10ms
}

ISR(USART_RX_vect) {
    ch = UDR0; //Faz a leitura do buffer da serial

    UART_enviaCaractere(ch); //Envia o caractere lido para o computador
    flag_com = 1; //Aciona o flag de comunicação
}
//------------------------------------------------------

void setDuty_1(int duty); //Seleciona o duty cycle na saída digital  3
void setFreq(char option); //Seleciona a frequência de operação do PWM
void setDuty_2(int duty);
int PID(int error, int tempo);
void frente();
void tras();
void esquerda();
void direita();
void motor_off();
void freio();
void entrou_na_curva(int sensor, int sensor2, int valor_erro, int tempo_passado);
int PID_Curva(int error_curva, int tempo_curva);
int parada(int sensor_esquerdo, int sensor_direito, int value_erro, int tempo_passed);

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


    DDRD = 0b01111000; //PD3 - PD6 definidos como saída, PD7 como entrada
    PORTD = 0b10000000; //inicializados em nível baixo
    DDRB = 0b00100000; //Habilita PB0 como entrada e PB5 como saída
    PORTB = 0b00000000; //PORTB inicializa desligado e saídas sem pull up


    //esquerdo pino 4 - PD2
    UART_config(); //Inicializa a comunicação UART
    inicializa_ADC(); //Configura o ADC
    UART_enviaString(s); //Envia um texto para o computador

    TCCR0B = 0b00000111; //TC0 com prescaler de 1024
    TCNT0 = 255; //Inicia a contagem em 100 para, no final, gerar 1ms
    TIMSK0 = 0b00000001; //habilita a interrupção do TC0

    TCCR1A = 0xA2; //Configura operação em fast PWM, utilizando registradores OCR1x para comparação

    setFreq(3); //Seleciona opção para frequência

    sei(); //Habilita as interrupções


    //----> Calibração dos Sensores frontais <----\\

    for (int i = 0; i < 120; i++) {
        int sensores_frontais[] = {le_ADC(0), le_ADC(1), le_ADC(2), le_ADC(3), le_ADC(4), le_ADC(5)};
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
        int sensores_frontais[] = {le_ADC(0), le_ADC(1), le_ADC(2), le_ADC(3), le_ADC(4), le_ADC(5)};
        for (int i = 0; i < 7; i++) {
            sprintf(buffer, "%4d", sensores_frontais[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espaço
        }
        UART_enviaCaractere(0x0D); //pula linha


        //região que seta os valores nos sensores frontais após a calibração

        //----------------------------------------------------------------//

        for (int j = 0; j < 3; j++) {
            soma_esquerdo += (sensores_frontais[j] * peso[j]);
            soma_direito += (sensores_frontais[5 - j] * peso[5 - j]);
        }

        soma_total = (soma_esquerdo + soma_direito) / (denominador_esquerdo + denominador_direito);

        erro = 0 - soma_total; //valor esperado(estar sempre em cima da linha) - valor medido

        sprintf(buffer, "%45\n", erro); //Converte para string
        UART_enviaString(buffer); //Envia para o computador
        UART_enviaCaractere(0x0D); //pula linha

        //--------------->AREA DO PID<---------------

        soma_esquerdo = 0;
        soma_direito = 0;
        soma_total = 0;
        
        u = PID(erro, delta_T);

        PWMA = PWMR - u;
        PWMB = PWMR + u;

        //--------------->AREA DOS SENSORES<---------------

        switch (ejetor) {
            case 0:
                if ((!(tst_bit(PORTB, sensor_de_curva))) || (!(tst_bit(PORTD, sensor_de_parada))))//verifica se sos sensores estão em nível 0
                {
                    timer2 = delta_T;
                    ejetor = 1;
                }
                break;

            case 1:
                if ((delta_T - timer2) > TempoEspera) {
                    parada(sensor_de_curva, sensor_de_parada, erro, delta_T); // Verifica se é um marcador de parada
                    ejetor = 2;
                }
                break;

            case 2:
                if ((tst_bit(PORTB, sensor_de_curva)) && (tst_bit(PORTD, sensor_de_parada))) {
                    timer2 = 0;
                    ejetor = 0;
                }
                break;
        }

        //-----> Área do senstido de giro

        if (erro < 0) //virar para a esquerda
        {
            entrou_na_curva(sensor_de_curva, sensor_de_parada, erro, delta_T);
            set_bit(PORTB, PB5); //liga o LED
            while (erro < 0) {
                esquerda();
            }
            clr_bit(PORTB, PB5);

        } else if (erro > 0) {
            entrou_na_curva(sensor_de_curva, sensor_de_parada, erro, delta_T);
            set_bit(PORTB, PB5); //liga o LED
            while (erro > 0) {
                direita();
            }
            clr_bit(PORTB, PB5);
        }

        //A função que fazia o robô rodar em seu próprio eixo foi removida


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

    OCR1B = duty;

} //end setDuty

void setDuty_1(int duty) //MotorA
{

    OCR1A = duty; //valores de 0 - 1023

} //end setDuty

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

int PID(int error, int tempo) {
    p = (error * Kp) / prescale; // Proporcao

    integral += error; // Integral
    i = ((Ki * integral) / prescale) * tempo;

    d = ((Kd * (error - erroAnterior)) / prescale) / tempo; // Derivada
    erroAnterior = error;

    Turn = p + i + d;
    return Turn; //retorna os valores após o PID
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

    //calibração dos sensores frontais - seta o valor médio
}

void esquerda() {
    clr_bit(PORTD, AIN1);
    set_bit(PORTD, AIN2); //direita frente
    clr_bit(PORTD, BIN2);
    set_bit(PORTD, BIN1); //esquerda trás

    setDuty_1(PWMA_C);
    setDuty_2(PWMB_C);


    //calibração dos sensores frontais - seta o valor médio
}

void entrou_na_curva(int sensor, int sensor2, int valor_erro, int tempo_passado) {
    if ((!tst_bit(PORTD, sensor_de_curva)) && tst_bit(PORTD, sensor_de_parada)) {
        switch (entrou) {
            case 0: //entrou na curva
                u_curva = PID_Curva(valor_erro, tempo_passado);
                PWMA_C = PWM_Curva - u_curva;
                PWMB_C = PWM_Curva + u_curva;
                setDuty_1(PWMA_C);
                setDuty_2(PWMB_C);
                entrou = 1;
                break;

            case 1:
                entrou = 0;
                setDuty_1(PWMA); //témino da curva
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
    return Turn_curva; //retorna os valores após o PID
}

int parada(int sensor_esquerdo, int sensor_direito, int value_erro, int tempo_passed) {
    if ((!tst_bit(PORTD, sensor_de_curva)) && tst_bit(PORTD, sensor_de_parada)) {
        contador++;
        entrou_na_curva(sensor_esquerdo, sensor_direito, value_erro, tempo_passed); // Verifica se é uma curva
    } else if ((!tst_bit(PORTD, sensor_de_curva)) && (!tst_bit(PORTD, sensor_de_parada))) //verifica se é crizamento
    {
        setDuty_1(PWMA);
        setDuty_2(PWMB);
    }

    while (contador == numParada) {
        freio();
    }
}