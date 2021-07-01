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
//#include "configbits.txt"   //configura os fusíveis

//variáveis de comando para os registradores
#define set_bit(y,bit) (y|=(1<<bit)) //coloca em 1 o bit x da variável Y
#define clr_bit(y,bit) (y&=~(1<<bit)) //coloca em 0 o bit x da variável Y
#define cpl_bit(y,bit) (y^=(1<<bit)) //troca o estado lógico do bit x da variável Y
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

int Kp = 2; //prescale de 100 - prescale int
int Kd = 1; //prescale de 100 - prescale int
int Ki = 1; // Variáveis que são modificadas no PID - prescale de 100 ou mais
int Kp_tras = 2, Kd_tras = 1, Ki_tras = 1;
int PWMR = 200; // valor da força do motor em linha reta
int PWM_Curva = 150; //PWM ao entrar na curva
int erro, p, d, erroAnterior = 0, i, integral = 0, Turn = 0; //Área PID -erro também possui prescale = 100
int pt, dt, it;
int u = 0; //valor de retorno do PID
int u_curva = 0;
int prescale = 2000; //prescale das constantes * prescale do erro
int PWMA, PWMB; // Modulação de largura de pulso enviada pelo PID
int PWMA_C, PWMB_C; //PWM de curva com ajuste do PID
int entrou;
int erroAnterior_curva = 0, erroAnterior_traseiro = 0;
int Turn_curva, p_curva, d_curva, i_curva, integral_curva = 0;
int position_line = 0; // Método com o Read Line;
int contador = 0, numParada = 4; // Borda
int curvaValor = 0, entrou = 0;
unsigned int timer2, TempoEspera = 100;
int ejetor = 0;
int prescale_tras = 10, erro_tras = 0, u_tras = 0;
int integral_traseiro = 0, Turn_traseiro = 0;

unsigned int delta_T = 0;
int peso [] = {-3, -2, -1, 1, 2, 3}; //utilizando um prescale de 100
int soma_direito = 0, soma_esquerdo = 0;
int denominador_direito = 6;
int denominador_esquerdo = 6;
int soma_total = 0;
int tempo_atual = 0;

int valor_max [] = {1023, 1023, 1023, 1023, 1023, 1023};
int valor_min [] = {0, 0, 0, 0, 0, 0};
int valor_min_abs = 0, valor_max_abs = 1023;


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


unsigned int millis = 0;

ISR(TIMER0_OVF_vect) {
    TCNT0 = 240; //Recarrega o Timer 0 para que a contagem seja 1ms novamente
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
int PID(int error);
void frente();
void tras();
void esquerda();
void direita();
void motor_off();
void freio();
void entrou_na_curva(int valor_erro, int u_traseiro);
int PID_Curva(int error_curva);
int PID_traseiro(int erro_traseiro);
int parada(int value_erro, int u_traseir);
int calibra_sensores();
int seta_calibracao();
int sensores();

int main(void) {


    int sensores_traseiros [] = {(tst_bit(PIND, sensor_tras_direito) >> sensor_tras_direito), (tst_bit(PIND, sensor_tras_esquerdo) >> sensor_tras_esquerdo)};
    int soma_tras = 0;

    DDRD =  0b00101000; //PD5 e PD3 como saída e PD2, PD4, PD6 e PD7 como entrada 
    PORTD = 0b11010100; //inicializados em nível baixo e com pull aticadi nas saídas
    DDRB =  0b00101111; //Habilita PB1, PB3, PB2, PB0 e PB5 como saída
    PORTB = 0b00000000; //PORTB inicializa desligado e saídas sem pull up
    //DDRC =  0b00000000; //PORTC como entrada
    //PORTC = 0b00111111; //PORTC com pull up (somente se o sensores tiverem um RC)


    //esquerdo pino 4 - PD2
    UART_config(); //Inicializa a comunicação UART
    inicializa_ADC(); //Configura o ADC
    UART_enviaString(s); //Envia um texto para o computador

    TCCR0B = 0b00000101; //TC0 com prescaler de 1024
    TCNT0 = 240; //Inicia a contagem em 100 para, no final, gerar 1ms
    TIMSK0 = 0b00000001; //habilita a interrupção do TC0

    TCCR2A = 0xA3; //Configura operação em fast PWM, utilizando registradores OCR1x para comparação

    setFreq(6); //Seleciona opção para frequência

    sei(); //Habilita as interrupções


    //----> Calibração dos Sensores frontais <----\\
    //----> Calibração dos Sensores frontais <----//
    set_bit(PORTB, PB5); //subrotina de acender e apagar o LED 13
    calibra_sensores(); //calibração dos sensores
    sensores(); //determina o limiar dos sensores e printa seus valores na tela
    //========================//

    set_bit(PORTB, PB5); //subrotina de acender e apagar o LED 13
    _delay_ms(1000);
    clr_bit(PORTB, PB5);
    _delay_ms(500);
    set_bit(PORTB, PB5);
    _delay_ms(500);
    clr_bit(PORTB, PB5);


    while (1) {
        tempo_atual = millis;
        delta_T = tempo_atual - timer2;
        int sensores_frontais[] = {le_ADC(0), le_ADC(1), le_ADC(2), le_ADC(3), le_ADC(4), le_ADC(5)};

        for (int i = 0; i < 7; i++) {
            sprintf(buffer, "%4d", sensores_frontais[i]); //Converte para string
            UART_enviaString(buffer); //Envia para o computador
            UART_enviaCaractere(0x20); //espaço
        }
        UART_enviaCaractere(0x0A); //pula linha

        for (int i = 0; i < 3; i++) {
            soma_tras += sensores_traseiros[i];
        }

        soma_tras = (soma_tras / 2) * prescale_tras; //média do valor dos sensores traseiros

        erro_tras = 0 - soma_tras;

        u_tras = PID_traseiro(erro_tras);


        //região que seta os valores nos sensores frontais após a calibração
        sensores();
        //----------------------------------------------------------------//

        for (int j = 0; j < 3; j++) {
            soma_esquerdo += (sensores_frontais[j] * peso[j]);
            soma_direito += (sensores_frontais[5 - j] * peso[5 - j]);
        }

        soma_total = (soma_esquerdo + soma_direito) / (denominador_esquerdo + denominador_direito);

        erro = 0 - soma_total; //valor esperado(estar sempre em cima da linha) - valor medido

        soma_esquerdo = 0;
        soma_direito = 0;
        soma_total = 0; //precisa-se zeraar para não gerar o  acúmulo nessas variáveis

        sprintf(buffer, "%5d\n", erro); //Converte para string
        UART_enviaString(buffer); //Envia para o computador
        UART_enviaCaractere(0x0A); //pula linha

        //--------------->AREA DO PID<---------------

        u = PID(erro);

        PWMA = PWMR - u + u_tras;
        PWMB = PWMR + u - u_tras;

        if (delta_T >= TempoEspera) {

            timer2 = 0;
        }


        //--------------->AREA DOS SENSORES<---------------

        switch (ejetor) {
            case 0:
                if ((!(tst_bit(PIND, sensor_de_curva) >> sensor_de_curva))
                        || (!(tst_bit(PIND, sensor_de_parada) >> sensor_de_parada)))//verifica se sos sensores estão em nível 0
                {
                    timer2 = tempo_atual;
                    ejetor = 1;
                }
                break;

            case 1:
                if ((delta_T) > TempoEspera) {
                    parada(erro, u_tras); // Verifica se é um marcador de parada
                    ejetor = 2;
                }
                break;

            case 2:
                if ((tst_bit(PIND, sensor_de_curva) >> sensor_de_curva)
                        && (tst_bit(PIND, sensor_de_parada) >> sensor_de_parada)) {
                    timer2 = 0;
                    ejetor = 0;
                }
                break;
        }

        //-----> Área do senstido de giro

        if (erro < 0) //virar para a esquerda
        {
            entrou_na_curva(erro, u_tras);
            set_bit(PORTB, PB5); //liga o LED
            /*while (erro < 0) {
                esquerda();
            }*/

        } else if (erro > 0) {
            entrou_na_curva(erro, u_tras);
            set_bit(PORTB, PB5); //liga o LED
            /*while (erro > 0) {
                direita();
            }*/
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

int PID(int error) {
    p = (error * Kp) / prescale; // Proporcao

    integral += error; // Integral
    i = ((Ki * integral) / prescale);

    d = ((Kd * (error - erroAnterior)) / prescale); // Derivada
    erroAnterior = error;

    Turn = p + i + d;
    return Turn; //retorna os valores após o PID
}

void frente() {

    clr_bit(PORTB, AIN1);
    set_bit(PORTB, AIN2); //frente direita
    clr_bit(PORTD, BIN2);
    set_bit(PORTB, BIN1); //frente esquerda
}

void tras() {
    set_bit(PORTB, AIN1);
    clr_bit(PORTB, AIN2); //frente direita
    set_bit(PORTD, BIN2);
    clr_bit(PORTB, BIN1); //frente esquerda

}

void motor_off() {
    clr_bit(PORTB, AIN1);
    clr_bit(PORTB, AIN2);
    clr_bit(PORTD, BIN2);
    clr_bit(PORTB, BIN1);
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

/*void direita() { //pode criar uma função só para curva
    clr_bit(PORTB, AIN1);
    set_bit(PORTB, AIN2); //frente direita
    clr_bit(PORTD, BIN2);
    set_bit(PORTB, BIN1); //frente esquerda

    setDuty_1(PWMA_C);
    setDuty_2(PWMB_C);

    //calibração dos sensores frontais - seta o valor médio
}

void esquerda() {
    clr_bit(PORTB, AIN1);
    set_bit(PORTB, AIN2); //frente direita
    clr_bit(PORTD, BIN2);
    set_bit(PORTB, BIN1); //frente esquerda

    setDuty_1(PWMA_C);
    setDuty_2(PWMB_C);


    //calibração dos sensores frontais - seta o valor médio
}*/

void entrou_na_curva(int valor_erro, int u_traseiro) {
    if ((!tst_bit(PIND, sensor_de_curva) >> sensor_de_curva)
            && tst_bit(PIND, sensor_de_parada) >> sensor_de_parada) {
        switch (entrou) {
            case 0: //entrou na curva
                u_curva = PID_Curva(valor_erro);
                PWMA_C = PWM_Curva - u_curva + u_tras;
                PWMB_C = PWM_Curva + u_curva - u_tras;
                frente();
                setDuty_1(PWMA_C);
                setDuty_2(PWMB_C);
                entrou = 1;
                break;

            case 1:
                entrou = 0;
                frente();
                setDuty_1(PWMA); //témino da curva
                setDuty_2(PWMB);
                break;
                clr_bit(PORTB, PB5);
        }
    }
}

int PID_Curva(int error_curva) {
    p_curva = (error_curva * Kp) / prescale; // Proporcao

    integral_curva += error_curva; // Integral
    i_curva = ((Ki * integral_curva) / prescale);

    d_curva = ((Kd * (error_curva - erroAnterior_curva)) / prescale); // Derivada
    erroAnterior_curva = error_curva;

    Turn_curva = p_curva + i_curva + d_curva;
    return Turn_curva; //retorna os valores após o PID
}

int PID_traseiro(int erro_traseiro) {
    pt = (erro_traseiro * Kp) / prescale_tras; // Proporcao

    integral_traseiro += erro_traseiro; // Integral
    it = ((Ki_tras * integral_traseiro) / prescale_tras);

    dt = ((Kd_tras * (erro_traseiro - erroAnterior_traseiro)) / prescale_tras); // Derivada
    erroAnterior_traseiro = erro_traseiro;

    Turn_traseiro = pt + it + dt;
    return Turn_traseiro; //retorna os valores após o PID
}

int parada(int value_erro, int u_traseir) {
    if ((!tst_bit(PIND, sensor_de_curva) >> sensor_de_curva)
            && tst_bit(PIND, sensor_de_parada) >> sensor_de_parada) {
        contador++;
        entrou_na_curva(value_erro, u_traseir); // Verifica se é uma curva
    } else if ((!tst_bit(PIND, sensor_de_curva) >> sensor_de_curva)
            && (!tst_bit(PIND, sensor_de_parada) >> sensor_de_parada)) //verifica se é crizamento
    {
        frente();
        setDuty_1(PWMA);
        setDuty_2(PWMB);
    }

    if (contador == numParada) {
        freio();
    }
}

int calibra_sensores() {
    int calibrado = 0;
    //=====Função que inicializa a calibração====//
    for (int i = 0; i < 120; i++) {
        int sensores_frontais[] = {le_ADC(3), le_ADC(2), le_ADC(1), le_ADC(0), le_ADC(7), le_ADC(6)};
        for (int i = 0; i < 6; i++) {
            if (valor_min [i] < sensores_frontais [i]) {
                valor_min[i] = sensores_frontais[i];
            } 
            else if (valor_max [i] > sensores_frontais[i]) {
                valor_max[i] = sensores_frontais [i];
            }
        }

        /*
        Após isso determinar o limiar de todos os sensores para que eles tenham os mesmos valores do AD. 
        Para que todos tenham um limite inferior e superior igual.
         */
    }

    calibrado = seta_calibracao();
    return calibrado;
}

int seta_calibracao() {
    //----> Calibração dos Sensores frontais <----\\

    //função que seta o limiar dos sensores
    for (int i = 0; i < 6; i++) {
        if (valor_min_abs < valor_min [i]) {
            valor_min_abs = valor_min [i];
        }
        else if (valor_max_abs > valor_max [i]) {
            valor_max_abs = valor_max [i];
        }
    }
    //valores que os sensores não poderiam ultrapassar
    //return (valor_min_abs, valor_max_abs);
}

int sensores() {
    seta_calibracao(); //Estabelece os limites dos sensores

    int sensores_frontais[6] = {le_ADC(3), le_ADC(2), le_ADC(1), le_ADC(0), le_ADC(7), le_ADC(6)};
    //======Estabelece o limiar da leitura dos sensores====//
    //função de correção da calibração
    for (int i = 0; i < 6; i++) {
        if (valor_min_abs < sensores_frontais[i]) {
            sensores_frontais[i] = valor_min_abs;
        } 
        else if (valor_max_abs > sensores_frontais[i]) {
            sensores_frontais [i] = valor_max_abs;
        }

        sprintf(buffer, "%4d", sensores_frontais[i]); //Converte para string
        UART_enviaString(buffer); //Envia para o computador
        UART_enviaCaractere(0x20); //espaço
    }
    UART_enviaCaractere(0x0A); //pula linha
}