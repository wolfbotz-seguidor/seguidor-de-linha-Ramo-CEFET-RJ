/*
 * File:   main.c
 * Author: Johann
 *
 * Created on 30 de Maio de 2021, 16:52
 * Last update on December 16th of 2021 at 17:55
 */

/*Este código é de um robô seguidor de linha da equipe Wolfbotz.
 * Aqui nós vemos o controle do robô bem como as tomadas de decisão de acordo com os padrões da pista*/
#include "HAL_atmega328p.h"
#include <stdio.h>            //Bilioteca do C
#include "UART.h"             //Biblioteca da comunicação UART
#include "ADC.h"              //Biblioteca do conversor AD
#include "PWM_10_bits.h"      //Biblioteca de PWM fast mode de 10 bits
#include "Driver_motor.h"     //Biblioteca das funções de controle dos motores  //usado para ponte H tb6612fng
#include "PID.h"              //Biblioteca do controle PID
#include "sensor_logic.h"     //lógica utilizando os sensores
//#include "dados.h"            //biblioteca que contém as funções atraladas ao envio de informações via UART
/*============================================================*/

/*==============================================================*/

/*Macros*/
#define NOP() __asm__ __volatile__ ("nop")

/*Variáveis globais*/
unsigned int PWMA = 0, PWMB = 0; // Modulação de largura de pulso enviada pelo PID
//variáveis de controle
char f_parada= 0;       //variável que comanda quando o robô deve parar e não realizar mais sua rotina
char flag = 0;          //variável de controle para identificar o momento de parada
char f_motor = 0;       //variável de controle da calibração automática
char flag_curva = 0;    //cronometragem entre as retas e as cruvas
char flag_parada = 0;   //inicia e encerra a cronometragem da pista
char f_calibra = 0;     //vai pra 1 quando a calibração é finalizada

unsigned char f_stop = 0;           //encerra a rotina
unsigned char f_millis = 0;         //controle do cronômetro
unsigned int millisegundos = 0;        //millisegundos

/*Variáveis do encoder*/
unsigned char pulse_numberR = 0, pulse_numberL = 0; //variáveis para contagem dos pulsos dos encoders

/*Variáveis da UART*/
volatile char ch; //armazena o caractere lido
volatile char flag_com = 0; //flag que indica se houve recepção de dado*/


/*Protótipo das funções*/
void setup();
void setup_logica();        //variáveis utilizadas na lógica
void loop();
void estrategia();          //estrategia do robô
//---------------------------------------------------------------//
void Auto_calibration(void);
void calibration();         //contêm toda a rotina de calibração
//--------------------------------------------------------------------//
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

/*Interrupções*/
ISR(USART_RX_vect) {
    ch = UDR0; //Faz a leitura do buffer da serial

    UART_enviaCaractere(ch); //Envia o caractere lido para o computador
    flag_com = 1; //Aciona o flag de comunicação
}

ISR(TIMER0_OVF_vect) 
{
    TCNT0 = 56; //Recarrega o Timer 0 para que a contagem seja 100us novamente
    
    f_timers(); //função de temporização das rotinas   
}//end TIMER_0

ISR(ADC_vect)
{
    ADC_maq();  //máquina de estado do conversor AD
}//end ADC_int

/*Função principal*/
int main(void) 
{
    setup();
    while (1) loop();
    return 0;
}//end main

/*RTOS primitivo*/

#define temp_min 1
/*Parte não visível ao usuário*/
void f_timers (void) 
{
    static unsigned char c_timer1 = 0;
    static unsigned char c_timer2 = 0;
    static unsigned char c_timer3 = 0;
    static unsigned char c_timer4 = 0;
    static unsigned char c_timer5 = 0;
        
    //funções a cada 200us
    if(c_timer1 < 2 - temp_min)      //tempo que quer menos 1
    {
        c_timer1++;
    }

    else
    {
        f_timer1();
        c_timer1 = 0;
    }

    /*300us*/
    if (c_timer2 < 3 - temp_min)   //o 0 conta na contagem -> 3-1
    {
        c_timer2++; //100us -1; 200us-2;300 us-3; 300us de intervalo de tempo
    }

    else    //a cada 300us
    {
        f_timer2();
        c_timer2 = 0;
    }

    if(c_timer3 < 100 - temp_min)   //10ms
    {
        c_timer3++;
    }

    else
    {
        f_timer3();
        c_timer3 = 0;
    }

    //Timer
    //1ms
    if(c_timer4 < 10 - temp_min)
    {
        c_timer4++;
    }

    else
    {
        f_timer4();
        c_timer4 = 0;
    }
    
    if(c_timer5 < 10 - temp_min)   //1000us
    {
        c_timer5++;
    }

    else
    {
        f_timer5();
        c_timer5 = 0;
    }
    
}//fim do RTOS


//===Funções não visíveis ao usuário======//
void setup() 
{

    setup_Hardware();   //setup das IO's e das interrupções
    sei();              //Habilita as interrupções
    calibration();      //rotina de calibração
    setup_logica();     //definição das variáveis lógicas(vazio por enquanto)

}//end setup

void setup_logica() /*Função que passa ponteiros para funções como parâm*/
{
    
}


void loop()//loop vazio
{

}

void estrategia()
{
    if(f_calibra)//se f_calibra for 1...
    {
        if (!f_parada)  //se f_parada for 0... 
        {
            sensores();             //seta o limiar da leitura dos sensores
            calculo_do_erro();      //faz a média ponderada e calcula o erro
            sentido_de_giro();      //Verifica se precisa fazer uma curva e o cálculo do PID
            volta_pra_pista();      //corrige o robô caso saia da linha
        }
    }   
}

void Auto_calibration(void)/*Sem uso*/
{
    static unsigned char flag_D = 0, flag_E = 0;
    /*Calibração automática
     Robô gira um dos motores num sentido
    *num intervalo de tempo e depois mudar o sentido de giro.
    *Em seguida fazer o mesmo com a outra roda.*/
    
    if(!f_motor)
    {
        if(!flag_D)
        {
            direita_frente();
            setDuty_1(300);
            setDuty_2(0);
            flag_D = 1;
        }

        else if(flag_D)
        {
            direita_tras();
            setDuty_1(300);
            setDuty_2(0);
            flag_E = 1;
        }

        else if(flag_E)
        {
            esquerda_frente();
            setDuty_1(0);
            setDuty_2(300);
            flag_E = 0;
        }

        else if(!flag_E)
        {
            esquerda_tras();
            setDuty_1(0);
            setDuty_2(300);
            f_motor = 1;
        }
    }
}

void calibration()
{
     //----> Calibração dos Sensores frontais <----//
    set_bit(PORTB, led);
    ADC_maq();  //inicializa a conversão do AD
    //calibra_sensores(); //calibração dos sensores //A calibração vai conseguir acompanhar o AD
                                                  //ou pode ser que o vetor não seja preenchido a tempo?
                                                  //É necessário colocar um contador
                                                  //para depois chamar a função de calibração?
    
    seta_calibracao(); //estabelece o limiar dos sensores através dos valores da função de cima
    
    clr_bit(PORTB, led);
    _delay_ms(250);
    set_bit(PORTB, led); //subrotina de acender e apagar o LED 13
    _delay_ms(250);
    clr_bit(PORTB, led);
    _delay_ms(250);
    set_bit(PORTB, led);
    _delay_ms(250);
    clr_bit(PORTB, led);
    _delay_ms(1000);
    
    f_calibra = 1;
    
    /*if(f_motor) //para o robô para iniciar a rotina
    {
        motor_off();
        setDuty_1(0);
        setDuty_2(0);
    }*/
}



void parada() 
{     
    if(f_calibra)
    {
        le_marcadores();
        
    }

}


void fim_de_pista()
{
    static char parada = 0;
    
    if(flag)
    {
       parada++;
       flag = 0;
    }
    
    
    if(parada > 1)  //dois marcadores de parada
    {
        f_parada = 1;
        freio();
        parada = 0;
    }
    
}

/*funções do encoder*/

void count_pulsesD() 
{
    static int Encoder_C1Last = 0, direction_m;

    int Lstate = 0;//tst_bit(PIND, encoder_C1D); //variável de leitura de um dos pinos do encoderD

    if (!Encoder_C1Last && Lstate) { //Verifica se Encoder_C1Last é falso e Lstate é verdadeiro
        int val = 0;//tst_bit(PIND, encoder_C2D); //Variável de leitura do segundo pino do encoderD

        if (!val && direction_m) direction_m = 0; //sentido horário

        else if (val && !direction_m) direction_m = 1; //sentido anti-horário
    }

    Encoder_C1Last = Lstate;

    if (!direction_m) pulse_numberR++; //sentido horário
    else pulse_numberR--;



}

void count_pulsesE()
{
    static int Encoder_C1Last = 0, direction_m;

    int Lstate = 0;//tst_bit(PIND, encoder_C1E); //variável de leitura de um dos pinos do encoderD

    if (!Encoder_C1Last && Lstate) { //Verifica se Encoder_C1Last é falso e Lstate é verdadeiro
        int val = 0;//tst_bit(PIND, encoder_C2E); //Variável de leitura do segundo pino do encoderD

        if (!val && direction_m) direction_m = 0; //sentido horário

        else if (val && !direction_m) direction_m = 1; //sentido anti-horário
    }

    Encoder_C1Last = Lstate;

    if (!direction_m) pulse_numberR++; //sentido horário
    else pulse_numberL--; //sentido anti-horário


}


void millis(void)
{
    //static unsigned int f_read = 0;
    
    if (f_millis)
    {
        millisegundos = 0;
    }
    else
    {
        millisegundos++;
    }
    
}

void f_timer1(void)
{
    parada();
    fim_de_pista();         //Verifica se é o fim da pista
}

void f_timer2(void)
{
    estrategia();
}

void f_timer3(void)     //10ms
{   
    static unsigned char c_timer1 = 0, c_timer2 = 0;
    
    if(c_timer1 < 200 - temp_min)  //2000ms = 2s
    {
        c_timer1++;
    }

    else 
    {   
        //Auto_calibration();
        c_timer1 = 0;
    }
    
    if(c_timer2 < 50 - temp_min)  //500ms = 0,5s
    {
        c_timer2++;
    }

    else 
    {   
        //telemetria();
        c_timer2 = 0;
    }
}

void f_timer4(void)
{
    millis();   //função chamada a cada 1ms 

    //colocar millis aqui dentro
    //
}

void f_timer5(void)
{
    if(!f_stop)
    {
        //coleta_de_dados();
    }
}
