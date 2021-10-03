/*
 * File:   main.c
 * Author: Johann
 *
 * Created on 30 de Maio de 2021, 16:52
 * Last update on July 7th of 2021 at 15:43
 */

/*Este código é de um robô seguidor de linha da equipe Wolfbotz.
 * Aqui nós vemos o controle do robô bem como as tomadas de decisão de acordo com os padrões da pista*/

/*Bibliotecas e frequência do uc*/
#define F_CPU 16000000        //define a frequencia do uC para 16MHz
#include <avr/io.h>           //Biblioteca geral dos AVR
#include <avr/interrupt.h>    //Biblioteca de interrupção
#include <util/delay.h>       //Biblioteca geradora de atraso
#include <stdio.h>            //Bilioteca do C
#include "UART.h"             //Biblioteca da comunicação UART
#include "ADC.h"              //Biblioteca do conversor AD
#include "PWM_10_bits.h"      //Biblioteca de PWM fast mode de 10 bits
#include "Driver_motor.h"     //Biblioteca das funções de controle dos motores  //usado para ponte H tb6612fng
#include "PID.h"              //Biblioteca do controle PID
/*============================================================*/

//variáveis de comando para os registradores
#define set_bit(y,bit) (y|=(1<<bit)) //coloca em 1 o bit x da variável Y
#define clr_bit(y,bit) (y&=~(1<<bit)) //coloca em 0 o bit x da variável Y
#define cpl_bit(y,bit) (y^=(1<<bit)) //troca o estado lógico do bit x da variável Y
#define tst_bit(y,bit) (y&(1<<bit)) //retorna 0 ou 1 conforme leitura do bit
/*==============================================================*/

/*Mapeamento de Hardware*/
#define sensor_de_curva   PB0
#define sensor_de_parada  PD7
#define led               PB5
#define leitura_curva    PINB
#define leitura_parada   PIND

/*==============================================================*/

/*Variáveis globais*/
char erro = 0;      //variável para cáculo do erro da direção do robô em cima da linha
int PWMA = 0, PWMB = 0; // Modulação de largura de pulso enviada pelo PID
unsigned char sensores_frontais[6];

//Variáveis globais da calibração de sensores
unsigned char valor_max[6] = {0, 0, 0, 0, 0, 0};
unsigned char valor_min[6] = {255, 255, 255, 255, 255, 255};
unsigned char valor_max_abs = 255;
unsigned char valor_min_abs = 0;

//variáveis de controle
char f_parada= 0;   //variável que comanda quando o robô deve parar e não realizar mais sua rotina
char f_calibra = 0; //variável que indica o fim da calibração dos esnores e inicio da estratégia
char flag = 0;      //variável de controle para identificar o momento de parada

//Variáveis globais da UART
char buffer[5]; //String que armazena valores de entrada para serem printadas
volatile char ch; //armazena o caractere lido
volatile char flag_com = 0; //flag que indica se houve recepção de dado

/*tempo =65536 * Prescaler/Fosc = 65536 * 1024/16000000 = 4, 19s
 tempo = X_bit_timer * Prescaler/Fosc
 Valor inicial de contagem = 256 - tempo_desejado*Fosc/Prescaler = 256 - 0,001*16000000/1024 = 255
 Valor inicial de contagem = X_bit_timer - tempo_desejado*Fosc/Prescaler */
/*===========================================================================*/

/*Protótipo das funções*/
void ADC_maq();             //máquina de estado do conversor AD
void INT_INIT();            //Inicializador das interrupções (Timer0)
//void entrou_na_curva(char valor_erro);
void parada();              //Leitura dos sensores laterais
void calibra_sensores();    //calibra sensores manualmente 
void seta_calibracao();     //estabelece o limiar dos valores máximos e mínimos de leitura
void sensores();            //caso um sensor passe do valor, o mesmo é corrigido
void setup();
void setup_Hardware();      //define os registradores
void setup_logica();
void loop();
void sentido_de_giro();     //lê os sensores frontais e determina o sentido de giro dos motores com o PID
void PWM_limit();           //limita o PWM em 1000 caso a variável passe de 1023 
void estrategia();          //estrategia do robô
void calibration();         //contêm toda a rotina de calibração
void fim_de_pista();        //verifica se é o fim da psita
void f_timers (void);       //função de temporização das rotinas
/*===========================================================================*/


/*Interrupções*/
ISR(USART_RX_vect) {
    ch = UDR0; //Faz a leitura do buffer da serial

    UART_enviaCaractere(ch); //Envia o caractere lido para o computador
    //flag_com = 1; //Aciona o flag de comunicação
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

/*============================================================================*/


/*Função principal*/
int main(void) 
{
    setup();

    while (1) loop();
    return 0;
}//end main

//===Funções não visíveis ao usuário======//

void setup() 
{

    setup_Hardware();   //setup das IO's e das interrupções
    calibration();      //rotina de calibração
    setup_logica();     //definição das variáveis lógicas(vazio por enquanto)
    sei();              //Habilita as interrupções

}//end setup


void setup_Hardware()
{
    MCUCR &= 0xef;      //habilita pull up quando configurado e desabilita algumas configurações prévias do MCU

    DDRD = 0b01111010; //PD3 - PD6 definidos como saída, PD7 como entrada, PD0 como entrada(RX) e PD1 como saída(TX)
    PORTD = 0b10000000; //inicializados em nível baixo e PD7 com pull up
    DDRB = 0b00100110; //Habilita PB0 como entrada e PB5, PB1 e PB2 como saída
    PORTB = 0b00000001; //PORTB inicializa desligado e pull up no PB0
    DDRC = 0b00000000; //PORTC como entrada
    PORTC = 0b00001111; //PC3 - PC0 com pull up (colocar resistor de pull up nos pinos A6 e A7)
    TCCR1A = 0xA2; //Configura operação em fast PWM, utilizando registradores OCR1x para comparação

    //esquerdo pino 4 - PD2
    //UART_config(16); //Inicializa a comunicação UART com 57.6kbps
    
    setFreq(4); //Seleciona opção para frequência
    //62,5Hz de PWM
    
    ADC_init(); //Inicializa o AD
    INT_INIT(); //Inicializa o Timer0

    
}//end setup_hardware

void calibration()
{
     //----> Calibração dos Sensores frontais <----//
    set_bit(PORTB, led);
    ADC_maq();  //inicializa a conversão do AD
    calibra_sensores(); //calibração dos sensores //A calibração vai conseguir acompanhar o AD
                                                  //ou pode ser que o vetor não seja preenchido a tempo?
                                                  //É necessário colocar um contador
                                                  //para depois chamar a função de calibração?
    
    seta_calibracao(); //estabelece o limiar dos sensores através dos valores da função de cima
    
    clr_bit(PORTB, led);
    _delay_ms(500);
    set_bit(PORTB, led); //subrotina de acender e apagar o LED 13
    _delay_ms(1000);
    clr_bit(PORTB, led);
    _delay_ms(500);
    set_bit(PORTB, led);
    _delay_ms(500);
    clr_bit(PORTB, led);
    _delay_ms(2000);
    f_calibra = 1;  //flag para indicar fim da calibração
}

void setup_logica(){
   
    
    
}

void INT_INIT()
{    //fórmula nas primeiras linhas do código
    TCCR0B = 0b00000010; //TC0 com prescaler de 8
    TCNT0 = 56; //Inicia a contagem em 56 para, no final, gerar 100us
    TIMSK0 = 0b00000001; //habilita a interrupção do TC0
}

void loop()//loop vazio
{
    
}

void ADC_maq () 
{
    //inicializo no setup na função calibration e em seguida toda
    //vez que ocorre uma converção a interrupção do AD ocorre
    //e então esta função é chamada pelo vetor de interrupção
    //do AD, obtendo os dados da conversão em "paralelo" à rotina
    
    //Leio primeiro o default que seria o primeiro canal
    //e em seguida faço uma lógica circular de leitura dos canais
    
    static unsigned char estado = 10;
    
    switch (estado) {
        
        case 0:
            estado = 1;
            sensores_frontais[0] = ADC_ler();
            ADC_conv_ch(2);
            break;
            
        case 1:
            estado = 2;
            sensores_frontais[1] = ADC_ler();
            ADC_conv_ch(1);
            break;
            
        case 2:
            estado = 3;
            sensores_frontais[2] = ADC_ler();
            ADC_conv_ch(0);
            break;
            
        case 3:
            estado = 4;
            sensores_frontais[3] = ADC_ler();
            ADC_conv_ch(7);
            break;
            
        case 4:
            estado = 5;
            sensores_frontais[4] = ADC_ler();
            ADC_conv_ch(6);
            break;
            
        case 5:
            estado = 6;
            sensores_frontais[5] = ADC_ler();
            ADC_conv_ch(3);
            break;
            
        default:
            estado = 0;
            ADC_conv_ch(3);
            sensores_frontais[0] = ADC_ler();
            break; 
    }
    
}//end ADC_maq

//=========Funções visíveis ao usuário===========//
//Função só útil após o mapeamneto
/*void entrou_na_curva(int valor_erro) {
    int u_curva = 0;
    static unsigned int PWMA_C = 0, PWMB_C = 0, entrou = 0; //PWM de curva com ajuste do PID;
    static unsigned int PWM_Curva = 350; //PWM ao entrar na curva

    if ((!tst_bit(leitura_curva, sensor_de_curva)) && tst_bit(leitura_parada, sensor_de_parada))
        //li branco no sensor de curva e li preto no sensor de parada
    {
        switch (entrou) {
            case 0: //entrou na curva
                u_curva = PID(valor_erro);
                PWMA_C = PWM_Curva - u_curva;
                PWMB_C = PWM_Curva + u_curva;
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
                clr_bit(PORTB, led);
                break;
        }
    }
}*/

void parada() 
{   
    //leitura de marcador de curva
    /*static char contador = 0, num_curva = 6;
    if ((!tst_bit(leitura_curva, sensor_de_curva)) && (tst_bit(leitura_parada, sensor_de_parada)))
    {
        contador++;
    }*/
    
    //cruzamento
    //branco = 0, preto = 1
    if ((!tst_bit(leitura_curva, sensor_de_curva)) && (!tst_bit(leitura_parada, sensor_de_parada))) //verifica se é crizamento
    {
        /*frente();
        setDuty_1(PWMA);
        setDuty_2(PWMB);*/
        flag = 0;
    }
    
    //leitura de marcador de parada
    else if ((tst_bit(leitura_curva, sensor_de_curva)) && (!tst_bit(leitura_parada, sensor_de_parada)))
    {
        flag = 1;
    }

}

void calibra_sensores() 
{
    //=====Função que inicializa a calibração====//
    for (int i = 0; i < 120; i++) {
        for (int i = 0; i < 6; i++) {
            if (sensores_frontais[i] < valor_min [i]) {
                valor_min[i] = sensores_frontais[i];
            }
            if (sensores_frontais[i] > valor_max [i]) {
                valor_max[i] = sensores_frontais[i];
            }
        }

        _delay_ms(10);  //tempo o suficiente para o pessoa calibrar os sensores mecanicamente
        
        /*
        Após isso determinar o limiar de todos os sensores para que eles tenham os mesmos valores do AD. 
        Para que todos tenham um limite inferior e superior igual.
        */
    }

}

void seta_calibracao() {
    //----> Calibração dos Sensores frontais <----//

    //função que seta o limiar dos sensores
    //Este é o algoritmo a ser usado no robô. Desmcomente antes de compilar e comente o outro.
    for (int i = 0; i < 6; i++) {
        if (valor_min [i] > valor_min_abs)// && valor_min[i] !=0 ) //esse !0 foi colocado pois estava havendo um bug ao simular
        {
            valor_min_abs = valor_min [i];
        } 
        
        if (valor_max [i] < valor_max_abs) {
            valor_max_abs = valor_max [i];
        }

    }
}

void sensores() {

    //======Estabelece o limiar da leitura dos sensores====//
    //função de correção da calibração
    for (int i = 0; i < 6; i++) {
        if (sensores_frontais[i] < valor_min_abs) {
            sensores_frontais[i] = valor_min_abs;
        }
        if (sensores_frontais[i] > valor_max_abs) {
            sensores_frontais[i] = valor_max_abs;
        }

    }
}


void sentido_de_giro()
{
    //-----> Área do senstido de giro
    unsigned char u = 0; //valor de retorno do PID
    static unsigned int PWMR = 400; // valor da força do motor em linha reta
    unsigned char u_curva = 0; //valor de retorno do PID numa curva
    static unsigned int PWMA_C = 0, PWMB_C = 0; //PWM de curva com ajuste do PID;
    static unsigned int PWM_Curva = 350; //PWM ao entrar na curva

    if ((sensores_frontais[0] < 50 && sensores_frontais[5] > 225) || (sensores_frontais[0]  > 225 && sensores_frontais[5] < 50))    
        //se o primeiro sensor ou o último sensor estiverem lendo branco...
        //necessário teste com monitor serial
        //estudar a melhor quantidade de sensores e seu espaçamento
    {
        u_curva = PID(erro);
        PWMA_C = PWM_Curva - u_curva;
        PWMB_C = PWM_Curva + u_curva;
        frente();
        PWM_limit();
        setDuty_1(PWMA_C);
        setDuty_2(PWMB_C);
        //flag1 = 1;
    } //em cima da linha
        
    else
    { 
        //pra frente - reta
        //flag1 = 0x00;
        //--------------->AREA DO PID<---------------

        u = PID(erro);

        PWMA = PWMR - u;
        PWMB = PWMR + u;

        frente();
        PWM_limit();
        setDuty_1(PWMA);
        setDuty_2(PWMB);
    }

    //A função que fazia o robô rodar em seu próprio eixo foi removida

}

void PWM_limit() {
    //------> Limitando PWM

    if (PWMA > 1023) {
        PWMA = 1000;
    } 
    else if (PWMB > 1023) {
        PWMB = 1000;
    }
}

void calculo_do_erro()
{
    unsigned int soma_direito = 0, denominador_direito = 6, denominador_esquerdo = 6;
    int soma_esquerdo = 0;
    char soma_total = 0;   //caso aumente o peso da média_ponderada, tomar cuidado com a variável char
    
    static char peso [] = {-3, -2, -1, 1, 2, 3}; //utilizando um prescale de 2000
    //os pesos precisarão ser corrigidos pois os sensores do Van Grogue estão um pouco assimétricos
    
    for (int j = 0; j < 3; j++) 
    {
        soma_esquerdo += (sensores_frontais[j] * peso[j]);
        soma_direito += (sensores_frontais[5-j] * peso[5 - j]);
    }

    soma_total = (soma_esquerdo + soma_direito) / (denominador_esquerdo + denominador_direito);
    
    erro = 0 - soma_total;   //valor esperado(estar sempre em cima da linha) - valor medido
}

void estrategia()
{
    
    if (!f_parada)  //se f_parada dor 0... 
    {
        sensores();             //seta o limiar da leitura dos sensores
        calculo_do_erro();      //faz a média ponderada e calcula o erro
        sentido_de_giro();      //Verifica se precisa fazer uma curva e o cálculo do PID
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
        freio();
        f_parada = 1;
    }
}

void f_timers (void) {

    static unsigned char c_timer2 = 0; //c_timer1 = 0, 
    if(f_calibra)
    {
        
        //funções a cada 100us
        parada();
        fim_de_pista();         //Verifica se é o fim da pista
        
        if (c_timer2 < 3)
        {
            c_timer2++;
        }
        
        else    //a cada 400us
        {
            estrategia();
            c_timer2=0;
        }
    }
}//fim do programa