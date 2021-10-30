/*
 * File:   main.c
 * Author: Johann
 *
 * Created on 30 de Maio de 2021, 16:52
 * Last update on July 7th of 2021 at 15:43
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
/*============================================================*/

/*==============================================================*/

/*Variáveis globais*/
int erro = 0;      //variável para cáculo do erro da direção do robô em cima da linha
unsigned int PWMA = 0, PWMB = 0; // Modulação de largura de pulso enviada pelo PID
unsigned int PWMA_C = 0, PWMB_C = 0; //PWM de curva com ajuste do PID;
unsigned char sensores_frontais[5];
unsigned int PWMR = 220; // valor da força do motor em linha reta
unsigned int PWM_Curva = 180; //PWM ao entrar na curva

//Variáveis globais da calibração de sensores
unsigned char valor_max[5] = {0, 0, 0, 0, 0};
unsigned char valor_min[5] = {255, 255, 255, 255, 255};
unsigned char valor_max_abs = 255;
unsigned char valor_min_abs = 0;

//variáveis de controle
char f_parada= 0;   //variável que comanda quando o robô deve parar e não realizar mais sua rotina
char f_calibra = 0; //variável que indica o fim da calibração dos sensores e inicio da estratégia
char flag = 0;      //variável de controle para identificar o momento de parada
char flag_com = 0;
char f_motor = 0;   //variável de controle da calibração automática

volatile char ch; //armazena o caractere lido
char buffer[5]; //String que armazena valores de entrada para serem printadas
//Variáveis globais da UART
/*char buffer[5]; //String que armazena valores de entrada para serem printadas
volatile char ch; //armazena o caractere lido
volatile char flag_com = 0; //flag que indica se houve recepção de dado*/

/*tempo =65536 * Prescaler/Fosc = 65536 * 1024/16000000 = 4, 19s
 tempo = X_bit_timer * Prescaler/Fosc
 Valor inicial de contagem = 256 - tempo_desejado*Fosc/Prescaler = 256 - 0,001*16000000/1024 = 255
 Valor inicial de contagem = X_bit_timer - tempo_desejado*Fosc/Prescaler */
/*===========================================================================*/

/*Protótipo das funções*/
void ADC_maq();             //máquina de estado do conversor AD
void parada();              //Leitura dos sensores laterais
void calibra_sensores();    //calibra sensores manualmente 
void seta_calibracao();     //estabelece o limiar dos valores máximos e mínimos de leitura
void sensores();            //caso um sensor passe do valor, o mesmo é corrigido
void setup();
void setup_logica();
void loop();
void sentido_de_giro();     //lê os sensores frontais e determina o sentido de giro dos motores com o PID
void PWM_limit();           //limita o PWM em 1000 caso a variável passe de 1023 
void estrategia();          //estrategia do robô
void calibration();         //contêm toda a rotina de calibração
void fim_de_pista();        //verifica se é o fim da psita
void f_timers (void);       //função de temporização das rotinas
void Auto_calibration(void);
void volta_pra_pista(void);
void volta_pra_pista_calibracao(void);
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

//===Funções não visíveis ao usuário======//
void setup() 
{

    setup_Hardware();   //setup das IO's e das interrupções
    sei();              //Habilita as interrupções
    calibration();      //rotina de calibração
    setup_logica();     //definição das variáveis lógicas(vazio por enquanto)

}//end setup


void calibration()
{
     //----> Calibração dos Sensores frontais <----//
    set_bit(PORTB, led);
    ADC_maq();  //inicializa a conversão do AD
    calibra_sensores(); //calibração dos sensores //A calibração vai conseguir acompanhar o AD
                                                  //ou pode ser que o vetor não seja preenchido a tempo?
                                                  //É necessário colocar um contador
                                                  //para depois chamar a função de calibração?
    /*if(f_motor) //para o robô para iniciar a rotina
    {
        volta_pra_pista_calibracao();
        motor_off();
        setDuty_1(0);
        setDuty_2(0);
    }*/
    
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
    
    f_calibra = 1;  //flag para indicar fim da calibração
}

void setup_logica()
{
    
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
            ADC_conv_ch(1);
            break;
            
        case 1:
            estado = 2;
            sensores_frontais[1] = ADC_ler();
            ADC_conv_ch(0);
            break;
            
        case 2:
            estado = 3;
            sensores_frontais[2] = ADC_ler();
            ADC_conv_ch(7);
            break;
            
        case 3:
            estado = 4;
            sensores_frontais[3] = ADC_ler();
            ADC_conv_ch(6);
            break;
            
        case 4:
            estado = 0;
            sensores_frontais[4] = ADC_ler();
            ADC_conv_ch(2);
            break;
            
            
        default:
            estado = 0;
            ADC_conv_ch(2);
            sensores_frontais[0] = ADC_ler();
            break; 
    }
    
}//end ADC_maq

//=========Funções visíveis ao usuário===========//
void parada() 
{   
    //cruzamento
    //branco = 0, preto = 1
    static unsigned char flag_count = 0;
    
    //leitura de marcador de parada
    if ((tst_bit(leitura_curva, sensor_de_curva)) && (!tst_bit(leitura_parada, sensor_de_parada)) && !flag_count)
    {
        flag = 1;
        flag_count = 1;
        set_bit(PORTB, PB5);
    }
    
    else if ((!tst_bit(leitura_curva, sensor_de_curva)) && (!tst_bit(leitura_parada, sensor_de_parada)) && flag_count) //verifica se é cruzamento
    {
        flag = 1;
        flag_count = 0;
        clr_bit(PORTB, PB5);
    }
    
    else if ((tst_bit(leitura_curva, sensor_de_curva)) && (tst_bit(leitura_parada, sensor_de_parada)) && flag_count)
    {
        flag = 0;
        flag_count = 0;
        clr_bit(PORTB, PB5);
    }
    else if (!(tst_bit(leitura_curva, sensor_de_curva)) && (tst_bit(leitura_parada, sensor_de_parada)) && flag_count)
    {
        flag = 0;
        flag_count = 0;
        clr_bit(PORTB, PB5);
    }


}

void calibra_sensores() 
{
    //=====Função que inicializa a calibração====//
    for (int i = 0; i < 120; i++) {
        for (int i = 0; i < 5; i++) {
            if ((sensores_frontais[i] < valor_min [i]) && sensores_frontais[i] !=0) //correção na calibração
            {
                valor_min[i] = sensores_frontais[i];
            }
            if (sensores_frontais[i] > valor_max [i])
            {
                valor_max[i] = sensores_frontais[i];
            }
        }

        //_delay_ms(20);  //tempo o suficiente para o pessoa calibrar os sensores mecanicamente
        
        /*
        Após isso determinar o limiar de todos os sensores para que eles tenham os mesmos valores do AD. 
        Para que todos tenham um limite inferior e superior igual.
        */
    }
    
    //f_motor = 1;

}

void seta_calibracao() {
    //----> Calibração dos Sensores frontais <----//

    //função que seta o limiar dos sensores
    //Este é o algoritmo a ser usado no robô. Desmcomente antes de compilar e comente o outro.
    for (int i = 0; i < 5; i++) {
        if ((valor_min [i] > valor_min_abs)) //esse !0 foi colocado pois estava havendo um bug ao simular
        {
            valor_min_abs = valor_min [i];
        } 
        
        if (valor_max [i] < valor_max_abs) {
            valor_max_abs = valor_max [i];
        }
        

    }

    
    
    valor_min_abs = 100; //valores vistos pelo monitor serial
    valor_max_abs = 200;
}

void sensores() 
{

    //======Estabelece o limiar da leitura dos sensores====//
    //função de correção da calibração
    for (int i = 0; i < 5; i++)
    {
        if (sensores_frontais[i] < valor_min_abs) 
        {
            sensores_frontais[i] = valor_min_abs;
        }
        if (sensores_frontais[i] > valor_max_abs)
        {
            sensores_frontais[i] = valor_max_abs;
        }

    }
    
    
}

void Auto_calibration(void)
{
    static unsigned char flag_rotate = 0;
    /*Calibração automática
     Robô gira um dos motores num sentido
    *num intervalo de tempo e depois mudar o sentido de giro.
    *Em seguida fazer o mesmo com a outra roda.*/
    
    switch(flag_rotate)
    {
        case 0:
            flag_rotate = 1;
            giro_direita();
            setDuty_1(PWM_Curva);
            setDuty_2(PWM_Curva);
            break;
            
        case 1:
            flag_rotate = 2;
            giro_esquerda();
            setDuty_1(PWM_Curva);
            setDuty_2(PWM_Curva);
            break;

        case 2:
            flag_rotate = 3;
            giro_esquerda();
            setDuty_1(PWM_Curva);
            setDuty_2(PWM_Curva);
            break;
            
        case 3:
            flag_rotate = 0;
            giro_direita();
            setDuty_1(PWM_Curva);
            setDuty_2(PWM_Curva);
            break;
        
    }
    
}

void sentido_de_giro()
{
    //-----> Área do senstido de giro
    static int u = 0; //valor de retorno do PID
    static int u_curva = 0; //valor de retorno do PID numa curva
    
    if ((sensores_frontais[0] < 116 && sensores_frontais[4] > 196) || (sensores_frontais[0]  > 196 && sensores_frontais[4] < 109))    
        //Valores vistos na serial
        //se o primeiro sensor ou o último sensor estiverem lendo branco...
        //necessário teste com monitor serial
        //estudar a melhor quantidade de sensores e seu espaçamento
    {
        u_curva = PID(erro);
        PWMA_C = PWM_Curva + u_curva;
        PWMB_C = PWM_Curva - u_curva;
        frente();
        PWM_limit();
        setDuty_1(PWMA_C);
        setDuty_2(PWMB_C);
    } //em cima da linha
       
    
    else
    { 
        //pra frente - reta
        //--------------->AREA DO PID<---------------

        u = PID(erro);

        PWMA = PWMR + u;
        PWMB = PWMR - u;

        frente();
        PWM_limit();
        setDuty_1(PWMA);
        setDuty_2(PWMB);
    }
    
    volta_pra_pista();
    //sprintf(buffer, "%d\n", u);
    //UART_enviaString(buffer);
    //A função que fazia o robô rodar em seu próprio eixo foi removida

}

void PWM_limit() {
    //------> Limitando PWM
    static int ExcessoB = 0, ExcessoA = 0;
    
    if (PWMA > 380)
    {
      ExcessoB = (PWMA - 380);
      PWMA = 380;
      PWMB -= ExcessoB;
    }

    else if (PWMB > 380)
    {
      ExcessoA = (PWMB - 380);
      PWMB = 380;
      PWMA -= ExcessoA;
    }

    if (PWMA < 0)
    {
      ExcessoB = (PWMA*(-1) * 2);
      PWMA += (PWMA*(-1)*2);
      PWMB += ExcessoB;
    }

    else if (PWMB < 0)
    {
      ExcessoA = (PWMB*(-1) * 2);
      PWMB += (PWMB*(-1)*2);
      PWMA += ExcessoA;
    }    
        
    if (PWMA_C > 320)
    {
      ExcessoB = (PWMA_C - 320);
      PWMA_C = 320;
      PWMB_C -= ExcessoB;
    }

    else if (PWMB_C > 320)
    {
      ExcessoA = (PWMB_C - 320);
      PWMB_C = 320;
      PWMA_C -= ExcessoA;
    }

    if (PWMA_C < 0)
    {
      ExcessoB = (PWMA_C*(-1)* 2);
      PWMA_C += (PWMA_C*(-1)*2);
      PWMB_C += ExcessoB;
    }

    else if (PWMB_C < 0)
    {
      ExcessoA = (PWMB_C*(-1) * 2);
      PWMB_C += (PWMB_C*(-1)*2);
      PWMA_C += ExcessoA;
    }

}

void volta_pra_pista(void)
{
     /*Área de voltar para a pista*/
    if ((sensores_frontais[0] < 101) && (sensores_frontais[4] > 190))
    {
      if (sensores_frontais[2] > 107)
      {
          
        giro_direita();
        setDuty_1(PWMA_C);
        setDuty_2(PWMB_C);

      }
    }
    else if ((sensores_frontais[4]< 101) && (sensores_frontais[0] > 190))
    {
      if (sensores_frontais[2] > 107)
      {
        
        giro_esquerda();
        setDuty_1(PWMA_C);
        setDuty_2(PWMB_C);
      }  
    }
    
    else if ((sensores_frontais[4] > 190) && (sensores_frontais[0] > 190) && sensores_frontais[2] > 190)
    {
        giro_direita();
        setDuty_1(PWMA_C);
        setDuty_2(PWMB_C); 
    }
    /*Fim de área para voltar para a pista*/
    //Obs.: Os valores mudam de acordo com o N° de sens. e suas posições
    //bem como a calibração dos mesmos.
}

void volta_pra_pista_calibracao(void)
{    
    if ((sensores_frontais[4] < 200) && (sensores_frontais[0] < 200))
    {
            while (sensores_frontais[1] < 101 && sensores_frontais[2] < 120)
            {
        
                giro_direita();
                setDuty_1(PWM_Curva);
                setDuty_2(PWM_Curva);

            }  
    }
    
    else if ((sensores_frontais[4] < 200) && (sensores_frontais[0] < 200))
    {
            while (sensores_frontais[3] < 100 && sensores_frontais[2] < 99)
            {
        
                giro_esquerda();
                setDuty_1(PWM_Curva);
                setDuty_2(PWM_Curva);

            }  
    }
}

void calculo_do_erro()
{
    int soma_esquerdo = 0, soma_direito = 0;
    static int denominador = 6;
    int soma_total = 0;   //caso aumente o peso da média_ponderada, tomar cuidado com a variável char
    
    static int peso [] = {-2, -1, 0, 1, 2};
    //os pesos precisarão ser corrigidos pois os sensores do Van Grogue estão um pouco assimétricos
    
    for (int j = 0; j < 2; j++) 
    {
        soma_esquerdo += (sensores_frontais[j] * peso[j]);
        soma_direito  += (sensores_frontais[4-j] * peso[4-j]);
    }

    soma_total = (soma_esquerdo + soma_direito)/ denominador;
    
    erro = 14 - soma_total;   //valor esperado(estar sempre em cima da linha) - valor medido

    
    if(erro > 33)   //corrigindo assimetria
    {
        erro = 33;
    }
    
    if(erro < -33)
    {
        erro = -33;
    }
    /*for(int i = 0; i < 5; i++)
    {
        sprintf(buffer, "%d\t", sensores_frontais[i]);
        UART_enviaString(buffer);
    }
    UART_enviaCaractere('\n');*/
    
    //sprintf(buffer, "%d\n", erro);
    //UART_enviaString(buffer);
}

void estrategia()
{
    
    if (!f_parada)  //se f_parada for 0... 
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
        //f_parada = 1;
        //freio();
        parada = 0;
    }
    
}

void f_timers (void) {

    static unsigned char c_timer1 = 0;
    static unsigned char c_timer2 = 0;
    static unsigned char c_timer3_ms = 0, c_timer3 = 0;
    if(f_calibra)
    {
        
        //funções a cada 100us
        if(c_timer1 < 1-1)
        {
            c_timer1++;
        }
        
        else
        {
            parada();
            fim_de_pista();         //Verifica se é o fim da pista
            c_timer1 = 0;
        }
        
        if (c_timer2 < 4-1)   //o 0 conta na contagem -> 4-1
        {
            c_timer2++; //100us -1; 200us-2;300 us-3; 400us de intervalo de tempo
        }
        
        else    //a cada 400us
        {
            estrategia();
            c_timer2=0;
        }
        
    }
    
    /*else
    {
        if(c_timer3_ms < 100 - 1)   //10ms
        {
            c_timer3_ms++;
        }
        
        else
        {
            if(c_timer3 < 30 - 1)   //300ms
            {
                c_timer3++;
            }
            
            else
            {
                if(!f_motor)
                {
                    Auto_calibration();
                    c_timer3 = 0;
                }
            }
            
            c_timer3_ms = 0;
        
        }
    }*/
}//fim do programa

/*Observações:
  Foram utilizados somente 5 sensores pois o módulo está assimétrico em relação ao robô, e
 *a forma mais simples de corrigir isso é usando os 5 sensores,
  mesmo não estando 100% simétrico.
  Por causa disso foi necessário alterar o setpoint do erro para termos um erro = 0
  quando o robô estivesse acima da linha.
  Foi visto também pela serial o limite superior e inferior dos sensores e foram
  setados em valor_max_abs e valor_min_abs pois a calibração não estava funcionando
  Além disso foi feito uma leitura serial dos sensores em uma curva,
  virando tanto pra esquerda quanto para a direita para saber os valores AD
  dos sensores extremos em cada situação para se iniciar uma curva*/
