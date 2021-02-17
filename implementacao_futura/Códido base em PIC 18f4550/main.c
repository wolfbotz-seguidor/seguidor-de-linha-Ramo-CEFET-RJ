/*Pinos de conexão pickit3
MCLR
VDD
VSS
PGD - RB7
PGC - RB6*/



//-------Conexão PWM-----//

/*CCP1 - RC2
CCP2 - RC1*/


//-----Pinos de leitura analógica--//

//RA0-RA3, RA5, RE0-RE1; -->Sensores frontais


//---Pinos digitais de entrada de dados----//

//sensores laterias

//RB3 e RB4

//---Pinos de leitura UART/Serial---//

//RC6/TX e RC7/RX

//--Pinos digitais de saída de dados---//

//RD4 - RD7


//Observações:
/*RB0 - RB2 são pinos que possuem interrupção externa
RB7 e RB6 são pinos de compilação, logo se for utilizar, robô deve estar desligado
Verificar a necessidade de utilizar as interrupções externas*/





//------------------------------------------------------------------------------------
//Código de base para qualquer outro robô, com algumas implementações diferentes como o sensor traseiro único de um robô, aqui estão todas as funções e ideias usadas
//até o momento, sendo estas comentadas para utilização futura.
//------------------------------------------------------------------------------------

// PWM       
// Kp:     
// Ki:       
// Kd:   
// Tensão: 

// PWM           
// Kp:          
// Kd:         
// Ki:          
// Tensão:      

#include "configbit.txt" //configurações dos bits
#include "PWM.h"         //biblioteca que gerencia o PWM
#include <xc.h>
#include <stdio.h>
#include "serial.h"
#include <stdlib.h>
#include <string.h>
#define _XTAL_FREQ 20000000 //Frequencia em 20MHz


//o PWM DESSE CÓDIGO VAI DE 0 A 962


//#define SaidaA RC1          //pino PWM do motorA  --> não precisa ser decretado
//#define SaidaB RC2          //pino PWM do motorB

//-motores-//
//Lado direito
#define AIN1 RD4            //sentido horário
#define AIN2 RD5            //sentido anti-horário

//Lado esquerdo
#define BIN1 RD6            //sentido horário
#define BIN2 RD7            //sentido anti-horário

//sensores laterais
#define sensor_de_parada RB3
#define sensor_de_curva  RB4

#define led RD0         //led indicador de calibração

int LeituraBranco = 0;         // Numero de vezes que o sensor de parada lera branco
double Kp = 0, Kd = 0, Ki = 0; // Variáveis que são modificadas no PID
int PWM = 0, PWMR = 0;         // valor da força do motor em linha reta
double erro, p, d, erroAnterior = 0, i, integral = 0, Turn = 0; //Área PID
double MotorA, MotorB; // PWM que será alterado pelo PID


int ExcessoA, ExcessoB;     //uTILIZADO PARA TRATAR COM EXCESSO DE PWM(?)

int contador = 0, acionador = 0; // Sensor de parada

//Área de definição das portas analógicas

int conversao_AD1(void){
    int conversao;
    ADCON0 = 0b00000001;// Seleciona canal AN0
    ADCON0bits.GO=1; // inicia a conversão
    while (ADCON0bits.GO); // Aguarda a o fim da conversão
    conversao = (ADRESL + (ADRESH << 8));
    return conversao;   //retorna valor de leitura em 10 bits
    
}


int conversao_AD2(void){
    int conversao2;
    ADCON0 = 0b00000101;// Seleciona canal AN1
    ADCON0bits.GO=1; // inicia a conversão
    while (ADCON0bits.GO); // Aguarda a o fim da conversão
    conversao2 = ADRESL + (ADRESH << 8);
    return conversao2;   //retorna valor de leitura em 10 bits
}

int conversao_AD3(void){
    int conversao3;
    ADCON0 = 0b00001001;// Seleciona canal AN2
    ADCON0bits.GO=1; // inicia a conversão
    while (ADCON0bits.GO); // Aguarda a o fim da conversão
    conversao3 = ADRESL + (ADRESH << 8);
    return conversao3;      //retorna valor de leitura em 10 bits
    
}

int conversao_AD4(void){
    int conversao4;
    ADCON0 = 0b00001101;// Seleciona canal AN3
    ADCON0bits.GO=1; // inicia a conversão
    while (ADCON0bits.GO); // Aguarda a o fim da conversão
    conversao4 = ADRESL + (ADRESH << 8);
    return conversao4;      //retorna valor de leitura em 10 bits

}

int conversao_AD5(){
    int conversao5;
    ADCON0 = 0b00010001;// Seleciona canal AN4
    ADCON0bits.GO=1; // inicia a conversão
    while (ADCON0bits.GO); // Aguarda a o fim da conversão
    conversao5 = ADRESL + (ADRESH << 8);
    return conversao5;      //retorna valor de leitura em 10 bits

}

int conversao_AD6(void){
    int conversao6;
    ADCON0 = 0b00010101;// Seleciona canal AN5
    ADCON0bits.GO=1; // inicia a conversão
    while (ADCON0bits.GO); // Aguarda a o fim da conversão
    conversao6 = ADRESL + (ADRESH << 8);
    return conversao6;      //retorna valor de leitura em 10 bits

}

int conversao_AD7(void){
    int conversao6;
    ADCON0 = 0b00011001;// Seleciona canal AN6
    ADCON0bits.GO=1; // inicia a conversão
    while (ADCON0bits.GO); // Aguarda a o fim da conversão
    conversao6 = ADRESL + (ADRESH << 8);
    return conversao6;      //retorna valor de leitura em 10 bits

}

int conversao_AD8(void){
    int conversao6;
    ADCON0 = 0b00011101;// Seleciona canal AN7
    ADCON0bits.GO=1; // inicia a conversão
    while (ADCON0bits.GO); // Aguarda a o fim da conversão
    conversao6 = ADRESL + (ADRESH << 8);
    return conversao6;      //retorna valor de leitura em 10 bits

}

char uart_rd;           //Variável de leitura serial a ser implementada no código
int leitura1 = 0;       //leitura analógica
int leitura2 = 0;
int leitura3 = 0;
int leitura4 = 0;
int leitura5 = 0;
int leitura6 = 0;
int leitura7 = 0;
int leitura8 = 0;
char txt[7];
char txt2[7]; //variáveis que enviam valores de leitura
char txt3[7];

void Frente();
void Tras();
void Girohorario();
void GiroAntihorario();
void Freio();


void main(){
    //Sensores analógicos
    TRISAbits.RA0 = 0x01;       //Determina sensor frontal como entrada
    TRISAbits.RA1 = 0x01;       //Determina como entrada
    TRISAbits.RA2 = 0x01;       //Determina como entrada
    TRISAbits.RA3 = 0x01;       //Determina como entrada
    TRISAbits.RA5 = 0x01;       //Determina como entrada
    TRISEbits.RE0 = 0x01;       //Determina como entrada
    TRISEbits.RE1 = 0x01;       //Determina sensor traseiro como entrada
    TRISEbits.RE2 = 0x01;       //Determina como entrada
    TRISBbits.RB3 = 0x01;       //Determina como entrada
    TRISBbits.RB4 = 0x01;       //Determina como entrada
    
    //sensores digitais
    TRISBbits.RB3 = 0x01;       //determina sensor de parada como entrada
    TRISBbits.RB4 = 0x01;       //determina sensor de curva como entrada
    
    
    //Saídas PWM
    TRISCbits.RC2 = 0x00;       //PWMA como saída
    TRISCbits.RC1 = 0x00;
    
    TRISCbits.RC7 = 0x01;       //pino RX como entrada
    TRISCbits.RC6 = 0x00;       //pino TX como saída
    
    
    //Saídas para os motores/ponte-h
    TRISDbits.RD4 = 0x00;       //AIN1
    TRISDbits.RD5 = 0x00;       //AIN2
    TRISDbits.RD6 = 0x00;       //BIN1
    TRISDbits.RD7 = 0x00;       //BIN2
    
    
    PORTA = 0xFF;               //Inicializa todo o PORTA em 1
    PORTE = 0xFF;               //Inicializa todo o PORTE em 0
    PORTC = 0x80;
    PORTD = 0x00;
    PORTB = 0x18;               //RB3 e RB4 em nível lógico alto
    
    //--Região de configuração dos registradores do ADC--//
    ADCON1 = 0b00000111;        //habilita as portas AN0 até AN7 como portas analógicas
    ADCON2 = 0b10010101;        //Justificado à direita
                                //TAD de 4
                                //Conversores AD select bits de Fosc/16
    CMCON = 0x07;               //desabilita comparadores
    
    
    //-----leitura dos sensores frontais-------//
    leitura1 = conversao_AD1();
    leitura2 = conversao_AD2();
    leitura3 = conversao_AD3();
    leitura4 = conversao_AD4();
    leitura5 = conversao_AD5();
    leitura6 = conversao_AD6();
        
    int sensores_frontais [] = {leitura1, leitura2, leitura3, leitura4, leitura5, leitura6};  //sensores frontais declarados como vetores
        
    //------leitura dos sensores traseiros---//
        
    leitura7 = conversao_AD7();
    leitura8 = conversao_AD8();
        
    int sensores_traseiros [] = {leitura7, leitura8};       //sensores traseiros

    serial_init(); //inicializa o serial com 9600 de baud rate
    PWM_Init();        //configura pwm em 1,388KHz
    __delay_ms(100);
    
    serial_tx_str("Robo ligado");
    serial_tx(10);
    serial_tx(13);      //quebra de linha
    
    
    //---->Calibração do sensor de parada<---// Ainda a ser implementada
    
    for (int i = 0; i < 70; i++) 
    {
      //bordaEsq.calibrate();
      //bordaDir.calibrate();
      __delay_ms(5);
    }

    led = 0x01;
    __delay_ms(1000);
    led = 0x00;
    
    
    //---->Calibração dos sensores frontais<---// Ainda a ser implementada
    for(int i = 0; i <120; i++){
        //calibração
        __delay_ms(5);
    }
    
    led = 0x01;
    __delay_ms(1000);
    led = 0x00;
    __delay_ms(500);
    led = 0x01;
    __delay_ms(500);
    led= 0x00;
    
    while(1){
    
        //função de calibração atribuindo ao sensor de curva
        
        led = 0x01;
        __delay_ms(200);
        led = 0x00;
        
        //--Cáculo do erro dos sensores frontais através de média ponderada--//
        int peso [] = {-3, -2, -1, 1, 2 , 3};
        float soma_direito, soma_esquerdo, denominador_direito, denominador_esquerdo;
        float Erro_direito, Erro_esquerdo;
        
        for(int j = 0; j < 3; j++){
            denominador_direito += sensores_frontais[j];
            soma_direito += (sensores_frontais[j] * peso[j]);
        
            denominador_esquerdo += sensores_frontais[5-j];
            soma_esquerdo += (sensores_frontais[5-j] * peso[5-j]);
        }

        Erro_direito = soma_direito / denominador_direito;
        Erro_esquerdo = soma_esquerdo / denominador_esquerdo;
        erro = Erro_direito - Erro_esquerdo;
    
        //----Área que imprime no serial o valor do erro---//
        serial_tx_str("Erro:");
        sprintf(txt3, "%i", erro);
        serial_tx(32);       //gera espaço
        serial_tx_str(txt3);
        serial_tx(13); //quebra de linha

    
        
        //------------AREA DO SENSOR DE PARADA A SER IMPLEMENTADO----------//
        
        
        if ((sensor_de_parada = 0x00) && (acionador == 0x00))   //leu linha branca, marcador de largada ou chegada
        {
        contador++;
        acionador = 1;
        }

        if ((sensor_de_parada > 0x01) && (acionador == 1))
        {
        acionador = 0;
        }

        while (contador >= LeituraBranco) 
        {
        Freio();
        }
        
        
        //obs.: implementar condição de cruzamento para não incrementar valor no contador das marcações
        
        
        
        //-------------ÁREA PARA VOLTAR A PISTA SERÁ IMPLEMENTADO---------------//
        if ((sensores_frontais[0] < 100) && (sensores_frontais[5] > 900))
        {
            led = 0x01;
            while (sensores_frontais[2] >= 300)
            {
                GiroAntihorario();  //Giroantihorario();

                PWM1_Set_Duty(PWMR);
                PWM2_Set_Duty(PWMR);      
    //          qtra.readCalibrated(sensor_values);     //função de calibração ainda a ser implementada
           }
        }
        else if ((sensores_frontais[5]< 100) && (sensores_frontais[0] > 900))
        {
            led = 0x01;
            while (sensores_frontais[3] >= 300)
            {
                Girohorario();    //Girohorario();
                
                PWM1_Set_Duty(PWMR);
                PWM2_Set_Duty(PWMR);
//              qtra.readCalibrated(sensor_values);       //função de calibração ainda a ser implementada
            }  
        }
        
        
        
        
        
        //--------AREA DO PID------//
        p = erro * Kp; // Proporcao
  
        integral += erro; // Integral
        i = Ki * integral;
 
        d = Kd * (erro - erroAnterior); // Derivada
        erroAnterior = erro;
  
        Turn = p + i + d;
  
        MotorA = PWM - Turn;   //lado direito
        MotorB = PWM + Turn;   //lado esquerdo
        
        
        
        
        //-----------ÁREA DO LIMITE DO PWM(?)-----------//
        //função a ser estudada para ver a real necessidade dela
        
        if (MotorA > 260)
        {
          ExcessoB = (abs(MotorA) - 250);
          MotorA = 250;
          MotorB -= ExcessoB;
        }

        else if (MotorB > 260)
        {
          ExcessoA = (abs(MotorB) - 250);
          MotorB = 250;
          MotorA -= ExcessoA;
        }

        if (MotorA < -260)
        {
          ExcessoB = (abs(MotorA) - 250);
          MotorA = 250;
          MotorB += ExcessoB;
        }

        else if (MotorB < -260)
        {
          ExcessoA = (abs(MotorB) - 250);
          MotorB = 250;
          MotorA += ExcessoA;
        }
        
        
        //------->AREA DO SENTIDO DAS RODAS<-----// lógica a ser revisada
        
        Frente();
        
        if (MotorA < 0) // Giro para a direita
        {
            AIN1 = 0x00;  //Sentido horário estado baixo
            AIN2 = 0x01;  //Sentido anti-horário estado alto
            
        }
        if (MotorB < 0) // Giro para a esquerda
        {
            BIN1 = 0x00;  //Sentido horário estado baixo
            BIN2 = 0x01;  //Sentido anti-horário estado alto
            
            
        }

        PWM1_Set_Duty(MotorA);
        PWM2_Set_Duty(MotorB); 

        
        
    }

}




void Frente(){

    AIN1 = 0x01;
    AIN2 = 0x00;
    BIN1 = 0x01;
    BIN2 = 0x00;

}

void Tras(){        //função fora de uso no momento

    AIN1 = 0x00;
    AIN2 = 0x01;
    BIN1 = 0x00;
    BIN2 = 0x01;


}


void Girohorario(){

    AIN1 = 0x00;
    AIN2 = 0x01;
    BIN1 = 0x01;
    BIN2 = 0x00;


}

void GiroAntihorario(){

    AIN1 = 0x01;
    AIN2 = 0x00;
    BIN1 = 0x00;
    BIN2 = 0x01;


}


void Freio(){       //Essa função deve ser reformulada

    Frente();
    
    PWM1_Set_Duty(187);     //20% de duty cycle
    PWM2_Set_Duty(187); 
    
    
    __delay_ms(500);
    
    AIN1 = 0x01;
    AIN2 = 0x01;
    BIN1 = 0x01;
    BIN2 = 0x01;
    
    PWM1_Set_Duty(0);     //0% de duty cycle
    PWM2_Set_Duty(0);
    
    __delay_ms(2000);
    
    AIN1 = 0x00;
    AIN2 = 0x00;
    BIN1 = 0x00;
    BIN2 = 0x00;
    
    __delay_ms(10000);      //10 segundos de delay

}


//obs.: estudar e implementar todas as funções que faltam
//e as que possuem comentário para revisão