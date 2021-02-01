// Este código tem como objetivo ser a continuação do ReadLine manual
// utilizando a ideia da linha ser branca e o fundo ser preto.                    
// 

// PWM         50     -----       60     -----       70     -----       80    -----    90       -----     110   -----   
// Kp:      0.043     -----     0.04     -----     0.043    -----     0.19    -----    0.065    -----           -----
// Ki:          0     -----        0     -----     0.00001  -----             -----    0.3      -----           -----
// Kd:       0.20     -----      0.3     -----      0.24    -----      0.9    -----    0.0001   -----           -----
// Tensão:      -     -----     7.97     -----     7.67     -----     7.32    -----    7.6      -----           -----

#include <QTRSensors.h>
#include <SoftwareSerial.h> //INCLUSÃO DE BIBLIOTECA
#include <Mouse.h>

const int pinoRX = 0; 
const int pinoTX = 1;

SoftwareSerial bluetooth(pinoRX, pinoTX);  

void ModoManual_Frente();
void ModoManual_direita();
void Manual_Esquerda(); 
void ModoManual_Tras();

#define SaidaA 9
#define SaidaB 3

//Lado direito
#define AIN2 6 // Quando em HIGH, roda direita anda para frente
#define AIN1 5 

//Lado Esquerdo
#define BIN1 4 // Quando em HIGH, roda esquerda anda para frente
#define BIN2 2

double Kp = 0, Kd = 0, Ki =0; // Variáveis que são modificadas no PID
int PWM = 0; // valor da força do motor em linha reta

double erro, p, d, erroAnterior = 0, i, integral = 0, Turn = 0; //Área PID
double MotorA, MotorB; // Tensao enviada pelo PID

int position_line = 0; // Método com o Read Line;

int contador = 0, acionador = 0; // Borda

int Saida = 0;
int tamanhoComando;
char recebeCaracter, tipo;
String comando, cte;

//QTRSensorsRC borda ((unsigned char[]) {A3}, 1); //Sensor de borda
//QTRSensorsAnalog qtra  ((unsigned char[]) {A2, A1, A0, A7, A6, A4}, 6); //Sensores frontais

unsigned int sensor_values[6];
unsigned int BordaValor[1];

void setup() {
  Serial.begin(9600); // Comunicacao com o Serial
  bluetooth.begin(9600); //Inicializa a Serial do Bluetooth
  delay(100);
  
  pinMode(A0,      INPUT);
  pinMode(A1,      INPUT);
  pinMode(A2,      INPUT);
  pinMode(A3,      INPUT);
  pinMode(A4,      INPUT);
  pinMode(A6,      INPUT);
  pinMode(A7,      INPUT);
  pinMode(AIN1,   OUTPUT);
  pinMode(AIN2,   OUTPUT);
  pinMode(BIN1,   OUTPUT);
  pinMode(BIN2,   OUTPUT);
  pinMode(MotorA, OUTPUT);
  pinMode(MotorB, OUTPUT);

//---->Pegando os Valores das constantes<----\\

while (Saida != 1) {
  
  while(bluetooth.available())
  {
    delay(100);

    recebeCaracter = bluetooth.read();
    
    comando += recebeCaracter;

     if (recebeCaracter == ';')
       {        
        tamanhoComando = comando.length();
        
        cte = comando.substring(1, tamanhoComando - 1);

        if (comando.substring(0,1) == "S") // Seguir
          {
            Saida = 1;
          }
        else if (comando.substring(0,1) == "P")
          {
           Kp = cte.toDouble();
          }
        else if (comando.substring(0,1) == "I")
          {
           Ki = cte.toDouble();
          }
        else if (comando.substring(0,1) == "D")
          {
           Kd = cte.toDouble();
          }
        else if (comando.substring(0,1) == "W")
          {
           PWM = cte.toDouble();
          }

        comando = "";
       }
  }  
}
  Serial.print("Kp: ");
  Serial.print(Kp);
  Serial.print("| Ki: ");
  Serial.print(Ki);
  Serial.print("| Kd: ");
  Serial.print(Kd);
  Serial.print("| PWM: ");
  Serial.print(PWM);

  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);

  //----> Calibração do sensor de borda <----\\

  for (int i = 0; i < 70; i++)
    {
      borda.calibrate(); 
      delay(5);
    }

  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  
  //----> Calibração dos Sensores frontais <----\\

  for (int i = 0; i < 120; i++)
    {
      qtra.calibrate();
      delay(5);
    }

  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(500);
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
 
}


void loop() {

  //----------------------> Lista de Comandos <----------------------\\
  //Parar    - R
  //Seguir   - S
  //Cima     - F
  //Esquerdo - E
  //Direito  - O
  //Baixo    - V
  
  while (bluetooth.available())
  {
    delay(100);

    recebeCaracter = bluetooth.read();
    
    Serial.print("recebeCaracter: ");
    Serial.println(recebeCaracter);
    
    comando += recebeCaracter;

     if (recebeCaracter == ';')
       {
        Serial.print(" Comando Recebido: ");
        Serial.println(comando);

        if (comando.substring(0,1) == "S") // Seguir
          {
            Saida = 1;
          }
        else if (comando.substring(0,1) == "R") // Parar
          {
           digitalWrite(SaidaA, 0);
           digitalWrite(SaidaB, 0);
           Saida = 0;
          }
        else if (comando.substring(0,1) == "F") // Frente
          {
           ModoManual_Frente();
          }
        else if (comando.substring(0,1) == "E") // Esquerda
          {
           Manual_Esquerda();
          }
        else if (comando.substring(0,1) == "O") // Direita
          {
           ModoManual_direita();
          }
        else if (comando.substring(0,1) == "V") // Baixo
          {
           ModoManual_Tras();
          }

        comando = "";
       }
  }
   
if (Saida == 1)
  {

//---->Área do Erro<----\\
  
  borda.readCalibrated(BordaValor);

  position_line = qtra.readLine(sensor_values,QTR_EMITTERS_ON, 1);
  erro = position_line - 2500;

  if ((BordaValor[0] < 550) && (acionador == 0))
    {
      contador++;
      acionador = 1;
    }

  if ((BordaValor[0] > 550) && (acionador == 1))
    {
      acionador = 0;
    }

  while (contador == 6) 
    {
      analogWrite(SaidaA, LOW);
      analogWrite(SaidaB, LOW);
    }

  //--------------->AREA DO PID<---------------\\ 

  p = erro * Kp; // Proporcao
  
  integral += erro; // Integral
  i = Ki * integral;
 
  d = Kd * (erro - erroAnterior); // Derivada
  erroAnterior = erro;
  
  Turn = p + i + d;
  
  MotorA = PWM - Turn;
  MotorB = PWM + Turn;

  //--------------->AREA DO SENTIDO DAS RODAS<---------------\\ 

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN2, LOW);
  digitalWrite(BIN1, HIGH);

  if (MotorA < 0) // Giro para a direita
    {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }
  if (MotorB < 0) // Giro para a esquerda
    {
      digitalWrite(BIN2, HIGH);
      digitalWrite(BIN1, LOW);
    }
    
  analogWrite(SaidaA, MotorA);
  analogWrite(SaidaB, MotorB); 
  Serial.println("ENTROU NO CODIGO NORMAL");

  }
}


void ModoManual_Frente()
{
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(SaidaA, 40);
  analogWrite(SaidaB, 40);
  
  Serial.println("FRENTE CHEGOU");
}

void ModoManual_Tras()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(SaidaA, 40);
  analogWrite(SaidaB, 40);
  
  Serial.println("BAIXO CHEGOU");
}


void Manual_Esquerda() 
{
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(SaidaA, 40);
  analogWrite(SaidaB, 40);
  
  Serial.println("ESQUERDA CHEGOU");
}

void ModoManual_direita()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(SaidaA, 40);
  analogWrite(SaidaB, 40);
  
  Serial.println("DIREITA CHEGOU");
}
