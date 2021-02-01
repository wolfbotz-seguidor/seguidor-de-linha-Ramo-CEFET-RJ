// Este código tem como objetivo ser a continuação do ReadLine manual
// utilizando a ideia da linha ser branca e o fundo ser preto.                    
// 

// PWM         50     -----       60     -----       70     -----       80    -----    100    -----           -----   
// Kp:      0.043     -----     0.04     -----     0.043    -----     0.19    -----           -----           -----
// Ki:          0     -----        0     -----     0.00001  -----             -----           -----           -----
// Kd:       0.20     -----      0.3     -----      0.24    -----      0.9    -----           -----           -----
// Tensão:      -     -----     7.97     -----     7.67     -----     7.32    -----           -----           -----

#include <QTRSensors.h>

#define SaidaA 9
#define SaidaB 3

//Lado direito
#define AIN2 6 // Quando em HIGH, roda direita anda para frente
#define AIN1 5 

//Lado Esquerdo
#define BIN1 4 // Quando em HIGH, roda esquerda anda para frente
#define BIN2 2

double Kp = 0.036, Kd = 0.25 , Ki = 0.00001; // Variáveis que são modificadas no PID
int PWM = 70; // valor da força do motor em linha reta

double erro, p, d, erroAnterior = 0, i, integral = 0, Turn = 0; //Área PID
double MotorA, MotorB; // Tensao enviada pelo PID

double leitura1 = 0, leitura2 = 0; // Método usado com o readCalibrated

double valorLine, numerador, denominador, somador; // Método usado para imitar o readLine();

int position_line = 0; // Método com o Read Line;

int contador = 0, acionador = 0; // Borda

QTRSensorsRC borda ((unsigned char[]) {A3}, 1); //Sensor de borda
QTRSensorsAnalog qtra  ((unsigned char[]) {A2, A1, A0, A7, A6, A4}, 6); //Sensores frontais

unsigned int sensor_values[6];
unsigned int BordaValor[1];

void setup() {
  Serial.begin(9600); // Comunicacao com o Serial
  
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

  //----> Calibração do sensor de borda <----\\

      
  Serial.println("Iniciando calibração do sensor de borda...");

  for (int i = 0; i < 5; i++)
    {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN2, HIGH);
      digitalWrite(BIN1, LOW);

      analogWrite(SaidaA, 51);
      analogWrite(SaidaB, 47);
      
      borda.calibrate();   
      delay(120);

      analogWrite(SaidaA, LOW);
      analogWrite(SaidaB, LOW);  
      delay(300);
      
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN2, LOW);
      digitalWrite(BIN1, HIGH);

      analogWrite(SaidaA, 51);
      analogWrite(SaidaB, 47);
      
      borda.calibrate();   
      delay(120);

      analogWrite(SaidaA, LOW);
      analogWrite(SaidaB, LOW);  
      delay(300); 
    }

  //----> Calibração dos Sensores frontais <----\\
    delay(500); 

      for (int i = 0; i < 2; i++)
    {
      digitalWrite(AIN1, HIGH); // Sentido do Giro para a direita
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN2, LOW);
      digitalWrite(BIN1, HIGH);

      analogWrite(SaidaA, 40);
      analogWrite(SaidaB, 40);
      
      qtra.calibrate();   
      delay(300);

      analogWrite(SaidaA, LOW);
      analogWrite(SaidaB, LOW);  
      delay(300);

      digitalWrite(AIN1, LOW); // Sentindo do Giro para a esquerda
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN2, HIGH);
      digitalWrite(BIN1, LOW);
      
      analogWrite(SaidaA, 40);
      analogWrite(SaidaB, 40);
      
      qtra.calibrate();   
      delay(300);

      analogWrite(SaidaA, LOW);
      analogWrite(SaidaB, LOW);  
      delay(300);

      digitalWrite(AIN1, LOW); // Sentindo do Giro para a esquerda
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN2, HIGH);
      digitalWrite(BIN1, LOW);
      
      analogWrite(SaidaA, 40);
      analogWrite(SaidaB, 40);
      
      qtra.calibrate();   
      delay(300);

      analogWrite(SaidaA, LOW);
      analogWrite(SaidaB, LOW);  
      delay(300);

      digitalWrite(AIN1, HIGH); // Sentido do Giro para a direita
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN2, LOW);
      digitalWrite(BIN1, HIGH);

      analogWrite(SaidaA, 40);
      analogWrite(SaidaB, 40);
      
      qtra.calibrate();   
      delay(300);

      analogWrite(SaidaA, LOW);
      analogWrite(SaidaB, LOW);  
      delay(300);
    }
 
//  for (int i = 0; i < 10; i++)
//    {
//      digitalWrite(AIN1, HIGH); // Sentido do Giro para a direita
//      digitalWrite(AIN2, LOW);
//      digitalWrite(BIN2, LOW);
//      digitalWrite(BIN1, HIGH);
//
//      analogWrite(SaidaA, 40);
//      analogWrite(SaidaB, 40);
//      
//      qtra.calibrate();   
//      delay(300);
//
//      analogWrite(SaidaA, LOW);
//      analogWrite(SaidaB, LOW);  
//      delay(300);
//
//      digitalWrite(AIN1, LOW); // Sentindo do Giro para a esquerda
//      digitalWrite(AIN2, HIGH);
//      digitalWrite(BIN2, HIGH);
//      digitalWrite(BIN1, LOW);
//      
//      analogWrite(SaidaA, 40);
//      analogWrite(SaidaB, 40);
//      
//      qtra.calibrate();   
//      delay(300);
//
//      analogWrite(SaidaA, LOW);
//      analogWrite(SaidaB, LOW);  
//      delay(300);
//    }

      
}

void loop() {

//  leitura1 = 0;
//  leitura2 = 0;
  
  //qtra.readCalibrated(sensor_values);
  borda.readCalibrated(BordaValor);

  //----> 

  position_line = qtra.readLine(sensor_values,QTR_EMITTERS_ON,1 );
  erro = position_line - 2500;

  //----> Variáveis usadas no método ReadLine <----\\
   
//  somador = 0;
//  numerador = 0;
//  denominador = 0;

  //----> Método Usando o ReadCalibrated() <----\\
  
//  for(int i= 0; i<3; i++){
//      leitura1 += sensor_values[i];
//      leitura2 += sensor_values[i+3];    
//    }
//  erro = leitura1 - leitura2;

  //----> Area do Sensor de Borda <----\\
/
  if ((BordaValor[0] < 550) && (acionador == 0))
    {
      contador++;
      acionador = 1;
    }

  if ((BordaValor[0] > 550) && (acionador == 1))
    {
      acionador = 0;
    }

  if (contador == 6) 
    {
      analogWrite(SaidaA, LOW);
      analogWrite(SaidaB, LOW);
    }
  else {

    //----> Método Usando o ReadLine <----\\

//  for (int i = 0; i < 6; i++)
//    {
//      numerador += somador*(1000 - sensor_values[i]);
//      denominador += sensor_values[i];
//      somador += 1000;
//    }
//
//  //----> Erro <----\\
//
//  valorLine = numerador/denominador;
//  erro = valorLine - 2500;

  //--------------->AREA DO PID<---------------\\ 

  p = erro * Kp; // `Proporcao
  
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
  }

}
