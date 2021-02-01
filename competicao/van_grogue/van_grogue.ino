// CODIGO ATUALIZADO DO VAN GROGUE!!!!!

// PWM         50     -----       60     -----       70     -----       80    -----    90       -----     100  
// Kp:      0.043     -----     0.04     -----     0.043    -----     0.04    -----    0.065    -----     0.075  
// Ki:          0     -----        0     -----     0.00001  -----             -----    0.3      -----     0.   
// Kd:       0.20     -----      0.3     -----      0.24    -----      0.2    -----    0.0001   -----     0.5 
// Tensão:      -     -----     7.97     -----     7.67     -----     7.32    -----    7.6      -----     7.7   

// PWM            130      ----   150      ----   170      ----   190       ----   200       ----  240        ----   255
// Kp:            0.13     ----   0.14     ----   0.21     ----   0.055      ----   0.037      ----  0.036    ----   0.038
// Kd:            0.43     ----   0.35     ----   0.42     ----   0.1      ----   0.08      ----  0.082       ----   0.082
// Ki:            0.000015 ----   0.00002  ----   0.00003  ----   0  ----   0  ----  0.000027   ----   0      ----     0
// Tensão:        7.67     ----   7.79     ----   7.83     ----   7.61      ----   8.09      ----  8.21       ----   8.21


#include <QTRSensors.h>

#define SaidaA 9
#define SaidaB 10
  
//Lado direito
#define AIN2 6 // Quando em HIGH, roda direita anda para frente
#define AIN1 5 

//Lado Esquerdo
#define BIN1 4 // Quando em HIGH, roda esquerda anda para frente
#define BIN2 3

double Kp = 0.07, Kd = 0.5, Ki = 0, kpc = 0.02 , kdc = 0.2; // Variáveis que são modificadas no PID
int PWM = 80, PWMR = 40; // valor da força do motor em linha reta
int correction,lastcorrection,C,Cpd;

double erro, p, d, erroAnterior = 0, i, integral = 0, Turn = 0; //Área PID
double MotorA, MotorB; // Tensao enviada pelo PID
\
int position_line = 0; // Método com o Read Line;

int ExcessoA, ExcessoB;

int contador = 0, acionador = 0; // Borda

QTRSensorsRC borda ((unsigned char[]) {A5}, 1);                         //Sensor de borda
QTRSensorsAnalog qtra  ((unsigned char[]) {A3, A2, A1, A0, A7, A6}, 6); //Sensores 

unsigned int sensor_values[6];
unsigned int BordaValor[1];

void setup() {
  Serial.begin(9600); // Comunicacao com o Serial
  
  pinMode(A0,      INPUT);
  pinMode(A1,      INPUT);
  pinMode(A2,      INPUT);
  pinMode(A3,      INPUT);
  pinMode(A4,      INPUT);
  pinMode(A5,      INPUT);
  pinMode(A6,      INPUT);
  pinMode(A7,      INPUT);
  pinMode(AIN1,   OUTPUT);
  pinMode(AIN2,   OUTPUT);
  pinMode(BIN1,   OUTPUT);
  pinMode(BIN2,   OUTPUT);
  pinMode(MotorA, OUTPUT);
  pinMode(MotorB, OUTPUT);

  //----> Calibração do sensor de borda <----\\

  for (int i = 0; i < 120; i++)
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
  
  borda.readCalibrated(BordaValor);
  qtra.readCalibrated(sensor_values);
  digitalWrite(13, LOW);

  position_line = qtra.readLine(sensor_values,QTR_EMITTERS_ON, 1);
  erro = position_line - 2500;
  
//--------------->AREA DO SENSOR DE0 PARADA<---------------

  if ((BordaValor[0] < 300) && (acionador == 0))
    {
      contador++;
      acionador = 1;
    }

  if ((BordaValor[0] > 500) && (acionador == 1))
    {
      acionador = 0;
    }

  while (contador >= 12) 
    {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN2, HIGH);
      digitalWrite(BIN1, LOW);

      analogWrite(SaidaA, 50);
      analogWrite(SaidaB, 50);
      
      delay(500);
      
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN2, HIGH);
      digitalWrite(BIN1, HIGH);

      analogWrite(SaidaA, LOW);
      analogWrite(SaidaB, LOW);
      
      delay (2000);

      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN2, LOW);
      digitalWrite(BIN1, LOW);

      delay(60000);
    }

  //-----> Área para voltar à pista

   if ((sensor_values[0] < 50) && (sensor_values[5] > 950))
    {
      digitalWrite(13, HIGH);
      while (sensor_values[4] >= 300)
      {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN2, LOW);
      digitalWrite(BIN1, HIGH);

      correction = PWMR + ((300 - sensor_values[3])*kpc);
      C = (correction - lastcorrection)*kdc;
      Cpd = correction + C;
      lastcorrection = correction;
      

      analogWrite(SaidaA, Cpd);
      analogWrite(SaidaB, Cpd);

      qtra.readCalibrated(sensor_values);
      }
    }
 else if ((sensor_values[5]< 50) && (sensor_values[0] > 950))
  {
    digitalWrite(13, HIGH);
      while (sensor_values[2] >= 300)
      {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN2, HIGH);
      digitalWrite(BIN1, LOW);

      correction = PWMR + ((300 - sensor_values[3])*0.02);
      C = (correction - lastcorrection)*0.2;
      Cpd = correction + C;
      lastcorrection = correction;
      

      analogWrite(SaidaA, Cpd);
      analogWrite(SaidaB, Cpd);
      
      qtra.readCalibrated(sensor_values);
      }  
  }
  
  //--------------->AREA DO PID<---------------

  p = erro * Kp; // Proporcao
  
  integral += erro; // Integral
  i = Ki * integral;
 
  d = Kd * (erro - erroAnterior); // Derivada
  erroAnterior = erro;
  
  Turn = p + i + d;
  
  MotorA = PWM - Turn;
  MotorB = PWM + Turn;

  //------> Limitando PWM

//  if (MotorA > 260)
//  {
//    ExcessoB = (abs(MotorA) - 250);
//    MotorA = 250;
//    MotorB -= ExcessoB;
//  }

//  else if (MotorB > 260)
//  {
//    ExcessoA = (abs(MotorB) - 250);
//    MotorB = 250;
//    MotorA -= ExcessoA;
//  }
//
//  if (MotorA < -260)
//  {
//    ExcessoB = (abs(MotorA) - 250);
//    MotorA = 250;
//    MotorB += ExcessoB;
//  }
//
//  else if (MotorB < -260)
//  {
//    ExcessoA = (abs(MotorB) - 250);
//    MotorB = 250;
//    MotorA += ExcessoA;
//  }

  //--------------->AREA DO SENTIDO DAS RODAS<--------------- 

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN2, HIGH);
  digitalWrite(BIN1, LOW);

  if (MotorA < 0) // Giro para a direita
    {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }
  if (MotorB < 0) // Giro para a esquerda
    {
      digitalWrite(BIN2, LOW);
      digitalWrite(BIN1, HIGH);
    }
    
  analogWrite(SaidaA, MotorA);
  analogWrite(SaidaB, MotorB); 
  
}
