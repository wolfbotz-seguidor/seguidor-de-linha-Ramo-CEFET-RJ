// ESTE CODIGO NAO ESTA ATUALIZADO, VEJA O DO OUTRO PEN DRIVE!!!!!

// PWM         50     -----       60     -----       70     -----       80    -----    90       -----     110  
// Kp:      0.043     -----     0.04     -----     0.043    -----     0.051    -----    0.065    -----     0.08  
// Ki:          0     -----        0     -----     0.00001  -----    -0.00002   -----    0.3      -----     0.3   
// Kd:       0.20     -----      0.3     -----      0.24    -----      0.21    -----    0.0001   -----    0.0001 
// Tensão:      -     -----     7.97     -----     7.67     -----     7.32    -----    7.6      -----     7.7   

// PWM            130      ----   150      ----   170      ----   190       ----   210       ----  230        ----   255
// Kp:            0.13     ----   0.14     ----   0.21     ----   0.18      ----   0.18      ----  0.2        ----   0.16
// Ki:            0.43     ----   0.35     ----   0.42     ----   0.51      ----   0.51      ----  0.52       ----   0.49
// Kd:            0.000015 ----   0.00002  ----   0.00003  ----   0.000024  ----   0.000026  ----  0.000027   ----   0.000025
// Tensão:        7.67     ----   7.79     ----   7.83     ----   7.61      ----   7.61      ----  7.59       ----   7.55

/*
 * int PWM, PWMstart, PWMconst, tempo;
PWM = PWMstart;

tempo = millis();


if (tempo < 1000) {
  if  (PWM < PWMconst){
    PWM += 10;
  }
}
else {
  PWM = PWMconst;
}

 */

// Incluindo a biblioteca que irá lidar com a leitura dos sensores
#include <QTRSensors.h>

/ Definindo as saídas dos motores
#define SaidaA 9
#define SaidaB 3

//Lado direito
#define AIN2 6 // Quando em HIGH, roda direita anda para frente
#define AIN1 5 

//Lado Esquerdo
#define BIN1 4 // Quando em HIGH, roda esquerda anda para frente
#define BIN2 2

double Kp = 0.18 , Kd = 0.50, Ki = 0,kpc = 0.08, kdc = 0.02; // Variáveis que são modificadas no PID
int PWM = 120, PWMR = 30; // valor da força do motor em linha reta

double erro, p, d, erroAnterior = 0, i, integral = 0, Turn = 0, correction,lastcorrection,C,Cpd; //Área PID
double MotorA, MotorB,ExcessoB,ExcessoA; // Tensao enviada pelo PID

int position_line = 0;// Método com o Read Line;

float timing;
int contador = 0, acionador = 0; // Borda

QTRSensorsRC borda ((unsigned char[]) {8}, 1);                         //Sensor de borda
QTRSensorsRC qtra  ((unsigned char[]) {A2, A1, A0, A3, A4, A5}, 6); //Sensores frontais

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
  pinMode(8,       INPUT);
  pinMode(AIN1,   OUTPUT);
  pinMode(AIN2,   OUTPUT);
  pinMode(BIN1,   OUTPUT);
  pinMode(BIN2,   OUTPUT);
  pinMode(SaidaA, OUTPUT);
  pinMode(SaidaB, OUTPUT);

  //----> Calibração do sensor de borda <----\\

  // Realiza a calibração enquanto a condição dentro do for
  // ser verdadeira, o delay ajuda a realizar uma leitura
  // mais precisa
  for (int i = 0; i < 100; i++)
    {
      borda.calibrate(); 
      delay(5);
    }

  // Ligando o LED do arduino para avisar que a calibração
  // terminou
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  
  //----> Calibração dos Sensores frontais <----\\

  // Mesmo processo que acima, porém com os sensores
  // da borda
  for (int i = 0; i < 120; i++)
    {
      qtra.calibrate();
      delay(5);
    }


  // Ligando de novo para avisar que a calibração acabou
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(500);
  digitalWrite(13, HIGH);
  delay(500);

  // Atribuindo a uma variável a função de cronometrar, usada
  // para criar a lógica de parada do seguidor após um dado
  // tempo dentro do código
  timing = millis();
 
}


void loop() {

  // Desligando o LED
  digitalWrite(13, LOW);

  // Realizando a leitura do sensor de borda e atribuindo à variável BordaValor
  borda.readCalibrated(BordaValor);

  // Realizando a leitura com média ponderada dos sensores da frente e atribuindo
  // á variávei position_line 
  position_line = qtra.readLine(sensor_values,QTR_EMITTERS_ON, 1);

  // Com esse cálculo verificamos o quanto o seguidor está centralizado em relação
  // à linha, pode-se verificar que esse valor vai de -2500 à 2500, sendo esses
  // valores representando o máx de erro para esquerda e para direita e, claro,
  // se position_line for 2500 significa que o seguidor está no meio da linha
  // e o erro é nulo
  erro = position_line - 2500;

  // Iniciando o cálculo do PID

  // Realizando o cálculo da variável P
  p = erro * Kp; 

  // Somando os erros na variável integral e utilizando para o I
  integral += erro; 
  i = Ki * integral;

  // Realizando a diferença entre o erro anterior e o novo para o D
  d = Kd * (erro - erroAnterior); // Derivada

  // Sobrescrevendo o valor da variável erroAnterior com o valor
  // da variável erro
  erroAnterior = erro;

  // Somando os valores obtidos e atribuindo à variável Turn
  Turn = p + i + d;

  // Atribuindo os valores para os motores ajustados pela variável
  // Turn, é necessário que os sinais sejam trocados pois caso contrário
  // ambos os motores sempre receberiam o mesmo valor independente do Turn
  // e sempre andariam em linha reta, assim sendo impossível passar pelos 
  // percursos da pista
  MotorA = PWM - Turn;
  MotorB = PWM + Turn;

  // Escrevendo nas saídas dos sentidos dos motores para que ele ande
  // para a frente
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, HIGH);
  digitalWrite(BIN1, LOW);

  // Caso o motorA ou B estejam com valor negativo significa que 
  // ele precisaria andar para trás, para tal nós necessitamos
  // apenas inverter o sentido escrito acima.
  if (MotorA < 0) // Giro para a direita
    {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    }
  if (MotorB < 0) // Giro para a esquerda
    {
      digitalWrite(BIN2, LOW);
      digitalWrite(BIN1, HIGH);
    }

  // Passando os valores obtidos às saídas dos motores
  analogWrite(SaidaA, MotorA);
  analogWrite(SaidaB, MotorB);
  

   //--------------->AREA PARA VOLTAR À PISTA<---------------

// Caso aconteça do seguidor estar muito perto de sair da pista
// nós realizamos testes práticos para saber quais valores os
// sensores estarão lendo e então realizamos ações específicas
// para esses momentos a fim de fazê-lo voltar ao percurso
//  Obs.: temos dois casos pois o seguidor pode sair da pista
// tanto pela esquerda quanto pela direita
if ((sensor_values[0] < 50) && (sensor_values[5] > 950))
    {
      // LED acende para sinalizar que está realizando uma ação especial
      digitalWrite(13, HIGH);

      // Enquanto um dos sensores do meio não ler a linha ele continuará 
      // realizando uma ação de giro
      while (sensor_values[4] >= 300)
      {
        // Sentidos colocados de tal forma que ele gire para um sentido
        digitalWrite(AIN1,HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN2, LOW);
        digitalWrite(BIN1, HIGH);

        // Recriando a ideia do PID mas utilizando dentro da lógica de voltar
        // à pista
        correction = PWMR + ((300 - sensor_values[3])*kpc);
        C = (correction - lastcorrection)*kdc;
        Cpd = correction + C;
        lastcorrection = correction;
        
        analogWrite(SaidaA, Cpd);
        analogWrite(SaidaB, Cpd);
  
        qtra.readCalibrated(sensor_values);
      }
    }

 // As mesmas coisas ditas acima valerão para os valores abaixo
 else if ((sensor_values[5]< 50) && (sensor_values[0] > 950))
  {
    digitalWrite(13, HIGH);
      while (sensor_values[2] >= 300)
      {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN2, HIGH);
        digitalWrite(BIN1, LOW);
  
        correction = PWMR + ((300 - sensor_values[3])*kpc);
        C = (correction - lastcorrection)*kdc;
        Cpd = correction + C;
        lastcorrection = correction;
        
  
        analogWrite(SaidaA, Cpd);
        analogWrite(SaidaB, Cpd);
        
        qtra.readCalibrated(sensor_values);
      }  
  }

  //--------------->AREA DO PID<---------------\\ 

//------> Limitando PWM

//  if (MotorA > 260)
//  {
//    ExcessoB = (abs(MotorA) - 250);
//    MotorA = 250;
//    MotorB -= ExcessoB;
//  }
//
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

  //--------------->AREA DO SENTIDO DAS RODAS<---------------\\ 

  // Variável necessária para saber se o seguidor deve parar ou não
  // Caso o valor da borda seja menor que 500 ele estará sobre uma
  // linha braca, o que sinaliza que seu contador deve aumentar em 1
  // para que quando chegue no valor necessário ele realize a função
  // de parar de correr.
  // A variável acionador está sendo usada apenas para evitar que o seguidor
  // aumente o contador enquanto seu sensor de borda ainda está sobre
  // a mesma linha branca, visto que o tempo ao qual o código é executado
  // é rápido o suficiente para que esse valor seja incrementado diversas
  // vezes antes do mesmo de fato sair da faixa.
  if ((BordaValor[0] < 500) && (acionador == 0))
    {
      contador++;
      acionador = 1;
    }

  // Quando o borda voltar a ler mais que 800 sinalizando que não está
  // sobre a linha branca então o acionador poderá voltar a ser 0.
  if ((BordaValor[0] > 800) && (acionador == 1))
    {
      acionador = 0;
    }

  // Função criada utilizando a ideia de tempo, caso o tempo seja maior que o
  // estabelecido então a lógica de parada é ativada.
  while (contador >= 10000) 
    {
      // Indo para frente
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN2, HIGH);
      digitalWrite(BIN1, LOW);

      // Dando um valor baixo para quase
      // parar
      analogWrite(SaidaA, 50);
      analogWrite(SaidaB, 50);

      // Mantendo a configuração por 350 ms
      delay(350);

      // Parando de forma bruta todas as rodas
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN2, HIGH);
      digitalWrite(BIN1, HIGH);

      // Parando os motores
      analogWrite(SaidaA, LOW);
      analogWrite(SaidaB, LOW);

      // Mantendo a configuração por 2 segundos
      delay (2000);

      // Parando os motores
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN2, LOW);
      digitalWrite(BIN1, LOW);

      // Dando um delay grande o suficiente para ter tempo
      // do seguidor ser retirado da pista sem a lógica voltar
      // ao normal
      delay(60000);
    }

}
