//------------------------------------------------------------------------------------
//Código de base para qualquer outro robô, aqui estão todas as funções e ideias usadas
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

#include <QTRSensors.h>

#define SaidaA 10 //terminal PWM do motor direito
#define SaidaB 12 //terminal PWM do motor esquerdo
  
//Lado direito
#define AIN1 6  //terminais como exemplo
#define AIN2 4

//Lado Esquerdo
#define BIN1 3
#define BIN2 9

int sensores_frontais[] = {A0, A1, A2, A3, A4, A5, A6, A7}; //Portas dos sensores frontais
int Leitura_dos_sensores[8]; // leitura dos sensores frontais

double Kp = 0, Kd = 0, Ki = 0; // Variáveis que são modificadas no PID
int PWM = 0, PWMR = 0;         // valor da força do motor em linha reta
int LeituraBranco = 0;         // Numero de vezes que o sensor de borda lera branco

double erro, p, d, erroAnterior = 0, i, integral = 0, Turn = 0; //Área PID
double MotorA, MotorB; // Tensao enviada pelo PID/ PWM ajustado pelo PID do motor direito e esquerdo

int position_line = 0; // Método com o Read Line;

int ExcessoA, ExcessoB;

int contador = 0, acionador = 0; // Borda

QTRSensorsRC borda ((unsigned char[]) {8}, 1);                         //Sensor de borda
QTRSensorsAnalog qtra  ((unsigned char[]) {A0, A1, A2, A3, A4, A5, A6, A7}, 8); //Sensores frontais

unsigned int sensor_values[6];
unsigned int BordaValor[1];

void Frente();
void Tras();
void GiroEsquerda();
void GiroDireita();
void Freio();
void Sem_biblioteca();

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
  pinMode(8,       INPUT);
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
  
  Sem_biblioteca(); //leitura dos sensores
  borda.readCalibrated(BordaValor);
  digitalWrite(13, LOW);

  position_line = qtra.readLine(sensor_values,QTR_EMITTERS_ON, 1);
  erro = position_line - 2500;
  
  //--------------->AREA DO SENSOR DE PARADA<---------------

 if ((BordaValor[0] < 300) && (acionador == 0))
    {
      contador++;
     acionador = 1;
    }

  if ((BordaValor[0] > 500) && (acionador == 1))
    {
      acionador = 0;
    }

  while (contador >= LeituraBranco) 
    {
      Freio();
      Sem_biblioteca();
    }

  //--------------->AREA PARA VOLTAR À PISTA<---------------

   if ((sensor_values[0] < 100) && (sensor_values[5] > 900))
   {
      digitalWrite(13, HIGH);
      while (sensor_values[2] >= 300)
        {
          GiroEsquerda();
          Sem_biblioteca();
        
          analogWrite(SaidaA, PWMR);
          analogWrite(SaidaB, PWMR);
      
          qtra.readCalibrated(sensor_values);
       }
   }
  else if ((sensor_values[5]< 100) && (sensor_values[0] > 900))
  {
    digitalWrite(13, HIGH);
    while (sensor_values[3] >= 300)
    {
        GiroDireita();
        Sem_biblioteca();

        analogWrite(SaidaA, PWMR);
        analogWrite(SaidaB, PWMR);
      
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

  //--------------->AREA DO LIMITE DO PWM<--------------- 

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

  //--------------->AREA DO SENTIDO DAS RODAS<--------------- 

  Frente();
  Sem_biblioteca();

  if (MotorA < 0) // Giro para a direita
    {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    }
  if (MotorB < 0) // Giro para a esquerda
    {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    }
    
  analogWrite(SaidaA, MotorA);
  analogWrite(SaidaB, MotorB); 
  
}

void Frente()
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
void Tras()
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);    
  }
void GiroEsquerda()
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);    
  }
void GiroDireita()
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);   
  }
void Freio()
  {
    Frente();

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


void Sem_biblioteca(){
  for(int i = 0; i < 8; i++){
    Leitura_dos_sensores[i] = analogRead(sensores_frontais[i]);
    Serial.print("Leitura do sensor");
    Serial.print((i+1));
    Serial.print(":");
    Serial.print(Leitura_dos_sensores[i]);
    Serial.print('\t'); //TAB literalmente
  }
  delay(2); //delay em teste
  Serial.println("");
}
