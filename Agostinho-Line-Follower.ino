#include <QTRSensors.h>
//Oscilando bastante durante a linha, porém não está saindo.
//https://github.com/Briza0407/Seguidor/blob/master/ChassiPlacaBom13.ino
unsigned int sensor_values[8];
//PWM        50       -----       60         -----     70       -----    80     -----     90          -----    100     -----      110*    -----     120   -------    130    -----    140    ----- 150 ------ 160 --------- 170           ----- 200      ---- 225           ----255          
//Kp:      0.04     -----        0.04       -----     0.18     -----    0.19   -----     0.19      -----      0.19    -----      0.12  -----       0.12 ------      0.120  -----     0.12   -----     ------     ---------               ----- 0.13     -----  0.17        ----
//Ki:        0        -----       0          -----              -----           -----                 -----            -----             -----           ------            -----             -----     ------     ---------              ----- 0.0004   ------ 0.0007      ----
//Kd:      0.20     -----        0.3        -----     0.4    -----      0.9   -----      1.10         -----   1.11     -----     0.8 -----        -0.80 ------       0.9  -----      1.1       -----     ------     ---------            ----- 2.2      ------   1          ----
//Tensão:    -        -----      7.97        -----    7.67      -----    7.32   -----    8.2        -----   7.9      -----      7.9    -----      7.9 -------       7.95   -----     7.5        -----     ------     ---------           ------7.73(2)  --------  7.61 (2)    ----



int leitura1 = 0, leitura2 = 0, aux = 0, aux2 = 1023, pwm = 255, pwmi = 120, outBack = 3150;
double offset, erro , p, integral = 0, kp = 0.127, ki = 0.0003, i, Turn, kd = 1.8, d, erroAnterior = 0 ; //kp = 0.32, ki = 0, kd = 2.2
double MotorA = 0, MotorB = 0; //Valor de PWM dos motores de 0 a 255
#define SaidaA 9
#define SaidaB 3

#define BIN2 2
#define BIN1 4
#define AIN2 6
#define AIN1 5

QTRSensorsAnalog qtra((unsigned char[]) {A5, A4, A6, A7, A0, A1, A2, A3}, 8);

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(SaidaA, OUTPUT);
  pinMode(SaidaB, OUTPUT);
  
  Serial.begin(9600);
  //Serial.println("Iniciando calibração");
int i;
  analogWrite(SaidaA, 0);
  analogWrite(SaidaB, 0);
  for (i = 0; i < 140; i++)  // make the calibration take about 2 seconds
  {
    qtra.calibrate();
    delay(5);

    qtra.read(sensor_values);
    if(aux < sensor_values[0]){
      aux = sensor_values[0];
    }
    if(aux2 > sensor_values[0]){
      aux2 = sensor_values[0];
    }
  }
  
  offset = (aux + aux2)/2;

  Serial.print("Maior leitura:");
  Serial.println(aux);
  Serial.println("Menor leitura:");
  Serial.println(aux2);
  Serial.println("Terminando calibração");
}

void loop() {
  leitura1 = 0;
  leitura2 = 0;
  qtra.read(sensor_values);
  for(int i= 0; i<4; i++){
    leitura1 += sensor_values[i];
    leitura2 += sensor_values[i+4];    
  }
  
  erro = leitura1 - leitura2;

  if(sensor_values[0] < 200){ // Robo muito fora da linha sensor_values[0] < 50
    while(sensor_values[3] > 580){
  digitalWrite(BIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(SaidaA, pwmi);
  analogWrite(SaidaB, pwmi);
  qtra.read(sensor_values);
  leitura1 = 0; leitura2 = 0;
      for(int i= 0; i<4; i++){
  leitura1 += sensor_values[i];
  leitura2 += sensor_values[i+4]; 
  digitalWrite(13, HIGH);   
      } 
   }
  }
  if(sensor_values[7] < 200) {
    while(sensor_values[6] > 580){//sentido 2 sensor_values[8] < 50
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(AIN1, HIGH);
  analogWrite(SaidaA, pwmi);
  analogWrite(SaidaB, pwmi);
  qtra.read(sensor_values);
  leitura1 = 0; leitura2 = 0;
      for(int i= 0; i<4; i++){
  leitura1 += sensor_values[i];
  leitura2 += sensor_values[i+4];   
  digitalWrite(13, HIGH);
      }
    }
  }
  
  digitalWrite(13, LOW); 
  p = erro * kp;
  
  integral += erro;
  i = ki * integral;
 
  d = kd * (erro - erroAnterior);
  
  erroAnterior = erro;
  Serial.println(leitura1);
  //Serial.println(erro);
  Turn = p + i + d;
  MotorA = pwm + Turn;
  MotorB = pwm - Turn;

  digitalWrite(BIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(AIN1, HIGH);

  if(MotorA < 0){
     digitalWrite(AIN2, HIGH);
     digitalWrite(AIN1, LOW);
    
  }
  if(MotorB < 0){
    digitalWrite(BIN2, HIGH);
    digitalWrite(BIN1, LOW);
  }
  
  analogWrite(SaidaA, MotorA);
  analogWrite(SaidaB, MotorB);

}
