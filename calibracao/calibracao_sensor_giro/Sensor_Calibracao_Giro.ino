#include <QTRSensors.h>

#define SaidaA 9
#define SaidaB 3

// Lado Esquerdo
#define BIN2 2 // Trás
#define BIN1 4 // Frente
// Lado Direito
#define AIN2 6 // Frente
#define AIN1 5 // Trás

QTRSensorsAnalog qtra((unsigned char[]) {A4, A6, A7, A0, A1, A2}, 6);

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A4, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(SaidaA, OUTPUT);
  pinMode(SaidaB, OUTPUT);
  
  Serial.begin(9600);

  for (int i = 0; i < 100; i++) // make the calibration take about 10 seconds
  {
    qtra.calibrate(); // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)

    digitalWrite(AIN2, HIGH);
    digitalWrite(AIN1, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    analogWrite(SaidaA, 40);
    analogWrite(SaidaB, 40);

    delay(20);

    qtra.calibrate();

    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    analogWrite(SaidaA, 40);
    analogWrite(SaidaB, 40);

    delay(20);
  }

  Serial.begin(9600);
  
  for (int i = 0; i < 6; i++) 
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (int i = 0; i <6; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
 
  Serial.println("Fim da calibração");
}
