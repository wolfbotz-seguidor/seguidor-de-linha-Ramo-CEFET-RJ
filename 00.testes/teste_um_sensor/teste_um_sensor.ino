//Código usado para testar o sensor lateral do Agostinho

#include <QTRSensors.h>

unsigned int sensor_values[1];

QTRSensorsRC qtra  ((unsigned char[]) {A5}, 1); // Agostinho

void setup(){

  Serial.begin(9600);
  
  pinMode(A5,      INPUT);

  for (int i = 0; i < 200; i++)
    {
      qtra.calibrate();
      delay(5);
    }
}

void loop(){

  qtra.readCalibrated(sensor_values);

  for (int i = 0; i < 1; i++)
    {
     Serial.print(" [" + String(i+1) + "]: ");
     Serial.print(sensor_values[i]);
     Serial.print("  ");
    } 

  Serial.println("");
}
