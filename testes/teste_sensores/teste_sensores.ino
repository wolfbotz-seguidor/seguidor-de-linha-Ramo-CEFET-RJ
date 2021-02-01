//Código usado para testar os sensores frontais do Agostinho 2.0

#include <QTRSensors.h>

unsigned int sensor_values[6];

QTRSensorsAnalog qtra  ((unsigned char[]) {A3, A2, A1, A0, A7, A6}, 6); //Sensores 
int position_line = 0;
void setup(){
  
  Serial.begin(9600);
  
  pinMode(A0,      INPUT);
  pinMode(A1,      INPUT);
  pinMode(A2,      INPUT);
  pinMode(A3,      INPUT);
  pinMode(A6,      INPUT);
  pinMode(A7,      INPUT);
  
 Serial.println("Calibre: ");
  for (int i = 0; i < 200; i++)
    {
      qtra.calibrate();
      delay(5);
    }
}

void loop(){

//  qtra.readCalibrated(sensor_values);

  position_line = qtra.readLine(sensor_values,QTR_EMITTERS_ON, 1);
//  for (int i = 0; i < 6; i++)
//    {
//     Serial.print(" [" + String(i+1) + "]: ");
//     Serial.print(sensor_values[i]);
//     Serial.print("  ");
//    } 

   Serial.println("");
   Serial.println("POSITION LINE: ");
   Serial.print(position_line);
   delay(100);
}
