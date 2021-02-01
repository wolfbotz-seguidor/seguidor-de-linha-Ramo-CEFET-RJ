//Código de leitura de sensores qre com sua leitura enviada para um módulo bluetooth hc 05// 
//Código de exemplo em teste//

#include <QTRSensors.h>

//-------Método sem a biblioteca--------//

int sensores_frontais[] = {A0, A1, A2, A3, A4, A5, A6, A7};
int Leitura_dos_sensores[8]; // leitura dos sensores frontais

//----------------------------------//



//--------Com a Biblioteca :(----------//
QTRSensors qtr;
  
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
  


//-----Código para melhor visualização no serial-------------//
int sensor_1, sensor_2, sensor_3, sensor_4, sensor_5, sensor_6, sensor_7, sensor_8;

//-----------------------------------------------------------//

void setup() {
//-------Método sem a biblioteca--------//
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  Serial.begin(9600);
//-----------------------------------// 

//---------Método com a biblioteca----//
  
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount); //SensorCount estabelece o número de sensores no objeto
  qtr.setEmitterPin(2);
  Serial.begin(9600);
//---------------------------------//



}

void loop() {

QTRRead(); //Com a biblioteca

Sem_biblioteca();

Codigo_cru();


}



//---------Método com a biblioteca----//

// read raw sensor values
void QTRRead(){
  qtr.read(sensorValues);

  // print the sensor values as numbers from 0 to 1023, where 0 means maximum
  // reflectance and 1023 means minimum reflectance
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(250);
}
//---------------------------------//



//-------Método sem a biblioteca--------//
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
//--------------------------------------// 


//------Código abaixo tem a intenção de mostrar no serial-----//
//os valores na mesma linha para melhor visualização---------//

void Codigo_cru(){
  sensor_1 = analogRead(A0);
  sensor_2 = analogRead(A1);
  sensor_3 = analogRead(A2);
  sensor_4 = analogRead(A3);
  sensor_5 = analogRead(A4);
  sensor_6 = analogRead(A5);
  sensor_7 = analogRead(A6);
  sensor_8 = analogRead(A7);

  Serial.print("Sensor 1:");
  Serial.print(sensor_1);
  Serial.print('\t');
  
  Serial.print("Sensor 2:");
  Serial.print(sensor_2);
  Serial.print('\t');
  
  Serial.print("Sensor 3:");
  Serial.print(sensor_3);
  Serial.print('\t');
  
  Serial.print("Sensor 4:");
  Serial.print(sensor_4);
  Serial.print('\t');
  
  Serial.print("Sensor 5:");
  Serial.print(sensor_5);
  Serial.print('\t');
  
  Serial.print("Sensor 6:");
  Serial.print(sensor_6);
  Serial.print('\t');
  
  Serial.print("Sensor 7:");
  Serial.print(sensor_1);
  Serial.print('\t');
  
  Serial.print("Sensor 8:");
  Serial.println(sensor_1);
}
//----------------------------//
