#include <SoftwareSerial.h> //INCLUSÃO DE BIBLIOTECA
#include <Mouse.h>

const int pinoRX = 0; 
const int pinoTX = 1;

SoftwareSerial bluetooth(pinoRX, pinoTX);  

String comando;
char caracter;
int c = 0, curva[10][10];
void valores(int);


void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  delay(100);
  
  while(1)
    {
      delay(100);
      caracter = bluetooth.read();
      comando += caracter;
         
      switch(caracter)
        {
          case 'N': 
            valores(0);
            break;
            
          case 'P': 
            valores(1);
            break;
            
          case 'I': 
            valores(2); 
            break;
            
          case 'D':
            valores(3);
            break;
            
          case 'W':
            valores(4);
            c++;
            break;
        }
    }
}

void valores(int v)
{
  int tamanho = (comando.length() - 1);
  String cte = comando.substring(0, tamanho);
  curva[c][v] = cte.toDouble();
  comando = "";
}

void loop() {
  // put your main code here, to run repeatedly:

}
