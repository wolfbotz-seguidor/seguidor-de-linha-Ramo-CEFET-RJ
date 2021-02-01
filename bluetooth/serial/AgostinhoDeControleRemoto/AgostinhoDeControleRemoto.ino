#include <Mouse.h>
#include <SoftwareSerial.h> //INCLUSÃO DE BIBLIOTECA

const int pinoRX = 0; 
const int pinoTX = 1;  

SoftwareSerial bluetooth(pinoRX, pinoTX);  

void ModoManual_Frente();
void ModoManual_direita();
void Manual_Esquerda(); 
void ModoManual_Tras();

#define SaidaA 9
#define SaidaB 7

#define BIN2 8
#define BIN1 4
#define AIN2 6
#define AIN1 5

char recebeCaracter, tipo;
String comando;

void setup()
{
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(SaidaA, OUTPUT);
  pinMode(SaidaB, OUTPUT);

  Serial.begin(9600); //INICIALIZA A SERIAL
  bluetooth.begin(9600); //INICIALIZA A SERIAL DO BLUETOOTH
  delay(100); //INTERVALO DE 100 MILISSEGUNDOS

  Serial.println("Começando compilação");
}
void loop()
{
  while(bluetooth.available())
  {
    delay(100);

    recebeCaracter = bluetooth.read();
    Serial.print("recebeCaracter: ");
    Serial.println(recebeCaracter);
    comando += recebeCaracter;

     if (recebeCaracter == ';') {
        Serial.print(" Comando Recebido: ");
        Serial.println(comando);
     }

     
     if (comando.substring(0,1) == "I") {
       String canal = comando.substring(1,2);
       Serial.print("Canal: ");
       Serial.println(canal);
     }

     comando = "";

/*
    if (recebeCaracter == ';')
    {
      if (recebeCaracter == 'F')
      {
      // ModoManual_Frente();
       Serial.println("f");
      }
      else if (tipo == 'V')
      {
      //  ModoManual_Tras();
        Serial.println("V");
      }
      else if (tipo == 'E')
      {
       // Manual_Esquerda();
        Serial.println("E");
      }
      else if (tipo == 'O')
      {
       // ModoManual_direita();
        Serial.println("O");
      } 
      else if(tipo == 'R')
      {
        analogWrite(SaidaA, 0);
        analogWrite(SaidaB, 0);
      }
    }
*/
  }


}
/*
void ModoManual_Frente()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(SaidaA, 60);
  analogWrite(SaidaB, 60);
}

void ModoManual_Tras()
{
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(SaidaA, 60);
  analogWrite(SaidaB, 60);
}


void Manual_Esquerda() 
{
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(SaidaA, 60);
  analogWrite(SaidaB, 60);
}

void ModoManual_direita()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(SaidaA, 60);
  analogWrite(SaidaB, 60);
  
}
*/
