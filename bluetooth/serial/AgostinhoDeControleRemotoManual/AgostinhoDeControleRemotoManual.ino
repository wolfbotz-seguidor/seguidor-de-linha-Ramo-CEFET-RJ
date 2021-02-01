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
  //Parar - R
  //Seguir - S
  //Cima - F
  //Esquerdo - E
  //Direito - O
  //Baixo - V
  while(bluetooth.available())
  {
    delay(100);

    recebeCaracter = bluetooth.read();
    Serial.print("recebeCaracter: ");
    Serial.println(recebeCaracter);
    comando += recebeCaracter;

     if (recebeCaracter == ';')
       {
        Serial.print(" Comando Recebido: ");
        Serial.println(comando);

        if (comando.substring(0,1) == "S") // Seguir
          {
          }
        else if (comando.substring(0,1) == "R") // Parar
          {
           digitalWrite(SaidaA, 0);
           digitalWrite(SaidaB, 0);
          }
        else if (comando.substring(0,1) == "F") // Frente
          {
           ModoManual_Frente();
          }
        else if (comando.substring(0,1) == "E") // Esquerda
          {
           Manual_Esquerda();
          }
        else if (comando.substring(0,1) == "O") // Direita
          {
           ModoManual_direita();
          }
        else if (comando.substring(0,1) == "V") // Baixo
          {
           ModoManual_Tras();
          }

        comando = "";
       }
  }


}

void ModoManual_Frente()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(SaidaA, 60);
  analogWrite(SaidaB, 60);
  Serial.println("FRENTE CHEGOU");
}

void ModoManual_Tras()
{
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(SaidaA, 60);
  analogWrite(SaidaB, 60);
  Serial.println("BAIXO CHEGOU");
}


void Manual_Esquerda() 
{
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(SaidaA, 60);
  analogWrite(SaidaB, 60);
  Serial.println("ESQUERDA CHEGOU");
}

void ModoManual_direita()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(SaidaA, 60);
  analogWrite(SaidaB, 60);
  Serial.println("DIREITA CHEGOU");
}
