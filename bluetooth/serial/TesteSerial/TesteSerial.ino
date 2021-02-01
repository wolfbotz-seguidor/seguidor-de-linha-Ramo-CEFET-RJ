#include <SoftwareSerial.h> //INCLUSÃO DE BIBLIOTECA
#include <Mouse.h>

const int pinoRX = 0; 
const int pinoTX = 1;

SoftwareSerial bluetooth(pinoRX, pinoTX);  

void ModoManual_Frente();
void ModoManual_direita();
void Manual_Esquerda(); 
void ModoManual_Tras();

int Saida = 0;
int tamanhoComando;

#define SaidaA 9
#define SaidaB 3

//Lado direito
#define AIN2 6 // Quando em HIGH, roda direita anda para frente
#define AIN1 5 

//Lado Esquerdo
#define BIN1 4 // Quando em HIGH, roda esquerda anda para frente
#define BIN2 2

double kp, kd, ki, PWM;
char recebeCaracter, tipo;
String comando, cte;

void setup()
{
  pinMode(AIN1,  OUTPUT);
  pinMode(AIN2,  OUTPUT);
  pinMode(BIN1,  OUTPUT);
  pinMode(BIN2,  OUTPUT);
  pinMode(SaidaA,OUTPUT);
  pinMode(SaidaB,OUTPUT);

  Serial.begin(9600);    //Inicializa a Serial
  bluetooth.begin(9600); //Inicializa a Serial do Bluetooth
  
  delay(100);

  Serial.println("Começando compilação");

while (Saida != 1) {
  
  while(bluetooth.available())
  {
    delay(100);

    recebeCaracter = bluetooth.read();
    
    Serial.print("recebeCaracter_Setup: ");
    Serial.println(recebeCaracter);
    
    comando += recebeCaracter;

     if (recebeCaracter == ';')
       {
        Serial.print(" Comando Recebido: ");
        Serial.print(comando);
        
        tamanhoComando = comando.length();
        Serial.print("| tamanhoComando: ");
        Serial.print(tamanhoComando);
        
        cte = comando.substring(1, tamanhoComando - 1);

        Serial.print("| cte: ");
        Serial.println(cte);

        if (comando.substring(0,1) == "S") // Seguir
          {
            Saida = 1;
          }
        else if (comando.substring(0,1) == "P")
          {
           kp = cte.toDouble();

             Serial.print("Kp: ");
             Serial.println(kp);
          }
        else if (comando.substring(0,1) == "I")
          {
           ki = cte.toDouble();

             Serial.print("Ki: ");
             Serial.println(ki);
          }
        else if (comando.substring(0,1) == "D")
          {
           kd = cte.toDouble();

             Serial.print("Kd: ");
             Serial.println(kd);
          }
        else if (comando.substring(0,1) == "W")
          {
           PWM = cte.toDouble();

             Serial.print("PWM: ");
             Serial.println(PWM);
          }

        comando = "";
       }

  }
  
}
  Serial.print("Kp: ");
  Serial.print(kp);
  Serial.print("| Ki: ");
  Serial.print(ki);
  Serial.print("| Kd: ");
  Serial.print(kd);
  Serial.print("| PWM: ");
  Serial.print(PWM);
}


void loop()
{
  //----------------------> Lista de Comandos <----------------------\\
  //Parar    - R
  //Seguir   - S
  //Cima     - F
  //Esquerdo - E
  //Direito  - O
  //Baixo    - V
  
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
            Saida = 1;
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
       Serial.println("TA PEGANDO AQUI");


}

void ModoManual_Frente()
{
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(SaidaA, 40);
  analogWrite(SaidaB, 40);
  
  Serial.println("FRENTE CHEGOU");
}

void ModoManual_Tras()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(SaidaA, 40);
  analogWrite(SaidaB, 40);
  
  Serial.println("BAIXO CHEGOU");
}


void Manual_Esquerda() 
{
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(SaidaA, 40);
  analogWrite(SaidaB, 40);
  
  Serial.println("ESQUERDA CHEGOU");
}

void ModoManual_direita()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(SaidaA, 40);
  analogWrite(SaidaB, 40);
  
  Serial.println("DIREITA CHEGOU");
}
