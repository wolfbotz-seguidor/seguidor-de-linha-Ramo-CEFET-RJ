// Código para testar o sentido das rodas do Agostinho

#define SaidaA 9
#define SaidaB 10

//Lado direito
#define AIN2 6 // Quando em HIGH, roda direita anda para frente
#define AIN1 5 

//Lado Esquerdo
#define BIN1 4 // Quando em HIGH, roda esquerda anda para frente
#define BIN2 3

void setup(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(SaidaA, OUTPUT);
  pinMode(SaidaB, OUTPUT);
}

void loop(){

  digitalWrite(5, HIGH);  // Sentido do Agostinho
  digitalWrite(6, LOW);
  digitalWrite(BIN2, HIGH);
  digitalWrite(BIN1, LOW);

  analogWrite(SaidaA, 100);
  analogWrite(SaidaB, 100);
}
