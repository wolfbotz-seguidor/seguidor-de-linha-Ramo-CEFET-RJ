#define CLK 1 //porta que controla o clock do flip flop
#define D 2 //porta que controla o D do flip flop
#define sensor_tensao A0

float leitura_tensao;
String bluetooth;
float tensao;

void setup() {
  Serial.begin(9600);

  pinMode(CLK, OUTPUT);
  pinMode(D, OUTPUT);
  pinMode(sensor_tensao, INPUT);
}

void loop() {

  leitura_tensao = analogRead(sensor_tensao);
  tensao = 5 * (leitura_tensao/1023);
  
  while(Serial.available()){
    delay(5);

    char let = Serial.read();
    bluetooth += let;
  }

  if(bluetooth.length() > 0x00){
    Serial.println(bluetooth);
    
    if(bluetooth == "r") reset();

    if(bluetooth == "a"){
      while(bluetooth == "a"){
        leitura_1();
        Serial.print(leitura_tensao);
        Serial.println("V");
      }
    }

    if(bluetooth == "b"){
      while(bluetooth == "b"){
        leitura_2();
        Serial.print(leitura_tensao);
        Serial.println("V");
      }
    }

    if(bluetooth == "c"){
      while(bluetooth == "c"){
        leitura_3();
        Serial.print(leitura_tensao);
        Serial.println("V");
      }
    }

    if(bluetooth == "d"){
      while(bluetooth == "d"){
        leitura_4();
        Serial.print(leitura_tensao);
        Serial.println("V");
      }
    }

    if(bluetooth == "e"){
      while(bluetooth == "e"){
        leitura_5();
        Serial.print(leitura_tensao);
        Serial.println("V");
      }
    }

    if(bluetooth == "f"){
      while(bluetooth == "f"){
        leitura_6();
        Serial.print(leitura_tensao);
        Serial.println("V");
      }
    }
    
    bluetooth = "";
  }
  
}

int reset(){
  digitalWrite(D, LOW);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
}

int leitura_1() {
  digitalWrite(D, HIGH);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
}

int leitura_2() {
  digitalWrite(D, HIGH);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
  digitalWrite(D, LOW);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
}

int leitura_3() {
  digitalWrite(D, HIGH);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
}

int leitura_4() {
  digitalWrite(D, HIGH);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
  digitalWrite(D, LOW);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
}

int leitura_5() {
  digitalWrite(D, HIGH);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
  digitalWrite(D, LOW);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
  digitalWrite(D, HIGH);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
}

int leitura_6() {
  digitalWrite(D, HIGH);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
  digitalWrite(D, LOW);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
}
