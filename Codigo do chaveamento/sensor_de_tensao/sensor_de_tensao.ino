// Esse código tem o objetivo de criar um chaveamento com 3 flip flops do tipo D em série
//gerando 3 bits como resposta que irão para 3 entradas de portas NAND que resultaram no curto
//de um circuito por meio de um transistor NPN para então fazermos a leitura individual de cada região
//do robô utilizando somente um sensor de tensão. Tudo isso recebendo os comando por um módulo bluetooth, o HC-05
//Os bits mais significativos estão à direita

#define CLK 1 //porta que controla o clock do flip flop
#define D 2 //porta que controla o D do flip flop
#define sensor_tensao A0  //as portas selecionadas podem ser alteradas de acordo com a PCB do Robô

float leitura_tensao;
String bluetooth;
float tensao; //variável que converte os valores analógicos em valores de tensão de 0  5V

void setup() {
  Serial.begin(9600);

  pinMode(CLK, OUTPUT);  //Definição das portas como saída e entrada de dados
  pinMode(D, OUTPUT);
  pinMode(sensor_tensao, INPUT);
}

void loop() {

  leitura_tensao = analogRead(sensor_tensao); //Possibilitar a leitura do pino que selecionamos
  tensao = 5 * (leitura_tensao/1023);  //Conversão de analog para Volts
  
  while(Serial.available()){ //aguarda a entrada de dados
    delay(5);

    char let = Serial.read();  //Lê a entrada de dados no serial e agrega a uma varável
    bluetooth += let;          // adiciona esse caracter à variável bluetooth e por ser uma string suporta caracteres
  }

  if(bluetooth.length() > 0x00){  //comprimento maior que zero
    Serial.println(bluetooth);    //Imprime no monitor serial
    
    if(bluetooth == "r") reset();  //Reseta todos os bits de saída dos flip flops

    else if(bluetooth == 'a'){       //Comando do bluetooth através de caracteres
      while(bluetooth == 'a'){  //Loop para fazer a leitura e interromper ao receber outro comando
        leitura_1();            // Leitura1 --> bits: 100
        Serial.print(leitura_tensao);
        Serial.println("V");
      }
    }

    else if(bluetooth == 'b'){
      while(bluetooth == 'b'){
        leitura_2();            // Leitura2 --> bits: 010
        Serial.print(leitura_tensao);
        Serial.println("V");
      }
    }

    else if(bluetooth == 'c'){
      while(bluetooth == 'c'){
        leitura_3();            // Leitura3 --> bits: 110
        Serial.print(leitura_tensao);
        Serial.println("V");
      }
    }

    else if(bluetooth == 'd'){
      while(bluetooth == 'd'){
        leitura_4();            // Leitura4 --> bits: 001
        Serial.print(leitura_tensao);
        Serial.println("V");
      }
    }

    else if(bluetooth == 'e'){
      while(bluetooth == "e"){
        leitura_5();            // Leitura5 --> bits: 101
        Serial.print(leitura_tensao);
        Serial.println("V");
      }
    }

    else if(bluetooth == 'f'){
      while(bluetooth == 'f'){
        leitura_6();            // Leitura6 --> bits: 011
        Serial.print(leitura_tensao);
        Serial.println("V");
      }
    }

    else if(bluetooth == 'z'){
      bluetooth = "";  //Agora sim limpamos o carcater. Na prática veremos a necesidade de usar esse comando toda vez que trocar de leitura ou não.
    }

    
    //bluetooth = "";
  }
  
}

int reset(){ //função de reset utilizando os pulsos da porta D e do clock do flip flop
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

int leitura_1() {  //função de leitura_1 utilizando essas portas digitais
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
