// Esse código tem o objetivo de criar um chaveamento com 3 flip flops do tipo D em série
//gerando 3 bits como resposta que irão para 3 entradas de portas NAND que resultaram no curto
//de um circuito por meio de um transistor NPN para então fazermos a leitura individual de cada região
//do robô utilizando somente um sensor de tensão. Tudo isso recebendo os comando por um módulo bluetooth, o HC-05
//Os bits mais significativos estão à direita


//Dessa vez foi utilizado o comando switch case

#define CLK 1 //porta que controla o clock do flip flop
#define D 2 //porta que controla o D do flip flop
#define sensor_tensao A0  //as portas selecionadas podem ser alteradas de acordo com a PCB do Robô

float leitura_tensao;
char bluetooth;  //Utilização somente como caracter, não precisará utilizar o Serial.length para saber o tamanho da string
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
  
  if(Serial.available() > 0){ //aguarda a entrada de dados
    delay(5);                 //Por não ser uma string podemos utilizar somente o if ao invés de while

    bluetooth = Serial.read();  //Lê a entrada de dados no serial e agrega a uma varável
    
    Serial.println(bluetooth);    //Imprime no monitor serial


    switch(bluetooth){  //Utilização do swtich para otimização do código
        case 'a':
          Serial.println("Leitura do motor direito");   //Antes de realizar a função de leitura, confitmar no serial o que está sendo feito
          while(bluetooth == 'a'){  //Loop para fazer a leitura e interromper ao receber outro comando
            leitura_1();            // Leitura1 --> bits: 100
            Serial.print(leitura_tensao);
            Serial.println("V");
          }  

        case 'b':
          Serial.println("Leitura do motor esquerdo");
          while(bluetooth == 'b'){
            leitura_2();            // Leitura2 --> bits: 010
            Serial.print(leitura_tensao);
            Serial.println("V");
          }

        case 'c':
          Serial.println("Leitura do Vin da ponte H");
           while(bluetooth == 'c'){
            leitura_3();            // Leitura3 --> bits: 110
            Serial.print(leitura_tensao);
            Serial.println("V");
          }

        case 'd':
          Serial.println("Leitura do Vin do ATmega");
          while(bluetooth == 'd'){
            leitura_4();            // Leitura4 --> bits: 001
            Serial.print(leitura_tensao);
            Serial.println("V");
          }

        case 'e':
          Serial.println("Leitura da tensão dos sensores frontais e traseiros");
          while(bluetooth == "e"){
            leitura_5();            // Leitura5 --> bits: 101
            Serial.print(leitura_tensao);
            Serial.println("V");
          }

        case 'f':
          Serial.println("Leitura da tensão dos sensores laterias");
          while(bluetooth == 'f'){
            leitura_6();            // Leitura6 --> bits: 011
            Serial.print(leitura_tensao);
            Serial.println("V");
          }

      case 'r':
        Serial.println("Realizando o reset");
        reset();

      
      default:
        bluetooth = "";  //Agora sim limpamos o carcater. Na prática veremos a necesidade de usar esse comando toda vez que trocar de leitura ou não.
        //Serial.println(bluetooth);  //opcional para mostrar o caracter vazio na linha
        break;
      
    }
  
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
  leitura_1();  //Para reduzir os códigos, pegamos as partes redundantes e colocamos as funçoes anteriores dentro das próximas funções
  digitalWrite(D, LOW);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
}

int leitura_3() {
  leitura_1();
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
}

int leitura_4() {
  leitura_2();
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
}

int leitura_5() {
  leitura_2();
  digitalWrite(D, HIGH);
  delay(100);
  digitalWrite(CLK, HIGH);
  delay(100);
  digitalWrite(CLK, LOW);
  delay(100);
}

int leitura_6() {
  leitura_1();
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
