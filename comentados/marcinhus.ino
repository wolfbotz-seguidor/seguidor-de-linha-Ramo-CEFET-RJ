#include <QTRSensors.h>

// Criação de Macros para ajudar no programa

// Função que verifica o valor mínimo através de um operador ternário
#define MIN(a,b) (a<b?a:b)

// Função queverifica o valor máximo através de um operador ternário
#define MAX(a,b) (a>b?a:b)

// Utilizando MIN e MAX juntas podemos criar a ideia de limitar um valor
// e atribuir isso a uma nova função
#define RAN(a,b,c) (MAX(MIN(a,c),b))

// Função que pega o valor absoluto de um número qualquer
#define ABS(a) (a < 0 ? -a : a)
//----------------------------------
//Definição de sensores
#define EDGEVAL 800
#define NUMSENSORS 8

// Variáveis booleanas que indicam qual a cor da linha
#define BLACKL 0
#define WHITEL 1

// Posição média dos sensores do seguidor
#define MID ((NUMSENSORS - 1)*500)
#define TIMEOUT 3000

unsigned char edgesensor[2] = {22, 23};

// Criando o objeto qtra reponsável pelos sensores do seguidor
QTRSensorsAnalog qtra((unsigned char[]) {13,12,14,27,26,25,33,32}, NUMSENSORS, 4, 22);

QTRSensorsRC qtrc((unsigned char[]) {23}, 1);

unsigned int sensors[NUMSENSORS], sensors_e[2];
unsigned int e_min[2];
unsigned int e_max[2];
//-----------------------------------
//MOTORES
#define PWMB 21
#define BIN2 19
#define BIN1 18

#define PWMA 15
#define AIN2 2
#define AIN1 4

#define MA_PWMCHANNEL 0
#define MB_PWMCHANNEL 1

#define FREQ 5000
#define RESOLUTION  8

// Declarando as funções que serão usadas
void followLine(int velocidadeA, int velocidadeB, bool sa = HIGH, bool sb = HIGH);
void posLine(byte line);
void addToVec(double err);
unsigned long calibrationTime = 0;

//SETUP---------------------------
void setup() {

  Serial.begin(115200);

  // Setup do motor, todas as entradas são OUTPUT
  // pois estão apenas enviando valores
  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  ledcSetup(MA_PWMCHANNEL, FREQ, RESOLUTION);
  ledcAttachPin(PWMA, MA_PWMCHANNEL);
  ledcSetup(MB_PWMCHANNEL, FREQ, RESOLUTION);
  ledcAttachPin(PWMB, MB_PWMCHANNEL);
  //-------------------------

  // Calibração dos sensores
  Serial.println("CALIBRANDO");
  for (int i = 0; i < 800; i++)  // Esse for faz a calibração acontecer dentro de 10 segundos
  {
    qtra.calibrate();       // Método para a leitura dos sensores
  }

  // Mostrando no serial quando o robô for calibrado
  Serial.println("CALIBRADO");
  calibrationTime = millis();
  
  //--------------------------
}


//INICIA OS MOTORES

//Valores das constantes que serão usadas no PID
#define KP 6
#define KD 145
#define KI 0
double ajust = 0, sum, rd;
long pos = 0;
double error[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte sensorsInLine;
int vmax = 255;

//7.9V
bool foward = true;

void loop() {
  // If para verificar se o tempo da corrida já acabou, se sim o robô ativa a variável
  // fim e então o programa para de executar a parte de movimento dos motores
  // só pode ser usada após cronometrar o tempo na corrida
  if((millis()- calibrationTime) > 18000) {
  followLine(0,0);
  fim = true;
  
  }

  // Função que verifica a posição da linha em relação ao seguidor
  posLine(WHITEL);

  // Criando o erro através da diferença do valor pego pela
  // posição média do seguidor
  addToVec(pos - MID);

  // Realizando o cálculo do PID, a variável sum é usadas junto da
  // a constante de integração (KI)
  sum = 0;
  for (int i = 0; i < 10; i++) sum += error[i];
  ajust = KP*error[0] + KD*(error[0] - error[1]) + KI*sum;

  // Se o valor de ajuste for muito alto ele ativa uma função que
  // tem por objetivo girar o seguidor de tal forma que ele sempre
  // volte para a linha, assim evitando possíveis saídas da pista
  Serial.println(ajust);
  if(ABS(ajust) > 3500){
    if (ajust > 0)followLine(vmax *.8, vmax - ajust, true, false);
    else followLine(vmax + ajust,vmax *.8, false, true);
  }else{
    foward = true;
    if (ajust > 0)followLine(vmax, vmax - ajust);
    else followLine(vmax + ajust,vmax);
  }
  
   
}

void followLine(int velocidadeA, int velocidadeB, bool sa, bool sb){
  // Quando fim é ativado o código não executa mais a função de que faz o seguidor andar
  if(fim) return;

  // Utilizando abs e ran para limitar o valor da velocidade dentro de 0 até o vmax (255)
  velocidadeA = RAN(ABS(velocidadeA), 0, vmax);
  velocidadeB = RAN(ABS(velocidadeB), 0, vmax);

   // Colocando o sentido de giro do motor dependendo do valor da velocidade em A e B
  if( velocidadeA > 0){
    digitalWrite(AIN2, sa);
    digitalWrite(AIN1, !sa);
  }else{
    digitalWrite(AIN2, !sa);
    digitalWrite(AIN1, sa);
  }

  if(velocidadeB > 0){
    digitalWrite(BIN2,sb);
    digitalWrite(BIN1, !sb);
  }else{
    digitalWrite(BIN2, !sb);
    digitalWrite(BIN1, sb);
  }

  // Enviando o valor ajustado para os motores
  ledcWrite(MA_PWMCHANNEL, velocidadeA);
  ledcWrite(MB_PWMCHANNEL, velocidadeB);
  
}

// Função para parar o seguidor, fazendo todas as portas dos motores receberem TRUE
void Break(){
  digitalWrite(AIN2, true);
  digitalWrite(AIN1, true);
  digitalWrite(BIN2, true);
  digitalWrite(BIN1, true);
}

// Aqui estocamos os 9 últimos erros feitos, como forma de usar na parte da integral
void addToVec(double err){
  for(int i = 8; i >= 0; i--)error[i + 1] = error[i];
  error[0] = err;
}

// Função que retorna a posição do seguidor em relação à linha
// a variável line decide como a lógica será feita dependendo
// se a linha for branca ou preta
void posLine(byte line){

  // Leitura dos sensores
  qtra.read(sensors);

  // Criação de variáveis que serão usadas
  double w, v, sum = 0, division = 0;
  sensorsInLine = 0;

  // Método de média ponderada feito em cada sensor
  for(int j = 0; j < NUMSENSORS; j++){

    // Essa parte limita o valor do sensor dentro do mínimo e máximo obtidos na calibração
    w = RAN(sensors[j], qtra.calibratedMinimumOn[j], qtra.calibratedMaximumOn[j]);

    // Subtraindo o valor de 2 pelo valor mínimo lido nós fazemos o valor da variável w
    // ficar dentro dos limites de 0 até o valor máximo
    w -= qtra.calibratedMinimumOn[j];
    w /= (qtra.calibratedMaximumOn[j] - qtra.calibratedMinimumOn[j]);
    // O valor de w muda dependendo das cores da pista
    if (line) w = 1 - w;
    if (w > 0.6) sensorsInLine ++;
    sum += 1000*j*w;
    division += w;
  }
  
  // Caso os sensores não estejam na linha será enviado o valor máximo
  // sendo 0 para uma ponta e 2 vezes o valor médio para a outra.
  // Assim a lógica de voltar à pista é executada
  if (!sensorsInLine){
    if(pos > MID){
      pos = 2*MID;
    }else{
      pos = 0;
    }
  // Caso contrário o valor da pos é apenas a divisão entre a soma
  // pelos valores dos sensores (variável division)
  }else{
    pos = sum/division;
  }
  
}
