#include <QTRSensors.h>
#include <Servo.h>
#include "PID_v1.h"

#define VALID_THRESHOLD         200
#define NUM_SENSORS             8
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             9

int counter = 0;                            // VARIÁVEL DE CONTAGEM DE TEMPO PARA CALIBRAR O SENSOR QTR

int potenciaE = 3;                          // PINO PWM DE CONTROLE DA RODA ESQUERDA
int potenciaD = 6;                          // PINO PWM DE CONTROLE DA RODA DIREITA
int motorE1 = 2;                            // PINO DIGITAL RODA ESQUERDA POLO 1
int motorE2 = 4;                            // PINO DIGITAL RODA ESQUERDA POLO 2
int motorD1 = 7;                            // PINO DIGITAL RODA DIREITA POLO 1
int motorD2 = 5;                            // PINO DIGITAL RODA DIREITA POLO 2
int data[NUM_SENSORS];                      // ARRAY PARA ARMAZENAR OS DADOS DO SENSOR ANALÓGICO
unsigned int sensorValues[NUM_SENSORS];     // ARRAY DE LEITURA DOS DADOS DO SENSOR ANALÓGICO

int last_proportional,                      // TERMO DO PID, ÚLTIMA LEITURA PROPORCIONAL
    integral,                               // TERMO DO PID, SOMATÓRIA INTEGRAL
    power_difference,                       // TERMO DO PID, DIFERENÇA APLICADA NOS MOTORES
    proportional,                           // PROPORCIONAL ADAPTADA PARA OS MOTORES DA POLOLU
    derivative;                             // TERMO DO PID, DERIVATIVO

const int weight1 = 1;                      // TERMO DE PESO PARA SENSOR ANALÓGIO DO CENTRO
const int weight2 = 2;                      // TERMO DE PESO PARA SENSOR ANALÓGIO DO MEIO-CENTRO
const int weight3 = 3;                      // TERMO DE PESO PARA SENSOR ANALÓGIO DA MEIO-BORDA
const int weight4 = 4;                      // TERMO DE PESO PARA SENSOR ANALÓGIO DA BORDA

const int max = 200;                        // VELOCIDADE MÁXIMA ATINGIDA PELO MOTOR
int maxInverse = 70;                        // VELOCIDADE MÁXIMA DA INVERSA LIDA PELO MOTOR
int errorBound = 30;                        // CONFIGURA O ERRO MÁXIMO PARA QUE UM DOS MOTORES SEJA LIGADO A INVERSA


QTRSensorsAnalog qtra((unsigned char[]) {   // SENSOR ANALÓGICO INICIALIZAÇÃO
  //A0, A1, A2, A3, A4, A5, A6, A7
  A7, A6, A5, A4, A3, A2, A1, A0
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

double Output;                              // VARIÁVEL QUE ARMAZENA O ERRO CÁLCULADO DA ADAPTAÇÃO (João)

void setup() {
    
  pinMode(motorE1, OUTPUT);
  pinMode(motorE2, OUTPUT);
  pinMode(motorD1, OUTPUT);
  pinMode(motorD2, OUTPUT);
  pinMode(potenciaE, OUTPUT);
  pinMode(potenciaD, OUTPUT);

  digitalWrite(motorD1, HIGH);
  digitalWrite(motorD2, LOW);
  digitalWrite(motorE1, LOW);
  digitalWrite(motorE2, HIGH);

  qtra.emittersOn();
  qtra.calibrate();

  for(counter=0; counter < 80; counter++) {
    qtra.calibrate();
    delay(10);
  }

}


void loop() {

  qtra.read(sensorValues, QTR_EMITTERS_OFF);

  // Conversão para um array de inteiros
  data[0] = sensorValues[0];
  data[1] = sensorValues[1];
  data[2] = sensorValues[2];
  data[3] = sensorValues[3];
  data[4] = sensorValues[4];
  data[5] = sensorValues[5];
  data[6] = sensorValues[6];
  data[7] = sensorValues[7];

  // Valida se as leituras devem ser processadas, no caso de nenhuma linha não estar sendo lida o erro anterior
  // deve ser mantido, pressupõe-se que o sensor achou uma curva muito fechada e não consegui acompanhar
  // o grau da curva no tempo de leitura, por isso mantém o erro anterior.
  if(processValues(data) == 1) {

    // Cálcula o erro em relação a linha central, retorna entre os limites de -110 e 110, 0 para quando o sensor
    // está balanceado com a linha no centro do array
    Output = calcError(data);

    // O termo proporcional deve ser 0 quando estamos na linha, faz o mapeamento do erro lido entre 1000 e 6000
    // que são valores que se adaptam bem ao cálculo do PID.
    proportional = map(-Output, -110, 110, 1000, 6000) - 3500;

    // Calcula o termo derivativo (mudança) e o termo integral (soma) da posição
    derivative = proportional - last_proportional;
    integral += proportional;

    // Lembrando a ultima posição
    last_proportional = proportional;

    // Calcula a diferença entre o aranjo de potência dos dois motores
    // m1 - m2. Se for um número positivo, o robo irá virar para a
    // direita. Se for um número negativo, o robo irá virar para a esquerda
    // e a magnitude dos números determinam a agudez com que fará as curvas/giros
    power_difference = proportional / 10 + integral / 10000 + derivative * 2;

  }

  // Calcula a configuração atual dos motores.  Nunca vamos configurar
  // um motor com valor negativo
  if(power_difference > max) power_difference = max;
  else if(power_difference < -max) power_difference = -max;

  // Define qual motor que será reduzido, se o direito ou o esquerdo
  if(power_difference < 0) set_motors(max + power_difference, max, Output);
  else set_motors(max, max - power_difference, Output);

};

void set_motors(int left_speed, int right_speed, int error){

  // Se a velocidade do motor for positiva manda andar para frente
  if(right_speed > 0) {
    digitalWrite(motorD1, HIGH);
    digitalWrite(motorD2, LOW);
  }
  // Se a velocidade do motor for negativa verifica se deve inverter o motor,
  // caso a borda de erro ultrapasse o limit então o motor é ligado na inversa
  // fazendo-o andar para trás
  else {
    right_speed = 0;
    if(error < -errorBound) right_speed = maxInverse;
    digitalWrite(motorD1, LOW);
    digitalWrite(motorD2, HIGH);
  }

  // Se a velocidade do motor for positiva manda andar para frente
  if(left_speed > 0) {
    digitalWrite(motorE1, LOW);
    digitalWrite(motorE2, HIGH);
  }
  // Se a velocidade do motor for negativa verifica se deve inverter o motor,
  // caso a borda de erro ultrapasse o limit então o motor é ligado na inversa
  // fazendo-o andar para trás
  else {
    left_speed = 0;
    if(error > errorBound) left_speed = maxInverse;
    digitalWrite(motorE1, HIGH);
    digitalWrite(motorE2, LOW);
  }

  //Escreve potência
  analogWrite(potenciaD, right_speed);
  analogWrite(potenciaE, left_speed);

}

double calcError(int sensors[NUM_SENSORS]) {

  // Tamanho do array de intersercção
  int sensorSize = 7;

  // Pondera os pesos do array para que o mais da borda seja mais relevante e inverte o sinal para o lado esquerdo
  float leftHandAux[4] = {
    -sensors[0] * weight4,
    -sensors[1] * weight3,
    -sensors[2] * weight2,
    -sensors[3] * weight1
  };

  // Pondera os pesos do array para que o mais da borda seja mais relevante para o lado direito
  float rightHandAux[4] = {
    sensors[7] * weight4,
    sensors[6] * weight3,
    sensors[5] * weight2,
    sensors[4] * weight1
  };

  // Faz a intersecção de modo a considerar valores mais precisos, porque a linha pode estar entre dois sensores
  float leftIntersection[7] = {
    leftHandAux[3],
    (leftHandAux[3] + leftHandAux[2]) / 2,
    leftHandAux[2],
    (leftHandAux[2] + leftHandAux[1]) / 2,
    leftHandAux[1],
    (leftHandAux[1] + leftHandAux[0]) / 2,
    leftHandAux[0]
  };

  // Faz a intersecção de modo a considerar valores mais precisos, porque a linha pode estar entre dois sensores
  float rightIntersection[7] = {
    rightHandAux[3],
    (rightHandAux[3] + rightHandAux[3]) / 2,
    rightHandAux[2],
    (rightHandAux[2] + rightHandAux[1]) / 2,
    rightHandAux[1],
    (rightHandAux[1] + rightHandAux[0]) / 2,
    rightHandAux[0]
  };

  int lessLeftIdx = 0,        // VÁRIAVEL QUE DETERMINA O INDICE DO MENOR VALOR DO ARRAY DE INTERSECÇÃO ESQUERDO
      lessRightIdx = 0,       // VÁRIAVEL QUE DETERMINA O INDICE DO MENOR VALOR DO ARRAY DE INTERSECÇÃO DIREITO
      lessLeftAux = -9999,    // AUXILIAR USADA PARA COMPARAR E ARMAZENAR O MENOR VALOR ESQUERDO
      lessRightAux = 9999;    // AUXILIAR USADA PARA COMPARAR E ARMAZENAR O MENOR VALOR DIREITO
  float leftLessSum = 0,      // ARMAZENA SOMA DOS VALORES DO INDICE ESQUERDO DO MENOR PARA BAIXO
        leftMajorSum = 0,     // ARMAZENA SOMA DOS VALORES DO INDICE ESQUERDO DO MENOR PARA CIMA
        rightLessSum = 0,     // ARMAZENA SOMA DOS VALORES DO INDICE DIREITO DO MENOR PARA BAIXO
        rightMajorSum = 0;    // ARMAZENA SOMA DOS VALORES DO INDICE DIREITO DO MENOR PARA CIMA

  // Busca indice do menor valor no array de interseccção
  for (int i = 0; i < sensorSize; i++) {
    if (leftIntersection[i] > lessLeftAux) {
      lessLeftAux = leftIntersection[i];
      lessLeftIdx = i;
    }
  }
  // Cálcula soma da posição do array para baixo
  for (int i = 0; i <= lessLeftIdx; i++) leftLessSum += leftIntersection[i];
    // Cálcula soma da posição do array para cima
  for (int i = lessLeftIdx; i < sensorSize; i++) leftMajorSum += leftIntersection[i];


  // Busca indice do menor valor no array de interseccção
  for (int i = 0; i < sensorSize; i++) {
    if (rightIntersection[i] < lessRightAux) {
      lessRightAux = rightIntersection[i];
      lessRightIdx = i;
    }
  }
  // Cálcula soma da posição do array para baixo
  for (int i = 0; i <= lessRightIdx; i++) rightLessSum += rightIntersection[i];
  // Cálcula soma da posição do array para cima
  for (int i = lessRightIdx; i < sensorSize; i++) rightMajorSum += rightIntersection[i];

  // Calcula diferença entre as somatórias
  float calcLeftSomatory = leftLessSum - leftMajorSum;
  float calcRightSomatory = rightLessSum - rightMajorSum;

  // Retorna proporção em % em relação a linha central
  return ((calcLeftSomatory + calcRightSomatory) * 100) / 14849.5;
};


// Valida se todos os sensores não estão lendo preto para que seja mantido o erro anterior
int processValues(int s[8]) {
  if(s[0] < VALID_THRESHOLD || s[1] < VALID_THRESHOLD || s[2] < VALID_THRESHOLD || s[3] < VALID_THRESHOLD ||
     s[4] < VALID_THRESHOLD || s[5] < VALID_THRESHOLD || s[6] < VALID_THRESHOLD || s[7] < VALID_THRESHOLD) {
    return 1;
  } else {
    return 0;
  }
}
