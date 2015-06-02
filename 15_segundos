#include <QTRSensors.h>
#include <Servo.h>
#include "PID_v1.h"

#define VALID_THRESHOLD         200
#define NUM_SENSORS             8
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             9

int counter = 0;

int potenciaE = 3;
int potenciaD = 6;
int motorE1 = 2;
int motorE2 = 4;
int motorD1 = 5;
int motorD2 = 7;
int data[NUM_SENSORS];
int speedInitialD = 80;
int speedInitialE = 80;
unsigned int sensorValues[NUM_SENSORS];

int last_proportional,
    integral,
    power_difference,
    proportional;

const int weight1 = 1;
const int weight2 = 2;
const int weight3 = 3;
const int weight4 = 4;

QTRSensorsAnalog qtra((unsigned char[]) {
  A3, A5, A4, A7, A6, A1, A0, A2
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

double Setpoint, Input, Output;

void setup() {
  Serial.begin(9600);
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
  analogWrite(potenciaD, speedInitialD);
  analogWrite(potenciaD, speedInitialE);
  qtra.calibrate();

  Setpoint = 0;

  for(counter=0; counter < 80; counter++) {
    qtra.calibrate();
    delay(10);
  }

}


void loop() {

  qtra.read(sensorValues, QTR_EMITTERS_OFF);

  data[0] = sensorValues[0];
  data[1] = sensorValues[1];
  data[2] = sensorValues[2];
  data[3] = sensorValues[3];
  data[4] = sensorValues[4];
  data[5] = sensorValues[5];
  data[6] = sensorValues[6];
  data[7] = sensorValues[7];

  if(processValues(data) == 1) {

    Output = calcError(data);

    // O termo proporcional deve ser 0 quando estamos na linha
    proportional = map(-Output, -110, 110, 1000, 6000) - 3500;

    // Calcula o termo derivativo (mudança) e o termo integral (soma)
    // da posição
    int derivative = proportional - last_proportional;
    integral += proportional;

    // Lembrando a ultima posição
    last_proportional = proportional;

    // Calcula a diferença entre o aranjo de potência dos dois motores
    // m1 - m2. Se for um número positivo, o robot irá virar para a
    // direita. Se for um número negativo, o robot irá virar para a esquerda
    // e a magnetude dos números determinam a agudez com que fará as curvas/giros
    power_difference = proportional/10 + integral/10000 + derivative * 2;

  }

  // Calcula a configuração atual dos motores.  Nunca vamos configurar
  // um motor com valor negativo
  const int max = 230;
  if(power_difference > max)
    power_difference = max;
  if(power_difference < -max)
    power_difference = -max;
  if(power_difference < 0)
    set_motors(max + power_difference, max, Output);
  else
    set_motors(max, max - power_difference, Output);

  //printValues(data);
  /*Serial.print(" -> ");
  Serial.println(power_difference);*/

};

void set_motors(int left_speed, int right_speed, int error){

  /*Serial.print(left_speed);
  Serial.print(" | ");
  Serial.print(right_speed);
  Serial.print(" = ");
  Serial.println(error);*/

  if(right_speed > 0) {
    digitalWrite(motorD1, HIGH);
    digitalWrite(motorD2, LOW);
  } else {
    right_speed = 0;
    if(error < -40) right_speed = 70;
    digitalWrite(motorD1, LOW);
    digitalWrite(motorD2, HIGH);
  }

  if(left_speed > 0) {
    digitalWrite(motorE1, LOW);
    digitalWrite(motorE2, HIGH);
  } else {
    left_speed = 0;
    if(error > 40) left_speed = 70;
    digitalWrite(motorE1, HIGH);
    digitalWrite(motorE2, LOW);
  }
  analogWrite(potenciaD, right_speed);
  analogWrite(potenciaE, left_speed);
}

double calcError(int sensors[NUM_SENSORS]) {

  int sensorSize = 7;

  float leftHandAux[4] = {
    -sensors[0] * weight4,
    -sensors[1] * weight3,
    -sensors[2] * weight2,
    -sensors[3] * weight1
  };

  float rightHandAux[4] = {
    sensors[7] * weight4,
    sensors[6] * weight3,
    sensors[5] * weight2,
    sensors[4] * weight1
  };

  float leftIntersection[7] = {
    leftHandAux[3],
    (leftHandAux[3] + leftHandAux[2]) / 2,
    leftHandAux[2],
    (leftHandAux[2] + leftHandAux[1]) / 2,
    leftHandAux[1],
    (leftHandAux[1] + leftHandAux[0]) / 2,
    leftHandAux[0]
  };

  float rightIntersection[7] = {
    rightHandAux[3],
    (rightHandAux[3] + rightHandAux[3]) / 2,
    rightHandAux[2],
    (rightHandAux[2] + rightHandAux[1]) / 2,
    rightHandAux[1],
    (rightHandAux[1] + rightHandAux[0]) / 2,
    rightHandAux[0]
  };

  int lessLeft = 1, lessRight = 1, lessLeftIdx = 0, lessRightIdx = 0, lessLeftAux = -9999, lessRightAux = 9999;
  float leftLessSum = 0, leftMajorSum = 0, rightLessSum = 0, rightMajorSum = 0;

  for (int i = 0; i < sensorSize; i++) {
    if (leftIntersection[i] > lessLeftAux) {
      lessLeftAux = leftIntersection[i];
      lessLeftIdx = i;
    }
  }
  for (int i = 0; i <= lessLeftIdx; i++) leftLessSum += leftIntersection[i];
  for (int i = lessLeftIdx; i < sensorSize; i++) leftMajorSum += leftIntersection[i];


  for (int i = 0; i < sensorSize; i++) {
    if (rightIntersection[i] < lessRightAux) {
      lessRightAux = rightIntersection[i];
      lessRightIdx = i;
    }
  }
  for (int i = 0; i <= lessRightIdx; i++) rightLessSum += rightIntersection[i];
  for (int i = lessRightIdx; i < sensorSize; i++) rightMajorSum += rightIntersection[i];

  float calcLeftSomatory = leftLessSum - leftMajorSum;
  float calcRightSomatory = rightLessSum - rightMajorSum;

  return ((calcLeftSomatory + calcRightSomatory) * 100) / 14849.5;
};

int processValues(int s[8]) {
  if(s[0] < VALID_THRESHOLD || s[1] < VALID_THRESHOLD || s[2] < VALID_THRESHOLD || s[3] < VALID_THRESHOLD ||
     s[4] < VALID_THRESHOLD || s[5] < VALID_THRESHOLD || s[6] < VALID_THRESHOLD || s[7] < VALID_THRESHOLD) {
    return 1;
  } else {
    return 0;
  }
}

void printValues(int s[8]) {
  Serial.print(s[0]);
  Serial.print(" - ");
  Serial.print(s[1]);
  Serial.print(" - ");
  Serial.print(s[2]);
  Serial.print(" - ");
  Serial.print(s[3]);
  Serial.print(" - ");
  Serial.print(s[4]);
  Serial.print(" - ");
  Serial.print(s[5]);
  Serial.print(" - ");
  Serial.print(s[6]);
  Serial.print(" - ");
  Serial.print(s[7]);
  Serial.print("\t\t");
};
