#include "QTRSensors.h"

#define VALID_THRESHOLD         200
#define NUM_SENSORS             8
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             9

int counter = 0;
int potenciaE = 3;
int potenciaD = 6;
int motorE1 = 2;
int motorE2 = 4;
int motorD1 = 7;
int motorD2 = 5;
int flagStart = 1;

int led1 = 11;
int led2 = 10;

int data[NUM_SENSORS],
    last_proportional,
    integral,
    power_difference,
    proportional,
    derivative;

unsigned int sensorValues[NUM_SENSORS];

long expectedTime = 19500;
int initTime = 0;
int currentTime = 0;

const int weight1 = 1;
const int weight2 = 2;
const int weight3 = 3;
const int weight4 = 4;

// VARIÁVEIS DE VELOCIDADE
int max = 100;
int maxInverse = 50;
int errorBound = 5;
const int maxVel = 250;

QTRSensorsAnalog qtra((unsigned char[]) {
  A0, A1, A2, A3, A4, A5, A6, A7
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

double Output;

void setup() {

  pinMode(motorE1, OUTPUT);
  pinMode(motorE2, OUTPUT);
  pinMode(motorD1, OUTPUT);
  pinMode(motorD2, OUTPUT);
  pinMode(potenciaE, OUTPUT);
  pinMode(potenciaD, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  digitalWrite(motorD1, HIGH);
  digitalWrite(motorD2, LOW);
  digitalWrite(motorE1, LOW);
  digitalWrite(motorE2, HIGH);

  qtra.emittersOn();
  qtra.calibrate();

  led(true);
  for(counter = 0; counter < 80; counter++) {
    qtra.calibrate();
    delay(10);
  }
  led(false);

  delay(3000);

  //Serial.begin(9600);
  
}

void led(int status){
  if(status == 1){
    digitalWrite(led1, LOW);
    digitalWrite(led2, HIGH);
  }else{
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
  }
}

void loop() {

  qtra.read(sensorValues, QTR_EMITTERS_OFF);

  data[0] = 900 - sensorValues[0];
  data[1] = 900 - sensorValues[1];
  data[2] = 900 - sensorValues[2];
  data[3] = 900 - sensorValues[3];
  data[4] = 900 - sensorValues[4];
  data[5] = 900 - sensorValues[5];
  data[6] = 900 - sensorValues[6];
  data[7] = 900 - sensorValues[7];

  /*for(int i = 0; i < 8; i++) {
    Serial.print(data[i]);
    Serial.print(" - ");
  }
  Serial.println("");*/
             
  if(processValues(data) == 1) {

    Output = calcError(data);

    if(max < maxVel) max++;
    
    // PID
    proportional = map(-Output, -max, max, 1000, 6000) - 3500;
    derivative = proportional - last_proportional;
    integral += proportional;

    last_proportional = proportional;
    power_difference = proportional / 7 + integral / 10000 + derivative * 3;
  }

  if(Output <= 8 && Output >= -8){
    setPower(max, max);
  }else{
    if(power_difference > max) power_difference = max;
    else if(power_difference < -max) power_difference = -max;

    if(flagStart == 1){
      if(power_difference < 0) set_motors(max + power_difference, max, Output);
      else set_motors(max, max - power_difference, Output);
    }else{
      setPower(0, 0);
    }
  }

  if(initTime == 0) {
    initTime = millis();
    currentTime = 1;
    led(true);
  }

  if(currentTime != 0){
    currentTime = (millis() - initTime);
    if(currentTime == 0){
      currentTime = 1;
    }
    if(expectedTime <= currentTime){
      setPower(0, 0);
      flagStart = 0;
      led(false);
    }
  }

};

void set_motors(int left_speed, int right_speed, int error){

  if(right_speed > 0) {
    digitalWrite(motorD1, HIGH);
    digitalWrite(motorD2, LOW);
  } else {
    right_speed = 0;
    if(error < -errorBound) right_speed = maxInverse;
    digitalWrite(motorD1, LOW);
    digitalWrite(motorD2, HIGH);
  }

  if(left_speed > 0) {
    digitalWrite(motorE1, LOW);
    digitalWrite(motorE2, HIGH);
  } else {
    left_speed = 0;
    if(error > errorBound) left_speed = maxInverse;
    digitalWrite(motorE1, HIGH);
    digitalWrite(motorE2, LOW);
  }

  setPower(right_speed, left_speed);

}

void setPower(int right_speed, int left_speed) {
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

  int lessLeftIdx = 0,
      lessRightIdx = 0,
      lessLeftAux = -9999,
      lessRightAux = 9999;
  float leftLessSum = 0,
        leftMajorSum = 0,
        rightLessSum = 0,
        rightMajorSum = 0;

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
    if((s[7] < VALID_THRESHOLD || s[6] < VALID_THRESHOLD) && (s[0] < VALID_THRESHOLD || s[1] < VALID_THRESHOLD)) {
      return 0;
    }
    return 1;
  } else {
    return 0;
  }
}
