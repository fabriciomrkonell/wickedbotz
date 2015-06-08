#include <QTRSensors.h>
#include <Servo.h>

#define VALID_THRESHOLD         200
#define NUM_SENSORS             8
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             13

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

int sensorLeft = 9;
int lines = 0;
int linesAux = 0;
int linesLastAux = 0;

int data[NUM_SENSORS],
    last_proportional,
    integral,
    power_difference,
    proportional,
    derivative;

unsigned int sensorValues[NUM_SENSORS];

int expectedTime = 8500;
int initTime = 0;
int currentTime = 0;
int timeIgnoreLine = 0;
int timeIgnored = 0;

int timeTemp = 0;

const int weight1 = 1;
const int weight2 = 2;
const int weight3 = 3;
const int weight4 = 4;

// VARIÁVEIS DE VELOCIDADE
int _max = 220;
int max = _max;
int maxInverse = 110;
int errorBound = 3;
int ignoreLine = 0;

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
  pinMode(sensorLeft, INPUT);

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

  delay(2000);
}

void win(){
  led(true);
  delay(500);
  led(false);
  delay(500);
  led(true);
  delay(500);
  led(false);
  delay(500);
  led(true);
  delay(500);
  led(false);
  delay(500);
  led(true);
  delay(500);
  led(false);
  delay(500);
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

int isLine(){
  linesLastAux = linesAux;
  linesAux = !digitalRead(sensorLeft);

  if(linesAux == 1){
    if(linesLastAux != 1 && ignoreLine == 0){
      if(lines == 0){
        lines = 1;
      }else{
        if(lines >= 100){
          lines = (lines / 100) + 1;
        }else{
          lines = lines + 1;
        }
      }
    }
  }

  if((lines == 1) && (ignoreLine == 0)){
    lines = lines * 100;
    max = 100;
    ignoreLine = 1;
    timeIgnoreLine = millis();
    timeIgnored = 900;
    led(true);
  }else if((lines == 2) && (ignoreLine == 0)){
    lines = lines * 100;
    max = 65;
    ignoreLine = 1;
    timeIgnoreLine = millis();
    timeIgnored = 8300;
    led(true);
  }else if((lines == 3) && (ignoreLine == 0)){
    lines = lines * 100;
    max = 65;
    ignoreLine = 1;
    timeIgnoreLine = millis();
    timeIgnored = 1150;
    led(true);
  }else if((lines == 4) && (ignoreLine == 0)){
    lines = lines * 100;
    max = 100;
    ignoreLine = 1;
    timeIgnoreLine = millis();
    timeIgnored = 1150;
    led(true);
  }else if((lines == 5) && (ignoreLine == 0)){
    lines = lines * 100;
    max = 100;
    ignoreLine = 1;
    timeIgnoreLine = millis();
    timeIgnored = 5000;
    led(true);

    if(initTime == 0) {
      initTime = millis();
      currentTime = 1;
    }
  }

  if((timeIgnoreLine + timeIgnored) != 0){
    if((timeIgnoreLine + timeIgnored) <= (millis())){
      ignoreLine = 0;
      led(false);
    }
  }

  if((timeIgnoreLine + 600) != 0){
    if((timeIgnoreLine + 600) <= (millis())){
      max = _max;
    }
  }


}

void calculatePower(){

  if(processValues(data) == 1) {

    Output = calcError(data);

    proportional = map(-Output, -160, 160, 1000, 6000) - 3500;
    derivative = proportional - last_proportional;
    integral += proportional;

    last_proportional = proportional;
    power_difference = proportional /  7 + integral / 10000 + derivative * 3;
  }

  if((Output <= 8 && Output >= -8) && (flagStart == 1)){
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

  isLine();

  calculatePower();

  if(currentTime != 0){
    currentTime = (millis() - initTime);
    if(currentTime == 0){
      currentTime = 1;
    }
    if(expectedTime <= currentTime){
      setPower(0, 0);
      flagStart = 0;
      win();
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
