#include <QTRSensors.h>
#include <Servo.h>

#define NUM_SENSORS             6
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             9

int potenciaE = 3;
int potenciaD = 6;
int motorE1 = 2;
int motorE2 = 4;
int motorD1 = 5;
int motorD2 = 7;
int data[6];
int speedInitialD = 80;
int speedInitialE = 80;

const int weight1 = 1;
const int weight2 = 2;
const int weight3 = 3;

double error = 0;
double valueSensor = 0;
double lastValueSensor = 0;
double
  kP = 2,
  kI = 0,
  kD = 0;
double P = 0, I = 0, D = 0;
double PID = 0;
double setPoint = 0;
long lastValueTime = 0;

unsigned int sensorValues[NUM_SENSORS];

QTRSensorsAnalog qtra((unsigned char[]) {
  0, 1, 2, 3, 4, 5
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

Servo servoMotor;

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
  servoMotor.attach(8);
  servoMotor.write(90);
  qtra.emittersOn();
  analogWrite(potenciaD, speedInitialD);
  analogWrite(potenciaD, speedInitialE);
  delay(1000);
}


void loop() {
  qtra.calibrate();
  qtra.read(sensorValues, QTR_EMITTERS_OFF);

  data[0] = sensorValues[3];
  data[1] = sensorValues[5];
  data[2] = sensorValues[4];
  data[3] = sensorValues[1];
  data[4] = sensorValues[0];
  data[5] = sensorValues[2];

  printValues(data);

  if(data[0] < 100 || data[1] < 100 || data[2] < 100 || data[3] < 100 || data[4] < 100 || data[5] < 100 ) {

    valueSensor = calcError(data);

  }

  error = setPoint - valueSensor;
  float deltaTime = (millis() - lastValueSensor) / 1000;
  lastValueTime = millis();
  P = error * kP;
  I += (error * kI) * (deltaTime / 1000);
  D = (lastValueSensor - valueSensor) * kD / deltaTime;
  lastValueSensor = valueSensor;
  PID = P + I + D;

  setMotor(PID);

};

void printValues(int s[6]) {
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
  Serial.println("\t\t");
};

void setMotor(double error) {

  int controlSpeedInitialD;
  int controlSpeedInitialE;

  controlSpeedInitialD = 50 - error;
  controlSpeedInitialE = 50 + error;

  if(controlSpeedInitialD > 80) { controlSpeedInitialD = 80; }
  if(controlSpeedInitialE > 80) { controlSpeedInitialE = 80; }

  if(controlSpeedInitialD < 0 && error > 100) {
    digitalWrite(motorD1, LOW);
    digitalWrite(motorD2, HIGH);
    controlSpeedInitialD = 20;
  } else {
    if(controlSpeedInitialD < 0 && error <= 100) controlSpeedInitialD = 0;
    digitalWrite(motorD1, HIGH);
    digitalWrite(motorD2, LOW);
  }

  if(controlSpeedInitialE < 0 && error < -100) {
    digitalWrite(motorE1, HIGH);
    digitalWrite(motorE2, LOW);
    controlSpeedInitialE = 20;
  } else {
    if(controlSpeedInitialE < 0 && error >= -100) controlSpeedInitialE = 0;
    digitalWrite(motorE1, LOW);
    digitalWrite(motorE2, HIGH);
  }

  analogWrite(potenciaD, controlSpeedInitialD);
  analogWrite(potenciaE, controlSpeedInitialE);
};

double calcError(int sensors[NUM_SENSORS]) {

  int sensorSize = 5;

  float leftHandAux[3] = {
    -sensors[0] * weight3,
    -sensors[1] * weight2,
    -sensors[2] * weight1
  };

  float rightHandAux[3] = {
    sensors[5] * weight3,
    sensors[4] * weight2,
    sensors[3] * weight1
  };

  float leftIntersection[5] = {
    leftHandAux[2],
    (leftHandAux[2] + leftHandAux[1]) / 2,
    leftHandAux[1],
    (leftHandAux[1] + leftHandAux[0]) / 2,
    leftHandAux[0]
  };

  float rightIntersection[5] = {
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
