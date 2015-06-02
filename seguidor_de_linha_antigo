float Kp = 25, Ki = 0.6, Kd = 2;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0,
      previous_I=0;
int sensor[5] = { 0, 0, 0, 0, 0 };
int initial_motor_speed = 80;

const int motorA = 2;
const int motorB = 4;
const int motorA_PWM = 3;
const int motorB_PWM = 5;

int line;
int outline;

const int ss1 = 8;
const int ss2 = 9;
const int ss3 = 10;
const int ss4 = 11;
const int ss5 = 12;

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

void setup(){
  Serial.begin(9600);
  pinMode(motorA,OUTPUT);
  pinMode(motorB,OUTPUT);
  pinMode(motorA_PWM,OUTPUT);
  pinMode(motorB_PWM,OUTPUT);

  pinMode(ss1, INPUT);
  pinMode(ss2, INPUT);
  pinMode(ss3, INPUT);
  pinMode(ss4, INPUT);
  pinMode(ss5, INPUT);

  delay(500);
}

void loop(){
  read_sensor_values();
  calculate_pid();
  motor_control();
}

void read_sensor_values(){

  line = 1;
  outline = 0;

  sensor[0] = digitalRead(ss1);
  sensor[1] = digitalRead(ss2);
  sensor[2] = digitalRead(ss3);
  sensor[3] = digitalRead(ss4);
  sensor[4] = digitalRead(ss5);

  if((sensor[0]==outline)&&(sensor[1]==outline)&&(sensor[2]==outline)&&(sensor[3]==outline)&&(sensor[4]==line)){
    error = 4;
  }else if((sensor[0]==line)&&(sensor[1]==outline)&&(sensor[2]==outline)&&(sensor[4]==outline)&&(sensor[4]==outline)){
    error = -4;
  }

  if((sensor[0]==outline)&&(sensor[1]==outline)&&(sensor[2]==outline)&&(sensor[3]==line)&&(sensor[4]==outline)){
    error = 2;
  }else if((sensor[0]==outline)&&(sensor[1]==line)&&(sensor[2]==outline)&&(sensor[3]==outline)&&(sensor[4]==outline)){
    error = -2;
  }

  if((sensor[0]==outline)&&(sensor[1]==outline)&&(sensor[2]==line)&&(sensor[3]==line)&&(sensor[4]==outline)){
    error = 1;
  }else if((sensor[0]==outline)&&(sensor[1]==line)&&(sensor[2]==line)&&(sensor[3]==outline)&&(sensor[4]==outline)){
    error = -1;
  }

  if((sensor[1]==outline)&&(sensor[2]==line)&&(sensor[3]==outline)){
    error = 0;
  }

  Serial.println(error);

}

void calculate_pid(){
  P = error;
  I = I + previous_I;
  D = error-previous_error;

  PID_value = (Kp*P) + (Ki*I) + (Kd*D);

  previous_I=I;
  previous_error=error;
}

void motor_control(){
  int left_motor_speed;
  int right_motor_speed;

  left_motor_speed = initial_motor_speed + PID_value;
  right_motor_speed = initial_motor_speed - PID_value;

  if(left_motor_speed < 0)
    left_motor_speed = 0;
  if(right_motor_speed < 0)
    right_motor_speed = 0;

  analogWrite(motorA_PWM,left_motor_speed);
  analogWrite(motorB_PWM,right_motor_speed);

  digitalWrite(motorA,LOW);
  digitalWrite(motorB,LOW);
}

void stopMotors(){
  analogWrite(motorA_PWM,0);
  analogWrite(motorB_PWM,0);
}
