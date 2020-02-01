unsigned int s1, s2, s3, s4, s5, s6;
unsigned long sens_sum, trq_sum;

unsigned long trq1, trq2, trq3, trq4, trq5, trq6;
unsigned int  line_pos;
int speedinc;

int error, PID, proportional, integral, derivative, previousError;
float kp, kd, ki;

int pwmL = 3;
int dirPinL1 = 9;
int dirPinL2 = 6;
int pwmR = 5;
int dirPinR1 = 10;
int dirPinR2 = 11;
int basespeed;
int speedL, speedR;

void forward();
void pause();
void motorSpeed();
void setSensors();
void calculateStuff();
void calculatePID();
void motorDrive();
void serialStuff();

void setup() {
  Serial.begin(9600);
  pinMode(dirPinL1, OUTPUT);
  pinMode(dirPinL2, OUTPUT);
  pinMode(dirPinR1, OUTPUT);
  pinMode(dirPinR2, OUTPUT);
  pinMode(pwmL, OUTPUT);
  pinMode(pwmR, OUTPUT);
  digitalWrite(dirPinL1, LOW);
  digitalWrite(dirPinL2, LOW);
  digitalWrite(dirPinR1, LOW);
  digitalWrite(dirPinR2, LOW);
  s1 = analogRead(A0);
  s2 = analogRead(A1);
  s3 = analogRead(A2);
  s4 = analogRead(A3);
  s5 = analogRead(A4);
  s6 = analogRead(A5);
  delay(3000);
  forward();
}

void loop() {
  // put your main code here, to run repeatedly:
  s1 = analogRead(A0);
  s2 = analogRead(A1);
  s3 = analogRead(A2);
  s4 = analogRead(A3);
  s5 = analogRead(A4);
  s6 = analogRead(A5);
  //setSensors();
  calculateStuff();
  calculatePID();
  motorSpeed();
  motorDrive();
  //serialStuff();
  delay(10);
}

void calculateStuff() {
  sens_sum = (s1 / 10 + s2 / 10 + s3 / 10 + s4 / 10 + s5 / 10 + s6 / 10);
  trq1 = s1 / 10 * 10;
  trq2 = s2 / 10 * 20;
  trq3 = s3 / 10 * 30;
  trq4 = s4 / 10 * 40;
  trq5 = s5 / 10 * 50;
  trq6 = s6 / 10 * 60;
  trq_sum = (trq1 + trq2 + trq3 + trq4 + trq5 + trq6);
  line_pos = trq_sum / sens_sum;
}

void calculatePID() {
  kp = 1.5;
  ki = 0;
  kd = 0;
  error = 35 - line_pos;
  integral = integral + error;
  derivative = error - previousError;
  PID = kp * error; //+ ki * integral + kd * derivative;
  previousError = error;
}

void motorDrive() {
  analogWrite(pwmL, speedL);
  analogWrite(pwmR, speedR);
}

void motorSpeed() {
  basespeed = 70;
  speedL = basespeed - PID;
  speedR = basespeed + PID;
  if (speedL > 254) {
    speedL = 254;
  }
  if (speedL < 0) {
    speedL = 0;
  }
  if (speedR > 254) {
    speedR = 254;
  }
  if (speedR < 0) {
    speedR = 0;
  }
}

void serialStuff() {

  Serial.print(s1); Serial.print("|");
  Serial.print(s2); Serial.print("|");
  Serial.print(s3); Serial.print("|");
  Serial.print(s4); Serial.print("|");
  Serial.print(s5); Serial.print("|");
  Serial.print(s6); Serial.print("|");

  //Serial.println("");
  //Serial.print("last pos: ");
  //Serial.print(last_position);
  Serial.print(" | line pos: ");
  Serial.print(line_pos);
  Serial.print(" | err: ");
  Serial.print(error);
  Serial.print(" | PID: ");
  Serial.print(PID);
  Serial.print(" | Speeds A:");
  Serial.print(speedL);
  Serial.print(" B: ");
  Serial.println(speedR);

  delay(500);
}

void forward() {
  digitalWrite(dirPinL1, 0);
  digitalWrite(dirPinL2, 1);
  digitalWrite(dirPinR1, 1);
  digitalWrite(dirPinR2, 0);
}

void pause() {
  digitalWrite(dirPinL1, 0);
  digitalWrite(dirPinL2, 0);
  digitalWrite(dirPinR1, 0);
  digitalWrite(dirPinR2, 0);
}
