//works with spiky bot

// file setup
#include <Servo.h>
#include <math.h>
int pos = 0;
Servo headServo;
int count_R = 0;
int count_L = 0;
Servo servoLeft;   // Declare left servor
Servo servoRight;  // Declare right servo
float constant = 0.00641025;
float distance = 0;
float turn = 0.34;

//encoder setup and functions
void encoderSetup() {
  pinMode(3, INPUT);
  pinMode(2, INPUT);
  Serial.begin(9600);
  servoLeft.attach(12);
  servoRight.attach(13);
  attachInterrupt(digitalPinToInterrupt(3), interruptFunctionR, RISING);
  attachInterrupt(digitalPinToInterrupt(2), interruptFunctionL, RISING);
}
void interruptFunctionR()
{
  count_R++;
  distance = ((count_R + count_L) / 2) * constant ;
}
void interruptFunctionL()
{
  count_L++;
  distance = ((count_R + count_L) / 2) * constant ;
}

void turnAngle(float angle, int leftRight) {
  distance = 0;
  count_L = 0;
  count_R = 0;
  while (true) {
    Serial.println("angle:");
    Serial.println(angle);
    Serial.println("Count_Right:");
    Serial.println(count_R);
    Serial.println("Count_Left:");
    Serial.println(count_L);
    Serial.println("Distance:");
    Serial.println(distance);
    if (distance < (turn / (360 / angle) && leftRight == 1)) {
      servoLeft.writeMicroseconds(1530);
      servoRight.writeMicroseconds(1530);
    }
    if (distance < (turn / (360 / angle) && leftRight == 0)) {
      servoLeft.writeMicroseconds(1470);
      servoRight.writeMicroseconds(1470);
    }
    if (distance > (turn / (360 / angle))) {
      servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1500);
      distance = 0;
      count_L = 0;
      count_R = 0;
      return;
    }
  }
}

void moveDistance(float dist) {
  distance = 0;
  count_L = 0;
  count_R = 0;
  int eksdee = 0;
  if (dist < 0) {
    dist *= -1;
    eksdee = 1;
  }
  while (true) {
    Serial.println("dist:");
    Serial.println(dist);
    Serial.println("Count_Right:");
    Serial.println(count_R);
    Serial.println("Count_Left:");
    Serial.println(count_L);
    Serial.println("Distance:");
    Serial.println(distance);
    if (distance < dist && eksdee == 0) {
      servoLeft.writeMicroseconds(1530);
      servoRight.writeMicroseconds(1470);
    }
    if (distance < dist && eksdee == 1) {
      servoLeft.writeMicroseconds(1470);
      servoRight.writeMicroseconds(1530);
    }
    if (distance >= dist) {
      servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1500);
      distance = 0;
      count_L = 0;
      count_R = 0;
      return;
    }
  }
}



// ultrasound setup and functions
void ultrasoundSetup() {  // put in setup of the main code
  headServo.attach(11);
  headTurnLeft();
}

float ping() {  // returns the distance in cm
  long duration;
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  delayMicroseconds(2);
  digitalWrite(9, HIGH);
  delayMicroseconds(5);
  digitalWrite(9, LOW);
  pinMode(9, INPUT);
  duration = pulseIn(9, HIGH);
  return ((float) duration) / 38;
}

void headTurn(int deg) {
  if (pos > deg) {
    for (; pos > deg; pos--) {
      headServo.write(pos);
      delay(10);
    }
  }
  else {
    for (; pos < deg ; pos++) {
      headServo.write(pos);
      delay(10);

    }
  }
}

void headTurnLeft() {  // turns the head left
  for (; pos <= 180 ; pos++) {
    headServo.write(pos);
    delay(10);
  }
}

void headTurnRight() {
  for (; pos >= 0 ; pos--) {
    headServo.write(pos);
    delay(10);
  }
}

int sweep() {
  while (true) {
    headTurnLeft();
    if (ping() < 40) {
      return 0;
    }
    headTurn(165);
    if (ping() < 40) {
      return 0;
    }
    headTurn(150);
    if (ping() < 40) {
      return 0;
    }
    headTurn(135);
    if (ping() < 40) {
      return 0;
    }
    headTurn(120);
    if (ping() < 40) {
      return 0;
    }
    headTurn(105);
    if (ping() < 40) {
      return 0;
    }
    headTurn(90);
    if (ping() < 40) {
      return 0;
    }
    headTurn(75);
    if (ping() < 40) {
      return 0;
    }
    headTurn(60);
    if (ping() < 40) {
      return 0;
    }
    headTurn(45);
    if (ping() < 40) {
      return 0;
    }
    headTurn(30);
    if (ping() < 40) {
      return 0;
    }
    headTurn(15);
    if (ping() < 40) {
      return 0;
    }
    headTurn(0);
    if (ping() < 40) {
      return 0;
    }
    headTurn(15);
    if (ping() < 40) {
      return 0;
    }
    headTurn(30);
    if (ping() < 40) {
      return 0;
    }
    headTurn(45);
    if (ping() < 40) {
      return 0;
    }
    headTurn(60);
    if (ping() < 40) {
      return 0;
    }
    headTurn(75);
    if (ping() < 40) {
      return 0;
    }
    headTurn(90);
    if (ping() < 40) {
      return 0;
    }
    headTurn(105);
    if (ping() < 40) {
      return 0;
    }
    headTurn(120);
    if (ping() < 40) {
      return 0;
    }
    headTurn(135);
    if (ping() < 40) {
      return 0;
    }
    headTurn(150);
    if (ping() < 40) {
      return 0;
    }
    headTurn(165);
    if (ping() < 40) {
      return 0;
    }
  }
}

float rad2deg(float rad) {
  return rad * 57.3;
}

// mountain detection functions
int isPerpendicular() { // checking if the robot is perpendicular to wall, turns more if it is
  int bruh = pos;
  headTurn(70);
  delay(100);
  float right = ping();
  headTurn(110);
  delay(100);
  float left = ping();
  if (right - left < 5 and right - left > -5) {
    if (right > left) {
      headTurn(100);
      turnAngle(30, 1);
    }
    else if (left > right) {
      headTurn(80);
      turnAngle(30, 0);
    }
    return 1;
  }
  else {
    headTurn(bruh);
    return 0;
  }
}

void findParallel() { // turn the robot parallel to the wall
  int leftRight;
  //turn right if head facing left, vice versa if facing right
  isPerpendicular();
  if (pos <= 90) { //turn left
    headTurnRight();
    turnAngle(30, 0);
    moveDistance(-0.1);
    float before = ping();
    moveDistance(0.1);
    float opposite = ping() - before;
    if (opposite > 10) {
      return;
    }
    double angle = asin((double) opposite / 10); // approximate angle between robot trajectory and wall
    Serial.println("angel");
    Serial.println(angle);
    if (angle < 0) {
      angle *= -1;
      leftRight = 0;
    }
    else {
      leftRight = 1;
    }
    turnAngle(rad2deg(angle), leftRight);
  }
  else { //turn right
    headTurnLeft();
    turnAngle(30, 1);
    moveDistance(-0.1);
    float before = ping();
    moveDistance(0.1);
    float opposite = ping() - before;
    if (opposite > 10) {
      return;
    }
    double angle = asin((double) opposite / 10); // approximate angle between robot trajectory and wall
    Serial.println("angel");
    Serial.println(opposite);
    if (angle < 0) {
      angle *= -1;
      leftRight = 1;
    }
    else {
      leftRight = 0;
    }
    turnAngle(rad2deg(angle), leftRight);
  }
}

void parallelMove() { // move after turning parallel, corrects course and stops when the mountain is cleared
  int bruh = pos;
  headTurn(90);
  delay(100);
  if (ping() < 70) {
    return;
  }
  headTurn(bruh);
  float start = ping();
  servoLeft.writeMicroseconds(1530);
  servoRight.writeMicroseconds(1470);
  while (true) {
    delay(500);
    if (start - ping() < -10) {
      servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1500);
      return;
    }
    else if (start - ping() < -3) {
      servoRight.writeMicroseconds(1400);
      delay(150);
      servoRight.writeMicroseconds(1470);
    }
    else if (start - ping() > 3) {
      servoLeft.writeMicroseconds(1600);
      delay(150);
      servoLeft.writeMicroseconds(1530);
    }
  }
}



void setup() {
  encoderSetup();
  ultrasoundSetup();
  Serial.begin(9600);

  // move forward
  servoLeft.writeMicroseconds(1530);
  servoRight.writeMicroseconds(1470);
  sweep();
  // stop when detecting a mountain
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
  // turn to approximately parallel
  findParallel();
  // move until mountain is cleared
  parallelMove();

}

void loop() {}
