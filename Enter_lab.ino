
#include <Servo.h> 
int count_R=0;
int count_L=0; 
int bottomIR_R = A0;
int bottomIR_L = A3;
  Servo servoLeft;   // Declare left servor
  Servo servoRight;  // Declare right servo
  float constant = 0.00641025;
  float distance =0 ; 
  int turnright=1550;
  int turnleft=1450;  
  float turn=0.34;
  int turninput=90; 
  int turnside=0;
  int check=0;
  int bottomIR_R_read=0;
  int bottomIR_L_read=0;
  int angle=0;
  int leftright=0;
  int firstcheck=0;
  float distance_overall=0;
void setup()
{
  pinMode(3,INPUT);
    pinMode(2,INPUT);
  Serial.begin(9600); 
    servoLeft.attach(12);                     
  servoRight.attach(13);
  attachInterrupt(digitalPinToInterrupt(3),interruptFunctionR,RISING);
  attachInterrupt(digitalPinToInterrupt(2),interruptFunctionL,RISING);
}
 
void loop()
{ 
  bottomIR_R_read = analogRead(bottomIR_R);
  bottomIR_L_read = analogRead(bottomIR_L);
 Serial.print("Right: ");
 Serial.print(bottomIR_R_read);
 Serial.print(" || Left: ");
 Serial.println(analogRead(bottomIR_L_read ));

 if(check==0){
  driveForward();
  check=1;
 }
 if(bottomIR_R_read <400 && bottomIR_L_read){
  servoLeft.writeMicroseconds(1500);  
  servoRight.writeMicroseconds(1500);
  angle=90;
  leftright=0;
  
  turnAngle(angle,leftright);
  check=0;
  if(firstcheck=0){
    distance_overall=0;   
  }
 }
 if(bottomIR_R_read <400){
    servoLeft.writeMicroseconds(1500);  
  servoRight.writeMicroseconds(1500);
 }
 if(bottomIR_L_read <400){
    servoLeft.writeMicroseconds(1500);  
  servoRight.writeMicroseconds(1500);
 }

 
  
}
void interruptFunctionR()
{
  count_R++;
  distance = ((count_R+count_L)/2)*constant ; 
  distance_overall=((count_R+count_L)/2)*constant ;
}
void interruptFunctionL()
{
  count_L++;
  distance = ((count_R+count_L)/2)*constant ; 
  distance_overall=((count_R+count_L)/2)*constant ;
}
void driveForward(){
  for(int speed = 0; speed <= 100; speed += 2) 
  {
    servoLeft.writeMicroseconds(1500+speed);  
    servoRight.writeMicroseconds(1500-speed); 
    delay(20);                                
  }
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
      servoLeft.writeMicroseconds(turnright);
      servoRight.writeMicroseconds(turnright);
    }
    if (distance < (turn / (360 / angle) && leftRight == 0)) {
      servoLeft.writeMicroseconds(turnleft);
      servoRight.writeMicroseconds(turnleft);
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

        
