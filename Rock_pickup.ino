  #include <Servo.h> 
  int count_R=0;
  int count_L=0; 
  float     distance_IR=0;
  int    count_L_IR = 0;
  int     count_R_IR = 0;
  int bottomIR_R=A0;
  int bottomIR_L =A3;
  int frontIR_R=5;
  int frontIR_L=4;
  Servo myclaw;
  int clawpos=0;
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
    float angle=0;
    int leftright=0;
    int firstcheck=0;
    float distance_overall=0;
    int frontIR_L_read=0;
    int frontIR_R_read=0;
    int turncount=0;
  void setup() {
  Serial.begin(9600);
    pinMode(3,INPUT);
      pinMode(2,INPUT);
      servoLeft.attach(12);                     
    servoRight.attach(13);
    attachInterrupt(digitalPinToInterrupt(3),interruptFunctionR,RISING);
    attachInterrupt(digitalPinToInterrupt(2),interruptFunctionL,RISING);
  myclaw.attach(10);
  openclaw();
  }

void loop() {
bottomIR_R_read = analogRead(bottomIR_R);
bottomIR_L_read = analogRead(bottomIR_L);
frontIR_R_read=digitalRead(frontIR_R);
frontIR_L_read=digitalRead(frontIR_L);

 if(check==0){
  driveForward();
  check=1;
 }
  if(frontIR_R_read==1 && frontIR_L_read==1 && check==1){
    turncount=0;
    delay(500);
          servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1500);
     closeclaw(); 
     driveForward();
     check=2;
  }
  //pickup left
  if(frontIR_R_read==1 && frontIR_L_read==0 && check==1){
    distance_IR=0;
    count_L_IR = 0;
    count_R_IR = 0;
      while(frontIR_L_read==0){
frontIR_R_read=digitalRead(frontIR_R);
frontIR_L_read=digitalRead(frontIR_L);
delay(50);
  turnAngle(1,0);
  turncount++;
delay(50);
  
  } 
  }

  if(frontIR_R_read==0 && frontIR_L_read==1){
    turncount=turncount*3.5;
    turnAngle(turncount,1);
    driveForward();
    turncount=0;
  }
    check=2;
  }
 //pickup right
  if(frontIR_R_read==0 && frontIR_L_read==1 && check==1){
    distance_IR=0;
    count_L_IR = 0;
    count_R_IR = 0; 
      while(frontIR_R_read==0){
frontIR_R_read=digitalRead(frontIR_R);
frontIR_L_read=digitalRead(frontIR_L);
delay(50);
  turnAngle(1,1);
  turncount++;
delay(50);
  
  } 
  if(frontIR_R_read==1 && frontIR_L_read==0){
    turncount=turncount*3.5;
    turnAngle(turncount,0);
    driveForward();
    turncount=0;
    delay(500);
          servoLeft.writeMicroseconds(1500);
      servoRight.writeMicroseconds(1500);
     closeclaw(); 
     driveForward();
  }
    check=2;
  }

}
void interruptFunctionR()
{
  count_R++;
  count_R_IR++;
  distance = ((count_R+count_L)/2)*constant ; 
  distance_overall=((count_R+count_L)/2)*constant ;
  distance_IR=((count_R_IR+count_L_IR)/2)*constant;

}
void interruptFunctionL()
{
  count_L++;
  count_L_IR++;
  distance = ((count_R+count_L)/2)*constant ; 
  distance_overall=((count_R+count_L)/2)*constant ;
    distance_IR=((count_R_IR+count_L_IR)/2)*constant;
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
void driveForward(){
  for(int speed = 0; speed <= 70; speed += 2) 
  {
    servoLeft.writeMicroseconds(1500+speed);  
    servoRight.writeMicroseconds(1500-speed); 
    delay(20);                                
  }
}
void openclaw(){
  if(clawpos > 90){
    for(clawpos = 180; clawpos>= 90; clawpos--){
      myclaw.write(clawpos);
      delay(15);
    }
  }
 else{
      for(clawpos = 0; clawpos <90; clawpos ++){
        myclaw.write(clawpos);
        delay(15);
      }
    }
  }
void closeclaw(){
  for(clawpos =90; clawpos>= 0 ; clawpos-- ){
    myclaw.write(clawpos);
    delay(15);
  }
}
