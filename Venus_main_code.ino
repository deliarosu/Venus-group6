/* Main code for project Venus group 6 */
#include <Servo.h>                           // Include servo library
#include <math.h>

//Variables and defines
  Servo servoLeft;   // Declare left servo
  Servo servoRight;  // Declare right servo
  Servo myclaw;      // Declare claw servo
  Servo myservo;     // Declare ultrasound servo

  int no_move = 1500;
  int right_spin_forward = 1400;
  int right_spin_backward = 1600;
  int left_spin_forward = 1600;
  int left_spin_backward = 1400;
  int clawpos = 0;
  int done = 0;
  int detected = 0;
  int detectedRock = 0;
  int detectedMountain = 0;
  int detectedCliff = 0;
  int detectedLab = 0;
  int count_R = 0;
  int count_L = 0; 
  float constant = 0.00641025;
  float distance = 0;  
  float turn = 0.34;
  int turninput = 90; 
  int turnside = 0;
  int cen = 85, pos = 0;
  int headLeft = 180;
  int headRight = 0;
  long duration = 0; 
  long distanceMountain = 0;
  long initv = 0;
  bool mountain = false;
  float circum = 34.56;
  int bottomIR_R = A0;
  int bottomIR_L = A3;
  float constant = 0.00641025;
  float trn=0.14;
  int check = 0;
  int check1 = 0;
  int leftright=0;
  int firstcheck=0;
  float save_dist = 0;
  float dist_capt = 0;
  int bottomIR_R_read=0;
  int bottomIR_L_read=0;
  uint8_t turn_counter = 0;       
  float set_distance = 0.03;
  int measured = 0;
  int curr_right = 1500;
  int curr_left = 1500;
  int turnright = 1530;
  int turnleft = 1470;  
  int count_R = 0; int save_R = 0;
  int count_L = 0; int save_L = 0;
  int count_Right = 0; int count_Left = 0;
  float distanceCliff = 0; float turn_dist = 0;
  int m = 0;
  int firstcheck=0;
  float distance_overall=0;
  int frontIR_L_read=0;
  int frontIR_R_read=0;
  int turncount=0;

void setup() {  // put your setup code here, to run once:
  Serial.begin(9600);                        // Set Baudrate for proper communication
  servoLeft.attach(12);                      // Attach left signal to pin 12
  servoRight.attach(13);                     // Attach right signal to pin 13
  myclaw.attach(10);                         // Attach claw signal to pin 10
  myservo.attach(11);                        // Attach head signal to pin 11
  pinMode(3,INPUT);
  pinMode(2,INPUT);
  attachInterrupt(digitalPinToInterrupt(3),interruptFunctionR,RISING);
  attachInterrupt(digitalPinToInterrupt(2),interruptFunctionL,RISING);
}

void loop() { // put your main code here, to run repeatedly:
  //driving
  servoLeft.writeMicroseconds(1550); 
  servoRight.writeMicroseconds(1450);
  //detect object
  IRdetection();
  USdetection();
  if(detected){
    go('s');
    //case rock
    if(detectedRock){
      //pick up rock and drive to saved lab location
        //rotate till both IR sensors detect rock, then rock is in front of car
        openClaw();                   //make sure claw is fully open
        go('f');                      //drive to location of rock so that it's between the claws
        delay(1000);
        go('s');
        closeClaw();                  //close claws and drive away with rock to lab
    }
    //case mountain
    if(detectedMountain){
      //turn and drive around it
      parallelMove();
      detected = 0;
      detectedMountain = 0;
    }
    //case cliff
    if(detectedCliff){
      avoid_cliff(m);
    }
    //case lab
    if(detectedLab){
      //drop rock, drive backwards and continue searching
      go('f');
      delay(2500);
      go('s');
      openClaw();
      closeClaw();
      go('b');
      delay(2500);
      go('t');
    }
  }
  if(done){
    disableServos();
    exit(0);
  }
}

void forwardToStill(){
  for(int speed = 100; speed >= 0; speed -= 2){
    servoLeft.writeMicroseconds(1500+speed);  
    servoRight.writeMicroseconds(1500-speed); 
    delay(20);                                
  }
}

void driveForward(){
  for(int speed = 0; speed <= 100; speed += 2){
    servoLeft.writeMicroseconds(1500+speed);  
    servoRight.writeMicroseconds(1500-speed); 
    delay(20);                                
  }
}

void backwardToStill(){
  for(int speed = 100; speed >= 0; speed -= 2){
    servoLeft.writeMicroseconds(1500-speed);
    servoRight.writeMicroseconds(1500+speed);  
    delay(20);                                 
  }
}

void driveBackward(){
  for(int speed = 0; speed <= 100; speed += 2){
    servoLeft.writeMicroseconds(1500-speed); 
    servoRight.writeMicroseconds(1500+speed); 
    delay(20);                                 
  }
}

void turnLeft(void){
  for(int speed = 0; speed <= 100; speed += 4){
    servoLeft.writeMicroseconds(1500-speed); 
    servoRight.writeMicroseconds(1500-speed); 
    delay(20);                                 
  }
  for(int speed = 0; speed <= 100; speed += 4){
    servoLeft.writeMicroseconds(1400+speed); 
    servoRight.writeMicroseconds(1400+speed); 
    delay(20);                                 
  }
}

void turnRight(void){
  for(int speed = 0; speed <= 100; speed += 4){
    servoLeft.writeMicroseconds(1500+speed); 
    servoRight.writeMicroseconds(1500+speed); 
    delay(20);                                 
  }
  for(int speed = 0; speed <= 100; speed += 4){
    servoLeft.writeMicroseconds(1600-speed); 
    servoRight.writeMicroseconds(1600-speed); 
    delay(20);                                 
  }
}

void openClaw(){
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

void closeClaw(){
  for(clawpos =0; clawpos<= 180 ; clawpos++ ){
    myclaw.write(clawpos);
    delay(15);
  }
}

long ping() {
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  delayMicroseconds(2);
  digitalWrite(9, HIGH);
  delayMicroseconds(5);
  digitalWrite(9, LOW);
  pinMode(9, INPUT);
  duration = pulseIn(9, HIGH);
  distanceMountain = duration/38;
  Serial.println(distanceMountain);
  return distanceMountain;
}

void headTurnRight() {
  for (; pos >= headRight; pos--) {
    myservo.write(pos);
    delay(15);
  }
}

void headTurnLeft() {
  for (; pos <= headLeft ; pos++) {
    myservo.write(pos);
    delay(15);
  }
}

void headCenter() {
  if (pos > cen) {
    for (; pos > cen; pos--) {
      myservo.write(pos);
      delay(15);
    }
  }
  else {
    for (; pos < cen ; pos++) {
      myservo.write(pos);
      delay(15);

    }
  }
}

void headTurn(int deg) {
  if (pos > deg) {
    for (; pos > deg; pos--) {
      myservo.writeMicroseconds(pos);
      delay(15);
    }
  }
  else {
    for (; pos < deg ; pos++) {
      myservo.writeMicroseconds(pos);
      delay(15);
    }
  }
}

void turnRightShort() {
  servoLeft.writeMicroseconds(1470);
  servoRight.writeMicroseconds(1470);
  headTurn(pos + 15);
  delay(100);
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
}

void turnLeftShort() {
  servoLeft.writeMicroseconds(1530);
  servoRight.writeMicroseconds(1530);
  headTurn(pos - 15);
  delay(100);
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
}

void sweep() {  // returns 0 if a close object is detected, run this on loop for it to sweep constantly
  headTurnLeft();
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(165);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(150);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(135);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(120);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(105);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(90);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(75);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(60);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(45);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(30);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(15);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(0);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(15);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(30);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(45);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(60);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(75);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(90);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(105);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(120);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(135);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(150);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
  headTurn(165);
  if (ping() < 40) {
    detected = 1;
    detectedMountain = 1;
    return 0;
  }
}

void turnAngle(float angle, int leftRight) {
  distance = 0;
  count_L = 0;
  count_R = 0;
  while (true) {
    /*Serial.println("angle:");
    Serial.println(angle);
    Serial.println("Count_Right:");
    Serial.println(count_R);
    Serial.println("Count_Left:");
    Serial.println(count_L);
    Serial.println("Distance:");
    Serial.println(distance);*/
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

float rad2deg(float rad) {
  return rad * 57.3;
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
    /*Serial.println("dist:");
    Serial.println(dist);
    Serial.println("Count_Right:");
    Serial.println(count_R);
    Serial.println("Count_Left:");
    Serial.println(count_L);
    Serial.println("Distance:");
    Serial.println(distance);*/
    if (distance < dist && eksdee == 0) {
      servoLeft.writeMicroseconds(turnright);
      servoRight.writeMicroseconds(turnleft);
    }
    if (distance < dist && eksdee == 1) {
      servoLeft.writeMicroseconds(turnleft);
      servoRight.writeMicroseconds(turnright);
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

int isPerpendicular() {
  int originalPos = pos;
  Serial.println("entered isperpendicular");
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
    headTurn(originalPos);
    return 0;
  }
}

void findParallel() {
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
    //Serial.println("angel");
    //Serial.println(angle);
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
    //Serial.println("angel");
    //Serial.println(opposite);
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

void parallelMove() {
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

void interruptFunctionR()
{
  count_R++;
  distance = ((count_R+count_L)/2)*constant;
  count_Right++;
  distanceCliff = ((count_R+count_L)/2)*constant ; 
  turn_dist = ((count_Right+count_Left)/2)*constant ; 
  distance_overall=((count_R+count_L)/2)*constant ;
  distance_IR=((count_R_IR+count_L_IR)/2)*constant;
}

void interruptFunctionL()
{
  count_L++;
  distance = ((count_R+count_L)/2)*constant;
  count_Left++;
  distanceCliff = ((count_R+count_L)/2)*constant ; 
  turn_dist = ((count_Right+count_Left)/2)*constant ;  
  distance_overall=((count_R+count_L)/2)*constant ;
  distance_IR=((count_R_IR+count_L_IR)/2)*constant;
}

int measure(){
  bottomIR_R_read = analogRead(bottomIR_R);
  bottomIR_L_read = analogRead(bottomIR_L);

  measured = 0;
  if (bottomIR_R_read < 400 || bottomIR_L_read < 400){
    measured = 1;
    Serial.println(" ");
    Serial.print("Measured L = "); Serial.println(bottomIR_L_read);
    Serial.print("Measured R = "); Serial.println(bottomIR_R_read);
    if (bottomIR_L_read > bottomIR_R_read){
      return 0; 
    } 
    else if (bottomIR_R_read > bottomIR_L_read){
      return 1;
    }
  }

   return -1;
}

void avoid_cliff(int dirct){                                      //input is that it measured something and which direction to go to
  while(1){
    dist_capt = distanceCliff;                                         //capt current distance
    while(measured == 0){                                         //measuring loop (doesn't enter on first iteration)
      measure();   
      Serial.print("...measuring..");//ping IR and if sees cliff it sets measured to 1
      if (measured == 1){drive_straight(0); break;}                                  //if cliff was measured, continue with the rest of the algorimth
      else if (distanceCliff - dist_capt >= set_distance && check1 == 0){
        Serial.println("start getting back to original trajectory");
        check1 = 1; 
        break;}
      
      go('f');
      //Serial.print("Distance - dist_capt:"); Serial.println(distance - dist_capt);
      if(measured == 0 && check1 == 1 &&(distanceCliff-dist_capt) >= set_distance*5){                           //if it went 2*set_distance without encountering cliff
        
        turn('s',abs(dirct-1), 90);                                         //turn to head back to previous trajectory
        dist_capt = distanceCliff;                                               //capt current distance
        while((distance-dist_capt) <= set_distance*turn_counter){           //drive untill there
          Serial.println("driving back");
          drive_straight(1);                                      
          delay(20);
        }
        drive_straight(0);
        turn('s',dirct, 90);                                                //turn; you are now at the other side of cliff
        Serial.println("done");
        return;                                                             //return to main function
      }
    }
    
    turn('s', dirct, 90);  
    if (turn_counter >= 5){return;}
    measure();
    Serial.println(" ");
    Serial.println("...measuring(corner)...");
    if (measured == 1){turn('s', dirct, 90); Serial.println("CORNER"); return;}
    go('f');                                                    //drive forward
    while(distanceCliff - dist_capt <= set_distance){delay(20);}
    go('s');                                       //stop
    turn_counter++;                                               //this is needed to get back to previous trajectory
    Serial.print("turncounter = ");
    
    Serial.println(turn_counter);
    turn('s',abs(dirct-1), 90);                                       //turn towards cliff
    go('f')                                          //drive forward
    measured = 0; 
  }                                                               //reset whether it measured something
}

void turn(char s, int dir, float angle){
  //if (s == 's'){
    turn_dist = 0;
    save_dist = distanceCliff;
    count_Left = 0;
    count_Right = 0;
    save_L = count_L;
    save_R = count_R;
    while (true) {

    if (turn_dist < (trn / (360 / 90) && dir == 1)) {
      servoLeft.writeMicroseconds(turnright);
      servoRight.writeMicroseconds(turnright);
      Serial.print("");
      //Serial.println(turn_dist);
    }
    if (turn_dist < (trn / (360 / 90) && dir == 0)) {
      servoLeft.writeMicroseconds(turnleft);
      servoRight.writeMicroseconds(turnleft);
      //Serial.print("check 2 ");
      //Serial.println(turn_dist);
    }
    if (turn_dist > (trn / (360 / 90))) {
        servoLeft.writeMicroseconds(1500);
        servoRight.writeMicroseconds(1500);
        distance = save_dist;
        count_L = save_L;
        count_R = save_R;
        //Serial.println("Turned");
        return;
      }
    }
}

void IRdetection(){
  m = measure();
  if (measured == 1){
    detected = 1;
    detectedCliff = 1;
  }
}

void USdetection(){
  sweep();
}

void detectRock(){
  //input code from subgroup
}

void detectMountain(){
  //input code from subgroup
}

void detectCliff(){
  //input code from subgroup
}

void communication(){
  
}

void disableServos(){                        // Halt servo signals{                                            
  servoLeft.detach();                        // Stop sending servo signals
  servoRight.detach();
  myclaw.detach();
}

void go(char c){                             // go function
  switch(c){                                 // Switch to code based on c
    case 'f':                                // c contains 'f'
      driveForward();                        // Full speed forward
      break;
    case 'b':                                // c contains 'b'
      driveBackward();                       // Full speed backward
      break;
    case 'l':                                // c contains 'l'
      turnLeft();                            // Rotate left in place
      break;
    case 'r':                                // c contains 'r'
      turnRight();                           // Rotate right in place
      break;
    case 's':                                // c contains 's'
      forwardToStill();                      // Stop driving forward
      break;
    case 't':                                // c contains 't'
      backwardToStill();                      // Stop driving backward
      break;
  }
}
