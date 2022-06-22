#include <Servo.h>
int count_R = 0; int save_R = 0;
int count_L = 0; int save_L = 0;
int count_Right = 0; int count_Left = 0;
float distance = 0; float turn_dist = 0;

int bottomIR_R = A0;
int bottomIR_L = A3;
float constant = 0.00641025;
  //float trn=0.34;
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

int no_move = 1500;
int curr_right = 1500;
int curr_left = 1500;
int turnright = 1530;
int turnleft = 1470;  
Servo servoLeft, servoRight;



void setup() {
  pinMode(3,INPUT);
  pinMode(2,INPUT);
  Serial.begin(9600); 
  servoLeft.attach(12);                     
  servoRight.attach(13);
  attachInterrupt(digitalPinToInterrupt(3),interruptFunctionR,RISING);
  attachInterrupt(digitalPinToInterrupt(2),interruptFunctionL,RISING);
}

void loop(){  
  Serial.print("main");
  if(curr_left == 1500 || curr_right == 1500){
    drive_straight(1);
  }
  int m = measure();
  if (measured == 1){
    drive_straight(0); 
    avoid_cliff(m);
  }
}


void avoid_cliff(int dirct){                                      //input is that it measured something and which direction to go to
  while(1){
    dist_capt = distance;                                         //capt current distance
    while(measured == 0){                                         //measuring loop (doesn't enter on first iteration)
      measure();   
      Serial.print("...measuring..");//ping IR and if sees cliff it sets measured to 1
      if (measured == 1){drive_straight(0); break;}                                  //if cliff was measured, continue with the rest of the algorimth
      else if (distance - dist_capt >= set_distance && check1 == 0){
        Serial.println("start getting back to original trajectory");
        check1 = 1; 
        break;}
      
      drive_straight(1);
      //Serial.print("Distance - dist_capt:"); Serial.println(distance - dist_capt);
      if(measured == 0 && check1 == 1 &&(distance-dist_capt) >= set_distance*5){                           //if it went 2*set_distance without encountering cliff
        
        turn('s',abs(dirct-1), 90);                                         //turn to head back to previous trajectory
        dist_capt = distance;                                               //capt current distance
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
    drive_straight(1);                                                    //drive forward
    while(distance - dist_capt <= set_distance){delay(20);}
    drive_straight(0);                                         //stop
    turn_counter++;                                               //this is needed to get back to previous trajectory
    Serial.print("turncounter = ");
    Serial.println(turn_counter);
    turn('s',abs(dirct-1), 90);                                       //turn towards cliff
    drive_straight(1);                                           //drive forward
    measured = 0; 
  }                                                               //reset whether it measured something
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


void interruptFunctionR(){
  count_R++;
  count_Right++;
  distance = ((count_R+count_L)/2)*constant ; 
  turn_dist = ((count_Right+count_Left)/2)*constant ; 
}
void interruptFunctionL(){
  count_L++;
  count_Left++;
  distance = ((count_R+count_L)/2)*constant ; 
  turn_dist = ((count_Right+count_Left)/2)*constant ; 
}


void drive_straight(int v){ //v=velocity (-1, 0, 1, 2) -1 = reverse, 0 = stop, 1 = no acceleration, 2 = with accelleration
  if (v == 0){
      curr_right = 1500; curr_left = 1500;
      servoRight.writeMicroseconds(curr_right);  
      servoLeft.writeMicroseconds(curr_left);
      //Serial.println("stop");
  }
  if (v == 1){
      curr_right = 1470; curr_left = 1530;
      servoRight.writeMicroseconds(curr_right);  
      servoLeft.writeMicroseconds(curr_left); 
  }
}

void turn(char s, int dir, float angle){
    turn_dist = 0;
    save_dist = distance;
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
