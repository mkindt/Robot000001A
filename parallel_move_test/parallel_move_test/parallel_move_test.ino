#include <Servo.h>
#include <SoftwareSerial.h>
SoftwareSerial pololu(12,13);
//integers//
float maxDistanceFromWall, minDistanceFromWall;
int hardLeftCount = 0;
int start = 0;
int topSpeed = 120;
int southBlockCount = 0;
int blockSize = 0;



void setup(){
  
}
void loop(){
  parallelMove(60);
}
  void parallelMove(int SetTopSpeed) { // standard KEY DISTANCE FROM WALL: 6.5 inches or 16.5cm
  if (start > 2) {
    topSpeed = SetTopSpeed + 15;
  }
  int maxDistanceFromWall, minDistanceFromWall;
  if (hardLeftCount == 0) {
    maxDistanceFromWall = 9.5; //14;
    minDistanceFromWall = 8; //12.5;
  }
  else if (southBlockCount == 0) {
    maxDistanceFromWall = 5.0;
    minDistanceFromWall = 3.5;
  } 
  else if ((hardLeftCount - 2)%4 == 0 && blockSize > 0) { // going south with block (need rear reading)
    maxDistanceFromWall = 26.3;//26; // 24.5; //21 //7.25 inches... // also need cushion for turn to east wall
    minDistanceFromWall = 25.0; //18.5
  }
  else if ((hardLeftCount - 2)%4 == 0) { // going south
    maxDistanceFromWall = 20.5; //21 //7.25 inches...  //perfect so far was 21.0
    minDistanceFromWall = 19.0; //17.5; //18.5 //perfect so far was 18.8
  }
  else if ((hardLeftCount - 4)%4 == 0 && blockSize == 1) { //returning north from delivering south block
    maxDistanceFromWall = 24.5; //7.25 inches... // was 26.0 for a long time
    minDistanceFromWall = 22.0; // was 23.0 for a long time
  }
  else if ((hardLeftCount - 3)%4 == 0) { // SOUTH WALL
    maxDistanceFromWall = 19.9; //20; //16.5; //FINAL 20.3 almost perfect or 20.7
    minDistanceFromWall = 17.9; //17.5; //14; //FINAL 17.5 almost perfect or 17.9
  }
  else  { //delivering east block
    maxDistanceFromWall = 19.9;//19.0; //20; //16.5; //FINAL 20.3 almost perfect
    minDistanceFromWall = 18.4;//17.0; //17.5; //14; //FINAL 17.5 almost perfect
  }
  if (SetTopSpeed == 999) { // to soften a hardLeft turn
    topSpeed = 110;
    maxDistanceFromWall = 23;
    minDistanceFromWall = 20;
  } 
  float distAveSideFront = pingWall(3); 
  float distAveSideRear = pingWall(2);
  // start by getting to the right distance from the wall
  // if almost parallel but too far from wall: 
  if (distAveSideFront > maxDistanceFromWall && distAveSideRear - distAveSideFront > 1 ){// 20 AND 7 originally
    straight(); //if already turned, don't turn more
  }
  else if (distAveSideFront < minDistanceFromWall && distAveSideFront - distAveSideRear > 1 ){ // 18 AND 7 originally
    straight(); //if already turned, don't turn more
  }
  else if (distAveSideFront > maxDistanceFromWall ){ //&& distAveSideRear - distAveSideFront < 7 ){// 20 AND 7 originally
    right();
  }
  else if (distAveSideFront < minDistanceFromWall ){ //&& distAveSideFront - distAveSideRear < 7){ // 18 AND 7 originally
    left();
  }
  else // if (distAveFront > 24) // smaller parallel adjustments
  {
    if (distAveSideFront > (distAveSideRear - 1)) //0.4))
    {
      fineRight();
    }
    else if (distAveSideFront < (distAveSideRear + 1)) //0.4))
    {
      fineLeft();
    }
    else
    {
      straight();
    }
  }
}
  
  
  float pingWall (int pingPin) {                             // Ping signal
  pinMode(pingPin, OUTPUT);                               // Same pin is used as input and output
  digitalWrite(pingPin, LOW);                             // Low signal to ensure a clean high signal
  delayMicroseconds(500);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(500);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  return microsecondsToCentimeters(pulseIn(pingPin, HIGH));
}
void straight() {
 SetSpeed(0, false, int(topSpeed)); //*0.98)); //0.96 // 45); //left wheel moves faster, 0.98 may be best
 SetSpeed(1, false, topSpeed); 
 //delay(100);
}

void right() {
 SetSpeed(0, false, topSpeed); // 45);
 SetSpeed(1, false, int(topSpeed*0.76)); //0.76 //0.7 //37);
 //delay(100);
}
void left() {
 SetSpeed(0, false, int(topSpeed*0.73)); //0.7 //74
 SetSpeed(1, false, topSpeed); //90
 //delay(100);
}
void fineLeft() {
 SetSpeed(0, false, int(topSpeed*0.89)); //0.87 //74
 SetSpeed(1, false, topSpeed); //90
 //delay(100);
}

void fineRight() {
 SetSpeed(0, false, topSpeed); // 45);
 SetSpeed(1, false, int(topSpeed*0.91)); //0.90 //37);
 //delay(100);
}
float microsecondsToCentimeters(long microseconds) {       // converts duration of PING signal return to a distance value
  return microseconds / 29.387 / 2;
}
void SetSpeed(int MotorIndex, boolean Forward, int Speed) {
  if(MotorIndex < 0 || MotorIndex > 1)            // Validate motor index
    return;
 
  if(Speed < 0)                                   // Validate speed
    Speed = 0;
  else if(Speed > 120)
    Speed = 120;
 
  unsigned char SendByte = 0;                     // Send the "set" command based on the motor
  if(MotorIndex == 0)                             // Choose between accelerate and instantly set speed
    SendByte = 0xC6;    //accel C6  //set C2
  else if(MotorIndex == 1)
    SendByte = 0xCE;    //accel CE  //set CA
 
  if(!Forward)                                    // If we go backwards, the commands are the same
    SendByte--;                                   // but minus one
                                                  // Send the set speed command byte
                                                  // pololu.print(SendByte, BYTE);
  pololu.write(SendByte);                         // Send the speed data byte
  pololu.write(Speed);                            // pololu.print(Speed, BYTE);}
}

