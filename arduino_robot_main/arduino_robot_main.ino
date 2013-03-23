
#include <SoftwareSerial.h>                       // Included for serial communication
 
#define TXPIN 12                                  // Define pins you're using for serial communication
#define RXPIN 13                                  // Do not use pins 0 or 1 as they are reserved for
    // 9 to 23                                              // standard I/O and programming 
    // 8 to 24
    //10 to 22    
SoftwareSerial pololu(RXPIN, TXPIN);              // Create an instance of the software serial
int frontSonarTrigger = 52;
int rearSonarTrigger = 53;
char * colors[] = {"error", "red", "orange", "yellow", "green", "blue", "brown"}; 
// north distances to front wall(inches): 50.25, 47.25, 44.25, 41.25, 38.25, 35.25
// north distances to rear wall(inches): 43.75, 46.75, 49.75, 52.75, 55.75, 58.75
// east distances to front wall(inches): 39, 36, 33, 30, 27, 24
// east distances to rear wall(inches): 8.5, 11.5, 14.5, 17.5, 20.5, 23.5
// south distances to front wall(inches): 
// south distances to rear wall(inches):
// 3 inches = 7.62cm
// first six are traveling north, last six are traveling east
//total north = 94.5 inches
float eastLocF[] = { 115.6, 108.0, 100.4, 92.8, 85.2, 77.5 };//adjusted for center of robot
float southLocF[] = { 88.3, 80.7, 73.1, 65.5, 57.9, 50.2 }; //adjusted for center of robot
float eastLocR[] = { 103.0, 110.0, 117.0, 124.0, 131.0, 138.0 };
//float disF[] = { 132.0, 122.0, 114.0, 107.0, 99.0, 91.0, 53.0, 45.0, 37.0, 29.0, 21.0, 13.0 };
float disR[] = { 103.0, 110.0, 117.0, 124.0, 131.0, 138.0 };
// char * colors[] = {"error", "red", "orange", "yellow", "green", "blue", "brown"}; 
// int eastColorLoc[] = { 0, 0, 0, 0, 0, 0 };
// int southColorLoc[] = { 0, 0, 0, 0, 0, 0 }; 
// colors below will be measured and recorded in first passes unless we have access to competition boards
// colorLoc indexes eastLocF and eastLocR: i.e. eastLocF[colorLoc[RED]]
// testing order is green = 0th, orange = 1st, blue = 2nd, brown = 3rd, yellow = 4th, red = 5th
int eastColorLoc[] = { 5, 2, 4, 0, 1, 3 };
int southColorLoc[] = { 5, 2, 4, 0, 1, 3 };
// block draw location : 0 = 728.65
float loadingLoc[] = { 138.11, 130.5, 122.87, 115.25, 107.63, 100.01, 92.39, 84.77, 77.15, 69.53, 61.91, 54.29, 46.67, 39.05, 0, 0};
int blockCount = 0; //tracks number of blocks picked up / delivered
int blockSize = 0; // 0 for air/default, 1 for south, 2 for east
int testLoadingColors[] = { 0, 2, 0, 2, 5, 1, 4, 3, 0, 2, 5, 1, 4, 3 }; // needed only for testing nav
int testLoadingSize[] =   { 0, 1, 2, 0, 1, 2, 1, 2, 1, 2, 2, 1, 2, 1 }; // needed only for testing nav
int currentBlockColor = -1; // (0-5)
unsigned long timeRef;
unsigned long time;
float prevCm = 1000;                               // communication object. This represents the
int topSpeed = 120;                                // interface with the TReX Jr device
int northCount = 0;
int hardLeftCount = 0;
long QTIref = 1000;
float pulse;
float cmR = 1000;
float cmF = 1000;
int start = 0;
int hardLeftTurnCounter = 0;

void setup() {                                     // Main application entry point
  pinMode(RXPIN, INPUT);                          // Define the appropriate input/output pins
  pinMode(TXPIN, OUTPUT);
  pinMode(frontSonarTrigger, OUTPUT);
  pinMode(rearSonarTrigger, OUTPUT);
  
  Serial.begin(9600);                             // Begin communicating with the pololu interface
  pololu.begin(19200);
  Serial1.begin(115200);
}

void loop() {
  digitalWrite(frontSonarTrigger, LOW);
  digitalWrite(rearSonarTrigger, LOW);
  if (start == 0) { //startup calibration for sonars
    delay(2000);
    debugPrint("");
    start = 1;
  }
  debugPrint("");
  debugPrint("hardLeftCounter is ");
  // need to calibrate QTI beforehand to get black versus colors/white...
  //Serial.println(RCTime(11));
  pinMode(4, INPUT);
  //pinMode(5, INPUT);
  digitalWrite(frontSonarTrigger, HIGH); //turn on front sonar
  pulse = pulseIn(4, HIGH);
  //float pulse2 = pulseIn(5, HIGH);
  cmF = pulse * 0.0173;
  //float cmR = pulse2 * 0.0173;
  if (cmF < prevCm - 8 || cmF > prevCm) {
    delay(50);
    pulse = pulseIn(4, HIGH);
    cmF = pulse * 0.0173;
   //Serial.print("correction is ");
   //Serial.print(cmF);
  }
  digitalWrite(frontSonarTrigger, LOW); //turn off front sonar
  //Serial.print("front is ");
  //Serial.print(cmF);
  prevCm = cmF;
  if (start == 1) { //startup calibration for timer
    timeRef = millis();
    start = 2;
    topSpeed = 70;
  }
  if (start == 2) {
    if (millis() > timeRef + 500) {
      topSpeed = 120;
      start = 3;
      debugPrint("");
      debugPrint("HI ");
    }
  }
 //////START OF MAIN STATES/////
  if (hardLeftCount == 0) {
    readEastColors();
  }
  //else if (hardLeftCount == 50) {
  //  meltDown();
  //}
  else if ((hardLeftCount - 1)%4 == 0) { //(hardLeftCount == 1)
    goWest();
  }
  else if ((hardLeftCount - 2)%4 == 0) { //(hardLeftCount == 2)
    goSouthForBlock();
  }
  else if ((hardLeftCount - 3)%4 == 0) { //(hardLeftCount == 3)
    goEast();
  }
  else if ((hardLeftCount - 4)%4 == 0) { //(hardLeftCount == 4)
    goNorth();
  }
}

void goWest() {
  straight();
  blockSize = 0;
  if (cmF < 24){
    hardLeft(1);
  }
}

void goSouthForBlock() {
  switch(blockSize) {
    case 0:  // air block and default case 
      // pick up blocks on hardLeftCount == 2, 6, 10, 14, 18, 22, ... when (HLC - 2)%4 ==0
      if (cmF > loadingLoc[blockCount]) {
        parallelMove(100);
      }
      else if (cmF < (loadingLoc[blockCount] - 3)) {
        pickUpBlock();
      }
      else {
        parallelMove(90);
      }
      break;
    case 1:  // south block
      if (cmF > 24.0) { //ideally 25.5
        parallelMove(100);
      }
      else if (cmF <= 24.0) {
        hardLeft(1);
      }
      break;
    case 2: // eastern bloc
      if (cmF > 112) {
        parallelMove(100);
      }
      else if (cmF <= 112) {
        hardLeft(1);
      }
      break;
  }
}

void goEast() {
  switch(blockSize) {
    case 0:
      freeze();
      delay(10000);
      Serial.print("error in goEast");
      break;
    case 1: // deliver south block
      if (cmF > southLocF[southColorLoc[currentBlockColor]]) {
        parallelMove(90);
      }
      else if (cmF <= southLocF[southColorLoc[currentBlockColor]]) {
        dropOffBlock();
        timeRef = millis();
        digitalWrite(frontSonarTrigger, HIGH);
        while (cmF > 25 || (millis() < timeRef + 500)){ //25.5)  //southLocF[5]) //BROKEN!!
          parallelMove(100);
          pulse = pulseIn(4, HIGH);
          cmF = pulse * 0.0173;
          delay(40); //for the sonar....
        }
        digitalWrite(frontSonarTrigger, LOW);
        hardLeft(1);
      }
      break;
    case 2: //deliver east block
      if (cmF > 25.5) {
        straight(); //parallelMove(110);
      }
      else if (cmF <= 25.5) {
        hardLeft(1);
      }
      break;
  }
}

void goNorth() {
  switch(blockSize) {
    case 1: { //delivered south block
      digitalWrite(rearSonarTrigger, HIGH); //turn on front sonar
      pulse = pulseIn(5, HIGH);
      cmR = pulse * 0.0173;
      if (cmR < (loadingLoc[blockCount]) || (millis() < timeRef + 3500)) {
        parallelMove(120); // speed 5
        dPrint("made it to goNorth, cmR = ", cmR);
      }
      else if (cmR >= (loadingLoc[blockCount])) {
        hardLeft(1);
        digitalWrite(rearSonarTrigger, LOW);
      }
      break;
    }
    case 2: { //delivering east block
      if (cmF > eastLocF[eastColorLoc[currentBlockColor]]) {
        parallelMove(90); // speed 2
      }
      else if (cmF <= eastLocF[eastColorLoc[currentBlockColor]]) {
        dropOffBlock();
        timeRef = millis();
        while (cmF > 85) { //BROKEN!!
          parallelMove(120);  //speed 5
          digitalWrite(frontSonarTrigger, HIGH);
          pulse = pulseIn(4, HIGH);
          cmF = pulse * 0.0173;
          delay(40); //for the sonar....
        }
        digitalWrite(frontSonarTrigger, LOW);
        hardLeft(0);
      }
      break;
    }
  }
}

void pickUpBlock() {
  freeze();
  blockSize = testLoadingSize[blockCount];
  currentBlockColor = testLoadingColors[blockCount];
  if (blockSize == 0) { // air block
    Serial.print("air block rejected, saving location");
    Serial.println();
    if (loadingLoc[14] == 0) {
      loadingLoc[14] = loadingLoc[blockCount];
      Serial.print("location ");
      Serial.print(loadingLoc[14]);
      Serial.println();
    }
    else {
      loadingLoc[15] = loadingLoc[blockCount];
      Serial.print("location ");
      Serial.print(loadingLoc[15]);
      Serial.println();
    }
    dropOffBlock();
  }
  blockCount = blockCount + 1;
  delay(1000);
}

void dropOffBlock() {
  freeze();
  currentBlockColor = 0;
  delay(1000);
}

void readEastColors() {
  if (cmF < 24){
    hardLeft(0);
  }
  if (cmF > 140){
    northCount = 0;
  }
  if (cmF < 85 && northCount > 4){
    hardLeft(0);
    northCount = 0;
  }
  // north motion -- not getting zero sometimes
  if (cmF > eastLocF[0]+20 && northCount == 0){
    parallelMove(120);
  }
  else if (cmF > eastLocF[0] && northCount == 0 && millis() > timeRef + 1200) {
    parallelMove(90);
  }
  else if (cmF < eastLocF[0]-10 && cmR >disR[0]-10.0 && northCount == 0 && RCTime(11) < QTIref && millis() > timeRef + 1300) { // < 8000)
    topSpeed = 80;
    freeze();
    delay(600);
    northCount++;
  }
  else if (cmF < eastLocF[1]-10 && cmR >disR[1]-10.0 && northCount == 1 && RCTime(11) < QTIref) { // < 8000)
    topSpeed = 80;
    freeze();
    delay(600);
    northCount++;
  }
  else if (cmF < eastLocF[2]-10 && cmR >disR[2]-10.0 && northCount == 2 && RCTime(11) < QTIref) { // < 8000)
    //freeze();
    //delay(600);
    northCount++;
  }
  else if (cmF < eastLocF[3]-10 && cmR >disR[3]-10.0 && northCount == 3 && RCTime(11) < QTIref) { // < 8000)
    //freeze();
    //delay(600);
    northCount++;
  }
  else if (cmF < eastLocF[4]-10 && cmR >disR[4]-10.0 && northCount == 4 && RCTime(11) < QTIref) { // < 8000)
    //freeze();
    //delay(600);
    northCount++;
  }
  else if (cmF < eastLocF[5]-10 && cmR >disR[5]-10.0 && northCount == 5 && RCTime(11) < QTIref) { // < 8000)
    //freeze();
    //delay(600);
    northCount++;
  }
  else{
    parallelMove(80);
    //QTIref = RCTime(11);
    //QTIref = (QTIref + RCTime(11))/2 ;
  }
  //Serial.print(northCount);
  Serial.println();  
 }

void parallelMove(int SetTopSpeed) { // standard KEY DISTANCE FROM WALL: 6.5 inches or 16.5cm
  if (start > 2) {
    topSpeed = SetTopSpeed;
  }
  int maxDistanceFromWall, minDistanceFromWall;
  if (hardLeftCount == 0) {
    maxDistanceFromWall = 14;
    minDistanceFromWall = 11;
  }
  else {
    maxDistanceFromWall = 20; //16.5;
    minDistanceFromWall = 17.5; //14;
  }
    
  float distAveSideFront = pingWall(3); 
  float distAveSideRear = pingWall(2);
  // start by getting to the right distance from the wall
  // if almost parallel but too far from wall: 
  if (distAveSideFront > maxDistanceFromWall && distAveSideRear - distAveSideFront > 5 ){// 20 AND 7 originally
    straight(); //if already turned, don't turn more
    //delay(100);
    //straight();
  }
  // if 
  else if (distAveSideFront < minDistanceFromWall && distAveSideFront - distAveSideRear > 5){ // 18 AND 7 originally
    straight(); //if already turned, don't turn more
    //delay(100);
    //straight();
  }
  else if (distAveSideFront > maxDistanceFromWall ){ //&& distAveSideRear - distAveSideFront < 7 ){// 20 AND 7 originally
    right();
    //delay(100);
    //straight();
  }
  // if 
  else if (distAveSideFront < minDistanceFromWall ){ //&& distAveSideFront - distAveSideRear < 7){ // 18 AND 7 originally
    left();
    //delay(100);
    //straight();
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

void hardLeft(int NWSE_0123) {
 topSpeed = 110;
 int minF, maxF, minR, maxR;
 if (NWSE_0123 == 0){
   minF = 48;
   maxF = 98;
   minR = 0;
   maxR = 0; 
 }
 else if (NWSE_0123 != 0) {
   minF = 100; //24;
   maxF = 160; //105;
   minR = 0;
   maxR = 0;
 }
 SetSpeed(0, false, 0);
 SetSpeed(1, false, int(topSpeed)); //90 // 45);
 delay(100); //100
 SetSpeed(0, true, topSpeed*0.7);
 SetSpeed(1, false, topSpeed);
 delay(500);
 SetSpeed(0, true, 0);
 SetSpeed(1, false, topSpeed);
 
 float sideFront = pingWall(3); 
 float sideRear = pingWall(2);
 /*digitalWrite(frontSonarTrigger, HIGH);
 pulse = pulseIn(4, HIGH);
 cmF = pulse * 0.0173;
 
 prevCm = cmF; */
 if (NWSE_0123 == 0){
   hardLeftTurnCounter = 0;
   while (sideFront - sideRear < 0){
     delay(40);
     hardLeftTurnCounter = hardLeftTurnCounter + 1;
     sideFront = pingWall(3); 
     sideRear = pingWall(2);
       debugPrint("");
       debugPrint("hardLeftCounter is ");
       debugPrint(""+String(int(hardLeftTurnCounter)));
 }
  /* while (cmF <= prevCm) {// Sonar ping from the front sensor
     prevCm = cmF;
     hardLeftTurnCounter = hardLeftTurnCounter + 1;
       dPrint("hardLeftTurnCounter is ", hardLeftTurnCounter);
     delay(40);
     pulse = pulseIn(4, HIGH);
     cmF = pulse * 0.0173; 
   } */
 } 
 else if (NWSE_0123 > 0) {
   for (int k = 0; k < hardLeftTurnCounter; k++){
     delay(40);
     sideFront = pingWall(3); 
     sideRear = pingWall(2);
       dPrint("made it to ", hardLeftTurnCounter);
     /* delay(40);
     pulse = pulseIn(4, HIGH); // keep the timing symmetrical
     cmF = pulse * 0.0173; */
   }
 }
 //digitalWrite(frontSonarTrigger, LOW);
 SetSpeed(0, true, 0);
 SetSpeed(1, false, 0);
 hardLeftCount++;
 //delay(200);
}

void left() {
 SetSpeed(0, false, int(topSpeed*0.7)); //74
 SetSpeed(1, false, topSpeed); //90
 //delay(100);
}

void right() {
 SetSpeed(0, false, topSpeed); // 45);
 SetSpeed(1, false, int(topSpeed*0.7)); //37);
 //delay(100);
}
void fineLeft() {
 SetSpeed(0, false, int(topSpeed*0.9)); //74
 SetSpeed(1, false, topSpeed); //90
 //delay(100);
}

void fineRight() {
 SetSpeed(0, false, topSpeed); // 45);
 SetSpeed(1, false, int(topSpeed*0.9)); //37);
 //delay(100);
}

void straight() {
 SetSpeed(0, false, topSpeed); // 45);
 SetSpeed(1, false, topSpeed);
 //delay(100);
}

void freeze() {
  SetSpeed(0, false, 0);
  SetSpeed(1, false, 0);
}

// ______QTI____________________
long RCTime(int sensorIn) {
  long duration = 0;
  pinMode(sensorIn, OUTPUT); // Make pin OUTPUT
  digitalWrite(sensorIn, HIGH); // Pin HIGH (discharge capacitor)
  delay(1); // Wait 1ms
  pinMode(sensorIn, INPUT); // Make pin INPUT
  digitalWrite(sensorIn, LOW); // Turn off internal pullups
  while(digitalRead(sensorIn)){ // Wait for pin to go LOW
    duration++;
  }
  return duration;
}

// MOTORS_____________Set the motor index, direction, and speed
// Motor index should either be a 0 or 1
// Direction should be either true for forward or false for backwards
// Speed should range between 0 and 127 (inclusivly)
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

// SONAR FUNCTIONS::::::::::::::::::::::::::::::::::::  
float microsecondsToCentimeters(long microseconds) {       // converts duration of PING signal return to a distance value
  return microseconds / 29.387 / 2;
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

float medianer (float *x) {
  for (int i = 0; i<3-1; ++i) {  //size function?
    for (int j = i+1; j<3; ++j) {
      if (x[j] < x[i]) {
        float temp = x[i];
        x[i] = x[j];
        x[j] = temp;
      }
    }
  }
  return x[1]; // length divided by 2?
}
  
void meltDown() {
  freeze();
  Serial.print("the end");
  Serial.println();
  delay(20000);
}
void dPrint(String string, float z){
  debugPrint("");
  debugPrint(string);
  debugPrint(""+String(int(z)));
  debugPrintLn("");
}

void debugPrintLn(String string){
  Serial.println(string);
  Serial1.println(string);
}
void debugPrint(String c){
  Serial.print(c);
  Serial1.print(c);
}

