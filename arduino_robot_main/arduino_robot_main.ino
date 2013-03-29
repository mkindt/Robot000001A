
#include <SoftwareSerial.h>                       // Included for serial communication
#include <Servo.h>
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
//length of robot: 22.2 cm

//////////////// GRIPPER
Servo myservo1;  // small servo
             
Servo myservo2;   // large servo

 int inpin = 7;  // Press Sensor Pin
 int val = 0;    // Variable to store the read value
 int pos1 = 0;    // variable to store the small servo position
 int pos2 = 0;    // variable to store the big servo position

int S0 = 8;//pinB  don't use this pin
int S1 = 29;//pinA
int S2 = 32;//pinE
int S3 = 31;//pinF
int out = 30;//pinC
int LED = 27;//pinD
//////////////////
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
int eastColorLoc[] = { 4, 5, 3, 0, 1, 2 };
int southColorLoc[] = { 2, 3, 0, 4, 1, 5 };
// block draw location : 0 = 728.65
float loadingLoc[] = { 138.11, 130.5, 122.87, 115.25, 107.63, 100.01, 92.39, 84.77, 77.15, 69.53, 61.91, 54.29, 46.67, 39.05, 0, 0};
float loadingLocR[] = { 23.1, 30.71, 38.32, 45.93, 53.54, 61.15, 68.76, 76.37 };
int blockCount = 0; //tracks number of blocks picked up / delivered
int blockSize = 0; // 0 for air/default, 1 for south, 2 for east
int testLoadingColors[] = { 0, 2, 0, 2, 5, 1, 4, 3, 0, 2, 5, 1, 4, 3 }; // needed only for testing nav
int testLoadingSize[] =   { 0, 1, 2, 0, 1, 2, 1, 2, 1, 2, 2, 1, 2, 1 }; // needed only for testing nav
int currentBlockColor = -1; // (0-5)
unsigned long timeRef;
unsigned long turnTimer;
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
  TCS3200setup(); //color sensors
  pinMode(4, INPUT); // reading front sonar
  pinMode(5, INPUT); // reading rear sonar
  pinMode(frontSonarTrigger, OUTPUT); // off/on generate sonar
  pinMode(rearSonarTrigger, OUTPUT); // off/on generate sonar
  pinMode(inpin, INPUT); // Pin 7 is connected to press sensor ///GRIPPER///
  // Press sensor attaches to pin 7 and to ground
  //myservo1.attach(9);  // attaches the small servo on pin 9
  //myservo2.attach(8);  // attaches the large servo on digital pin 8
  digitalWrite(inpin, HIGH);
  
  Serial.begin(9600);                             // Begin communicating with the pololu interface
  pololu.begin(19200);
  Serial1.begin(115200);
}

void loop() {
  myservo1.detach();
  myservo2.detach();
  digitalWrite(frontSonarTrigger, LOW);
  digitalWrite(rearSonarTrigger, LOW);
  if (start == 0) { //startup calibration for sonars
    debugPrint("");
    debugPrint("test ");
    delay(2000);
    dPrint("test of dPrint ", cmF);
    start = 1;
  }
  

  // need to calibrate QTI beforehand to get black versus colors/white...
  //Serial.println(RCTime(11));
  setCmF();

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
  if (hardLeftCount < 2) { //34 inches for rear?
    setCmR();
    if (cmR > 86) {
      hardLeft(0, 0);
    }
  }
  else {
    if (cmF < 28){ //24
      hardLeft(0, 0);
    }
  }
}

void goSouthForBlock() {
  switch(blockSize) {
    case 0:  // air block and default case 
      // pick up blocks on hardLeftCount == 2, 6, 10, 14, 18, 22, ... when (HLC - 2)%4 ==0
      setCmR();
      if (cmR < loadingLocR[blockCount] || RCTime(11) > QTIref ) {
        parallelMove(80);
      }
      else if (cmR > (loadingLocR[blockCount] + 3) && RCTime(11) < QTIref ) {
        pickUpBlock();
      }
      else {
        parallelMove(70);
      }
      break;
    case 1:  // south block
      if (cmF > 27) { //ideally 25.5
        parallelMove(100);
      }
      else if (cmF <= 27) { //24
        hardLeft(1, 0);
      }
      break;
    case 2: // eastern bloc
      if (cmF > 112) {
        parallelMove(100);
      }
      else if (cmF <= 112) {
        hardLeft(1, true); //softening might not work as robot gets closer to south wall
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
        while (cmF > 30 || (millis() < timeRef + 200)){ //25.5) //changed cmF > 28 for softening
          parallelMove(100);
          pulse = pulseIn(4, HIGH);
          cmF = pulse * 0.0173;
          delay(40); //for the sonar....
        }
        digitalWrite(frontSonarTrigger, LOW);
        hardLeft(1, 0);
      }
      break;
    case 2: //deliver east block
      if (cmF > 28) {
        straight(); //parallelMove(110);
      }
      else if (cmF <= 28) {
        hardLeft(0, 0);
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
      if (cmR < (loadingLoc[blockCount]) || (millis() < timeRef + 1200)) { //timeRef from hardLeft
        parallelMove(100); // speed 5
        dPrint("made it to goNorth, cmR = ", cmR);
      }
      else if (cmR >= (loadingLoc[blockCount])) {
        hardLeft(1, 0); //dont soften turn
        digitalWrite(rearSonarTrigger, LOW);
      }
      break;
    }
    case 2: { //delivering east block
      if (cmF > eastLocF[eastColorLoc[currentBlockColor]]) {
        parallelMove(90); // speed 2
      }
      else if (cmF <= eastLocF[eastColorLoc[currentBlockColor]]) {
        if (eastColorLoc[currentBlockColor] != 5) {       
          dropOffBlock();
        }
        else {
          dropOffBlock();
          timeRef = millis();
          while (millis() - timeRef < 1000) {
          reverse();
          }
          freeze();
        }
        timeRef = millis();
        while (cmF > 91) {  // > 85
          parallelMove(100);  //speed 5
          digitalWrite(frontSonarTrigger, HIGH);
          pulse = pulseIn(4, HIGH);
          cmF = pulse * 0.0173;
          delay(40); //for the sonar....
        }
        digitalWrite(frontSonarTrigger, LOW);
        hardLeft(1, 0);
      }
      break;
    }
  }
}

void pickUpBlock() {
  freeze();
    myservo1.attach(9);
    myservo2.attach(8);
    lowerarm();           // Lower the arm to the block 
    closesmallservo();      //  Close gripper
    delay(500);
  // blockSize = testLoadingSize[blockCount];
  // currentBlockColor = testLoadingColors[blockCount];
  if (blockSize == 0) { // air block
    Serial.print("air block rejected, saving location");
    Serial.println();
    if (loadingLoc[14] == 0) {  //save location of airblock
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
    //dropOffBlock();
      opensmallservo();      // Release block
      delay(500);
  }
  liftarm();             // Lift the gripper arm  
  blockCount = blockCount + 1;
  delay(1000); // waits one second for other servo to lift arm
     myservo1.detach();
    myservo2.detach(); 
}

void dropOffBlock() {
  freeze();
    myservo1.attach(9);
    myservo2.attach(8);
    lowerarm();            // Lower the gripper arm
    opensmallservo();      // Release block
    delay(500);
    liftarm();            // Raise the gripper arm out of the way
    delay(1000);           // waits one second for other servo to lift arm            
  currentBlockColor = 0;
       myservo1.detach();
    myservo2.detach(); 
  // delay(1000);
}

void readEastColors() {
  if (cmF < 24){
    hardLeft(1, 0);
  }
  if (cmF > 140){
    northCount = 0;
  }
  if (cmF < 92 && northCount > 5){  //originally <85
    hardLeft(1, 0);
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
    freeze();
    delay(200);
    northCount++;
    timeRef = millis();
    while (millis() - timeRef < 1000) {
      reverse();
    }
    freeze();
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
  else if ((hardLeftCount - 2)%4 == 0) {
    maxDistanceFromWall = 21.0; //7.25 inches...
    minDistanceFromWall = 18.5;
  }
  else {
    maxDistanceFromWall = 20; //16.5;
    minDistanceFromWall = 17.5; //14;
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
    //delay(100);
    //straight();
  }
  // if 
  else if (distAveSideFront < minDistanceFromWall && distAveSideFront - distAveSideRear > 1 ){ // 18 AND 7 originally
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

void hardLeft(boolean calibrate, boolean soften) {
  if (soften == true) {
    timeRef = millis();
    while (millis() - timeRef < 400) {
    parallelMove(999);
    }
  }
 topSpeed = 110;
 int minF, maxF, minR, maxR;
 if (calibrate == false){
   minF = 48;
   maxF = 98;
   minR = 0;
   maxR = 0; 
 }
 else if (calibrate == true) {
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
 if (calibrate == false) { //recalibrate
   hardLeftTurnCounter = 0;
   timeRef = millis();
   while (sideFront - sideRear < 0){ //strange reading from this corner, previously < 0
     delay(40);
     //hardLeftTurnCounter = hardLeftTurnCounter + 1;
     //side calibration worked great until we increased height of side sonars, now rear value is small
     sideFront = pingWall(3); 
     sideRear = pingWall(2);
       debugPrint("");
       debugPrint("hardLeftCounter is ");
       debugPrint(""+String(int(hardLeftTurnCounter)));
   }
   turnTimer = millis() - timeRef;
   dPrint("turnTimer = ", turnTimer);
 } 
 else if (calibrate == true) { //don't recalibrate //BACKWARDS boolean blah
   /*for (int k = 0; k < hardLeftTurnCounter; k++)
     delay(40);
     sideFront = pingWall(3); // keep the timing symmetrical
     sideRear = pingWall(2);
       dPrint("made it to ", hardLeftTurnCounter);
   } */
   delay(300); // turnTimer); //tests returned 319
 }
 SetSpeed(0, true, 0);
 SetSpeed(1, false, 0);
 hardLeftCount++;
}

void left() {
 SetSpeed(0, false, int(topSpeed*0.7)); //0.7 //74
 SetSpeed(1, false, topSpeed); //90
 //delay(100);
}

void right() {
 SetSpeed(0, false, topSpeed); // 45);
 SetSpeed(1, false, int(topSpeed*0.76)); //0.7 //37);
 //delay(100);
}
void fineLeft() {
 SetSpeed(0, false, int(topSpeed*0.87)); //74
 SetSpeed(1, false, topSpeed); //90
 //delay(100);
}

void fineRight() {
 SetSpeed(0, false, topSpeed); // 45);
 SetSpeed(1, false, int(topSpeed*0.90)); //37);
 //delay(100);
}

void straight() {
 SetSpeed(0, false, int(topSpeed*0.96)); // 45); //left wheel moves faster
 SetSpeed(1, false, topSpeed); 
 //delay(100);
}

void freeze() {
  SetSpeed(0, false, 0);
  SetSpeed(1, false, 0);
}

void reverse() {
  SetSpeed(0, true, topSpeed*0.7);
  SetSpeed(1, true, topSpeed*0.7);
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

void setCmF() {
  digitalWrite(frontSonarTrigger, HIGH); //turn on front sonar
  pulse = pulseIn(4, HIGH);
  //float pulse2 = pulseIn(5, HIGH);
  cmF = pulse * 0.0173;
  //float cmR = pulse2 * 0.0173;
  if (cmF < prevCm - 8 || cmF > prevCm) { //
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
}

void setCmR() {
  digitalWrite(rearSonarTrigger, HIGH); //turn on rear sonar
  pulse = pulseIn(5, HIGH);
  cmR = pulse * 0.0173;
  if (cmR < prevCm - 8 || cmR > prevCm) { //
    delay(50);
    pulse = pulseIn(5, HIGH);
    cmR = pulse * 0.0173;
  }
  digitalWrite(rearSonarTrigger, LOW); //turn off rear sonar
  prevCm = cmR;
}
//////////// GRIPPER ///////////////////////

void opensmallservo()
{
//  Serial.print("Opening Gripper. Final Position: ");
//  myservo2.write(pos2);
   for(pos1 = 56; pos1 < 145; pos1++)  // small servo opens 
 {                                  // in steps of 1 degree
  myservo1.write(pos1);              // tell servo to go to position in variable 'pos'
   delay(15);                       // waits 15ms for the servo to reach the position
  }
//   myservo2.write(pos2);
//   delay(15);
//  Serial.println(pos1);
}

void closesmallservo()
{
//  Serial.print("Closing Gripper. ");
//  Serial.println();
    // Pull Press Sensor input up
   pos1 = 145;           // inititalize small servo position
   while(pos1 > 56)
   {                
      myservo1.write(pos1);          // tell servo to go to position in variable 'pos'
      delay(15);                     // waits 15ms for the servo to reach the position
      if (digitalRead(inpin)==LOW) {
     //  Serial.print("Final closed Position: ");  // Monitor the last position of the servo
     //  Serial.println(pos1);
       break;
      }
      pos1--;
   }
 //   Serial.print("Final Position: ");  // Monitor the last position of the servo
  Serial.println(pos1);       //  Add language to show size of block based on pos1 value
if ((pos1>=104)&&(pos1<=125))
{color();
blockSize = 2;
Serial.println("Rail block");}
else if ((pos1>=85)&&(pos1<=100))
{color();
blockSize = 1;
Serial.println("Sea block");}
else if ((pos1>=57)&&(pos1<=80))
{color();
blockSize = 0;
Serial.println("Air block");}
else Serial.println("Shit");

}

void liftarm()
{
//  Serial.print("Lifting Arm");
//  Serial.println();
   for(pos2 = 7; pos2 < 100; pos2 += 1)  // big servo lifts arm
  {                                  // in steps of 1 degree
    myservo2.write(pos2);              // tell big servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void lowerarm()
{
//  Serial.print("Lowering Arm");
//  Serial.println();
  for(pos2 = 100; pos2>=7; pos2-=1)     // big servo lowers arm
  {
    myservo2.write(pos2);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void color() {
  currentBlockColor = detectColor(out);
  //need to have statement here to detect again if no color is undetermined
  //define integer in void color scope.
 // Serial.print("\n\n\n");
  delay(1000);
}
int detectColor(int taosOutPin){
  //isPresentTolerance will need to be something small if used in high light environment, large if used in dark environment.
  //the color detection will work either way, but the larger isPresentTolerance is, 
  //the closer the object will need to be in front of sensor
  double isPresentTolerance = 3;
  double isPresent = colorRead(taosOutPin,0,0)/colorRead(taosOutPin,0,1);//number gets large when something is in front of sensor. 
  //Serial.print("isPresent:");
  //Serial.println(isPresent,2);
  //Serial.print("isPresentTolerance currently set to:");
  //Serial.println(isPresentTolerance,2);
  if(isPresent < isPresentTolerance){
    Serial.println("nothing is in front of sensor");
    return 0;
  }
  double red,blue,green;
  double white = colorRead(taosOutPin,0,1);
  red = white/colorRead(taosOutPin,1,1)*255;
  blue = white/colorRead(taosOutPin,2,1)*255;
  green = white/colorRead(taosOutPin,3,1)*255;

  //Prints out RBG value right here.
// Serial.print("red ");
//  Serial.println(red);
//  Serial.print("blue ");
//  Serial.println(blue);
//  Serial.print("green ");
//  Serial.println(green);

if(red > 175 && red < 205 && blue > 45 && blue < 62 && green > 30 && green < 45){
    Serial.println("Red Detected");
    return 0;
  }

 else if(red > 175 && red < 205 && blue > 38 && blue < 53 && green > 35 && green < 50){
    Serial.println("Orange Detected");
    return 1;
  }

 else if(red > 65 && red < 80 && blue > 80 && blue < 100 && green > 90 && green < 120){
    Serial.println("Green Detected");
    return 3;
  }

 else if(red > 118 && red < 145 && blue > 65 && blue < 83 && green > 59 && green < 75){
    Serial.println("Brown Detected");
    return 5;
  }

 else if(red > 20 && red < 45 && blue > 150 && blue < 170 && green > 70 && green < 90){
    Serial.println("Blue Detected");
    return 4;
  }

 else if(red > 115 && red < 138 && blue > 40 && blue < 60 && green > 80 && green < 100){
    Serial.println("Yellow Detected");
    return 2;
  }
  else Serial.println(" :( I didn't read the fucking color :( ");
}
/*
This method will return the pulseIn reading of the selected color.
 Since frequency is proportional to light intensity of the selected color filter, 
 the smaller pulseIn is, the more light there is of the selected color filter.  
 It will turn on the sensor at the start taosMode(1), and it will power off the sensor at the end taosMode(0)
 color: 0=white, 1=red, 2=blue, 3=green
 if LEDstate is 0, LED will be off. 1 and the LED will be on.
 taosOutPin is the ouput of the TCS3200. If you have multiple TCS3200, all wires can be combined except the out pin
 */
double colorRead(int taosOutPin, int color, boolean LEDstate){
  //make sure that the pin is set to input
  pinMode(taosOutPin, INPUT);
  //turn on sensor with highest frequency settingtaosMode(1);
  //delay to let the sensor sit before taking a reading. Should be very small with this sensor
  int sensorDelay = 1;
  //set the pins to select the color  
  if(color == 0){//white
    digitalWrite(S3, LOW); //S3
    digitalWrite(S2, HIGH); //S2
    // Serial.print(" w");
  }
  else if(color == 1){//red
    digitalWrite(S3, LOW); //S3
    digitalWrite(S2, LOW); //S2
    // Serial.print(" r");
  }
  else if(color == 2){//blue
    digitalWrite(S3, HIGH); //S3
    digitalWrite(S2, LOW); //S2 
    // Serial.print(" b");
  }
  else if(color == 3){//green
    digitalWrite(S3, HIGH); //S3
    digitalWrite(S2, HIGH); //S2 
    // Serial.print(" g");
  }
  double readPulse;
  if(LEDstate == 0){
    digitalWrite(LED, LOW);
  }
  if(LEDstate == 1){
    digitalWrite(LED, HIGH);
  }
  delay(sensorDelay);
  readPulse = pulseIn(taosOutPin, LOW, 80000);
  //if the pulseIn times out, it returns 0 and that throws off numbers. just cap it at 80k if it happens
  if(readPulse < .1){
    readPulse = 80000;
  }
  //turn off color sensor and white LED to save power 
  taosMode(0);
  return readPulse;
}
//setting mode to zero will put taos into low power mode. taosMode(0);
void taosMode(int mode){
  if(mode == 0){
    //power OFF
    digitalWrite(LED, LOW);
    digitalWrite(S0, LOW); //S0
    digitalWrite(S1, LOW); //S1
    //  Serial.println("mOFFm");
  }
  else if(mode == 1){
    //this will put in 1:1
    digitalWrite(S0, HIGH); //S0
    digitalWrite(S1, HIGH); //S1
    // Serial.println("m1:1m");
  }
  else if(mode == 2){
    //this will put in 1:5
    digitalWrite(S0, HIGH); //S0
    digitalWrite(S1, LOW); //S1
    //Serial.println("m1:5m");
  }
  else if(mode == 3){
    //this will put in 1:50
    digitalWrite(S0, LOW); //S0
    digitalWrite(S1, HIGH); //S1 
    //Serial.println("m1:50m");
  }
  return;
}
void TCS3200setup(){
  //initialize pins
  pinMode(LED,OUTPUT); //LED pinD
  //color mode selection
  pinMode(S2,OUTPUT); //S2 pinE
  pinMode(S3,OUTPUT); //s3 pinF
  //color response pin (only actual input from taos)
  pinMode(out, INPUT); //out pinC
  //communication freq output divider
  pinMode(S0,OUTPUT); //S0 pinB
  pinMode(S1,OUTPUT); //S1 pinA 
  return;
}

