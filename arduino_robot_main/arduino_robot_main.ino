
#include <SoftwareSerial.h>                       // Included for serial communication
 
#define TXPIN 12                                  // Define pins you're using for serial communication
#define RXPIN 13                                  // Do not use pins 0 or 1 as they are reserved for
    // 9 to 23                                              // standard I/O and programming 
    // 8 to 24
    //10 to 22    
SoftwareSerial pololu(RXPIN, TXPIN);              // Create an instance of the software serial

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
float disF[] = { 115.6, 108.0, 100.4, 92.8, 85.2, 77.5 }; // eastLocF[]
//float disF[] = { 132.0, 122.0, 114.0, 107.0, 99.0, 91.0, 53.0, 45.0, 37.0, 29.0, 21.0, 13.0 };
float disR[] = { 103.0, 110.0, 117.0, 124.0, 131.0, 138.0 };
class Space {
  public:
    Space();
    int getColor();
    int color;
    boolean filled;   
};
Space s[12];
int www = s[5].color;
int rrr = s[5].getColor();
unsigned long timeRef;
unsigned long time;
float prevCm = 1000;                               // communication object. This represents the
int topSpeed = 120;                                // interface with the TReX Jr device
int northCount = 0;
int hardLeftCount = 0;
long QTIref = 1000;
float cmR = 1000;
int start = 0;

void setup() {                                     // Main application entry point
  pinMode(RXPIN, INPUT);                          // Define the appropriate input/output pins
  pinMode(TXPIN, OUTPUT);
  
  Serial.begin(9600);                             // Begin communicating with the pololu interface
  pololu.begin(19200);
  Serial1.begin(115200);
}

void loop() {
  if (start == 0) { //startup calibration for sonars
    delay(1000);
    start = 1;
  }
  Serial.print(s[5].color);
  // check 
  //  check distance from side wall -- is inside of:
  // check distance from front/back wall -- is inside of:
  // check location on loop -- is inside of:
   // check progress of deliveries
  
  
  // need to calibrate QTI beforehand to get black versus colors/white...
  float frontTotal = 0.00;
  float sideFrontTotal = 0.00;  
  float sideRearTotal = 0.00; 
  pinMode(4, INPUT);
  //pinMode(5, INPUT);
  //Serial.println(RCTime(11));
  /* float cmFarray[3];
  for (int i = 0; i < 3; ++i){  // too slow!!!
    float pulse = pulseIn(4, HIGH);
    cmFarray[i] = pulse * 0.0173;
  }
  // having the gripper down destroys side measurements
  float cmF = medianer(cmFarray);     */ 
    float pulse = pulseIn(4, HIGH);
    //float pulse2 = pulseIn(5, HIGH);
    float cmF = pulse * 0.0173;
    //float cmR = pulse2 * 0.0173;
    if (cmF < prevCm - 8 || cmF > prevCm) {
      delay(100);
      pulse = pulseIn(4, HIGH);
      cmF = pulse * 0.0173;
     //Serial.print("rear is ");
     //Serial.print(cmR);
    }
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
    }
  }
    
  //float cmF = pingWall(4);   // Sonar ping from the front sensor
  if (hardLeftCount == 1) { //west
    straight();
    if (cmF < 24){
      hardleft(1);
    }
    
  }
   else if (hardLeftCount == 2) { //south
     straight();
     if (cmF < 24){
      hardleft(2);
    }
   }
   else if (hardLeftCount > 2) { //east
    freeze();
    
   }
  else if (hardLeftCount == 0) {
    if (cmF < 24){
      hardleft(0);
    }
    if (cmF > 140){
      northCount = 0;
    }
    if (cmF < 85 && northCount > 4){
      hardleft(0);
      northCount = 0;
    }
    for (int var = 1; var <= 3; ++var){                                // Loop to get average distance readings
      //float cmF2 = pingWall(4);
      // if (cmF2 > cmF){
      //    cmF = cmF2;
      // }   
      float cmSF = pingWall(3);                     // Sonar ping from the side front sensor
      float cmSR = pingWall(2);                    // Sonar ping from the side rear sensor
      // frontTotal = (frontTotal + cmF);
      sideFrontTotal = (sideFrontTotal + cmSF);     // Adds the values in preparation for averaging, average value of 5
      sideRearTotal = (sideRearTotal + cmSR);       // distance readings are used to minimize 'hunting' and overcompensation while turning
    }
    float distAveFront = 30; // = (frontTotal/3);            // Distance from the wall in front of robot, determines a hard left turn
    float distAveSideFront = (sideFrontTotal/3);    // Distance from the wall to the side of the robot, used for navigation 
    float distAveSideRear = (sideRearTotal/3);
    Serial.print(distAveSideFront);
    Serial.print(" ");
    Serial.print(distAveSideRear);
    Serial.println();
    
    // north motion -- not getting zero sometimes
    if (cmF > disF[0]+20 && northCount == 0){
      turn (distAveFront, distAveSideFront, distAveSideRear);
    }
    else if (cmF > disF[0] && northCount == 0 && millis() > timeRef + 1200) {
      topSpeed = 60;
      turn (distAveFront, distAveSideFront, distAveSideRear);
    }
    else if (cmF < disF[0]-10 && cmR >disR[0]-10.0 && northCount == 0 && RCTime(11) < QTIref && millis() > timeRef + 1300) { // < 8000){
      topSpeed = 60;
      freeze();
      delay(600);
      northCount++;
    }
    else if (cmF < disF[1]-10 && cmR >disR[1]-10.0 && northCount == 1 && RCTime(11) < QTIref) { // < 8000){
      topSpeed = 60;
      freeze();
      delay(600);
      northCount++;
    }
    else if (cmF < disF[2]-10 && cmR >disR[2]-10.0 && northCount == 2 && RCTime(11) < QTIref) { // < 8000){
      freeze();
      delay(600);
      northCount++;
    }
    else if (cmF < disF[3]-10 && cmR >disR[3]-10.0 && northCount == 3 && RCTime(11) < QTIref) { // < 8000){
      freeze();
      delay(600);
      northCount++;
    }
    else if (cmF < disF[4]-10 && cmR >disR[4]-10.0 && northCount == 4 && RCTime(11) < QTIref) { // < 8000){
      freeze();
      delay(600);
      northCount++;
    }
    else if (cmF < disF[5]-10 && cmR >disR[5]-10.0 && northCount == 5 && RCTime(11) < QTIref) { // < 8000){
      freeze();
      delay(600);
      northCount++;
    }
    else{
      turn (distAveFront, distAveSideFront, distAveSideRear);
      //QTIref = RCTime(11);
      //QTIref = (QTIref + RCTime(11))/2 ;
    }
    //Serial.print(northCount);
    Serial.println();  
   }
 }
 // END OF VOID LOOP!!! 


void turn(float distAveFront, float distAveSideFront, float distAveSideRear) {
  // start by getting to the right distance from the wall
  // if almost parallel but too far from wall: 
    if (distAveSideFront > 17 ){ //&& distAveSideRear - distAveSideFront < 7 ){// 20 AND 7 originally
      right();
      delay(100);
      straight();
    }
    // if 
    else if (distAveSideFront < 15 ){ //&& distAveSideFront - distAveSideRear < 7){ // 18 AND 7 originally
      left();
      delay(100);
      straight();
    }
    else // if (distAveFront > 24) // smaller parallel adjustments
    {
      if (distAveSideFront > (distAveSideRear - 0.4))
      {
        right();
      }
      else if (distAveSideFront < (distAveSideRear + 0.4))
      {
        left();
      }
      else
      {
        straight();
      }
    }
    // else
    // {
    //    hardleft();
    // }
    
}
    


void hardleft(int NWSE_0123) {  // Cuts out the left motor to turn the robot hard to the left
 topSpeed = 90;
 int minF, maxF, minR, maxR;
 if (NWSE_0123 == 0){                                                  // when a wall is detected to the front
   minF = 24;
   maxF = 105;
   minR = 0;
   maxR = 0; 
 }
 else if (NWSE_0123 != 0) {
   minF = 24;
   maxF = 105;
   minR = 0;
   maxR = 0;
 }
 SetSpeed(0, false, 0);
 SetSpeed(1, false, int(topSpeed)); //90 // 45);
 delay(100); //100
 SetSpeed(0, true, topSpeed*0.6);
 SetSpeed(1, false, topSpeed);
 delay(500);
 float pulse = pulseIn(4, HIGH);
 float cmF = pulse * 0.0173;
 while (cmF < minF || cmF > maxF) {// Sonar ping from the front sensor
   delay(30);
   pulse = pulseIn(4, HIGH);
   cmF = pulse * 0.0173; 
 }
 hardLeftCount++;
}

void left() {                                                // Cuts out the left motor to turn the robot left
 SetSpeed(0, false, int(topSpeed*0.7)); //74
 SetSpeed(1, false, topSpeed); //90
 delay(100);
}

void right() {                                              // Cuts out the right motor to turn the robot right
 SetSpeed(0, false, topSpeed); // 45);
 SetSpeed(1, false, int(topSpeed*0.7)); //37);
 delay(100);
}

void straight() {                                           // Both motors on to go straight
 SetSpeed(0, false, topSpeed); // 45);
 SetSpeed(1, false, topSpeed);
 delay(100);
}

void freeze() {
  SetSpeed(0, false, 0);
  SetSpeed(1, false, 0);
}

void movement() {
    float frontTotal = 0.00;
  float sideFrontTotal = 0.00;  
  float sideRearTotal = 0.00; 
    // Sonar ping from the front sensor
  for (int var = 1; var <= 3; ++var) {                   
    float cmSF = pingWall(3);                     // Sonar ping from the side front sensor
    float cmSR = pingWall(2);                    // Sonar ping from the side rear sensor
    sideFrontTotal = (sideFrontTotal + cmSF);
    sideRearTotal = (sideRearTotal + cmSR);  
  }
  float distAveFront = 30; // = (frontTotal/50);  
  float distAveSideFront = (sideFrontTotal/3); 
  float distAveSideRear = (sideRearTotal/3);
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
  else if(Speed > 127)
    Speed = 127;
 
  unsigned char SendByte = 0;                     // Send the "set" command based on the motor
  if(MotorIndex == 0)                             // Note that we do not accelerate to the
    SendByte = 0xC6;    //accel C6  //set C2                       // speed, we just instantly set it
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
Space::Space() {
  color = 0;
  filled = 0;
}
int Space::getColor() {
  return color;
}

