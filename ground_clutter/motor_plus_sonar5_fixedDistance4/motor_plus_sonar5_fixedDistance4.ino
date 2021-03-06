
#include <SoftwareSerial.h>                       // Included for serial communication
 
#define TXPIN 12                                  // Define pins you're using for serial communication
#define RXPIN 13                                  // Do not use pins 0 or 1 as they are reserved for
                                                  // standard I/O and programming 
                                                  
SoftwareSerial pololu(RXPIN, TXPIN);              // Create an instance of the software serial
                                                  // communication object. This represents the
                                                // interface with the TReX Jr device
int northCount = 0;
void setup()                                      // Main application entry point
{
  pinMode(RXPIN, INPUT);                          // Define the appropriate input/output pins
  pinMode(TXPIN, OUTPUT);
  
  Serial.begin(9600);                             // Begin communicating with the pololu interface
  pololu.begin(19200);
}

void loop()
{
  float frontTotal = 0.00;
  float sideFrontTotal = 0.00;  
  float sideRearTotal = 0.00; 
  pinMode(4, INPUT);
  float pulse = pulseIn(4, HIGH);
  float cmF = pulse * 0.0173;
  //float cmF = pingWall(4);   // Sonar ping from the front sensor
   if (cmF < 24){
      hardleft();
    }
  if (cmF > 140){
    northCount = 0;
  }
  for (int var = 1; var <= 3; ++var)                                  // Loop to get average distance readings
  {
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
  //Serial.print(cmF*0.39); // show inches...
  //Serial.println(); 
  //Serial.println(); 
  //Serial.print(distAveSideFront*0.39); 
  //Serial.println();    
  //Serial.print(distAveSideRear*0.39);             
  Serial.println();
  
  // north motion -- not getting zero sometimes
  if (cmF > 132 && northCount == 0){
    turn (distAveFront, distAveSideFront, distAveSideRear);
  }
  else if (cmF < 132 && cmF > 122 && northCount == 0){
    freeze();
    delay(600);
    northCount++;
  }
  else if (cmF < 122 && cmF > 114 && northCount == 1){
    freeze();
    delay(600);
    northCount++;
  }
  else if (cmF < 114 && cmF > 107 && northCount == 2){
    freeze();
    delay(600);
    northCount++;
  }
  else if (cmF < 107 && cmF > 99 && northCount == 3){
    freeze();
    delay(600);
    northCount++;
  }
  else if (cmF < 99 && northCount == 4){
    freeze();
    delay(600);
    northCount++;
  }
  else if (cmF < 91 && northCount == 5){
    freeze();
    delay(600);
    northCount++;
  }
  else{
    turn (distAveFront, distAveSideFront, distAveSideRear);
  }
  Serial.print(northCount);
  Serial.println();  
 }
 
// Set the motor index, direction, and speed
// Motor index should either be a 0 or 1
// Direction should be either true for forward or false for backwards
// Speed should range between 0 and 127 (inclusivly)
void SetSpeed(int MotorIndex, boolean Forward, int Speed)
{
  if(MotorIndex < 0 || MotorIndex > 1)            // Validate motor index
    return;
 
  if(Speed < 0)                                   // Validate speed
    Speed = 0;
  else if(Speed > 127)
    Speed = 127;
 
  unsigned char SendByte = 0;                     // Send the "set" command based on the motor
  if(MotorIndex == 0)                             // Note that we do not accelerate to the
    SendByte = 0xC2;                              // speed, we just instantly set it
  else if(MotorIndex == 1)
    SendByte = 0xCA;
 
  if(!Forward)                                    // If we go backwards, the commands are the same
    SendByte--;                                   // but minus one
                                                  // Send the set speed command byte
                                                  // pololu.print(SendByte, BYTE);
  pololu.write(SendByte);
                                                  // Send the speed data byte
                                                  // pololu.print(Speed, BYTE);}
  pololu.write(Speed);
}

// SONAR FUNCTIONS::::::::::::::::::::::::::::::::::::  
float microsecondsToCentimeters(long microseconds)        // converts duration of PING signal return to a distance value
{
  return microseconds / 29.387 / 2;
}

float pingWall (int pingPin)                              // Ping signal
{
  pinMode(pingPin, OUTPUT);                               // Same pin is used as input and output
  digitalWrite(pingPin, LOW);                             // Low signal to ensure a clean high signal
  delayMicroseconds(500);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(500);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  return microsecondsToCentimeters(pulseIn(pingPin, HIGH));
}

void turn(float distAveFront, float distAveSideFront, float distAveSideRear)                // Use average distances to determine which way to turn 
{
    if (distAveSideFront > 20 && distAveSideRear - distAveSideFront < 7 ){
      right();
    }
    else if (distAveSideFront < 18 && distAveSideFront - distAveSideRear < 7){
      left();
    }
    else // if (distAveFront > 24)
    {
      float distAveSide = ((distAveSideFront + distAveSideRear) / 2);
      //Serial.print(distAveSideRear);                //Diagnostic tool
      //Serial.println(); 
      //if (distAveSide > 11) //casey had 11
      if (distAveSideFront > (distAveSideRear - 0.4))
      {
        right();
      }
      // else if (distAveSide < 10) // casey had 10
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
    
void left()                                                // Cuts out the left motor to turn the robot left
{
 SetSpeed(0, false, 37);
 SetSpeed(1, false, 45);
 //delayMicroseconds(500);
}

void hardleft()                                            // Cuts out the left motor to turn the robot hard to the left
{                                                          // when a wall is detected to the front
 SetSpeed(0, false, 0);
 SetSpeed(1, false, 45);
 delay(100);
 SetSpeed(0, true, 45);
 SetSpeed(1, false, 45);
 delay(500);
}

void right()                                               // Cuts out the right motor to turn the robot right
{
 SetSpeed(0, false, 45);
 SetSpeed(1, false, 37);
 //delayMicroseconds(250);
}

void straight()                                            // Both motors on to go straight
{
 SetSpeed(0, false, 45);
 SetSpeed(1, false, 45);
}

void freeze(){
  SetSpeed(0, false, 0);
  SetSpeed(1, false, 0);
}

void movement(){
    float frontTotal = 0.00;
  float sideFrontTotal = 0.00;  
  float sideRearTotal = 0.00; 
  float cmF = pingWall(4);   // Sonar ping from the front sensor
   if (cmF < 24){
      hardleft();
    }
  for (int var = 1; var <= 3; ++var)
  {                   
    float cmSF = pingWall(3);                     // Sonar ping from the side front sensor
    float cmSR = pingWall(2);                    // Sonar ping from the side rear sensor
    frontTotal = (frontTotal + cmF);
    sideFrontTotal = (sideFrontTotal + cmSF);
    sideRearTotal = (sideRearTotal + cmSR);  
  }
  float distAveFront = 30; // = (frontTotal/50);  
  float distAveSideFront = (sideFrontTotal/3); 
  float distAveSideRear = (sideRearTotal/3);
}
  
