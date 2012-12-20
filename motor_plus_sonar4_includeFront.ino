// SonarNavigation version 7
//
// Casey Darden, Michael Kindt, Dakota Lazenby
//
// Allows a small robot equipped with one forward facing PING sensor and one side facing PING sensor to Navigate in a counterclockwise fashion,
// maintaining a distance of 10 cm from the wall to its right, turning left when it approaches a wall to the front
//

#include <SoftwareSerial.h>                       // Included for serial communication
 
#define TXPIN 12                                  // Define pins you're using for serial communication
#define RXPIN 13                                  // Do not use pins 0 or 1 as they are reserved for
                                                  // standard I/O and programming 
                                                  
SoftwareSerial pololu(RXPIN, TXPIN);              // Create an instance of the software serial
                                                  // communication object. This represents the
                                                  // interface with the TReX Jr device

 
 
void setup()                                      // Main application entry point
{
  pinMode(RXPIN, INPUT);                          // Define the appropriate input/output pins
  pinMode(TXPIN, OUTPUT);
  
  Serial.begin(9600);                             // Begin communicating with the pololu interface
  pololu.begin(19200);
}
 

void loop()                                       // Main application loop
{
  float frontTotal = 0.00;                        // Zeros out the variables used to calculate the average distance
  float sideFrontTotal = 0.00;  
  float sideRearTotal = 0.00;  
  int var = 1;
  while(var < 5)                                  // Loop to get average distance readings
  {
    float cmF = pingWall(4);                      // Sonar ping from the front sensor
    float cmSF = pingWall(3);                     // Sonar ping from the side front sensor
    float cmSR = pingWall(2);                     // Sonar ping from the side rear sensor
    frontTotal = (frontTotal + cmF);
    sideFrontTotal = (sideFrontTotal + cmSF);     // Adds the values in preparation for averaging, average value of 5
    sideRearTotal = (sideRearTotal + cmSR);       // distance readings are used to minimize 'hunting' and overcompensation while turning
    var++;
  }
  float distAveFront = (frontTotal/5);            // Distance from the wall in front of robot, determines a hard left turn
  float distAveSideFront = (sideFrontTotal/5);    // Distance from the wall to the side of the robot, used for navigation 
  float distAveSideRear = (sideRearTotal/5);
  turn (distAveFront, distAveSideFront, distAveSideRear);
  //Serial.print(distAveSideRear);                //Diagnostic tool
  //Serial.println(); 
  
                                                  // end SONAR code 
  
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
                                                          // SONAR FUNCTIONS:
  
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
    if (distAveFront > 24)
    {
      float distAveSide = ((distAveSideFront + distAveSideRear) / 2);
      //Serial.print(distAveSide);                //Diagnostic tool
      //Serial.println(); 
      if (distAveSide > 11)
      {
        right();
      }
      else if (distAveSide < 10)
      {
        left();
      }
      else
      {
        straight();
      }
    }
    else
    {
      hardleft();
    }
    
}
    
void left()                                                // Cuts out the left motor to turn the robot left
{
 SetSpeed(0, false, 37);
 SetSpeed(1, false, 45);
 //delayMicroseconds(500);
}

void hardleft()                                            // Cuts out the left motor to turn the robot hard to the left
{                                                          // when a wall is detected to the front
 SetSpeed(0, false, 37);
 SetSpeed(1, false, 45);
 delayMicroseconds(4500);                                  //Time delay to complete 90 degree turn
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
  
