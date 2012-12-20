
// Included for serial communication
#include <SoftwareSerial.h>
 
// Define pins you're using for serial communication
// Do not use pins 0 or 1 as they are reserved for
// standard I/O and programming
#define TXPIN 12
#define RXPIN 13
 
// Create an instance of the software serial
// communication object. This represents the
// interface with the TReX Jr device
SoftwareSerial pololu(RXPIN, TXPIN);
 
// Main application entry point 
void setup()
{
  // Define the appropriate input/output pins
  pinMode(RXPIN, INPUT);
  pinMode(TXPIN, OUTPUT);
  // Begin communicating with the pololu interface
  Serial.begin(9600);
  pololu.begin(19200);
}
 
// Main application loop
void loop()
{
  float frontTotal = 0.00;                              // Zeros out the variables used to calculate the average distance
  float rearTotal = 0.00;                                  
  int var = 1;
  while(var < 500)                                       // Loop to get average distance readings
  {
    float cm1 = pingWall(2);                            // Sonar ping from the rear sensor
    float cm2 = pingWall(4);                            // Sonar ping from the front sensor
    rearTotal = (rearTotal + cm1);                      // Adds the values in preparation for averaging
    frontTotal = (frontTotal + cm2);
    var++;
  }
  float distAveR = (rearTotal/500);                      // Distances from the wall, used for navigation 
  float distAveF = (frontTotal/500);
  turn (distAveR, distAveF);  
  // end SONAR code 
 // delay(5000);
 // SetSpeed(0, true, 70);
 // SetSpeed(1, true, 70);
 // delay(1000);
 // SetSpeed(0, false, 70);
 // SetSpeed(1, true, 70);
// SetSpeed(0, true, 0);
// SetSpeed(1, true, 0);  
}
 
// Set the motor index, direction, and speed
// Motor index should either be a 0 or 1
// Direction should be either true for forward or false for backwards
// Speed should range between 0 and 127 (inclusivly)
void SetSpeed(int MotorIndex, boolean Forward, int Speed)
{
  // Validate motor index
  if(MotorIndex < 0 || MotorIndex > 1)
    return;
 
  // Validate speed
  if(Speed < 0)
    Speed = 0;
  else if(Speed > 127)
    Speed = 127;
 
  // Send the "set" command based on the motor
  // Note that we do not accelerate to the
  // speed, we just instantly set it
  unsigned char SendByte = 0;
  if(MotorIndex == 0)
    SendByte = 0xC2;
  else if(MotorIndex == 1)
    SendByte = 0xCA;
 
  // If we go backwards, the commands are the same
  // but minus one
  if(!Forward)
    SendByte--;
 
  // Send the set speed command byte
  //pololu.print(SendByte, BYTE);
  pololu.write(SendByte);
  // Send the speed data byte
  //pololu.print(Speed, BYTE);}
  pololu.write(Speed);
}
  // SONAR FUNCTIONS:
  float microsecondsToCentimeters(long microseconds)      // converts duration of signal return to a distance value
{
  return microseconds / 29.387 / 2;
}

float pingWall (int pingPin)                            // Ping signal
{
  pinMode(pingPin, OUTPUT);                             // Same pin is used as input and output
  digitalWrite(pingPin, LOW);                           // Low signal to ensure a clean high signal
  delayMicroseconds(500);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(500);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  return microsecondsToCentimeters(pulseIn(pingPin, HIGH));
}
float turn(float distAveR, float distAveF)               // Use average distances to determine which way to turn 
{
    float difference = (distAveR-distAveF);
    if (difference > .25)
    {
      left();
    }
    else if (difference < -.25)
    {
      right();
    }
    else
    {
      straight();
    }
    //Serial.print(difference);
    //Serial.println();
}
void left()                                             // Cuts out the left motor to turn the robot left
{
 SetSpeed(0, false, 60);
 SetSpeed(1, false, 65);
}

void right()                                            // Cuts out the right motor to turn the robot right
{
 SetSpeed(0, false, 65);
 SetSpeed(1, false, 60);
}

void straight()                                         // Both motors on to go straight
{
 SetSpeed(0, false, 65);
 SetSpeed(1, false, 65);
}
  


