// Included for serial communication
#include <SoftwareSerial.h>
 
// Define pins you're using for serial communication
// Do not use pins 0 or 1 as they are reserved for
// standard I/O and programming
#define TXPIN 12
#define RXPIN 13
// 17, 18 bluetooth
 
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
  Serial1.begin(115200);
}
 
// Main application loop
void loop()
{ 
  Serial1.print("Setting speed to: ");
  delay(4000);
   // Loop through 127 to 0, forward
  for(int i = 120; i >= 60; i--)
  {
    // Say that we are setting our speed
    Serial1.print("Setting speed to: ");
    Serial1.println(i, DEC);
 
    // Set speed to motor 0 and forward
    SetSpeed(0, false, i);
      SetSpeed(1, false, i);
    delay(500);
  } 
 
  // Loop through 0 to 127, backward
  for(int i = 0; i < 106; i++)
  {
    // Say that we are setting our speed
    Serial1.print("Setting speed to: ");
    Serial1.println(i, DEC);
 
    // Set speed to motor 0 and forward
    SetSpeed(0, true, i);
    SetSpeed(1, true, i);
    delay(500);
  }
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
  else if(Speed > 107)
    Speed = 107;
 
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
  pololu.write(SendByte);
 
  // Send the speed data byte
  pololu.write(Speed);
}
