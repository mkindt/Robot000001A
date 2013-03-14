// This program sends information from a MaxStream XBee radio.
// It polls a switch and sends out a zero when the switch is open or a 1 when the switch is closed.
// rob@faludi.com

// serial out is on port 1
// serial in is on port 0

// a digital input is on port 12
int switchPin = 12;

// a status light is on port 13
int ledPin = 13;

// a byte to send out data:
char thisByte = 0;


void setup () {
  // set pins to input and output appropriately
  pinMode(ledPin, OUTPUT);
  pinMode(switchPin, INPUT);

  // start up the serial connection with 9600-8-n-1-true (non-inverted):
  Serial.begin(9600);

  // blink the status LED
  blinkLED(ledPin, 3);

  // for some reason it seems to help to send an arbitrary character first
  //then pause for the guard time before requesting command mode
  Serial.print("X");
  delay(1100);
  // put the XBee in command mode
  Serial.print("+++");
   delay(1100);
  // wait for a response from the XBee for 2000 ms, or start
  // over with setup if no valid response comes

  
  if (returnedOK() == 'T') {
    // if an OK was received then continue 
  }
  else {
    setup(); // otherwise go back and try setup again
  }
  


  // set the PAN (personal area network) ID number
  // this example uses 0x3330, but you'll want to choose your own 
  // unique hexadecimal number between 0x0 and 0xFFFE
  // (note the comma at the end of the command which indicates that another command will follow)
  Serial.print("ATID3330,");
  // set the Destination High to 0x0
  // to select 16 bit addressing mode. These addresses can
  // be assigned and changed by sending commands from a microcontroller
  Serial.print("DH0,");
  // set the Destination Low (16 bit address)
  // this example uses 0x0 for send and 0x1 for receive but you'll
  // want to choose your own hexadecimal numbers between 0x0 and 0xFFFE
  Serial.print("DL1,");
  // exit command mode (note that we use Serial.printLN here to issue a linefeed that completes the command sequence)
  Serial.println("CN");
  
  // the preceeding commands can also be sent on a single line like this, using a single AT command with commas:
  // Serial.println("ATID3330,DH0,DL1,CN");
  
  // the preceeding command line could also be sent as separate commands, by reissuing the AT command:
  // Serial.println("ATID3330");
  // Serial.println("ATDH0");
  // Serial.println("ATDL1");
  // Serial.println("ATCN");
  
  // wait for a response from the XBee for 2000 ms, or start
  // over with setup if no valid response comes
  
   if (returnedOK() == 'T') {
    // if an OK was received then continue 
  }
  else {
    setup(); // otherwise go back and try setup again
  }

}


void loop () {
  // read the switch:
  thisByte = digitalRead(switchPin);
  // convert it to a readable ASCII value, send it out the serial port:
  Serial.print(thisByte, DEC);
}

void blinkLED(int targetPin, int numBlinks) {
  // this function blinks the status LED light as many times as requested
  for (int i=0; i<numBlinks; i++) {
    digitalWrite(targetPin, HIGH);   // sets the LED on
    delay(250);                     // waits for a second
    digitalWrite(targetPin, LOW);    // sets the LED off
    delay(250);
  }
}


char returnedOK () {
  // this function checks the response on the serial port to see if it was an "OK" or not
  char incomingChar[3];
  char okString[] = "OK";
  char result = 'n';
  int startTime = millis();
  while (millis() - startTime < 2000 && result == 'n') {  // use a timeout of 10 seconds
    if (Serial.available() > 1) {
      // read three incoming bytes which should be "O", "K", and a linefeed:
      for (int i=0; i<3; i++) {
        incomingChar[i] = Serial.read();
      }
      if ( strstr(incomingChar, okString) != NULL ) { // check to see if the respose is "OK"
//      if (incomingChar[0] == 'O' && incomingChar[1] == 'K') { // check to see if the first two characters are "OK"
        result = 'T'; // return T if "OK" was the response
      }  
      else {
        result = 'F'; // otherwise return F
      }
    }
  }
  return result;
}

