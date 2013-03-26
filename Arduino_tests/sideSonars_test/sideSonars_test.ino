

void setup(){
  Serial.begin(9600);
}

void loop(){
  float sideFrontTotal = 0;
  float sideRearTotal = 0;
 for (int var = 1; var <= 1; ++var){                                // Loop to get average distance readings
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
    float distAveSideFront = (sideFrontTotal);    // Distance from the wall to the side of the robot, used for navigation 
    float distAveSideRear = (sideRearTotal);
    /////********removed loops for testing********
    /////******loops are also too slow for realtime movement******
    Serial.print(distAveSideFront);
    Serial.print(" ");
    Serial.print(distAveSideRear);
    Serial.print(" ");
    Serial.print(distAveFront);
    Serial.println();
  delay(500);

}  

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

