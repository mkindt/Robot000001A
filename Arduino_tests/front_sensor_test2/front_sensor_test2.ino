
float prevCmF = 1000;
float prevCmR = 1000;
float prevCmRR = 1000;
// includes Nick Pelone's debugPrint for bluetooth
int frontSonarTrigger = 52;
int rearSonarTrigger = 53;
int rearRightSonarTrigger = 49;
float cmF, cmR, cmRR, pulse;

void setup(){
 Serial.begin(9600);
 Serial1.begin(115200);
 pinMode(frontSonarTrigger, OUTPUT);
 pinMode(rearSonarTrigger, OUTPUT);
 pinMode(rearRightSonarTrigger, OUTPUT);
}

void loop(){
 digitalWrite(frontSonarTrigger, LOW);
 digitalWrite(rearSonarTrigger, LOW);
 digitalWrite(rearRightSonarTrigger, LOW);
 pinMode(4, INPUT);
 pinMode(5, INPUT);
 pinMode(6, INPUT);
 /*float cmFArray[3];
 digitalWrite(frontSonarTrigger, HIGH);
  for (int i = 0; i < 3; ++i){  // too slow!!!
    float pulse = pulseIn(4, HIGH);
    cmFArray[i] = pulse * 0.0173; 
  }
  
 float pulse = pulseIn(4, HIGH);
 //delay(100);
 //float pulse2 = pulseIn(5, HIGH);
 //delay(100);
 float cmF = pulse * 0.0173;
 //float cmR = pulse2 * 0.0173;
//Serial.println();
debugPrint("");
 debugPrint("front is ");
 debugPrint(""+String(int(cmF)));
 //Serial.println();
 debugPrint(" ");
 debugPrint(""+String(int(medianer(cmFArray))));

 if (cmF < prevCm - 8 || cmF > prevCm) {
   delay(100);
   pulse = pulseIn(4, HIGH);
   cmF = pulse * 0.0173;
 }
 prevCm = cmF;
  debugPrint(" ");
 debugPrint(""+String(int(cmF)));
 debugPrintLn("");
 */
 /////// ~8 inch radius sight at 35 inches
 /////// at distances around 22 inches sight expands slightly beyond 8 inches --
 ///////// now object sometimes detected in the 8 inch radius,
 /// slight balloon shape, see published diagrams
 setCmR(); 
 // *******minimum value is ~17cm, any closer reads 29cm or more*************
 dPrint("rear is ", cmR);
 Serial.print(cmR);
 //delay(100);
  setCmF(); 
  setCmRR();
 // *******minimum value is ~17cm, any closer reads 29cm or more*************
 Serial.print("    front is ");
 dPrint("    front is ", cmF);
 Serial.print(", rightRear is ");
 dPrint(", rightRear is ", cmRR);
 Serial.println();
 //delay(200);
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
  if (cmF < prevCmF - 8 || cmF > prevCmF) { //
    delay(50);
    pulse = pulseIn(4, HIGH);
    cmF = pulse * 0.0173;
    //Serial.print("correction is ");
    //Serial.print(cmF);
  }
  digitalWrite(frontSonarTrigger, LOW); //turn off front sonar
  //Serial.print("front is ");
  //Serial.print(cmF);
  prevCmF = cmF;
}

void setCmR() {
  digitalWrite(rearSonarTrigger, HIGH); //turn on rear sonar
  pulse = pulseIn(5, HIGH);
  cmR = pulse * 0.0173;
  if (cmR < prevCmR - 8 || cmR > prevCmR) { //
    delay(50);
    pulse = pulseIn(5, HIGH);
    cmR = pulse * 0.0173;
  }
  digitalWrite(rearSonarTrigger, LOW); //turn off rear sonar
  prevCmR = cmR;
}

void setCmRR() {
  digitalWrite(rearRightSonarTrigger, HIGH); //turn on rear sonar
  pulse = pulseIn(6, HIGH);
  cmRR = pulse * 0.0173;
  if (cmRR < prevCmRR - 8 || cmRR > prevCmRR) { //
    delay(50);
    pulse = pulseIn(6, HIGH);
    cmRR = pulse * 0.0173;
  }
  digitalWrite(rearRightSonarTrigger, LOW); //turn off rear sonar
  prevCmRR = cmRR;
}

