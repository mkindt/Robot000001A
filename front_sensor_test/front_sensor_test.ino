
float prevCm = 1000;

void setup(){
 Serial.begin(9600);
   pololu.begin(19200);
 Serial1.begin(115200);
}

void loop(){
 pinMode(4, INPUT);
 //pinMode(5, INPUT);
 float cmFArray[3];
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
 //Serial.print("rear is ");
 //Serial.print(cmR);
 }
 prevCm = cmF;
  debugPrint(" ");
 debugPrint(""+String(int(cmF)));
 debugPrintLn("");
 delay(300);
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
void debugPrintLn(String string){
  Serial.println(string);
  Serial1.println(string);
}
void debugPrint(String c){
  Serial.print(c);
  Serial1.print(c);
}
