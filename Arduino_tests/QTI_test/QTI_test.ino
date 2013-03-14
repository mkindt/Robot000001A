

void setup(){
  Serial.begin(9600);

}

void loop(){

  delay(500);
  Serial.print(RCTime(11));
  Serial.println();

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

