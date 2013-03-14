
int IRpina = 2; // analog pin for reading the IR sensor
int IRpinb = 3;

void setup() {
Serial.begin(9600); // start the serial port
}
float voltsa;
int distancea;
float voltsb;
int distanceb;

void loop() {
voltsa = digitalRead(IRpina)*0.0048828125; // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
distancea = pow(voltsa, -1.10);
// Reads about 348 if nothing there, 0 if something is there.
voltsb = digitalRead(IRpinb)*0.0048828125; // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
distanceb = pow(voltsb, -1.10);
delay(500); // arbitary wait time.
//Serial.print("Sensor A: ");
//Serial.print(distancea); //Optional for debugging
//Serial.print(" .... Sensor B: ");
//Serial.println(distanceb); //Optional for debugging
if((distancea)&&(distanceb)){
Serial.println("There's a Block in place!");
}
else{
if (distancea)
Serial.println("I detect a block in front of Sensor B");
if (distanceb)
Serial.println("I detect a block in front of Sensor A");
}
}
