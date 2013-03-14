/*
/////////////////////////////////
Htachi HM55B Compass Module
parallax (#)
AUTHOR: kiilo kiilo@kiilo.org
License: http://creativecommons.org/licenses/by-nc-sa/2.5/ch/
Navigation Team EGM 373
/////////////////////////////////
*/

//// VARS
byte CLK_pin = 8;
byte EN_pin = 9;
byte DIO_pin = 10;

float X_Data = 0;
float Y_Data = 0;
float angle;
float mainangle=16;

//// FUNCTIONS
void ShiftOut(int Value, int BitsCount) {
  for(int i = BitsCount; i >= 0; i--) {
    digitalWrite(CLK_pin, LOW);
    if ((Value & 1 << i) == ( 1 << i)) {
      digitalWrite(DIO_pin, HIGH);
      // Serial.print("1");
    }
    else {
      digitalWrite(DIO_pin, LOW);
      // Serial.print("0");
    }
    digitalWrite(CLK_pin, HIGH);
    delayMicroseconds(1);
  }
  //Serial.print(" ");
}

float ShiftIn(int BitsCount) {
  int ShiftIn_result;
  ShiftIn_result = 0;
  pinMode(DIO_pin, INPUT);
  for(int i = BitsCount; i >= 0; i--) {
    digitalWrite(CLK_pin, HIGH);
    delayMicroseconds(1);
    if (digitalRead(DIO_pin) == HIGH) {
      ShiftIn_result = (ShiftIn_result << 1) + 1;
      // Serial.print("x");
    }
    else {
      ShiftIn_result = (ShiftIn_result << 1) + 0;
      // Serial.print("_");
    }
    digitalWrite(CLK_pin, LOW);
    delayMicroseconds(1);
  }
  // Serial.print(":");
  
  // below is difficult to understand:
  // if bit 11 is Set the value is negative
  // the representation of negative values you
  // have to add B11111000 in the upper Byte of
  // the integer.
  // see: http://en.wikipedia.org/wiki/Two%27s_complement
  if ((ShiftIn_result & 1 << 11) == 1 << 11) {
    ShiftIn_result = (B11111000 << 8) | ShiftIn_result;
  }
  return ShiftIn_result;
}

void HM55B_Reset() {
  pinMode(DIO_pin, OUTPUT);
  digitalWrite(EN_pin, LOW);
  ShiftOut(B0000, 3);
  digitalWrite(EN_pin, HIGH);
}
  
void HM55B_StartMeasurementCommand() {
  pinMode(DIO_pin, OUTPUT);
  digitalWrite(EN_pin, LOW);
  ShiftOut(B1000, 3);
  digitalWrite(EN_pin, HIGH);
}
  
int HM55B_ReadCommand() {
  int result = 0;
  pinMode(DIO_pin, OUTPUT);
  digitalWrite(EN_pin, LOW);
  ShiftOut(B1100, 3);
  result = ShiftIn(3);
  return result;
}

void firstread() {
  int c;
  float X_sum=0, Y_sum=0, X_ave, Y_ave;
  HM55B_StartMeasurementCommand(); // necessary!!
  delay(40); // the data is 40ms later ready
  HM55B_ReadCommand(); // read data and print Status
  //Serial.print(" ");
  for (c=0; c<10; c++)
  {
  X_Data = ShiftIn(11); // Field strength in X
  Y_Data = ShiftIn(11); // and Y direction
  X_sum = X_sum + X_Data;
  Y_sum = Y_sum + Y_Data;
  }
  X_ave = X_sum/10;
  Y_ave = Y_sum/10;
  Serial.print("X: ");
  Serial.print(X_ave); // print X strength
  Serial.print(" ");
  Serial.print("Y: ");
  Serial.print(Y_ave); // print Y strength
  Serial.print(" ");
  digitalWrite(EN_pin, HIGH); // ok deselect chip
  mainangle = 180 * (atan2(-1*Y_ave , X_ave) / 3.1416); // angle is atan( -y/x) !!!
  if (mainangle<0)
  mainangle = (mainangle + 360);
  Serial.print("mainangle = ");
  Serial.print(mainangle); // print angle
  Serial.println("");
}

void setup() {
  Serial.begin(2400);
  pinMode(EN_pin, OUTPUT);
  pinMode(CLK_pin, OUTPUT);
  pinMode(DIO_pin, INPUT);
  Serial.print("Taking initial measurements....");
  Serial.println("");
  HM55B_Reset();
  delay(50);
  firstread();
  firstread();
  Serial.print("mainangle = ");
  Serial.print(mainangle); // print angle
  Serial.println("");
  Serial.print("Currently facing 'NORTH'");
  Serial.println("");
}

void loop() {
  int c;
  float X_sum=0, Y_sum=0, X_ave, Y_ave;
  Serial.println();
  HM55B_StartMeasurementCommand(); // necessary!!
  delay(40); // the data is 40ms later ready
  HM55B_ReadCommand(); // read data and print Status
  //Serial.print(" ");
  for (c=0; c<10; c++)
  {
  X_Data = ShiftIn(11); // Field strength in X
  Y_Data = ShiftIn(11); // and Y direction
  X_sum = X_sum + X_Data;
  Y_sum = Y_sum + Y_Data;
  }
  X_ave = X_sum/10;
  Y_ave = Y_sum/10;
  //Serial.print("X: ");
  //Serial.print(X_ave); // print X strength
  //Serial.print(" ");
  // Serial.print("Y: ");
  // Serial.print(Y_ave); // print Y strength
  //Serial.print(" ");
  digitalWrite(EN_pin, HIGH); // ok deselect chip
  angle = 180 * (atan2(-1*Y_ave , X_ave) / 3.1416); // angle is atan( -y/x) !!!
  if (angle<0)
  angle = (angle + 360);
  if(((angle-mainangle)>(-45))&&((angle-mainangle)<45))
  Serial.print("NORTH * ");
  if(((angle-mainangle)>(45))&&((angle-mainangle)<135))
  Serial.print("EAST * ");
  if((((angle-mainangle)>(135))||((angle-mainangle)<-135)))
  Serial.print("SOUTH * ");
  if(((angle-mainangle)>(-135))&&((angle-mainangle)<(-45)))
  Serial.print("WEST * ");
  //Serial.println("");
  Serial.print("angle = ");
  Serial.print(angle);
  Serial.print(" // ");
  Serial.print("angle difference = ");
  Serial.print(angle-mainangle); // print angle
  Serial.println("");
  delay(500);
}
