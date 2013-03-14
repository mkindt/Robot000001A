
// March 12, 2013: Robot Movement: Michael Kindt
// north distances to front wall(inches): 50.25, 47.25, 44.25, 41.25, 38.25, 35.25
// north distances to rear wall(inches): 43.75, 46.75, 49.75, 52.75, 55.75, 58.75
// east distances to front wall(inches): 39, 36, 33, 30, 27, 24
// east distances to rear wall(inches): 8.5, 11.5, 14.5, 17.5, 20.5, 23.5
// south distances to front wall(inches): 
// south distances to rear wall(inches):
// 3 inches = 7.62cm
// total north = 94.5 inches
// west wall distances: 14.875 + 14*3 + 0.5 + 14.875 + 22.25
// float eastLocF[] = { 132.0, 122.0, 114.0, 107.0, 99.0, 91.0 }; //old, inaccurate
// float eastLocF[] = { 127.6, 120.0, 112.4, 104.8, 97.2, 89.5 }; //real measurements
float eastLocF[] = { 115.6, 108.0, 100.4, 92.8, 85.2, 77.5 };//adjusted for center of robot
// float southLocF[] = { 53.0, 45.0, 37.0, 29.0, 21.0, 13.0 }; // what is this??
// float southLocF[] = { 100.3, 92.7, 85.1, 77.5, 69.9, 62.2 }; //original measurements
float southLocF[] = { 88.3, 80.7, 73.1, 65.5, 57.9, 50.2 }; //adjusted for center of robot
float eastLocR[] = { 103.0, 110.0, 117.0, 124.0, 131.0, 138.0 };
float cmF = 209.5; // centimeters measured from front sonar
float cmR = 500; // centimeters from rear
int northCount = 0; // needed in RL to force a stable Count during read of east colors
int hardLeftCount = 0; // main tracking of sequence/state
final int RED1 = 0;
final int ORANGE1 = 1;
final int YELLOW1= 2;
final int GREEN1 = 3;
final int BLUE1 = 4;
final int BROWN1 = 5;
// char * colors[] = {"error", "red", "orange", "yellow", "green", "blue", "brown"}; 
// int eastColorLoc[] = { 0, 0, 0, 0, 0, 0 };
// int southColorLoc[] = { 0, 0, 0, 0, 0, 0 }; 
// colors below will be measured and recorded in first passes unless we have access to competition boards
// colorLoc indexes eastLocF and eastLocR: i.e. eastLocF[colorLoc[RED]]
// testing order is green = 0th, orange = 1st, blue = 2nd, brown = 3rd, yellow = 4th, red = 5th
int eastColorLoc[] = { 5, 2, 4, 0, 1, 3 };
int southColorLoc[] = { 5, 2, 4, 0, 1, 3 };
// block draw location : 0 = 728.65
float loadingLoc[] = { 138.11, 130.5, 122.87, 115.25, 107.63, 100.01, 92.39, 84.77, 77.15, 69.53, 61.91, 54.29, 46.67, 39.05, 0, 0};
int blockCount = 0; //tracks number of blocks picked up / delivered
int blockSize = 0; // 0 for air/default, 1 for south, 2 for east
int currentBlockColor = -1; // (0-5)
int testLoadingColors[] = { 0, 2, 0, 2, 5, 1, 4, 3, 0, 2, 5, 1, 4, 3 }; // needed only for this simulation
int testLoadingSize[] =   { 0, 1, 2, 0, 1, 2, 1, 2, 1, 2, 2, 1, 2, 1 }; // needed only for this simulation

void setup() {
  size(1200, 600); //board is 240cm x ~121cm
}

void draw() {
  background (0);
  drawEastWall();
  drawWestWall();
  drawSouthWall();
  drawBlocks();
  //////START OF ROBOT CODE/////
  if (hardLeftCount == 0) {
    readEastColors();
  }
  else if (hardLeftCount == 50) {
    meltDown();
  }
  else if ((hardLeftCount - 1)%4 == 0) { //(hardLeftCount == 1) {
    goWest();
  }
  else if ((hardLeftCount - 2)%4 == 0) { //(hardLeftCount == 2) {
    goSouthForBlock();
  }
  else if ((hardLeftCount - 3)%4 == 0) { //(hardLeftCount == 3) {
    goEast();
  }
   else if ((hardLeftCount - 4)%4 == 0) { //(hardLeftCount == 4) {
    goNorth();
  }
}
// for purposes of this model, this function draws the path of robot //
// in RL, this is the main movement parallel to wall using side sonars //
void parallelMove (int speed) {
  delay(50/speed);
  if (hardLeftCount == 0) { //going north
    cmF = cmF - 4;
    fill (255);
    ellipse (1200 - (5*cmF), 515, 30, 30);
    rect (1120 - (5*cmF), 500, 80, 60);
  }
  else if ((hardLeftCount - 1)%4 == 0) { //going west
    cmF = cmF - 4;
    fill (255);
    switch(blockSize) {
      case 1: //240-loadingLoc[blockCount]
        ellipse ((loadingLoc[blockCount]+20)*5, 5*cmF-40, 30, 30);
        rect (((loadingLoc[blockCount]+20)*5)-15, 5*cmF-40, 60, 80);
        break;
      default:
        ellipse (1200-400, 5*cmF-40, 30, 30);
        rect (1200-415, 5*cmF-40, 60, 80);
        break;
    }
  }
  else if ((hardLeftCount - 2)%4 == 0) { //going south
    cmF = cmF - 4;
    fill (255);
    ellipse (5*cmF, 127.5, 30, 30); // KEY DISTANCE FROM WALL: 6.5 inches or 82.55 pixels plus size of robot: 25.5cm
    rect (5*cmF, 82.5, 80, 60);
  }
  else if ((hardLeftCount - 3)%4 == 0) { // draw going east
    cmF = cmF - 4;
    fill (255);
    switch(blockSize) {
      case 1: // south block
        ellipse (127.5, 600-(5*cmF), 30, 30);  //600-(5*cmF)
        rect (82.5, 520-(5*cmF), 60, 80);
        break;
      case 2: // to east dock
        if (blockCount > 4) { 
          ellipse (5*(loadingLoc[blockCount] - 3), 600-(5*cmF), 30, 30);
          rect (5*(loadingLoc[blockCount]-3)-45, 520-(5*cmF), 60, 80); //x was (111*5)-45
        }
        else {
          ellipse (111*5, 600-(5*cmF), 30, 30);
          rect ((111*5)-45, 520-(5*cmF), 60, 80);
        }
        break;
    }
  }
  else if ((hardLeftCount - 4)%4 == 0) { // draw going north
    cmF = cmF - 4;
    fill (255);
    ellipse (1200-(5*cmF), 600-127.5, 30, 30); //y originally 600-312.5
    rect (1120-(5*cmF), 585-127.5, 80, 60);  //y originally 585-312.5
  }
}

// this function sets a front sonar reading found after a 90 degree turn //
// in RL this function turns the robot 90 degrees //
void hardLeft () {
  hardLeftCount++;
  if ((hardLeftCount - 1)%4 == 0) { //start of facing west
    cmF = 103;
  }
  else if ((hardLeftCount - 2)%4 == 0) { //start of facing south
    switch(blockSize) {
      case 1:
        cmF = loadingLoc[blockCount]+20;
        break;
      default:
        cmF = 160;
        break;
    }
    blockSize = 0; // undefined
  }
  else if ((hardLeftCount - 3)%4 == 0) { // start of facing east
    switch(blockSize) {
      case 1: // south block
        cmF = 94.5;
        break;
      case 2:  //east block
        cmF = 94.5;
        break;
    }
  }
  else if ((hardLeftCount - 4)%4 == 0) { // start of facing north
    switch(blockSize) {
      case 1: // from south wall
        cmF = 214.5;
        break;
      case 2:  //along east wall
        if (blockCount > 4) {
          cmF = 240 - loadingLoc[blockCount];
        }
        else {
          cmF = 127.6;
        }
        break;
    }
  }
}

void goWest() {
  if (cmF > 25.5) {
    parallelMove(5);
  }
  else if (cmF <= 25.5) {
  hardLeft();
  }
}

void goEast() {
  switch(blockSize) {
    case 0:
      print("error in goEast");
      break;
    case 1: // deliver south block
      if (cmF > southLocF[southColorLoc[currentBlockColor]]) {
        parallelMove(2);
      }
      else if (cmF <= southLocF[southColorLoc[currentBlockColor]]) {
        dropOffBlock();
        while (cmF > 25.5) {  //southLocF[5]) {
          parallelMove(1);
        }
        hardLeft();
      }
      break;
    case 2: //deliver east block
      if (cmF > 25.5) {
        parallelMove(5);
      }
      else if (cmF <= 25.5) {
        hardLeft();
      }
      break;
  }
}

void goNorth() {
  switch(blockSize) {
    case 1:
      if (cmF > (240-loadingLoc[blockCount])) {
        parallelMove(5);
      }
      else if (cmF <= (240-loadingLoc[blockCount])) {
        hardLeft();
      }
      break;
    case 2:
      if (cmF > eastLocF[eastColorLoc[currentBlockColor]]) {
        parallelMove(2);
      }
      else if (cmF <= eastLocF[eastColorLoc[currentBlockColor]]) {
        dropOffBlock();
        while (cmF > 85) {
          parallelMove(5);
        }
        hardLeft();
      }
      break;
  }
}

void goSouthForBlock() {
  switch(blockSize) {
    case 0:  // air block and default case 
      // pick up blocks on hardLeftCount == 2, 6, 10, 14, 18, 22, ... when (HLC - 2)%4 ==0
      if (cmF > loadingLoc[blockCount]) {
        parallelMove(5);
      }
      else if (cmF < (loadingLoc[blockCount] - 3)) {
        pickUpBlock();
      }
      else {
        parallelMove(2);
      }
      break;
    case 1:  // south block
      if (cmF > 24.0) { //ideally 25.5
        parallelMove(5);
      }
      else if (cmF <= 24.0) {
        hardLeft();
      }
      break;
    case 2: // eastern bloc
      if (cmF > 112) {
        parallelMove(5);
      }
      else if (cmF <= 112) {
        hardLeft();
      }
      break;
  }
}

void pickUpBlock() {
  blockSize = testLoadingSize[blockCount];
  currentBlockColor = testLoadingColors[blockCount];
  if (blockSize == 0) { // air block
    print("air block rejected, saving location");
    println();
    if (loadingLoc[14] == 0) {
      loadingLoc[14] = loadingLoc[blockCount];
      print("location " + loadingLoc[14]);
      println();
    }
    else {
      loadingLoc[15] = loadingLoc[blockCount];
      print("location " + loadingLoc[15]);
      println();
    }
    dropOffBlock();
  }
  blockCount = blockCount + 1;
  delay(1000);
}

void dropOffBlock() {
  currentBlockColor = 0;
  delay(1000);
}
  
void readEastColors() { //old function from robot tests
  if (cmF > 140){
    northCount = 0;
  }
  if (cmF < 85 && northCount > 4){
    hardLeft();
    northCount = 0;
  }
  if (cmF > eastLocF[0] && northCount == 0){
    parallelMove(5);
  }
  else if (cmF < eastLocF[0] && cmR >eastLocR[0]-10.0 && northCount == 0) { // && RCTime(11) < QTIref) { // < 8000){ 
    delay(200);
    northCount++;
  }
  else if (cmF < eastLocF[1] && cmR >eastLocR[1]-10.0 && northCount == 1) { //&& RCTime(11) < QTIref) { // < 8000){
    delay(200);
    northCount++;
  }
  else if (cmF < eastLocF[2] && cmR >eastLocR[2]-10.0 && northCount == 2){ //&& RCTime(11) < QTIref) { // < 8000){
    delay(200);
    northCount++;
  }
  else if (cmF < eastLocF[3] && cmR >eastLocR[3]-10.0 && northCount == 3){ //&& RCTime(11) < QTIref) { // < 8000){
    delay(200);
    northCount++;
  }
  else if (cmF < eastLocF[4] && cmR >eastLocR[4]-10.0 && northCount == 4){ //&& RCTime(11) < QTIref - 1000) { // < 8000){
    delay(200);
    northCount++;
  }
  else if (cmF < eastLocF[5] && cmR >eastLocR[5]-10.0 && northCount == 5){ // && RCTime(11) < QTIref - 1000) { // < 8000){
    delay(200);
    northCount++;
  }
  else{
    parallelMove(5);
    //QTIref = RCTime(11);
    //QTIref = (QTIref + RCTime(11))/2 ;
  }
}


void drawWestWall() {
  float x = 31.75;
  float y = 76.2;
  fill(255);
  rect(188.9, 0.1, 539.75, 82.55);
  float xx = 385.75 + 38.1;
  fill(100, 100, 100);
  rect(917.3, 0.1, 282.6, 282.6);
  fill(0);
  rect(195.25, 0.1, x, y);
  rect(233.35, 0.1, x, y);
  rect(271.45, 0.1, x, y);
  rect(309.55, 0.1, x, y);
  rect(347.65, 0.1, x, y);
  rect(385.75, 0.1, x, y);
  rect(xx, 0.1, x, y);
  rect(xx=xx+38.1, 0.1, x, y);
  rect(xx=xx+38.1, 0.1, x, y);
  rect(xx=xx+38.1, 0.1, x, y);
  rect(xx=xx+38.1, 0.1, x, y);
  rect(xx=xx+38.1, 0.1, x, y);
  rect(xx=xx+38.1, 0.1, x, y);
  rect(xx=xx+38.1, 0.1, x, y);
}

void drawBlocks() {
  for (int w = 13; w >= blockCount; w--) {
    if (testLoadingColors[w] == 0) { fill(255, 0, 0); }//red
    else if (testLoadingColors[w] == 1) { fill(255, 126, 0); }//orange
    else if (testLoadingColors[w] == 2) { fill(255, 255, 0); }//yellow
    else if (testLoadingColors[w] == 3) { fill(0, 255, 0); }//green
    else if (testLoadingColors[w] == 4) { fill(0, 0, 255); }//blue
    else if (testLoadingColors[w] == 5) { fill(61, 43, 31); }//brown
    rect(5*loadingLoc[w], 76.2-((2+testLoadingSize[w])*12.7), 19.05, (2+testLoadingSize[w])*12.7);
  }
}

void drawEastWall() {
  fill(255);
  rect(0, 447.6, 152.4, 6.35);
  rect(152.4, 447.6, 6.35, 152.4);
  rect(561.95, 530.1, 234.95, 69.85);
  fill(0, 255, 0);
  rect(568.3, 536, 31.75, 63.5);
  fill(0, 0, 255);
  rect(606.4, 536, 31.75, 63.5);
  fill(255, 126, 0);  //orange
  rect(644.5, 536, 31.75, 63.5);
  fill(61, 43, 31);  //brown
  rect(682.6, 536, 31.75, 63.5);
  fill(255, 255, 0);
  rect(720.7, 536, 31.75, 63.5);
  fill(255, 0, 0);
  rect(758.8, 536, 31.75, 63.5);
}

void drawSouthWall() {
  fill(255);
  rect(0, 107.95, 57.15, 234.95);
  fill(0, 255, 0);
  rect(0, 114.3, 50.8, 31.75);
  fill(0, 0, 255);
  rect(0, 152.4, 50.8, 31.75);
  fill(255, 126, 0); //orange
  rect(0, 190.5, 50.8, 31.75);
  fill(61, 43, 31);  //brown
  rect(0, 228.6, 50.8, 31.75);
  fill(255, 255, 0);
  rect(0, 266.7, 50.8, 31.75);
  fill(255, 0, 0);
  rect(0, 304.8, 50.8, 31.75);
}

void meltDown() {
    fill(255);
    textSize(64);
    text("Game Over Man, GAME OVER!", 100, 200); 
    rect(0, 0, random(1200), random(600));
}
  
