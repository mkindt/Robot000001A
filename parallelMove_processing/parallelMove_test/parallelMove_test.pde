float len = 100;
float x = 0;
float y = 100;
float offx,offy,diffx,diffy;
void setup() {
  size(600, 300);
  background(0);
}

void draw() {
  stroke(255);
  len+=5;
  for(float angle = 0; angle <=PI; angle+=0.1){
    offx = sin(angle) * len;
    offy = cos(angle) * len;
    diffx = offx - x;
    diffy = offy - y;
    x = x + diffx;
    y = y + diffy;
    line(x, y, offx+20, offy+20);
  }
 // if(mousePressed) {
  //  line(mouseX, mouseY, pmouseX, pmouseY);
  //}
}
