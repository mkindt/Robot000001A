float len = 100;
float x,y,offx,offy,diffx,diffy;
void setup() {
  size(600, 300);
  background(0);
}

void draw() {
  stroke(255);
  len+=5;
  for(float angle = 0; angle <=PI; angle+=0.1){
    y = cos(angle) * len;
    x = sin(angle) * len;
    line(0,0,x, y);
  }
 // if(mousePressed) {
  //  line(mouseX, mouseY, pmouseX, pmouseY);
  //}
}
