import processing.serial.*;

Serial myPort;
int val;

void setup() 
{
  size(1000, 1000);
  background(255);
  String portName = Serial.list()[1];
  println(portName);
  myPort = new Serial(this, portName, 115200);
}

void draw()
{
  if ( myPort.available() > 0) {
    String inBuffer = myPort.readStringUntil('\n');
    if (inBuffer != null) {
      String[] messages = inBuffer.split("\\s+");
      
      if (messages.length != 2) {        
        println("Invalid message size: ", messages.length);
        return;
      }
      
      Float h1 = Float.parseFloat(messages[0]);
      int d1 = Integer.parseInt(messages[1]);
      
      // scale and convert polar to cartesian
      float r = d1 / 1;
      float theta = radians(h1);
      float x = r * cos(theta);
      float y = r * sin(theta);

      fill(255, 0 , 0);
      stroke(255);
      ellipse(500, 500, 5, 5);

      stroke(0);
      point(500+x, 500+y);
   }
  }
}
