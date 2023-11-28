#include <Servo.h>
#include <math.h>
#include <Ramp.h>


Servo alpha_servo;  // create servo object to control a servo
Servo theta1_servo;  // create servo object to control a servo
Servo theta2_servo;  // create servo object to control a servo
Servo claw_servo;  // create servo object to control a servo
// twelve servo objects can be created on most boards


rampDouble  alpha_ramp;
rampDouble  theta1_ramp;
rampDouble  theta2_ramp;
rampDouble  claw_ramp;


double l1 = 7.4;
double l2 = 7.4; //TODO


int xPin = A2;
int yPin = A1;
int zPin = A0;
int clawPin = 10;


void inverseKinematics(double x, double y, double z) {
  double alpha = atan2(y, x);
  double theta_3 = -acos((x*x + y*y + z*z - l1*l1 - l2*l2) / (2 * l1 * l2));
  double theta_1 = atan(z / sqrt(x*x+y*y)) - atan(l2 * sin(theta_3) / (l1 + l2 * cos(theta_3)));
  double theta_2 = theta_1+theta_3;
  alpha *= 180 / PI;
  theta_2 *= 180 / PI;
  theta_1 *= 180 / PI;


  setState(alpha, theta_1, theta_2);
}


long ramp_duration = 500;


void setState(double alpha, double theta_1, double theta_2) {
  if(isnan(alpha) || isnan(theta_1) || isnan(theta_2)) {
    Serial.println("bad position");
    return;
  }
  Serial.print((String)alpha + ", " + (String)theta_1 + ", " + (String)theta_2);
  if(alpha<0||alpha>75||theta_1<38||theta_1>133||theta_2<-66||theta_2>20)Serial.print(" |OBO| ");
  alpha = map(alpha, 0, 75, 119, 33);
  theta_1 = map(theta_1, 38, 133, 145, 43);
  theta_2 =  map(theta_2, -66, 20, 30, 132);
  Serial.println("==>" + (String)alpha + ", " + (String)theta_1 + ", " + (String)theta_2);
 
  alpha_servo.write(alpha);
  theta1_servo.write(theta_1);
  theta2_servo.write(theta_2);
}


void setStateRaw(double alpha, double theta_1, double theta_2) {
  if(isnan(alpha) || isnan(theta_1) || isnan(theta_2)) {
    Serial.println("bad position");
    return;
  }
  // Serial.print((String)alpha + ", " + (String)theta_1 + ", " + (String)theta_2);
  // alpha = map(alpha, 0, 70, 119, 33);
  // theta_1 = map(theta_1, 38, 133, 145, 43);
  // theta_2 = map(theta_2, -145, -70, 30, 132);
  // Serial.print("==>" + (String)alpha + ", " + (String)theta_1 + ", " + (String)theta_2);
 
  alpha_servo.write(alpha);
  theta1_servo.write(theta_1);
  theta2_servo.write(theta_2);
}




void setup() {
  alpha_servo.attach(3);  // attaches the servo on pin 9 to the servo object
  theta1_servo.attach(5);  // attaches the servo on pin 9 to the servo object
  theta2_servo.attach(6);  // attaches the servo on pin 9 to the servo object
  claw_servo.attach(11);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
}


double lerp(double min, double max, double t) {
  return min + (max - min) * t;
}


void circle() {
  long ms = millis();
  double s = (double)ms/1000;
  double x = 7+3 * sin(s);
  double y = 3.5+3*cos(s);
  double z = 2;
  inverseKinematics(x, y, z);
}


void circle2() {
  long ms = millis();
  double s = (double)ms/500;
  double x = 5+3 * cos(s);
  double y = 5-3*cos(s);
  double z = 4+3*sin(s);
  inverseKinematics(x, y, z);
}


void rectangle() {
  long ms = millis();
  double s = fmod((double)ms/500, 6.0);
  double z = 3;
  double x;
  double y;
  if(s < 1.5) {
    y = 0;      // (3,0)->(9,0)
    x = s/1.5;
  } else if(s < 3) { //(9,0)->(9,9)
    x = 1;
    y = 1/1.5*(s-1.5);
  } else if (s < 4.5) { //9,9->3,9
    y = 1;
    x = 1-1/1.5*(s-3);
  } else {
    x = 0;
    y = 1-1/1.5*(s-4.5);
  }
  x*=6;
  x+=3;
  y*=6;
  y+=0;


  Serial.print((String)x+","+(String)y+","+(String)z+" ");
  inverseKinematics(x,y,z);
}


double x = 0; // [0,inf]
double y = 0; //
double z = 0;
bool claw = false;
bool lastClaw = false;


void thefunny() {
  claw_servo.write(25);
  delay(1000);
  inverseKinematics(7.31,0.01,5.88);
  delay(1000);
  claw_servo.write(70);
  delay(1000);
  inverseKinematics(7.31,0.01,1.88);
  delay(1000);
  thefunny();
}


void loop() {
  double analogX = (double)analogRead(xPin) / (1<<10);
  double analogY = (double)analogRead(yPin) / (1<<10);
  double analogZ = (double)analogRead(zPin) / (1<<10);
  thefunny();
  // IK
  x = lerp(5, 10, analogX);
  y = lerp(0, 10, analogY);
  z = lerp(0, 15, analogZ);




  // rectangle();






  // direct
  // setStateRaw(lerp(119,33,analogX), lerp(145,43,analogY), lerp(30,132,analogZ));
  // alpha is 0->70  map(alpha, 0, 70, 119, 33)
  // theta 38->133 map(theta_1, 38, 133, 145, 43)
  // theta2 -70->-145 map(theta_2, -66, 20, 30, 132)




  // alpha_ramp.update();
  // theta1_ramp.update();
  // theta2_ramp.update();
  // alpha_servo.write(alpha_ramp.getValue());
  // theta1_servo.write(theta1_ramp.getValue());
  // theta2_servo.write(theta2_ramp.getValue());
  // Serial.println(alpha_ramp.getValue());


  bool curClaw = digitalRead(clawPin) == HIGH;
  if (curClaw && !lastClaw) {
    claw = !claw;
  }
  lastClaw = curClaw;
  if(claw) {
    claw_servo.write(45);
    inverseKinematics(x, y, z);


  } else {
    inverseKinematics(7.31,0.01,10.88);
    delay(75);
    claw_servo.write(70);
  }
  // claw_servo.write(claw ? 45 : 70);
  // claw_ramp.update();
  // claw_servo.write(claw_ramp.getValue());
 
  Serial.print("x: ");
  Serial.print(x);
  Serial.print(", y: ");
  Serial.print(y);
  Serial.print(", z:");
  Serial.print(z);
  // Serial.print(", claw: ");
  // Serial.print(claw ? "CLOSED, " : "OPEN, ");
  // Serial.print("alpha:");
  // Serial.print(alpha_servo.read());
  // // Serial.print(theta1_ramp.update());
  // Serial.print(", theta1:");
  // Serial.print(theta1_servo.read());
  // Serial.print(", theta2:");
  // Serial.print(theta2_servo.read());
  // Serial.print(", claw:");
  // Serial.println(claw_servo.read());


}
