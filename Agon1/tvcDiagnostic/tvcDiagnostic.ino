#include <Servo.h>

//TVC
int TVC_TOP_PIN = 5;
int TVC_X_PIN = 20;
int TVC_Y_PIN = 21;
Servo tvcTop;
Servo tvcX;
Servo tvcY;

int a=78;
int b=88;
float beta=.9094;
int movement=45;
int dConstant=500;

void setup() {
  
  // put your setup code here, to run once:
  tvcX.attach(TVC_X_PIN);
  tvcY.attach(TVC_Y_PIN);

  //tvcY.write(b);tvcX.write(a);delay(500);
  
  tvc(0, 0, beta, dConstant*5);
  tvc(movement, 0, beta, dConstant*5);
  tvc(0, 0, beta, dConstant*5);
  tvc(-movement, 0, beta, dConstant*5);
  tvc(0, 0, beta, dConstant*5);

  tvc(0, 0, beta, dConstant*5);
  tvc(0, movement, beta, dConstant*5);
  tvc(0, 0, beta, dConstant*5);
  tvc(0, -movement, beta, dConstant*5);
  tvc(0, 0, beta, dConstant*5);
  
  
  
  
  
}

void tvc(int x, int y, double gamma, int d){
  double u=(x)*cos(gamma)+(y)*sin(gamma);
  double v=(y)*cos(gamma)-(x)*sin(gamma);

  //double r=(a)*cos(gamma)+(b)*sin(gamma);
  //double s=(b)*cos(gamma)-(a)*sin(gamma);
  
  tvcY.write(v+b);tvcX.write(u+a); delay(d);
}

void loop() {
  // put your main code here, to run repeatedly:

}
