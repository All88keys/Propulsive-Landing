#include <Servo.h>

//TVC
int TVC_TOP_PIN = 5;
int TVC_X_PIN = 20;
int TVC_Y_PIN = 21;
Servo tvcTop;
Servo tvcX;
Servo tvcY;
int tvcXoffset=85;
int tvcYoffset=83;
//float beta=.9094;
float beta=.6;
int gear=9;
float damper=1.5;
int tvcDelay=10;

long uLast [2]={0, 0};



void setup() {
  // put your setup code here, to run once:
  tvcX.attach(TVC_X_PIN);
  tvcY.attach(TVC_Y_PIN);
  tvcTop.attach(TVC_TOP_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:
    tvc(0, 0, beta, tvcDelay*100);
    /*
    tvc(30, 0, beta, tvcDelay*100);
    tvc(-30, 0, beta, tvcDelay*100);
    tvc(0, 0, beta, tvcDelay*100);
    tvc(0, 30, beta, tvcDelay*100);
    tvc(0, -30, beta, tvcDelay*100);
    tvc(0, 0, beta, tvcDelay*100);
    */
    delay(2000);
}

void tvc(int x, int y, double gamma, int d){
  double u=(x)*cos(gamma)+(y)*sin(gamma);
  double v=(y)*cos(gamma)-(x)*sin(gamma);
  /*
  if (u-uLast[0]>2){
    u=uLast[0]+2;
  }
  else if (u-uLast[0]<-2){
    u=uLast[0]-2;
  }

  if (v-uLast[1]>2){
    v=uLast[1]+2;
  }
  else if (v-uLast[1]<-2){
    v=uLast[1]-2;
  }
  uLast[0]=u; uLast[1]=v;
  */
  tvcY.write(v+tvcYoffset);tvcX.write(u+tvcXoffset); delay(d);
  
}
