//includes
#include "I2Cdev.h"     //I2C protocol
#include "MPU6050.h"    //imu
#include "Wire.h"
#include <Eigen313.h>
//#include <stlport.h>
//#include <Eigen30.h>
using namespace Eigen;
#include <LU>
#include <SD.h>
#include<SPIFlash.h>
#include <Servo.h>

long MEGA=1000000;
long pi=31416;

//modes
int n=0;
int c=0;

//clock
long t0;
long t;
long h;

//mission parameters
long tb=.5*MEGA;
long tbEstDelay=.25*MEGA;
long setupWait=60; //120
long warningWait=60;  //120
long warning2Wait=30;  //30
long countdownWait=10; //10
long calibrateTime=0;

float dropTBuffer=.05*MEGA;
long dropBuffer=3276/8;  //~1 m/s^2
long dropGoal=-32768/8; //-g

float fireTBuffer=.05*MEGA;
float fireBuffer=3276/4/8; //~.25 m/s^2
float fireGoal=-32768/8; //-g

float landTBuffer=.5*MEGA;
float landBuffer=3276/2/8; //
float landGoal=0;

//mision variables
long dropT;
long dropTime;
long fireTime;
long landTime;
bool buffStatus;
float buffCountDown;
//float accAvg;
//float accCount;

//get accel, process y
int16_t ax, ay, az;
int16_t gx, gy, gz;
long offsets [3];
long O1, O2, O3;
int imuRange=32768;         //range of raw values that accelerometer will output
int aSensitivity=8;         //how many g's represented in that range
int gSensitivity=250;
float aMult=imuRange/9.81/aSensitivity;
//long gMult=imuRange/gSensitivity*180/pi*10000;
float gMult=imuRange/gSensitivity;
MPU6050 imu;
long yaw;

//controller
MatrixXf Ka(2,6);
VectorXi x(6); 
VectorXf xControl(6); 
VectorXf ua(2);
float ub [2];
float uc [2];
MatrixXi A [9];
MatrixXi B [9];
float maxU=5*3.14159/180;
long uLast [2]={0, 0};

//TVC
int TVC_TOP_PIN = 5;
int TVC_X_PIN = 20;
int TVC_Y_PIN = 21;
Servo tvcTop;
Servo tvcX;
Servo tvcY;
int tvcXoffset=85;
int tvcYoffset=83;
float beta=.6;
int gear=9;
float damper=1.5;
int tvcDelay=10;

//pyro
int pyroPin = 22;

//blink
int B_LED_1 = 6;
int G_LED_1 = 4;
int B_LED_2 = 9;
int G_LED_2 = 8;
//each pattern has 9 values
//0: Hz
//1-4: A, command for each, green1, blue1, green2, blue2,
//5-8: B, command for each
int blink0 [] ={1, 1, 0, 1, 0, 1, 0, 1, 0};
int blink1 [] ={1, 0, 1, 0, 1, 0, 1, 0, 1};
int blink2 [] ={1, 0, 1, 0, 1, 1, 0, 1, 0};
int blink3 [] ={1, 1, 0, 1, 0, 0, 0, 0, 0};
int blink4 [] ={1, 0, 0, 0, 0, 0, 0, 0, 0};
int blink5 [] ={1, 0, 1, 0, 1, 0, 1, 0, 1};
int blink6 [] ={1, 0, 0, 0, 0, 0, 0, 0, 0};
int blink7 [] ={1, 1, 0, 1, 0, 1, 0, 1, 0};
int blink8 [] ={1, 0, 1, 0, 1, 1, 0, 1, 0};
int blink9 [] ={1, 0, 1, 0, 1, 0, 1, 0, 1};
int blinkMaster [90];
int blinkCommand;
long nextBlink;
bool blinkFlag=false;

long dispGap=100000;
long nextDisplay=dispGap;

#define CHIPSIZE MB64
SPIFlash flash(1);


void setup() {

    pinMode(G_LED_1, OUTPUT);
    pinMode(B_LED_1, OUTPUT);
    pinMode(G_LED_2, OUTPUT);
    pinMode(B_LED_2, OUTPUT);
    
    digitalWrite(G_LED_1, LOW);
    digitalWrite(B_LED_1, LOW);
    digitalWrite(G_LED_2, LOW);
    digitalWrite(B_LED_2, LOW);
    
    // accel, processY
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    

    flash.begin(9600);
    //flash.eraseChip();

    tvcX.attach(TVC_X_PIN);
    tvcY.attach(TVC_Y_PIN);
    tvcTop.attach(TVC_TOP_PIN);

    tvcTop.write(10); delay(1000);
    tvc(0, 0, beta, tvcDelay*100);
    tvc(30, 0, beta, tvcDelay*100);
    tvc(-30, 0, beta, tvcDelay*100);
    tvc(0, 0, beta, tvcDelay*100);
    tvc(0, 30, beta, tvcDelay*100);
    tvc(0, -30, beta, tvcDelay*100);
    tvc(0, 0, beta, tvcDelay*100);
    
    
  //blink
    for (int i=0; i<9; i++){
      blinkMaster[i+9*0]=blink0[i]; blinkMaster[i+9*1]=blink1[i];
      blinkMaster[i+9*2]=blink2[i]; blinkMaster[i+9*3]=blink3[i];
      blinkMaster[i+9*4]=blink4[i]; blinkMaster[i+9*5]=blink5[i];
      blinkMaster[i+9*6]=blink6[i]; blinkMaster[i+9*7]=blink7[i];
      blinkMaster[i+9*8]=blink8[i]; blinkMaster[i+9*9]=blink9[i];
    }

  pinMode(pyroPin, OUTPUT);
  digitalWrite(pyroPin, LOW);
    
  x<<0,0,0,0,0,0;
  Ka<<0.34641, 1.72254, 0.32694, 0, 0, 0, 0, 0, 0, 0.34641, -1.88376, -0.3991;
  n=0;
  t=0;
}

void loop() {

  //clk update
  
  t=micros();
  h=t-t0;
  t0=t;

  // check state
  switch(n){
    case 0: //setup and idle  &&& green
      if (t>setupWait*MEGA){
        n++; blinkFlag=true;
      }
    break;
    case 1: //warning  &&&   blue
      if (t>(setupWait+warningWait)*MEGA){
        n++; blinkFlag=true;
        //imu.CalibrateGyro(20);
        //imu.CalibrateAccel(50);
      }
    break;
    case 2: //clear the pad, calibrate   &&&   blue/green
      if (t>(setupWait+warningWait+warning2Wait)*MEGA){
        n++; blinkFlag=true;

          digitalWrite(G_LED_1, LOW);
          digitalWrite(B_LED_1, LOW);
          digitalWrite(G_LED_2, LOW);
          digitalWrite(B_LED_2, LOW);
          
          imu.initialize();
          imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
          imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
          imu.CalibrateGyro(30); // Fine tune after setting offsets with less Loops.
          imu.CalibrateAccel(30);
      
          Serial.print(">");
          long aSums [3]={0,0,0};
          //float offsets
          for (int i=0; i<10; i++){
            for (int j=0; j<250; j++){
              imu.getMotion6(&ax, &az, &ay, &gx, &gz, &gy);
              aSums[0]+=ax;
              aSums[1]+=ay;
              aSums[2]+=az; 
      
            }
            Serial.print(".");
            delay(500);
          }
          Serial.println();
          for (int i=0; i<3; i++){
            aSums[i]/=2500;
            //Serial.println(aSums[i]);
          }
      
          offsets[0]=aSums[0];
          offsets[1]=aSums[1]-imuRange/2;
          offsets[2]=aSums[2];            

          calibrateTime=micros();
      }
    break;
    case 3: //countdown   &&&   green//off
      if (t>calibrateTime+countdownWait*MEGA){
        n++; blinkFlag=true;
        buffStatus=false; buffCountDown=dropTBuffer;
        tvcTop.write(150); delay(150);
        dropT=micros();
      }
    break;
    case 4: //drop   &&&   nothing
      if(az<dropGoal+dropBuffer){
        if (buffStatus){buffCountDown-=h; }
        else{buffStatus=true;}
      }
      else{
        if (buffStatus){buffStatus=false; buffCountDown=dropTBuffer; }
      }
      if ((buffCountDown<0) || (t>dropT+MEGA*.25)){
        dropTime=t-dropTBuffer;
        buffStatus=false; buffCountDown=dropTBuffer;
        //accAvg=0;accCount=0;
        n++; blinkFlag=true;
      }
    break;
    case 5: //ballistic (control tvc)   &&&  blue
      digitalWrite(pyroPin, HIGH);
      if (t>dropTime+tb-tbEstDelay){
        digitalWrite(pyroPin, HIGH);
        n++; blinkFlag=true; 
      }
    break;
    case 6: //fire pyros, check for accel   &&& nothing
      digitalWrite(pyroPin, HIGH);
      if(az>fireGoal+fireBuffer){
        if (buffStatus){buffCountDown-=h;}
        else{buffStatus=true;}
      }
      else{
        if (buffStatus){buffStatus=false; buffCountDown=fireTBuffer;}
      }
      if (buffCountDown<0){
        fireTime=t-fireTBuffer;
        buffStatus=false; buffCountDown=landTBuffer;
        //accAvg=0;accCount=0;
        n++; blinkFlag=true;
      }
    break;
    case 7: //tvc    &&& green
      digitalWrite(pyroPin, HIGH);
      if((az<landGoal+landBuffer) && (az>landGoal-landBuffer)){
        if (buffStatus){buffCountDown-=h;}
        else{buffStatus=true;}
      }
      else{
        if (buffStatus){buffStatus=false; buffCountDown=landTBuffer;}
      }
      if (buffCountDown<0){
        landTime=t-landTBuffer;
        n++; blinkFlag=true;
      }
    break;
    case 8: //land, turn off, transfer data     &&&     blue/green
      digitalWrite(pyroPin, LOW);
      //turn on leds
      //turn off stuff
      //transfer data
      blinkFlag=true;
    break;
  }
  if (n>2){
    c++;
    float Ox=x(4)*3.14/180/gMult/MEGA; 
    float Oy=x(1)*3.14/180/gMult/MEGA;
    float Oz=yaw*3.14/180/gMult/MEGA; 
  
    imu.getMotion6(&ax, &az, &ay, &gx, &gz, &gy);
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.println("\t");
  
    ax=-ax+offsets[0];
    ay=-ay+offsets[1]+imuRange/2;
    az=az-offsets[2];
  
    
    
    Matrix3f bodyEuler;
    bodyEuler<< 1,0,-sin(Ox),0,cos(Oy),sin(Oy)*cos(Ox),0,-sin(Oy),cos(Oy)*cos(Ox);
    Vector3f deuler;
    deuler<<gx, gy, gz; deuler*=(-1);
    deuler=bodyEuler.inverse()*deuler;
    if (c>5){
    x(4)+=deuler(0)*h; x(1)+=deuler(1)*h; yaw+=deuler(2)*h;
    }
    x(5)=deuler(0); x(2)=deuler(1);
    
    MatrixXf R1(3,3); R1<<1, 0, 0, 0, cos(Ox), sin(Ox), 0, -sin(Ox), cos(Ox);
    MatrixXf R2(3,3); R2<<cos(Oy), 0, -sin(Oy), 0, 1, 0, sin(Oy), 0, cos(Ox);
    MatrixXf R3(3,3); R3<<cos(Oz), sin(Oz), 0, -sin(Oz), cos(Oz), 0, 0, 0, 1;
    MatrixXf R(3,3); R=R1*R2*R3; 
    Vector3f ve; ve<<ax, ay, az;
    ve=R*ve;
    if (c>10){
      x(0)+=ve[0]*h; x(3)+=ve[1]*h;
    }
  
    xControl(0)=x(0)/aMult/MEGA/50;
    xControl(3)=x(3)/aMult/MEGA/50;
    xControl(1)=x(1)/gMult/MEGA/180*3.14;
    xControl(4)=x(4)/gMult/MEGA/180*3.14;
    xControl(2)=x(2)/gMult/180*3.14;
    xControl(5)=x(5)/gMult/180*3.14;
  
    ua=-Ka*xControl;
    
    float uNorm=sqrt(ua(0)*ua(0)+ua(1)*ua(1));
    //Serial.print("     "); Serial.print(uNorm); Serial.print("     "); Serial.println(maxU);
    if (uNorm>maxU){
      //Serial.print("TRUE  ");
      ub[0]=ua(0)/uNorm*maxU;
      ub[1]=ua(1)/uNorm*maxU;
    }
    else {
      ub[0]=ua(0); ub[1]=ua(1);
    }
  
    uc[0]=(ub[0])*cos(Oz)+(ub[1])*sin(Oz);
    uc[1]=(ub[1])*cos(Oz)-(ub[0])*sin(Oz);
  
    //Serial.print(u(0)); Serial.print("\t"); 
    //Serial.println(u(1));
    
    if((n>3)&(n<8)){
      int x=uc[0]*180*gear/3.14;
      int y=uc[1]*180*gear/3.14;
      bool newU=((x!=uLast[0]) || (y!=uLast[1]));
      if (newU){
        tvc(x, y, beta, tvcDelay);
        uLast[0]=x; uLast[1]=y;
      }
    }
    
    
    
    //Serial.print(xControl(1)); Serial.print("\t");
    //Serial.println(xControl(4));
  }

  //blink
  if ((t>nextBlink)||(blinkFlag)){
    blinkFlag=false;
    //if (blinkCommand==1) {blinkCommand=0;} else {blinkCommand=1;}
    blinkCommand=~blinkCommand;
    if (blinkMaster[9*n-4*blinkCommand+1]){
      digitalWrite(B_LED_1, LOW);
    }
    else{
      digitalWrite(B_LED_1, HIGH);
    }
    if (blinkMaster[9*n-4*blinkCommand+2]){
      digitalWrite(G_LED_1, LOW);
    }
    else{
      digitalWrite(G_LED_1, HIGH);
    }
    if (blinkMaster[9*n-4*blinkCommand+3]){
      digitalWrite(B_LED_2, LOW);
    }
    else{
      digitalWrite(B_LED_2, HIGH);
    }
    if (blinkMaster[9*n-4*blinkCommand+4]){
      digitalWrite(G_LED_2, LOW);
    }
    else{
      digitalWrite(G_LED_2, HIGH);
    }
    nextBlink=t+MEGA/blinkMaster[9*n];
  }

  if (n>2){
    void record();
  }


}

void tvc(int x, int y, double gamma, int d){
  double u=(x)*cos(gamma)+(y)*sin(gamma);
  double v=(y)*cos(gamma)-(x)*sin(gamma);
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
  tvcY.write(v+tvcYoffset);tvcX.write(u+tvcXoffset); delay(d);
}

void record(){
  long dataL [17]={n, t, ax, ay, az, gx, gy, gz, x(0), x(1), x(2), x(3), x(4), x(5), yaw, uLast[0], uLast[1]};
  float dataF [12]={xControl(0), xControl(1), xControl(2), xControl(3), xControl(4), xControl(5), ua(0), ua(1), ub[0], ub[1], uc[0], uc[1]};
  bool newPoint[4]={false, false, false, false};

  
  uint32_t floatAddr [12];
  uint32_t longAddr [17];
  uint32_t newPointAddr[4];
  bool success;

  for (uint32_t i = 0; i < arrayLen(newPointAddr); i++) {
    newPointAddr[i] = flash.getAddress(sizeof(bool));
    success=flash.writeLong(newPointAddr[i], newPoint[i]);
    if (!success){
      Serial.println("flashFail");
    }
  }
  for (uint32_t i = 0; i < arrayLen(longAddr); i++) {
    longAddr[i] = flash.getAddress(sizeof(long));
    success=flash.writeLong(longAddr[i], dataL[i]);
    if (!success){
      Serial.println("flashFail");
    }
  }
  for (uint32_t i = 0; i < arrayLen(floatAddr); i++) {
    floatAddr[i] = flash.getAddress(sizeof(float));
    success=flash.writeFloat(floatAddr[i], dataF[i]);
  }

  
}
