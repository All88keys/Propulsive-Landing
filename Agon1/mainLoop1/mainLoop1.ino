//includes
#include "I2Cdev.h"     //I2C protocol
#include "MPU6050.h"    //imu
#include "Wire.h"
#include <Eigen313.h>
//#include <stlport.h>
//#include <Eigen30.h>
using namespace Eigen;
#include <SD.h>
#include<SPIFlash.h>
#include <Servo.h>

long MEGA=1000000;
long pi=31415;

//get accel, process y
int16_t ax, ay, az;
int16_t gx, gy, gz;
int imuRange=32768;         //range of raw values that accelerometer will output
int aSensitivity=8;   //how many g's represented in that range
int gSensitivity=250;
long lastAcc [6];
long aMult=imuRange/9.81/aSensitivity;
long gMult=imuRange/gSensitivity;
MPU6050 imu;
MatrixXi bodyEuler(3,3);
MatrixXi R1(3,3);
MatrixXi R2(3,3);
MatrixXi R3(3,3);

//controller
float KABiterTimes [9];
int KABcount;
MatrixXi K(2, 6) [9];
VectorXi x(6);
VectorXi u(2);
VectorXi dx(6);
MatrixXi A [9];
MatrixXi B [9];
long maxU=5*MEGA/10000*pi/180;

//yaw
long yaw;
long lastyaw;

//observer
MatrixXi L(6,4);
MatrixXi C(4,6);
//initialize C<<1, 0, 0, 0, 0, 0....;
VectorXi y(4);

//modes
int n=0;

//clock
long t0;
long t;
long h;

//mission parameters
long tb;
long tbEstDelay;
long setupWait=600;
long warningWait=240;
long warning2Wait=50;
long countdownWait=10;

float dropTBuffer=.1;
float dropBuffer=1;
float dropGoal=-9.81;

float fireTBuffer=.1;
float fireBuffer=.25;
float fireGoal=-9.81;

float landTBuffer=1.5;
float landBuffer=.5;
float landGoal=0;

//mision variables
long dropTime;
long fireTime;
long landTime;
bool buffStatus;
float buffCountDown;
//float accAvg;
//float accCount;


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
int * blinkMaster;
int blinkCommand;
long nextBlink;
bool blinkFlag=false;

//sd card
Sd2Card card;
SdVolume volume;
SdFile root;
const int SDchipSelect = 0;

//flash
#define CHIPSIZE MB64
SPIFlash flash(1);

//record
long dp [31];
String sBegin="begin     ";
String sEnd="     end";
String newPoint="n";
String sModeSwitch="";
uint32_t dpAddr;

//TVC
int TVC_TOP_PIN = 5;
int TVC_X_PIN = 20;
int TVC_Y_PIN = 21;
Servo tvcTop;
Servo tvcX;
Servo tvcY;


void setup() {
  
  delay(2000);

  //blink
    blinkMaster=new int[9*10];
    pinMode(G_LED_1, OUTPUT);
    pinMode(B_LED_1, OUTPUT);
    pinMode(G_LED_2, OUTPUT);
    pinMode(B_LED_2, OUTPUT);
  
  // accel, processY
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    //initialize serial
    Serial.begin(38400);
    //initialize peripherals
    imu.initialize();
    imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    imu.CalibrateGyro(10);
    imu.CalibrateAccel(25);
    
    flash.begin(9600);
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
        imu.CalibrateGyro(20);
        imu.CalibrateAccel(50);
      }
    break;
    case 2: //clear the pad, calibrate   &&&   blue/green
      if (t>(setupWait+warningWait+warning2Wait)*MEGA){
        n++; blinkFlag=true;
      }
    break;
    case 3: //countdown   &&&   green//off
      accAvg+=az/MEGA;
      accCount++;
      if (t>(setupWait+warningWait+warning2Wait+countdownWait)*MEGA){
        //turn top servo
        n++; blinkFlag=true;
        buffStatus=false; buffCountDown=dropTBuff;
      }
    break;
    case 4: //drop   &&&   nothing
      if(ddz<accAvg/accCount-dropBuff){
        if (buffStatus){buffCountDown-=h;}
        else{buffStatus=true;}
      }
      else{
        if (buffStatus){buffStatus=false; buffCountDown=dropTBuff;}
      }
      if (buffCountDown<0){
        dropTime=t-dropTBuffer;
        buffStatus=false; buffCountDown=dropTFire;
        accAvg=0;accCount=0;
        n++; blinkFlag=true;
      }
    case 5: //ballistic (control tvc)   &&&  blue
      accAvg+=az/MEGA;
      accCount++;
      if (t>dropTime+tb-tbEstDelay){
        //fire pyros
        n++; blinkFlag=true;
      }
    break;
    case 6: //fire pyros, check for accel   &&& nothing
      if(ddz>accAvg/accCount+fireBuff){
        if (buffStatus){buffCountDown-=h;}
        else{buffStatus=true;}
      }
      else{
        if (buffStatus){buffStatus=false; buffCountDown=fireTBuff;}
      }
      if (buffCountDown<0){
        fireTime=t-fireTBuffer;
        buffStatus=false; buffCountDown=landTFire;
        accAvg=0;accCount=0;
        n++; blinkFlag=true;
      }
    break;
    case 7: //tvc    &&& green
      digitalWrite(G_LED_1, LOW);
      digitalWrite(G_LED_2, LOW);
      digitalWrite(B_LED_1, HIGH);
      digitalWrite(B_LED_2, HIGH);
      if((ddz+landBuff<landGoal) && (ddz-landBuff>landGoal)){
        if (buffStatus){buffCountDown-=h;}
        else{buffStatus=true;}
      }
      else{
        if (buffStatus){buffStatus=false; buffCountDown=landTBuff;}
      }
      if (buffCountDown<0){
        landTime=t-landTBuffer;
        n++; blinkFlag=true;
      }
    break;
    case 8: //land, turn off, transfer data     &&&     blue/green
      digitalWrite(G_LED_1, HIGH);
      digitalWrite(G_LED_2, HIGH);
      digitalWrite(B_LED_1, LOW);
      digitalWrite(B_LED_2, LOW);
      //turn on leds
      //turn off stuff
      //transfer data
      n++; blinkFlag=true;
    break;
    case 9: //ready to turn off         &&&     blue
      digitalWrite(G_LED_1, HIGH);
      digitalWrite(G_LED_2, HIGH);
      digitalWrite(B_LED_1, HIGH);
      digitalWrite(B_LED_2, HIGH);
      while(1){}
    break;
  }

  if ((n>1) & (n<9)){

    //iterKAB
    if (n==5) {KABcount=1;}
    else if (n>5){
      if ((t>tb+KABiterTimes[KABcount+1]*MEGA)&(KABcount<9)){KABcount++;}
    }
    
    dp[0]=t; dp[1]=n; 
    
    //get accel
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    dp[2]=ax; dp[3]=ay; dp[4]=az; dp[5]=gx; dp[6]=gy;  dp[7]=gz;

  /*
  deuler=inv([1 0 -sin(euler(1)); 0 cos(euler(2)) sin(euler(2))*cos(euler(1)); 0 -sin(euler(2)) cos(euler(2))*cos(euler(1))])*w;
    P=[1 0 0; 0 cos(euler(1)) sin(euler(1)); 0 -sin(euler(1)) cos(euler(1))];
    Q=[cos(euler(2)) 0 -sin(euler(2)); 0 1 0; sin(euler(2)) 0 cos(euler(2))];
    R=[cos(euler(3)) sin(euler(3)) 0; -sin(euler(3)) cos(euler(3)) 0; 0 0 1];
    DCM=inv(P*Q*R);
    ve=DCM*vb;
    */
    
    //processY
    bodyEuler(0, 0)=1; bodyEuler(0, 2)=-sin(x(0)); 
    bodyEuler(1, 0)=1; bodyEuler(1, 2)=-sin(x()); 
    bodyEuler(2, 0)=1; bodyEuler(2, 2)=-sin(x()); 
    long 

    
    y[0]=x[0]+h*(lastAcc[0]+ax)/2/aMult; lastAcc[0]=ax;
    y[2]=x[3]+h*(lastAcc[1]+ay)/2/aMult; lastAcc[1]=ay;
    //y[2]=tstep*(last[2]+az)/2/aMult; last[2]=az;
    y[3]=gx/gMult;
    y[1]=gy/gMult;
    yaw+=h*(gz+lastyaw)/2/gMult; lastyaw=gz;
    //body rates
    dp[8]=y[0]; dp[9]=y[1]; dp[10]=y[2]; dp[11]=y[3]; dp[12]=yaw;
  
    //iter K, A, B
  
    //process u
    u=K[KABcount]*x; 
    dp[13]=u[0]; dp[14]=u[1];
    long norm=sqrt(u(0)*u(1));
    if (norm>maxU){
      u(0)*=maxU/norm;
      u(1)*=maxU/norm;
    } 
    dp[15]=u[0]; dp[16]=u[1];
    float yawf=yaw/MEGA*3.14159/180;
    long a=u[0]; long b=u[1];
    u[0]=a*cos(yawf)+b*sin(yawf);
    u[1]=b*cos(yawf)-a*sin(yawf);
    dp[17]=u[0]; dp[18]=u[1];
  
    //send u
  
    //dx, x
      if (n<6){u*=0;}
      dx=A[KABcount]*x+B[KABcount]*u+L*(y-C*x);
      dp[19]=dx[0]; dp[20]=dx[1]; dp[21]=dx[2]; dp[22]=dx[3]; dp[23]=dx[4]; dp[24]=dx[5];
      x=x+dx*h;
      dp[25]=x[0]; dp[26]=x[1]; dp[27]=x[2]; dp[28]=x[3]; dp[29]=x[4]; dp[30]=x[5];
    
    //record
    dpAddr = flash.getAddress(sizeof(String));
    if(flash.writeStr(dpAddr, newPoint)){
        while(1){}
    }
    
    for (uint8_t i = 0; i < arrayLen(dp); i++) {
      dpAddr = flash.getAddress(sizeof(long));
      if (!flash.writeFloat(dpAddr, dp[i])){
        while(1){}
      }
    }
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
    Serial.println(nextBlink);
  }


}
