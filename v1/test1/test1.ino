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

//clock
long t0;
long t;
long h;

//get accel, process y
int16_t ax, ay, az;
int16_t gx, gy, gz;
long O1, O2, O3;
int imuRange=32768;         //range of raw values that accelerometer will output
int aSensitivity=8;   //how many g's represented in that range
int gSensitivity=250;
long lastAcc [6];
long aMult=imuRange/9.81/aSensitivity;
//long gMult=imuRange/gSensitivity*180/pi*10000;
long gMult=imuRange/gSensitivity;
MPU6050 imu;
MatrixXi L(6,4);
MatrixXi C(4,6);
//initialize C<<1, 0, 0, 0, 0, 0....;
VectorXi y(4);

long yaw;

//controller
float KABiterTimes [1];
int KABcount=0;
MatrixXf K(2, 6) [1];
VectorXi x(6);
VectorXi u(2);
VectorXi dx(6);
MatrixXi A [9];
MatrixXi B [9];
long maxU=5*MEGA/10000*pi/180;

K[0]<<1.57424, 7.41479, 1.3178, 0, 0, 0, 0, 0, 0, 1.57424, -8.15564, -1.63289;


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



void setup() {

    // accel, processY
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    imu.initialize();
    imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    
    pinMode(G_LED_1, OUTPUT);
    pinMode(B_LED_1, OUTPUT);
    pinMode(G_LED_2, OUTPUT);
    pinMode(B_LED_2, OUTPUT);

  //blink
    for (int i=0; i<9; i++){
      blinkMaster[i+9*0]=blink0[i]; blinkMaster[i+9*1]=blink1[i];
      blinkMaster[i+9*2]=blink2[i]; blinkMaster[i+9*3]=blink3[i];
      blinkMaster[i+9*4]=blink4[i]; blinkMaster[i+9*5]=blink5[i];
      blinkMaster[i+9*6]=blink6[i]; blinkMaster[i+9*7]=blink7[i];
      blinkMaster[i+9*8]=blink8[i]; blinkMaster[i+9*9]=blink9[i];
    }

}

void loop() {

  //clk update
  t=micros();
  h=t-t0;
  t0=t;

  if (t>nextDisplay){
    nextDisplay=t+dispGap;
    for (int i=0; i<6; i++){
      Serial.print(x(i)/MEGA); Serial.print("\t");
    }
    Serial.println();
  }

  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  float Ox=x(4)/MEGA*3.14159/180;
  float Oy=x(1)/MEGA*3.14159/180;
  float Oz=yaw/MEGA*3.14159/180;
  Matrix3f bodyEuler;
  bodyEuler<< 1,0,-sin(Ox),0,cos(Oy),sin(Oy)*cos(Ox),0,-sin(Oy),cos(Oy)*cos(Ox);
  Vector3f deuler;
  deuler<<gx, gy, gz;
  bodyEuler=bodyEuler.inverse();
  deuler=bodyEuler*(deuler/gMult);
  y(0)=deuler(0); y(1)=deuler(1);
  yaw+=deuler(2)*h;
  
  MatrixXf R1(3,3); R1<<1, 0, 0, 0, cos(Ox), sin(Ox), 0, -sin(Ox), cos(Ox);
  MatrixXf R2(3,3); R2<<cos(Oy), 0, -sin(Oy), 0, 1, 0, sin(Oy), 0, cos(Ox);
  MatrixXf R3(3,3); R2<<cos(Oz), sin(Oz), 0, -sin(Oz), cos(Oz), 0, 0, 0, 1;
  MatrixXf R(3,3); R=R1*R2*R3; 
  Vector3f ve; ve<<ax, ay, az; ve=ve*h/aMult;
  ve=R*ve; y(2)=ve(0); y(3)=ve(1);
  
  //dx=A[KABcount]*x+B[KABcount]*u+L*(y-C*x);
  x=x+dx*h;

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


}
