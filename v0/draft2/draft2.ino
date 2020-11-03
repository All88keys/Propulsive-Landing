//includes
#include "I2Cdev.h"     //I2C protocol
#include "MPU6050.h"    //imu
#include "Wire.h"

//imu raw values
int16_t ax, ay, az;
int16_t gx, gy, gz;

int imuRange=32768;         //range of raw values that accelerometer will output
int aSensitivity=8;   //how many g's represented in that range
int gSensitivity=250;
long x [9];
long last [6];
float disp [9];
long aMult=imuRange/9.81/aSensitivity;
long gMult=imuRange/gSensitivity;

long lastT=0;
long t=0;
long tstep=0;

int Pyro1 = 22;
int Pyro2 = 23;

//peripheral declarations
MPU6050 imu;

void setup() {
  delay(2000);
  
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
    imu.CalibrateAccel(5);
   Serial.println("hello");
   delay(6000);

    //The following code MUST BE REMOVED before you connect anything to the pyro channels
    

    
}

void loop() {
  /*
  Serial.println("We'll now cycle through each channel, turning each one on for 2 seconds");
    delay(1000);
    digitalWrite(Pyro1, HIGH);
    Serial.println("Pyro 1 is on!");
    delay(5000);
    digitalWrite(Pyro1, LOW);
    Serial.println("Pyro 1 is off");
    delay(5000);
    digitalWrite(Pyro2, HIGH);
    Serial.println("Pyro 2 is on!");
    delay(5000);
    digitalWrite(Pyro2, LOW);
    Serial.println("Pyro 2 is off");
    delay(1000);
    */
  
  t=micros(); tstep=t-lastT; lastT=t;
  //Serial.println(tstep);

  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  az-=imuRange/aSensitivity;
  
  x[0]+=tstep*(last[0]+ax)/2/aMult; last[0]=ax;
  x[1]+=tstep*(last[1]+ay)/2/aMult; last[1]=ay;
  x[2]+=tstep*(last[2]+az)/2/aMult; last[2]=az;

  x[3]=gx/gMult;
  x[4]=gy/gMult;
  x[5]=gz/gMult;

  x[6]+=tstep*(last[3]+x[3])/2; last[3]=x[3];
  x[7]+=tstep*(last[4]+x[4])/2; last[4]=x[4];
  x[8]+=tstep*(last[5]+x[5])/2; last[5]=x[5];

  for (int i=0; i<9; i++)
  {
    disp[i]=x[i];
    if ((i<3) | (i>5)){
      disp[i]/=1000000;
    }
    if (i>5){
      Serial.print(disp[i]); Serial.print("\t");
    }
  }
  Serial.println();
}
