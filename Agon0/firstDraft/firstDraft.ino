//includes
#include "I2Cdev.h"     //I2C protocol
#include "MPU6050.h"    //imu
#include "Wire.h"
#include <Servo.h>

int TVCXpin = 20;
int TVCYpin = 21;
int TVCTpin = 5;
Servo TVCXservo;
Servo TVCYservo;
Servo TVCTservo;

//pin constants
int LED_BLU1=6;
int LED_GRN1=4;
int LED_BLU2=9;
int LED_GRN2=8;

//imu raw values
int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t imuRange=32768;         //range of raw values that accelerometer will output
int16_t aSensitivity=8;   //how many g's represented in that range
int16_t gSensitivity=250;
int16_t metric [] = {0, 0, 0, 0, 0, 0};
int32_t v [] ={0, 0, 0};
int32_t x [] ={0, 0, 0, 0, 0, 0};
double p [] ={0, 0, 0, 0, 0, 0};
double g=9.81;

int16_t t=0;
int16_t tstep=0;

//peripheral declarations
MPU6050 imu;

void setup() {
  delay(2000);
  
  TVCXservo.attach(TVCXpin);
  TVCXservo.write(90); delay(500);
  TVCXservo.write(120); delay(500);
  TVCXservo.write(90); delay(500);
  TVCXservo.write(60); delay(500);
  TVCXservo.write(90); delay(500);
  delay(500);
  TVCYservo.attach(TVCYpin);
  TVCYservo.write(90); delay(500);
  TVCYservo.write(120); delay(500);
  TVCYservo.write(90); delay(500);
  TVCYservo.write(60); delay(500);
  TVCYservo.write(90); delay(500);
  delay(500);
  TVCTservo.attach(TVCTpin);
  TVCTservo.write(90); delay(500);
  TVCTservo.write(120); delay(500);
  TVCTservo.write(90); delay(500);
  TVCTservo.write(60); delay(500);
  TVCTservo.write(90); delay(500);

  
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

    // configure Arduino LED pin for output
    pinMode(LED_BLU1, OUTPUT);
    pinMode(LED_GRN1, OUTPUT);
    pinMode(LED_BLU2, OUTPUT);
    pinMode(LED_GRN2, OUTPUT);
    digitalWrite(LED_BLU1, LOW);
    digitalWrite(LED_GRN1, LOW);
    digitalWrite(LED_BLU2, LOW);
    digitalWrite(LED_GRN2, LOW);
    
    //Now the FLASH
    /*
  Serial.println("Looking for the SPI flash chip. Standby...");
  if (flash.begin(CHIPSIZE)){
    delay(500);
    Serial.println("Great, looks like there's one here!");
    Serial.println("Here's some info about it...");
    Serial.println();
    delay(1000);
    uint8_t b1, b2, b3;
    uint32_t JEDEC = flash.getJEDECID();
    b1 = (JEDEC >> 16);
    b2 = (JEDEC >> 8);
    b3 = (JEDEC >> 0);
    sprintf(printBuffer, "Manufacturer ID: %02xh\nMemory Type: %02xh\nCapacity: %02xh", b1, b2, b3);
    Serial.println(printBuffer);
    clearprintBuffer();
    sprintf(printBuffer, "JEDEC ID: %04lxh", JEDEC);
    Serial.println(printBuffer);
    Serial.println();
    Serial.println();
    delay(1000);
  }
  else{
    delay(500);
    Serial.println();
    Serial.println("Hmmm, looks like there's no flash chip here. Try checking the following:");
    Serial.println(" - Is the chip soldered on in the correct orientation?");
    Serial.println(" - Is the correct chip select pin defined in the SPIFlash constructor?");
    Serial.println(" - Is the correct CHIPSIZE defined?");
    delay(5000);
    Serial.println();
    Serial.println("Proceding with the rest of the startup process");
    delay(1000);
    Serial.println();
  }
  */
}

void loop() {
  tstep=(micros()-t);
  t=micros();
  //Serial.print(tstep); Serial.print("\t"); Serial.println(t);
  /*
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  v[0]=v[0]+(ax*g*aSensitivity/imuRange)*tstep/1000000;
  v[1]=v[1]+(ay*g*aSensitivity/imuRange)*tstep/1000000;
  v[2]=v[2]+(az*g*aSensitivity/imuRange-g)*tstep/1000000;
  for (int i=0; i<3; i++)
  {
    x[i]=x[i]+v[i]*tstep/1000000;
  }
  x[3]=x[3]+(gx*gSensitivity/imuRange)*tstep/1000000;
  x[4]=x[4]+(gy*gSensitivity/imuRange)*tstep/1000000;
  x[5]=x[5]+(gz*gSensitivity/imuRange)*tstep/1000000;
*/
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  v[0]=v[0]+(ax)*tstep;
  v[1]=v[1]+(ay)*tstep;
  v[2]=v[2]+(az-imuRange/aSensitivity)*tstep;
  for (int i=0; i<3; i++)
  {
    x[i]=x[i]+v[i]*tstep;
  }
  x[3]=x[3]+(gx)*tstep;
  x[4]=x[4]+(gy)*tstep;
  x[5]=x[5]+(gz)*tstep;

  for (int i=0; i<3; i++)
  {
    p[i]=x[i]*g*aSensitivity/imuRange/1000000000;
  }
  for (int i=3; i<6; i++)
  {
    p[i]=x[i]*gSensitivity/imuRange/1000;
  }
 
  //Serial.print("a/g:\t");
  for (int i=0; i<6; i++)
  {
    Serial.print(p[i]); Serial.print("\t");
  }
  Serial.println();
}
