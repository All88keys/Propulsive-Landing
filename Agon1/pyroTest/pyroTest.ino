#include <SPIFlash.h>
#include <SPI.h>
//#include <Wire.h>
#include <Servo.h>

int Pyro1 = 22;
int Pyro2 = 23;

//This is for the TVC servos
int tvcTopPin = 5;
Servo tvcTop;


void setup() {
tvcTop.attach(tvcTopPin);
  //delay(2000);
  tvcTop.write(10); delay(1000);
  tvcTop.write(150); delay(5000);
  tvcTop.write(10);
  

}

void loop() {
    // put your main code here, to run repeatedly:

  
  
  //tvcTop.write(-15); delay(1000);
  }
