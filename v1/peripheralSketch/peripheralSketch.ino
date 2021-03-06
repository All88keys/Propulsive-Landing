//includes
#include <SPIFlash.h>
#include <Servo.h>

int t = 5;
int x = 20;
int y = 21;
Servo sx;
Servo sy;
Servo st;

int tvcXoffset=78;
int tvcYoffset=88;
float beta=.9094;
int gear=9;
float damper=1.5;
int tvcDelay=10;

#define CHIPSIZE MB64
SPIFlash flash(1);
uint8_t pageBuffer[256];
String serialCommand;
char printBuffer[128];
uint16_t page;
uint8_t offset, dataByte;
uint16_t dataInt;
String inputString, outputString;


void setup() {
  // put your setup code here, to run once:
  sx.attach(x);
  sy.attach(y);
  st.attach(t);

  //Now the FLASH
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
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  sx.write(60); delay(1000);
  sx.write(120); delay(1000);
  sx.write(60); delay(1000);
  sx.write(120); delay(1000);

  sy.write(60); delay(1000);
  sy.write(120); delay(1000);
  sy.write(60); delay(1000);
  sy.write(120); delay(1000);

  st.write(60); delay(1000);
  st.write(120); delay(1000);
  st.write(60); delay(1000);
  st.write(120); delay(1000);
  */

  tvc(0, 0, beta, tvcDelay*100);
    tvc(30, 0, beta, tvcDelay*100);
    tvc(-30, 0, beta, tvcDelay*100);
    tvc(0, 0, beta, tvcDelay*100);
    tvc(0, 30, beta, tvcDelay*100);
    tvc(0, -30, beta, tvcDelay*100);
    tvc(0, 0, beta, tvcDelay*100);
}

void clearprintBuffer()
{
  for (uint8_t i = 0; i < 128; i++) {
    printBuffer[i] = 0;
  }
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
  sy.write(v+tvcYoffset);sx.write(u+tvcXoffset); delay(d);
}
