#include "Wire.h"
#include <SPI.h>
#include <SPIFlash.h>
#include <Servo.h>

//This is for the SPI Flash chip
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

}
