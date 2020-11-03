#include <SPIFlash.h>
#include <SPI.h>
//#include <Wire.h>

#define CHIPSIZE MB64;
uint32_t floatAddr1[2];
float testFloat1[] = {
  3.1415, 6.283
};
uint32_t floatAddr2[5];
float testFloat2[] = {
  3.1415, 444, -5.5, 6, 22
};
float readDat;

SPIFlash flash(1);

void getAddresses();
void writeDat();
void readDatf();

void setup(){
  Serial.println("Init Flash");
  flash.begin(9600);
  flash.eraseChip();
  //seq();
  getAddresses();
  writeDat();
  readDatf();
  
}

void loop(){
  
}

void getAddresses(){
  for (uint8_t i = 0; i < arrayLen(floatAddr1); i++) {
    floatAddr1[i] = flash.getAddress(sizeof(float));
    Serial.print("Float Address ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(floatAddr1[i]);
  }
  for (uint8_t i = 0; i < arrayLen(floatAddr2); i++) {
    floatAddr2[i] = flash.getAddress(sizeof(float));
    Serial.print("Float Address ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(floatAddr2[i]);
  }
}

void writeDat(){
  for (uint8_t i = 0; i < arrayLen(floatAddr1); i++) {
    if (flash.writeFloat(floatAddr1[i], testFloat1[i])) {
      Serial.print(testFloat1[i]);
      Serial.print(" written to ");
      Serial.println(floatAddr1[i]);
    }
  }
  for (uint8_t i = 0; i < arrayLen(floatAddr2); i++) {
    if (flash.writeFloat(floatAddr2[i], testFloat2[i])) {
      Serial.print(testFloat2[i]);
      Serial.print(" written to ");
      Serial.println(floatAddr2[i]);
    }
  }
}

void readDatf(){
  for (uint8_t i = 0; i < 2; i++) {
    readDat=flash.readFloat(floatAddr1[i]);
    Serial.println(readDat);
    }
  }

void seq(){
  for (uint8_t i = 0; i < arrayLen(floatAddr1); i++) {
    floatAddr1[i] = flash.getAddress(sizeof(float));
    Serial.print("Float Address ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(floatAddr1[i]);
  }
  for (uint8_t i = 0; i < arrayLen(floatAddr1); i++) {
    if (flash.writeFloat(floatAddr1[i], testFloat1[i])) {
      Serial.print(testFloat1[i]);
      Serial.print(" written to ");
      Serial.println(floatAddr1[i]);
    }
  }
  for (uint8_t i = 0; i < arrayLen(floatAddr2); i++) {
    floatAddr2[i] = flash.getAddress(sizeof(float));
    Serial.print("Float Address ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(floatAddr2[i]);
  }
  for (uint8_t i = 0; i < arrayLen(floatAddr2); i++) {
    if (flash.writeFloat(floatAddr2[i], testFloat2[i])) {
      Serial.print(testFloat2[i]);
      Serial.print(" written to ");
      Serial.println(floatAddr2[i]);
    }
  }
}
