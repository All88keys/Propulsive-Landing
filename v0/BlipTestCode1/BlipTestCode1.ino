
//Thanks for downloading the Blip test code, and being a patron of BPS.space!
//
//You can run this code on the regular Blip R1 computer to see if everything
//is hooked up correctly. You're welcome to modify or change this for your own
//non-commercial designs, but please do not distribute or publish it. - Joe Barnard


//#include "Wire.h"



//This is for the pyro channels
int Pyro1 = 20;
int Pyro2 = 21;
int Pyro3 = 22;
int Pyro4 = 23;

void setup(){

  //Test pyro channels
  Serial.println();
  Serial.println("To finish up here, let's test all the pyro channels");
  Serial.println();
  pinMode(Pyro1, OUTPUT);
  pinMode(Pyro2, OUTPUT);
  pinMode(Pyro3, OUTPUT);
  pinMode(Pyro4, OUTPUT);
  digitalWrite(Pyro1, LOW);
  digitalWrite(Pyro2, LOW);
  digitalWrite(Pyro3, LOW);
  digitalWrite(Pyro4, LOW);
  delay(1000);

          //The following code MUST BE REMOVED before you connect anything to the pyro channels
          Serial.println("We'll now cycle through each channel, turning each one on for 2 seconds");
          delay(1000);
          digitalWrite(Pyro1, HIGH);
          Serial.println("Pyro 1 is on!");
          delay(2000);
          digitalWrite(Pyro1, LOW);
          Serial.println("Pyro 1 is off");
          delay(1000);
          digitalWrite(Pyro2, HIGH);
          Serial.println("Pyro 2 is on!");
          delay(2000);
          digitalWrite(Pyro2, LOW);
          Serial.println("Pyro 2 is off");
          delay(1000);
          digitalWrite(Pyro3, HIGH);
          Serial.println("Pyro 3 is on!");
          delay(2000);
          digitalWrite(Pyro3, LOW);
          Serial.println("Pyro 3 is off");
          delay(1000);
          digitalWrite(Pyro4, HIGH);
          Serial.println("Pyro 4 is on!");
          delay(2000);
          digitalWrite(Pyro4, LOW);
          Serial.println("Pyro 4 is off");
          delay(1000);
  
  Serial.println();
  Serial.println("Done with the pyro channel testing");
  Serial.println();
  Serial.println();
  delay(500);
}

void loop() {}
  
 
