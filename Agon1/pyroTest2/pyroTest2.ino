#include "Wire.h"
int Pyro4 = 22;
int LED=6;

void setup() {
  // put your setup code here, to run once:
  pinMode(Pyro4, OUTPUT);
  digitalWrite(Pyro4, LOW);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  
  delay(20000);

  digitalWrite(LED, LOW);
  digitalWrite(Pyro4, HIGH);
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(Pyro4, LOW);
  delay(5000);
  digitalWrite(Pyro4, HIGH);
  delay(2000);
  
}
