#include <Servo.h>

int LED_BLU1=6;
int LED_GRN1=4;
int LED_BLU2=9;
int LED_GRN2=8;

int TVCXpin = 20;
int TVCYpin = 21;
Servo TVCXservo;
Servo TVCYservo;

void setup() {
  /**
  pinMode(LED_BLU1, OUTPUT);
  pinMode(LED_GRN1, OUTPUT);
  pinMode(LED_BLU2, OUTPUT);
  pinMode(LED_GRN2, OUTPUT);
  */
  TVCXservo.attach(TVCXpin);
  TVCXservo.write(90); delay(1000);
}

void loop() {
  TVCXservo.write(60); delay(1000);
  TVCXservo.write(120); delay(1000);
  /*
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BLU1, LOW);
  digitalWrite(LED_GRN1, HIGH);
  digitalWrite(LED_BLU2, HIGH);
  digitalWrite(LED_GRN2, LOW);
  delay(500);
  
  digitalWrite(LED_BLU1, HIGH);
  digitalWrite(LED_GRN1, LOW);
  digitalWrite(LED_BLU2, LOW);
  digitalWrite(LED_GRN2, HIGH);
  delay(500);
  */

  
}
