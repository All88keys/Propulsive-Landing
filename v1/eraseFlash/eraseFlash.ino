//includes
#include "Wire.h"
#include<SPIFlash.h>
#include <Servo.h>

int LED=6;


#define CHIPSIZE MB64
SPIFlash flash(1);


void setup() {

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    flash.begin(9600);
    flash.eraseChip();
    digitalWrite(LED, LOW);
}

void loop() {}
