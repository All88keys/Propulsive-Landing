#include <Wire.h>
#include <SD.h>
#include <SPI.h>

Sd2Card card;
SdVolume volume;
SdFile root;
const int SDchipSelect = 0;

void setup() {
  delay(2000);

  //Now the SD card
  Serial.print("Looking for an SD card. Standby...");
  if (!card.init(SPI_HALF_SPEED, SDchipSelect)) {
    Serial.println("Looks like there's no card here! Trying checking the following:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your soldering correct?");
    Serial.println("* is the chipselect pin correct?");
    Serial.println();
    delay(4000);
    Serial.println("We'll continue with the startup without the SD card now :(");
  } else {
    Serial.println("Found an SD card! Here's some information about it.");
    delay(1000);
    Serial.println();
    Serial.print("Card type:         ");
    switch (card.type()) {
      case SD_CARD_TYPE_SD1:
        Serial.println("SD1");
        break;
      case SD_CARD_TYPE_SD2:
        Serial.println("SD2");
        break;
      case SD_CARD_TYPE_SDHC:
        Serial.println("SDHC");
        break;
      default:
        Serial.println("Unknown");
    }
    if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    while (1);
  }

}
}

void loop() {
  // put your main code here, to run repeatedly:

}
