#include <Arduino.h>
#include "BQ79606M.h"


BQ79606M bms(11, 200000, 4, 5, 7, 18, 19); // Set pins for BQ79606



void setup() {
  Serial.begin(115200);
  bms.initializeBQs(); // Initialize BQ79606
  //bms.setBalance(0, 0, false, 0x00); // Set balance time and type
  //bms.setThresh(3.5);
  //bms.stopBalancing();

 // setPins(1,200000,4,5,7,8,9); // Set pins for BQ79606
}

void loop() {
  bms.getMeansure(); //Lee valores de temeraturas en los GPIO
  bms.printData();
  bms.stopBalancing();
  delay(5000);
}
