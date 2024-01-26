//This code drives the LED Matricies in the head of R2D2.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"


//Create Matrix Objects

Adafruit_8x16minimatrix frontMatrix1 = Adafruit_8x16minimatrix();
//Adafruit_8x16minimatrix frontMatrix2 = Adafruit_8x16minimatrix();
//Adafruit_8x16minimatrix backMatrix1 = Adafruit_8x16minimatrix();
//Adafruit_8x16minimatrix backMatrix2 = Adafruit_8x16minimatrix();

void setup() {
  Serial.begin(9600);
  
  //Initialize Matricies
  
  frontMatrix1.begin(0x70);  // pass in the address
  //frontMatrix2.begin(0x71);
  //backMatrix1.begin(0x72);
  //backMatrix2.begin(0x73);
}

long randNumber;
bool isOn;

void loop() {

  frontMatrix1.clear();
  //frontMatrix2.clear();
  //backMatrix1.clear();
  //backMatrix2.clear();

  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 16; j++) {
      randNumber = esp_random();
      if (randNumber % 4 == 0) {
        isOn = true;
      }
      else {
        isOn = false;
      }
      frontMatrix1.drawPixel(i, j, isOn);
    }
  }
  frontMatrix1.writeDisplay();

  /*

  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 16; j++) {
      randNumber = esp_random();
      if (randNumber % 4 == 0) {
        isOn = true;
      }
      else {
        isOn = false;
      }
      frontMatrix2.drawPixel(i, j, isOn);
    }
  }
  frontMatrix2.writeDisplay();
  */

  // ....e.g.

  delay(230); // If we implement dynamic features to these led matricies, such as text scrolling, we will have to implement timers for each
  //matrix, rather than calling delay()
  
  
}
