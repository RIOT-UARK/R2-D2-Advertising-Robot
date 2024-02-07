//This code drives the LED Matricies in the head of R2D2.

//TODO: update into functions and clean up this code

#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"


//Create Matrix Objects

Adafruit_8x16minimatrix frontMatrix1 = Adafruit_8x16minimatrix();
Adafruit_8x16minimatrix frontMatrix2 = Adafruit_8x16minimatrix();
Adafruit_8x16minimatrix backMatrix1 = Adafruit_8x16minimatrix();
Adafruit_8x16minimatrix backMatrix2 = Adafruit_8x16minimatrix();

void setup() {
  Serial.begin(9600);
  
  //Initialize Matricies
  
  frontMatrix1.begin(0x70);  
  frontMatrix2.begin(0x71);
  backMatrix1.begin(0x72);
  backMatrix2.begin(0x73);
}

long randNumber;
bool isOn;

void loop() {

  frontMatrix1.clear();
  frontMatrix2.clear();
  backMatrix1.clear();
  backMatrix2.clear();

  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 16; j++) {
      randNumber = esp_random();
      if (randNumber % 4 == 0) {  //There is a 1 in 4 chance that any LED in the matrix will be lit for any 'frame'
        isOn = true;
      }
      else {
        isOn = false;
      }
      frontMatrix1.drawPixel(i, j, isOn);
    }
  }
  frontMatrix1.writeDisplay();


  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 16; j++) {
      randNumber = esp_random();
      if (randNumber % 4 == 0) { //There is a 1 in 4 chance that any LED in the matrix will be lit for any 'frame'
        isOn = true;
      }
      else {
        isOn = false;
      }
      frontMatrix2.drawPixel(i, j, isOn);
    }
  }
  frontMatrix2.writeDisplay();

    for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 16; j++) {
      randNumber = esp_random();  //There is a 1 in 4 chance that any LED in the matrix will be lit for any 'frame'
      if (randNumber % 4 == 0) {
        isOn = true;
      }
      else {
        isOn = false;
      }
      backMatrix1.drawPixel(i, j, isOn);
    }
  }
  backMatrix1.writeDisplay();

      for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 16; j++) {
      randNumber = esp_random();  //There is a 1 in 4 chance that any LED in the matrix will be lit for any 'frame'
      if (randNumber % 4 == 0) {
        isOn = true;
      }
      else {
        isOn = false;
      }
      backMatrix2.drawPixel(i, j, isOn);
    }
  }
  backMatrix2.writeDisplay();

  if (esp_random() % 80 == 0) {
  frontMatrix1.setTextSize(1);
  frontMatrix1.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
  frontMatrix1.setTextColor(LED_ON);
  frontMatrix1.setRotation(3);

  frontMatrix2.setTextSize(1);
  frontMatrix2.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
  frontMatrix2.setTextColor(LED_ON);
  frontMatrix2.setRotation(3);

  for (int8_t x=19; x>=-20; x--) {
    frontMatrix1.clear();
    frontMatrix1.setCursor(x,0);
    frontMatrix1.print("JMP");
    frontMatrix1.writeDisplay();

    frontMatrix2.clear();
    frontMatrix2.setCursor(x,0);
    frontMatrix2.print("JMP");
    frontMatrix2.writeDisplay();

    for (int i = 0; i < 8; i++) {
      for (int j = 0; j < 16; j++) {
        randNumber = esp_random();  //There is a 1 in 4 chance that any LED in the matrix will be lit for any 'frame'
        if (randNumber % 4 == 0) {
          isOn = true;
        }
        else {
          isOn = false;
        }
        backMatrix1.drawPixel(i, j, isOn);
      }
    }
  backMatrix1.writeDisplay();

    for (int i = 0; i < 8; i++) {
      for (int j = 0; j < 16; j++) {
        randNumber = esp_random();  //There is a 1 in 4 chance that any LED in the matrix will be lit for any 'frame'
        if (randNumber % 4 == 0) {
          isOn = true;
        }
        else {
          isOn = false;
        }
      backMatrix2.drawPixel(i, j, isOn);
      }
    }
  backMatrix2.writeDisplay();
  

    delay(230);
  }
  frontMatrix1.setRotation(0);
  frontMatrix2.setRotation(0);
  }


  delay(230); // If we implement dynamic features to these led matricies, such as text scrolling, we will have to implement timers for each
  //matrix, rather than calling delay()
  
  
}
