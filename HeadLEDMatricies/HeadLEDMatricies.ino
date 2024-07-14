/*----------------------------------------------------------------------------

    HeadLEDMatricies.ino

    DESCRIPTION:
      This code drives the LED Matricies in the head of R2D2.

    MICROCONTROLLER:
      ESP-32

-----------------------------------------------------------------------------*/

/*-------------------------------------------------------------------
        INCLUDES
--------------------------------------------------------------------*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <R2D2_LIB.h>


/*-------------------------------------------------------------------
        GLOBAL VARIABLES
--------------------------------------------------------------------*/

//Create Matrix Objects
Adafruit_8x16minimatrix frontMatrix1 = Adafruit_8x16minimatrix();
Adafruit_8x16minimatrix frontMatrix2 = Adafruit_8x16minimatrix();
Adafruit_8x16minimatrix backMatrix1 = Adafruit_8x16minimatrix();
Adafruit_8x16minimatrix backMatrix2 = Adafruit_8x16minimatrix();

long randNumber;
bool isOn;        /* On/Off value for any given pixel */


/*----------------------------------------------------------------------

    Setup function

----------------------------------------------------------------------*/

void setup() {
  Serial.begin(9600);
  
  //Initialize Matricies
  
  frontMatrix1.begin(0x70);  //I2C Address
  frontMatrix2.begin(0x71);
  backMatrix1.begin(0x72);
  backMatrix2.begin(0x73);
}


/*----------------------------------------------------------------------

    Microcontroller Superloop

----------------------------------------------------------------------*/

void loop() {

  frontMatrix1.clear();
  frontMatrix2.clear();
  backMatrix1.clear();
  backMatrix2.clear();

  // For each pixel
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
  // Draw the entire display
  frontMatrix1.writeDisplay();

  // For each pixel
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
  // Draw the entire display
  frontMatrix2.writeDisplay();

  // For each pixel
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
  // Draw the entire display
  backMatrix1.writeDisplay();

  // For each pixel
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
  // Draw the entire display
  backMatrix2.writeDisplay();

  // There is a 1 in 120 chance that the text "JMP" will scroll across the front matricies
  if (esp_random() % 120 == 0) {
    frontMatrix1.setTextSize(1);
    frontMatrix1.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
    frontMatrix1.setTextColor(LED_ON);
    frontMatrix1.setRotation(3);

    frontMatrix2.setTextSize(1);
    frontMatrix2.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
    frontMatrix2.setTextColor(LED_ON);
    frontMatrix2.setRotation(3);

    // X values to scroll across
    for (int8_t x=19; x>=-20; x--) {
      frontMatrix1.clear();
      frontMatrix1.setCursor(x,0);
      frontMatrix1.print("JMP");
      frontMatrix1.writeDisplay();

      frontMatrix2.clear();
      frontMatrix2.setCursor(x,0);
      frontMatrix2.print("JMP");
      frontMatrix2.writeDisplay();

      // Keep updating the back matricies with random pixels
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
      // Draw the entire display
      backMatrix1.writeDisplay();

      // Keep updating the back matricies with random pixels
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
      // Draw the entire display
      backMatrix2.writeDisplay();

      delay(230);
    }
    frontMatrix1.setRotation(0);
    frontMatrix2.setRotation(0);
  }

  delay(230); // If we implement dynamic features to these led matricies, such as text scrolling, we will have to implement timers for each
  //matrix, rather than calling delay()
  
}
