/*----------------------------------------------------------------------------

    HeadLEDController.ino

    DESCRIPTION:
      This code drives the LED Controller for R2D2. This code controls the
      front and rear PSI light. The board recieves a digital signal (pin 7)
      from the audio controller board. This signal, which is high when R2 is
      'talking,' causes the front PSI light to change colors rapidly,
      mirroring how it works in the movies.

    MICROCONTROLLER:
      Arduino Nano

-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
HeadLEDController Pinout

  PIN 2  --> SIGNAL INPUT FOR PERISCOPE ACTION SIGNAL FROM HeadAudioBoard Pin XXXXXX
  PIN 3  --> PWM SIGNAL OUTPUT FOR PERISCOPE SERVO 1
  PIN 5  --> PWM SIGNAL OUTPUT FOR PERISCOPE SERVO 2
  PIN 6  --> SIGNAL OUTPUT FOR FRONT PSI LIGHT NEOPIXEL
  PIN 7  --> SIGNAL INPUT FOR 'TALK FAST' flag from HeadAudioBoard Pin 23 
  PIN 8  --> SIGNAL OUTPUT FOR BACK PSI LIGHT NEOPIXEL

-----------------------------------------------------------------------------*/



/*-------------------------------------------------------------------
        INCLUDES
--------------------------------------------------------------------*/

#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <R2D2_LIB.h>

/*-------------------------------------------------------------------
        DEFINES AND CONSTANTS
--------------------------------------------------------------------*/

#define FRONT_PSI_PIN    PIN_6
#define BACK_PSI_PIN     PIN_8
#define LED_COUNT        12

Adafruit_NeoPixel front(LED_COUNT, FRONT_PSI_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel back(LED_COUNT, BACK_PSI_PIN, NEO_GRB + NEO_KHZ800);

/*-------------------------------------------------------------------
        GLOBAL VARIABLES
--------------------------------------------------------------------*/

Servo servo1;               /* servo that controls TYPE ME                   */
Servo servo2;               /* servo that controls TYPE ME                   */

long time = 1;              /* current time                                  */
long periscopeTimer = 0;    /* time when periscope movement will finish      */
long frontDelayTime = 0;    /* time until front psi light next color change  */
long backDelayTime = 0;     /* time until back psi light next color change   */
int frontColorSelection;    /* selected color for front psi light            */
int backColorSelection;     /* selected color for rear psi light             */
int frontLastColor;         /* previously selected front psi color           */
int backLastColor;          /* previously selected rear psi color            */
int periscopeStage = 0;     /* motion stage of periscope                     */
bool talkFastFlag = false;  /* flag when fron psi should rapidly change      */
bool pDoOnce = true;        /* flag for resetting periscope movement         */

int r, g, b;
int fadeR, fadeG, fadeB;    /* Brightness values for r,g,b to be iterated    */
bool isFading = false;
bool fadeDown = true;

/*----------------------------------------------------------------------

    Setup function

----------------------------------------------------------------------*/

void setup() {
  Serial.begin(57600);

  front.begin();  // INITIALIZE NeoPixel
  back.begin();

  front.show();   // Turn OFF all for boot
  back.show();

  front.setBrightness(50);  /* uint8 brightness value (0-255)       */
  back.setBrightness(50);

  pinMode(PIN_7, INPUT);  /* Receive talkFast flag from audioBoard  */
  pinMode(PIN_2, INPUT);  /* Recieve Periscope flag from audioBoard */
  servo1.attach(PIN_3);   /* Periscope Servo 1                      */
  servo2.attach(PIN_5);   /* Periscope Servo 2                      */


  servo1.write(90);
  servo2.write(0);

  delay(2500);            /* wait 2.5 sec                           */

}


/*----------------------------------------------------------------------

    Microcontroller Superloop

      A signal of HIGH will be received on PIN_7 if R2D2 is supposed to
      be 'talking'. Receiving this signal will cause R2's front psi 
      light to rapidly change colors, as it does in the movies

----------------------------------------------------------------------*/

void loop() {
  time = millis();

  // If delayTime set in frontTalk has passed and PIN_7 is LOW:
  if ((time - frontDelayTime > 0) && (digitalRead(PIN_7) != HIGH)) {
    talkFastFlag = false;
    frontTalk(1800, 5000, front, frontDelayTime, frontLastColor);
  }
  // If PIN_7 is HIGH and R2 isn't talking fast.
  else if (digitalRead(PIN_7) == HIGH && !talkFastFlag) {
    talkFastFlag = true;
    frontDelayTime = time - 10;
    frontTalk(200, 450, front, frontDelayTime, frontLastColor);
  }
  // Keep talking fast if PIN_7 is still HIGH.
  else if ((time - frontDelayTime > 0) && digitalRead(PIN_7) == HIGH) { //TODO: Still switching very quickly *sometimes* when in fast talk mode
    frontTalk(200, 450, front, frontDelayTime, frontLastColor);
  }

  // Change back psi light randomly every 1.8 to 5 seconds
  if (time - backDelayTime > 0) {
    isFading = true;
    fadeDown = true;
    backTalk(1800, 5000, back, backDelayTime, backLastColor);
  }

  /*------------------------------------------------------------------------
     If a signal of HIGH is received on PIN_2, go through an 8 second long
     action of raising the periscope, looking from size to side, then bring
     the periscope back down
  ------------------------------------------------------------------------*/

  if (periscopeStage == 0 && digitalRead(PIN_2) == HIGH && time > periscopeTimer ) {
    periscopeTimer = time + 8000;
    periscopeStage = 1;
    servo1.write(90);
    servo2.write(150);
    pDoOnce = true;
  }
  else if (periscopeStage == 1) {
    if (periscopeTimer - time > 5000 && periscopeTimer - time <= 6500) {
      servo1.write(150);
    }
    else if (periscopeTimer - time > 3000 && periscopeTimer - time <= 5000) {
     servo1.write(30);
    }
    else if (periscopeTimer - time > 1500 && periscopeTimer - time <= 3000) {
      servo1.write(90);
    }
    else if (periscopeTimer - time > 500 && periscopeTimer - time <= 1500) {
      servo1.write(90);
      servo2.write(0);
      periscopeStage = 0;
    }
  }
  else {
    servo1.write(90);
    servo2.write(0);
  }
  
  if (periscopeStage == 0 && pDoOnce) {
    servo1.write(90);
    servo2.write(0);
    pDoOnce = false;
  }

}

/*---------------------------------------------------------------

  frontTalk

    Change front psi light color. Randomly changes between red,
    blue and white. When R2D2 is 'talking', these lights will
    change much more rapidly, as they do in the movies.

----------------------------------------------------------------*/

void frontTalk(int min, int max, Adafruit_NeoPixel &strip, long &delayTime, int &lastColor) 
{
  frontColorSelection = random(10); //Randomly select color of light for front psi. 40% chance light will be red, 40% it will be blue, and 10% it will be white.
  time = millis();
  if (time - delayTime > 0) {
    delayTime = random(min, max) + time; //Assign a random delay time between a given min and max to hold the current light color for.
  }

  if (frontColorSelection >= 0 && frontColorSelection <= 4 && lastColor != 0) { //Set LEDs to Red
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(255, 10, 10)); //Red
    }

  strip.show();
  lastColor = 0;
  } 
  else if (frontColorSelection > 4 && frontColorSelection < 9 && lastColor != 1) { //Set LEDs to Blue
    for (int j = 0; j < strip.numPixels(); j++) {
      strip.setPixelColor(j, strip.Color(0, 85, 255)); //Blue
    }
  strip.show();
  lastColor = 1;
  } 
  else if (frontColorSelection == 9 && lastColor != 2) { //Set LEDs to White
    for (int k = 0; k < strip.numPixels(); k++) {
      strip.setPixelColor(k, strip.Color(244, 244, 244)); //White  
    }
    strip.show();
    lastColor = 2;
  }
  else {  
    //If the same color is selected twice in a row, selection is updated and talk() is recursively called until a new color is selected.
    frontTalk(min, max, strip, delayTime, lastColor);
  }
}


  /*---------------------------------------------------------------

    backTalk
  
      Randomly change back psi light. Changes between red, green, 
      and yellow.  

  ---------------------------------------------------------------*/

  void backTalk(int min, int max, Adafruit_NeoPixel &strip, long &delayTime, int &lastColor) 
{
  backColorSelection = random(10); //Back psi. 50% red, 40% green, 10% yellow
  time = millis();
  if (time - delayTime < max) {
    delayTime = random(min, max) + time; //Assign a random delay time between a given min and max to hold the current light color for.
  }

  if (backColorSelection >= 0 && backColorSelection <= 4 && lastColor != 0) { //Set LEDs to Red
      fadeR = r;
      fadeG = g;
      fadeB = b;
      r = 254; g = 10; b = 10;
      lastColor = 0;
  } 
  else if (backColorSelection > 4 && backColorSelection < 9 && lastColor != 1) { //Set LEDs to Green
      fadeR = r;
      fadeG = g;
      fadeB = b;
      r = 20; g = 200; b = 0;
      lastColor = 1;
  } 
  else if (backColorSelection == 9 && lastColor != 2) { //Set LEDs to Yellow
      fadeR = r;
      fadeG = g;
      fadeB = b;
      r = 200; g = 220; b = 0;
      lastColor = 2;
  }
  else {  
    //If the same color is selected twice in a row, selection is updated and talk() is recursively called until a new color is selected.
    backColorSelection = random(10);
    backTalk(min, max, strip, delayTime, lastColor);
  }

  for (int i = 0; i < back.numPixels(); i++) {
          back.setPixelColor(i, back.Color(r, g, b));
        }
        back.show();
}


  /*---------------------------------------------------------------

    fade
  
      Gives back psi light ability to fade in and out when
      changing colors
      CURRENTLY NOT USED. SHALL REIMPLEMENT IF HAVE TIME 

  ---------------------------------------------------------------*/

  void fade() {
    if (fadeR != r || fadeG != g || fadeB != b) {
      if (isFading && fadeDown) {
        if (fadeR > 0) {
          fadeR -= 2;
        }
        if (fadeG > 0) {
          fadeG -= 2;
        }
        if (fadeB > 0) {
          fadeB -= 2;
        }
        if (fadeR == 0 && fadeG == 0 && fadeB == 0) {
          fadeDown = false;
        }
        //TODO: TEST ADDED CODE: NOW UPDATING EVERY 15ms instead of every time
        if (time % 15 == 0) { //Only update pixels every 15ms
          for (int i = 0; i < back.numPixels(); i++) {
            back.setPixelColor(i, back.Color(fadeR, fadeG, fadeB));
          }
          back.show();
        }
               
      }
      else if (isFading && !fadeDown) {
        if (fadeR < r) {
          fadeR += 2;
        }
        if (fadeG < g) {
          fadeG += 2;
        }
        if (fadeB < b) {
          fadeB += 2;
        }

        if (fadeR == r && fadeG == g && fadeB == b) {
          isFading = false;
        }

        //TODO: TEST ADDED CODE: NOW UPDATING EVERY 15ms instead of every time
        if (time % 15 == 0) { //Only update pixels every 15ms
          for (int i = 0; i < back.numPixels(); i++) {
            back.setPixelColor(i, back.Color(fadeR, fadeG, fadeB));
          }
          back.show();
        }
      }
    }
  }