/*----------------------------------------------------------------------------

    HeadLEDController.ino

    DESCRIPTION:
      This code drives the LED Controller for R2D2. This code controls the
      front and rear PSI light. The board recieves a digital signal (pin 7)
      from the audio controller board. This signal, which is high when R2 is
      'talking,' causes the front PSI light to change colors rapidly,
      mirroring how it works in the movies.

-----------------------------------------------------------------------------*/

#include <Adafruit_NeoPixel.h>
#include <definitions.h>

#define FRONT_PSI_PIN    PIN_6
#define BACK_PSI_PIN     PIN_8
#define LED_COUNT        PIN_12

Adafruit_NeoPixel front(LED_COUNT, FRONT_PSI_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel back(LED_COUNT, BACK_PSI_PIN, NEO_GRB + NEO_KHZ800);

long time = 1; 
long frontDelayTime = 0;
long backDelayTime = 0;
int frontColorSelection;
int backColorSelection;
int frontLastColor;
int backLastColor;
bool talkFastFlag = false;

int r, g, b;

void setup() {
  Serial.begin(57600);

  front.begin();  // INITIALIZE NeoPixel
  back.begin();

  front.show();   // Turn OFF LEDs all for boot
  back.show();

  front.setBrightness(50);
  back.setBrightness(50);

  pinMode(PIN_7, INPUT);

}

//TODO: PROVIDE GOOD EXPLANATION OF HOW PIN 7 is set and how that effects speed of talk

void loop() {
  time = millis();
  frontColorSelection = ranom(10); //Randomly select color of light for front psi. 40% chance light will be red, 40% it will be blue, and 10% it will be white.
  backColorSelection = random(10); //Back psi. 40% red, 40% green, 10% yellow

  //If delayTime set in frontTalk has passed and PIN_7 is LOW:
  if ((time - frontDelayTime > 0) && (digitalRead(PIN_7) != HIGH)) {
    talkFastFlag = false;
    frontTalk(1800, 5000, front, frontDelayTime, frontLastColor);
    Serial.print("FST:  delaytime: ");
    Serial.print(frontDelayTime); Serial.print(" time: "); Serial.println(time);
  }
  //If PIN_7 is HIGH and R2 isn't talking fast.
  else if (digitalRead(PIN_7) == HIGH && !talkFastFlag) {
    talkFastFlag = true;
    frontDelayTime = millis() - 10;
    frontTalk(200, 450, front, frontDelayTime, frontLastColor);
  }
  else if ((time - frontDelayTime > 0) && digitalRead(7) == HIGH) { //TODO: Still switching very quickly *sometimes* when in fast talk mode
    frontTalk(200, 450, front, frontDelayTime, frontLastColor);
  }

  if (time - backDelayTime > 0) {
    
    backTalk(1800, 5000, back, backDelayTime, backLastColor);
  }


}

void frontTalk(int min, int max, Adafruit_NeoPixel &strip, long &delayTime, int &lastColor) 
{
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
    frontColorSelection = random(10);
    frontTalk(min, max, strip, delayTime, lastColor);
  }
}

  void backTalk(int min, int max, Adafruit_NeoPixel &strip, long &delayTime, int &lastColor) 
{
  time = millis();
  if (time - delayTime < max) {
    delayTime = random(min, max) + time; //Assign a random delay time between a given min and max to hold the current light color for.
  }

  if (backColorSelection >= 0 && backColorSelection <= 4 && lastColor != 0) { //Set LEDs to 
    while (r != 0 || g != 0 || b != 0) {
      if (r > 0) {
        r = r-5;
      }
      if (g > 0) {
        g = g-5;
      }
      if (b > 0) {
      b = b - 5;
      }
    for (int j = 0; j < strip.numPixels(); j++) {
      strip.setPixelColor(j, strip.Color(r, g, b)); //Red
    }
    strip.show();
    delay(7);
  }

  while (r != 255 || g != 10 || b != 10) {
      if (r < 255) {
        r = r+5;
      }
      if (g < 10) {
        g = g+5;
      }
      if (b < 10) {
      b = b + 5;
      }
    for (int j = 0; j < strip.numPixels(); j++) {
      strip.setPixelColor(j, strip.Color(r, g, b)); //Red
    }
    strip.show();
    delay(7);
  }
      lastColor = 0;
  } 
  else if (backColorSelection > 4 && backColorSelection < 9 && lastColor != 1) { //Set LEDs to Blue
    while (r != 0 || g != 0 || b != 0) {
      if (r > 0) {
        r = r-5;
      }
      if (g > 0) {
        g = g-5;
      }
      if (b > 0) {
      b = b - 5;
      }
    for (int j = 0; j < strip.numPixels(); j++) {
      strip.setPixelColor(j, strip.Color(r, g, b)); //Red
    }
    strip.show();
    delay(7);
  }

  while (r != 20 || g != 200 || b != 0) {
      if (r < 20) {
        r = r+5;
      }
      if (g < 200) {
        g = g+5;
      }
    for (int j = 0; j < strip.numPixels(); j++) {
      strip.setPixelColor(j, strip.Color(r, g, b)); //Red
    }
    strip.show();
    delay(7);
  }
    lastColor = 1;
  } 
  else if (backColorSelection == 9 && lastColor != 2) { //Set LEDs to White
        while (r != 0 || g != 0 || b != 0) {
      if (r > 0) {
        r = r-5;
      }
      if (g > 0) {
        g = g-5;
      }
      if (b > 0) {
      b = b - 5;
      }
    for (int j = 0; j < strip.numPixels(); j++) {
      strip.setPixelColor(j, strip.Color(r, g, b)); //Red
    }
    strip.show();
    delay(7);
  }

  while (r != 200 || g != 220 || b != 0) {
      if (r < 200) {
        r = r+5;
      }
      if (g < 220) {
        g = g+5;
      }
    for (int j = 0; j < strip.numPixels(); j++) {
      strip.setPixelColor(j, strip.Color(r, g, b)); //Red
    }
    strip.show();
    delay(7);
  }
    lastColor = 2;
  }
  else {  
    //If the same color is selected twice in a row, selection is updated and talk() is recursively called until a new color is selected.
    backColorSelection = random(10);
    backTalk(min, max, strip, delayTime, lastColor);
  }

}
