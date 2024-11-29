//    This code drives the LED Controller for R2D2.
//    This code controls the front and rear PSI light
//    The board recieves a digital signal (pin 7) from the audio controller board. This signal, which is 
//    high when R2 is 'talking,' causes the front PSI light to change colors rapidly, mirroring how it works in the movies.

#include <Adafruit_NeoPixel.h>

#define FRONT_PSI_PIN    6
#define BACK_PSI_PIN     8
#define LED_COUNT        12

Adafruit_NeoPixel front(LED_COUNT, FRONT_PSI_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel back(LED_COUNT, BACK_PSI_PIN, NEO_GRB + NEO_KHZ800);


void setup() {
  Serial.begin(57600);

  front.begin();  // INITIALIZE NeoPixel
  back.begin();

  front.show();   // Turn OFF all for boot
  back.show();

  front.setBrightness(50);
  back.setBrightness(50);

  pinMode(7, INPUT);

}

  long time = 1; 
  long frontDelayTime = 0;
  long backDelayTime = 0;
  int frontSelection;
  int backSelection;
  int frontLastColor;
  int backLastColor;
  bool talkFastFlag = false;

  int r, g, b;
  

void loop() {
  time = millis();
  frontSelection = random(10); //Randomly select color of light for front psi. 40% chance light will be red, 40% it will be blue, and 10% it will be white.
  backSelection = random(10); //Back psi. 40% red, 40% green, 10% yellow


  if ((time - frontDelayTime > 0) && (digitalRead(7) != HIGH)) {
    talkFastFlag = false;
    frontTalk(1800, 5000, front, frontDelayTime, frontLastColor);
    Serial.print("FST:  delaytime: ");
    Serial.print(frontDelayTime); Serial.print(" time: "); Serial.println(time);
  }
  
  if (digitalRead(7) == HIGH && !talkFastFlag) {
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

  if (frontSelection >= 0 && frontSelection <= 4 && lastColor != 0) { //Set LEDs to Red
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(255, 10, 10)); //Red
    }

  strip.show();
  lastColor = 0;
  } 
  else if (frontSelection > 4 && frontSelection < 9 && lastColor != 1) { //Set LEDs to Blue
    for (int j = 0; j < strip.numPixels(); j++) {
      strip.setPixelColor(j, strip.Color(0, 85, 255)); //Blue
    }
  strip.show();
  lastColor = 1;
  } 
  else if (frontSelection == 9 && lastColor != 2) { //Set LEDs to White
    for (int k = 0; k < strip.numPixels(); k++) {
      strip.setPixelColor(k, strip.Color(244, 244, 244)); //White  
    }
    strip.show();
    lastColor = 2;
  }
  else {  
    //If the same color is selected twice in a row, selection is updated and talk() is recursively called until a new color is selected.
    frontSelection = random(10);
    frontTalk(min, max, strip, delayTime, lastColor);
  }
}

  void backTalk(int min, int max, Adafruit_NeoPixel &strip, long &delayTime, int &lastColor) 
{
  time = millis();
  if (time - delayTime < max) {
    delayTime = random(min, max) + time; //Assign a random delay time between a given min and max to hold the current light color for.
  }

  if (backSelection >= 0 && backSelection <= 4 && lastColor != 0) { //Set LEDs to 
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
  else if (backSelection > 4 && backSelection < 9 && lastColor != 1) { //Set LEDs to Blue
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
  else if (backSelection == 9 && lastColor != 2) { //Set LEDs to White
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
    backSelection = random(10);
    backTalk(min, max, strip, delayTime, lastColor);
  }

}

