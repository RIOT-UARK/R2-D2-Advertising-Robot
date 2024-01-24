//    This code is an unfinished version of the LED Controller for R2D2.
//    In its current state, this code controls only the front PSI light: the most complicated light on R2's dome.
//    The board recieves a digital signal (pin 7) from the audio controller board. This signal, which is 
//    high when R2 is 'talking,' causes the front PSI light to change colors rapidly, mirroring how it works in the movies.

#include <Adafruit_NeoPixel.h>

#define LED_PIN    6
#define LED_COUNT 12

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(57600);

  strip.begin();           // INITIALIZE NeoPixel
  strip.show();            // Turn OFF all for boot
  strip.setBrightness(50); 
  pinMode(7, INPUT);       // 'Talk' signal pin from Audio Board.
}

  long atime = 1;
  long delayTime = 0;
  int selection;
  int lastColor;

void loop() {
  selection = random(10); //Randomly select color of light. 40% chance light will be red, 40% it will be blue, and 10% it will be white.
  Serial.println((atime - delayTime));
  //Loop to hold code before changing the led color again. Can be interrupted by 'talk' signal
  while (((atime - delayTime) < 0) && (digitalRead(7) != HIGH)) 
  { 
    
    delay(20);
    atime = millis();
    //Serial.print(digitalRead(7));
    //Serial.print(".");
    if ((atime - delayTime > 0) && (digitalRead(7) == LOW)) {
      break;
    }

  }

  while (digitalRead(7) == HIGH) 
  {
    talk(200, 450);                     //Quickly change colors 'talk' mode
    while ((atime - delayTime) < 0) {
      atime = millis();
      delay(20);
      Serial.print(digitalRead(7));
    }
    
  }

    talk(1800, 5000);  //Slowly change colors

 

/*
  Serial.print(digitalRead(7));
  Serial.print(" time = ");
  Serial.print(time);
  Serial.print("  DelayTime = ");
  Serial.print(delayTime);
  Serial.print("  time - delayTime: ");
  Serial.print(time - delayTime);
  Serial.print("   ");
  Serial.println(selection); 
  */
}

void talk(int min, int max) 
{
  atime = millis();
  if (atime - delayTime < max) {
    delayTime = random(min, max) + atime; //Assign a random delay time between a given min and max to hold the current light color for.
  }

  if (selection >= 0 && selection <= 4 && lastColor != 0) { //Set LEDs to Red
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(255, 10, 10)); //Red
    }
  strip.show();
  lastColor = 0;
  } 
  else if (selection > 4 && selection < 9 && lastColor != 1) { //Set LEDs to Blue
    for (int j = 0; j < strip.numPixels(); j++) {
      strip.setPixelColor(j, strip.Color(0, 85, 255)); //Blue
    }
  strip.show();
  lastColor = 1;
  } 
  else if (selection == 9 && lastColor != 2) { //Set LEDs to White
    for (int k = 0; k < strip.numPixels(); k++) {
      strip.setPixelColor(k, strip.Color(244, 244, 244)); //White  
    }
    strip.show();
    lastColor = 2;
  }
  else {  
    //If the same color is selected twice in a row, selection is updated and talk() is recursively called until a new color is selected.
    selection = random(10);
    talk(min, max);
  }

}
