/*
  ARDUINO UNO FOR CONTROLLING THE POLOLU DOME MOTOR
  This board receives an integer over serial (may change this to a faster comm method later) from the Main ESP32 Receiver Board,
  then uses that to control the speed and direction of the Dome Motor.

  PINOUT
Arduino Uno -> PROK Motor Controller
  
  5V -> 5V
  5V -> 5V
  GND -> GND
  GND -> GND
  Pin 2 -> IN1
  Pin 3 -> ENA1
  Pin 4 -> IN2
  Pin 6 -> ENA2
  Pin 7 -> IN3
  Pin 8 -> IN4
  
Arduino Uno -> ESP32 Receiver Board
  RX(PIN 0) -> Pin TX 
  Pin 5 -> Pin 33

  Using an Arduino Uno was necessary because the PROK Motor controller would only work with the Uno's 5V logic,
  and not an ESP32's 3V logic.
*/

// TODO: Implement Encoder Readings

unsigned long timeOfLastRec;
int driveValue;

unsigned long timeOfLastPam = 0; //time of last pamphlet
bool forwardReverse = true; //true = pamphlet dispenser forward, false = backwards
bool pamReady = true;

void setup() {

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, INPUT);
  pinMode(6, OUTPUT); //ENA
  pinMode(7, OUTPUT); //IN3
  pinMode(8, OUTPUT); //IN4

  Serial.begin(115200);


 delay(3000); //Delay because the ESP32 will spew garbage over serial upon startup.
}

void loop() {
 
 

  if (Serial.available() > 0) {
      driveValue = Serial.read();  
      timeOfLastRec = millis();
      
  Serial.println(driveValue);
  
    if (driveValue > 14 && driveValue < 100) {
      digitalWrite(2, LOW);
      digitalWrite(4, HIGH);
      
      driveValue = int(((float(driveValue) - 14) * 255.0) / 85.0);
      
      }
    else if (driveValue < 241 && driveValue > 155) {
      digitalWrite(2, HIGH);
      digitalWrite(4, LOW);
      
      driveValue = int(((240 - float(driveValue)) * 255.0) / 85.0);
      
      }
    else {
      driveValue = 0;
    }
  
  analogWrite(3, driveValue);

  }

  if ((millis() - timeOfLastRec) > 200) {
  analogWrite(3, 0);
  }

  if (pamReady && digitalRead(5) == HIGH && (millis() - timeOfLastPam > 5000)) {
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
    analogWrite(6, 60);
    timeOfLastPam = millis();
    pamReady = false;
  }
  else if (millis() - timeOfLastPam > 1500 && millis() - timeOfLastPam < 3300) {
    digitalWrite (7, HIGH);
    digitalWrite (8, LOW);
    analogWrite(6, 60);
  }
  else if (millis() - timeOfLastPam > 3300) {
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    analogWrite(6, 0);
    pamReady = true;
  }

}
