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

Arduino Uno -> Pamphlet Dispenser Door Servo Motor
  Pin 9 -> Orange Wire

  Using an Arduino Uno was necessary because the PROK Motor controller would only work with the Uno's 5V logic,
  and not an ESP32's 3V logic.
*/

// TODO: Implement Encoder Readings

#include <Servo.h>

unsigned long timeOfLastRec;
int driveValue;

unsigned long timeOfLastPam = 0; //time of last pamphlet
//bool forwardReverse = true; //true = pamphlet dispenser forward, false = backwards
//bool pamReady = true;
int stage = 0;

Servo servo;

void setup() {

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, INPUT);
  pinMode(6, OUTPUT); //ENA
  pinMode(7, OUTPUT); //IN3
  pinMode(8, OUTPUT); //IN4
  //pinMode(9, OUTPUT); //Servo Motor

  Serial.begin(115200);

  servo.attach(9);
  servo.write(0);


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

  if ((millis() - timeOfLastRec) > 200) { //If not recieving signal from controller, stop the motor.
  digitalWrite(2, LOW);
  digitalWrite(4, LOW);
  analogWrite(3, 0);
  }

  /*
  if (pamReady && digitalRead(5) == HIGH && (millis() - timeOfLastPam > 1000)) { //When dispense signal first received
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
    servo.write(90);
    analogWrite(6, 60);
    timeOfLastPam = millis();
    pamReady = false;
  }
  else if (millis() - timeOfLastPam > 1500 && millis() - timeOfLastPam < 3300) { //Reverse Motors to dispense pamphlet
    digitalWrite (7, HIGH);
    digitalWrite (8, LOW);
    servo.write(90);
    analogWrite(6, 60);
  }
  else if (millis() - timeOfLastPam > 3300 && millis() - timeOfLastPam < 5000) { //Wait until at least 5 seconds have passed
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    analogWrite(6, 0);
  }
  else if (!pamReady && digitalRead(5) == HIGH && (millis() - timeOfLastPam > 5000)) {
    servo.write(0);
    timeOfLastPam = millis();
    pamReady = true;
  }
  */
//--------------------------------------------------------------------------------------
//PAMPHLET DISPENSER + DOOR SERVO MOTOR CONTROL
//--------------------------------------------------------------------------------------

  if (stage == 0) {                                                     //Door is closed and Pamphlet Dispenser is ready to receive signal from controller
    if (digitalRead(5) == HIGH && (millis() - timeOfLastPam > 1000)) {
      stage++;
      timeOfLastPam = millis();
    }
  }
  if (stage == 1) {                                                     //Signal to print has been received. Load a pamphlet in paper feeder (move motor in one direction) and open the door
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
    servo.write(90);
    analogWrite(6, 80);
    if (millis() - timeOfLastPam > 1500) {
      stage++;
    }
  }
  else if (stage == 2) {                                                //Feed the paper out of the pamphlet dispenser.
    digitalWrite (7, HIGH);
    digitalWrite (8, LOW);
    servo.write(90);
    analogWrite(6, 80);
    if (millis() - timeOfLastPam > 3500) {
      stage++;
    }
  }
  else if (stage == 3) {                                                //Stop the paper feeder motor when the paper is just about out of the dispenser.
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    analogWrite(6, 0);
    if (millis() - timeOfLastPam > 5000) {
      stage++;
    }
  }
  else if (stage == 4) {                                                //Wait for an additional button press from the controller to signal to close the door
    if (digitalRead(5) == HIGH) {
      servo.write(0);
      stage++;
      timeOfLastPam = millis();
    }
  }
  else if (stage == 5) {                                                //Wait for the door to close and go back to stage 0: waiting for another pamphlet dispensing command
    servo.write(0);
    if (millis() - timeOfLastPam > 1100) {
      stage = 0;
      timeOfLastPam = millis();
    }
  }



}
