/*----------------------------------------------------------------------------

    Secondary_Motor_Driver.ino

    DESCRIPTION:
      Arduino Uno codde for controlling the Pololu Dome motor and pamphlet
      dispenser motor. This board receives an integer over serial (may change 
      this to a faster comm method later) from the Main ESP32 Receiver Board,
      then uses that to control the speed and direction of the Dome Motor.


    MICROCONTROLLER:
      Arduino Uno

-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------

    MICROCONTROLLER PINOUT

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
--------------------------------------------------------------------------------*/

// TODO: Implement Encoder Readings

/*-------------------------------------------------------------------
        INCLUDES
--------------------------------------------------------------------*/

#include <Servo.h>
#include <R2D2_LIB.h>


/*-------------------------------------------------------------------
        GLOBAL VARIABLES
--------------------------------------------------------------------*/

unsigned long curTime;            /* current time, time since boot in ms           */
unsigned long timeOfLastRec;      /* Time since last drive signal received         */
int driveValue;                   /* Drive signal to send to dome motor controller */
unsigned long timeOfLastPam = 0;  /* Time of last pamphlet staging event           */
int stage = 0;                    /* Pamphlet dispenser pamphlet feeding stage     */

//bool forwardReverse = true; //true = pamphlet dispenser forward, false = backwards
//bool pamReady = true;

Servo servo;                      /* Pamphlet door servo                           */


/*----------------------------------------------------------------------

    Setup function

----------------------------------------------------------------------*/

void setup() {

  pinMode(PIN_2, OUTPUT); /* IN1 DROK Motor controller direction */
  pinMode(PIN_3, OUTPUT); /* ENA1DROK Motor controller PWM Speed */
  pinMode(PIN_4, OUTPUT); /* IN2 DROK Motor controller direction */
  pinMode(PIN_5, INPUT);  /* Signal from BodyESP32 Receiver to dispense a pamphlet */
  pinMode(PIN_6, OUTPUT); /* ENA2                                */
  pinMode(PIN_7, OUTPUT); /* IN3                                 */
  pinMode(PIN_8, OUTPUT); /* IN4                                 */
  //pinMode(9, OUTPUT); //Servo Motor

  Serial.begin(115200);

  servo.attach(PIN_9);
  servo.write(0);

  delay(3500); //Delay because the ESP32 will spew garbage over serial upon startup.
}

/*----------------------------------------------------------------------

    Microcontroller Superloop

----------------------------------------------------------------------*/

void loop() {
  curTime = millis();
 
  if (Serial.available() > 0) {
      driveValue = Serial.read();  
      timeOfLastRec = curTime;
    //Serial.println(driveValue);
    //TODO: Provide a good explanation of how the signals to the DROK motor controller works
    // and setting pins 2 and 4 control direction/braking

    if (driveValue > 14 && driveValue < 100) {
      digitalWrite(PIN_2, LOW);
      digitalWrite(PIN_4, HIGH);
      driveValue = int(((float(driveValue) - 14) * 255.0) / 85.0);
      }
    else if (driveValue < 241 && driveValue > 155) {
      digitalWrite(PIN_2, HIGH);
      digitalWrite(PIN_4, LOW);
      driveValue = int(((240 - float(driveValue)) * 255.0) / 85.0);   
      }
    else {
      driveValue = 0;
    }
  
  // Write Dome driveValue as PWM to motor controller
  analogWrite(PIN_3, driveValue);
  }

  if ((curTime - timeOfLastRec) > 200) { //If not recieving signal from controller, stop the motor.
    digitalWrite(PIN_2, LOW);
    digitalWrite(PIN_4, LOW);
    analogWrite(PIN_3, 0);
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
/*--------------------------------------------------------------------------------------
  PAMPHLET DISPENSER + DOOR SERVO MOTOR CONTROL
--------------------------------------------------------------------------------------*/

  if (stage == 0) {            // Door is closed and Pamphlet Dispenser is ready to receive signal from controller
    // If receiving signal from receiver and has been >1sec since last pamphlet event
    if (digitalRead(PIN_5) == HIGH && (curTime - timeOfLastPam > 1000)) {
      stage++;
      timeOfLastPam = curTime;
    }
  }
  else if (stage == 1) {       // Signal to print has been received. Load a pamphlet in paper feeder (move motor in one direction) and open the door
    digitalWrite(PIN_7, LOW);         // Motor Controller feed motor direction
    digitalWrite(PIN_8, HIGH);    
    servo.write(90);                  // Pamphlet door servo position
    analogWrite(PIN_6, 80);           // PWM Speed signal to pamphlet feed motor
    if (curTime - timeOfLastPam > 1500) {
      stage++;
    }
  }
  else if (stage == 2) {      // Feed the paper out of the pamphlet dispenser.
    digitalWrite (PIN_7, HIGH);       // Motor Controller feed motor direction
    digitalWrite (PIN_8, LOW);
    servo.write(90);              // Pamphlet door servo position
    analogWrite(PIN_6, 80);           // PWM Speed signal to pamphlet feed motor
    if (curTime - timeOfLastPam > 3500) {
      stage++;
    }
  }
  else if (stage == 3) {      // Stop the paper feeder motor when the paper is just about out of the dispenser.
    digitalWrite(PIN_7, LOW);         // Motor Controller feed motor direction
    digitalWrite(PIN_8, LOW);
    analogWrite(PIN_6, 0);            // PWM Speed signal to pamphlet feed motor
    if (curTime - timeOfLastPam > 5000) {
      stage++;
    }
  }
  else if (stage == 4) {      // Wait for an additional button press from the controller to signal to close the door
    if (digitalRead(PIN_5) == HIGH) {
      servo.write(0);             // Pamphlet door servo position
      stage++;
      timeOfLastPam = curTime;
    }
  }
  else if (stage == 5) {      //Wait for the door to close and go back to stage 0: waiting for another pamphlet dispensing command
    servo.write(0);               // Pamphlet door servo position
    if (curTime - timeOfLastPam > 1100) {
      stage = 0;
      timeOfLastPam = curTime;
    }
  }

}
