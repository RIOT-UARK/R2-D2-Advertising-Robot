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

  Using an Arduino Uno was necessary because the PROK Motor controller would only
  work with the Uno's 5V logic, and not an ESP32's 3V logic.
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

unsigned long curTime;            /* current time, time since boot in ms              */
unsigned long timeOfLastRec;      /* Time since last drive signal received            */
short driveValue;                 /* Drive signal to send to dome motor controller    */
unsigned long timeOfLastPam = 0;  /* Time of last pamphlet staging event              */
int stage = 0;                    /* Pamphlet dispenser pamphlet feeding stage        */
char incomingSerial[3];           /* Serial communication 'packet'                    */
unsigned short badPktCounter = 0; /* How many invalid packets have we received        */
bool DomeMovementEnabled = true;  /* To turn off dome movement if too many bad pkts   */

//bool forwardReverse = true; //true = pamphlet dispenser forward, false = backwards
//bool pamReady = true;

Servo PDservo;                      /* Pamphlet door servo                              */
Servo IrisServo;


void emergencyDisconnect() {
  // Disconnect everything
  digitalWrite(PIN_2, LOW);
  digitalWrite(PIN_4, LOW);
  analogWrite(PIN_3, 0);
  analogWrite(PIN_6, 0);
  PDservo.detach();
  IrisServo.detach();
  
  // Enter an inescapable superloop
  while (1) {
    // DO NOT PUT ANY CODE HERE. THIS LOOP IS MEANT TO STOP ALL EXECUTION
  }
}


/*----------------------------------------------------------------------

    Setup function

----------------------------------------------------------------------*/

void setup() {

  pinMode(PIN_2, OUTPUT); /* IN1 DROK Motor controller direction                    */
  pinMode(PIN_3, OUTPUT); /* ENA1DROK Motor controller PWM Speed                    */
  pinMode(PIN_4, OUTPUT); /* IN2 DROK Motor controller direction                    */
  pinMode(PIN_5, INPUT);  /* Signal from BodyESP32 Receiver to dispense a pamphlet  */
  pinMode(PIN_6, OUTPUT); /* ENA2                                                   */
  pinMode(PIN_7, OUTPUT); /* IN3                                                    */
  pinMode(PIN_8, OUTPUT); /* IN4                                                    */
  pinMode(PIN_10, INPUT);
  //pinMode(9, OUTPUT); //Servo Motor

  Serial.begin(115200);

  PDservo.attach(PIN_9);
  PDservo.write(0);

  IrisServo.attach(PIN_11);
  IrisServo.write(0);

  delay(3500); //Delay because the ESP32 will spew garbage over serial upon startup.
}

/*----------------------------------------------------------------------

    Microcontroller Superloop

----------------------------------------------------------------------*/

void loop() {
  curTime = millis();
 
  if (Serial.available() > 0 && DomeMovementEnabled) {

      //Receive 3 byte serial packet containing driveValue
      Serial.readBytesUntil(SERIAL_PKT_TERMINATOR, incomingSerial, SERIAL_PKT_SIZE);
      driveValue = word( incomingSerial[0], incomingSerial[1] );
      Serial.println(driveValue);
      timeOfLastRec = curTime;
    //TODO: Provide a good explanation of how the signals to the DROK motor controller works
    // and setting pins 2 and 4 control direction/braking

    if (driveValue > 12 && driveValue < 103) {
      digitalWrite(PIN_2, LOW);
      digitalWrite(PIN_4, HIGH);
      driveValue = int(((float(driveValue) - 12) * 255.0) / 88.0);
      }
    else if (driveValue < -12 && driveValue > -103) {
      digitalWrite(PIN_2, HIGH);
      digitalWrite(PIN_4, LOW);
      driveValue = int(((float(abs(driveValue)) - 12) * 255.0) / 88.0);   
      }
    // If we receive emergency disconnect signal
    else if (driveValue == SERIAL_PKT_EMERGENCY_DISCONNECT) {
      emergencyDisconnect();
    }
    /* If we receive a lot of bad data, we disable dome movement for safety purposes */
    else if (driveValue < -103 && driveValue > 103) {
      badPktCounter++;
      driveValue = 0;

      if (badPktCounter >= 3000) {
        DomeMovementEnabled = false;
      }
    }
  
  // Write Dome driveValue as PWM to motor controller
  analogWrite(PIN_3, driveValue);
  }

  if ((curTime - timeOfLastRec) > 750) { //If not recieving signal from controller, stop the motor.
    digitalWrite(PIN_2, LOW);
    digitalWrite(PIN_4, LOW);
    analogWrite(PIN_3, 0);
  }

/*--------------------------------------------------------------------------------------
  IRIS SERVO
--------------------------------------------------------------------------------------*/

  if (digitalRead(PIN_10) == HIGH) {
    IrisServo.write(255);
  }
  else {
    IrisServo.write(0);
  }

  /*
  if (pamReady && digitalRead(5) == HIGH && (millis() - timeOfLastPam > 1000)) { //When dispense signal first received
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
    PDservo.write(90);
    analogWrite(6, 60);
    timeOfLastPam = millis();
    pamReady = false;
  }
  else if (millis() - timeOfLastPam > 1500 && millis() - timeOfLastPam < 3300) { //Reverse Motors to dispense pamphlet
    digitalWrite (7, HIGH);
    digitalWrite (8, LOW);
    PDservo.write(90);
    analogWrite(6, 60);
  }
  else if (millis() - timeOfLastPam > 3300 && millis() - timeOfLastPam < 5000) { //Wait until at least 5 seconds have passed
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    analogWrite(6, 0);
  }
  else if (!pamReady && digitalRead(5) == HIGH && (millis() - timeOfLastPam > 5000)) {
    PDservo.write(0);
    timeOfLastPam = millis();
    pamReady = true;
  }
  */
/*--------------------------------------------------------------------------------------
  PAMPHLET DISPENSER + DOOR SERVO MOTOR CONTROL
--------------------------------------------------------------------------------------*/

  if (stage == DOOR_CLOSED_PD_READY) {            // Door is closed and Pamphlet Dispenser is ready to receive signal from controller
    // If receiving signal from receiver and has been >1sec since last pamphlet event
    if (digitalRead(PIN_5) == HIGH && (curTime - timeOfLastPam > 1000)) {
      stage++;
      timeOfLastPam = curTime;
    }
  }
  else if (stage == LOAD_PAMPHLET_OPEN_DOOR) {    // Signal to print has been received. Load a pamphlet in paper feeder (move motor in one direction) and open the door
    digitalWrite(PIN_7, LOW);         // Motor Controller feed motor direction
    digitalWrite(PIN_8, HIGH);    
    PDservo.write(90);                  // Pamphlet door servo position
    analogWrite(PIN_6, 80);           // PWM Speed signal to pamphlet feed motor
    if (curTime - timeOfLastPam > 1500) {
      stage++;
    }
  }
  else if (stage == FEED_PAPER_OUT) {             // Feed the paper out of the pamphlet dispenser.
    digitalWrite (PIN_7, HIGH);       // Motor Controller feed motor direction
    digitalWrite (PIN_8, LOW);
    PDservo.write(90);              // Pamphlet door servo position
    analogWrite(PIN_6, 80);           // PWM Speed signal to pamphlet feed motor
    if (curTime - timeOfLastPam > 3500) {
      stage++;
    }
  }
  else if (stage == PAPER_IS_OUT_WAIT) {          // Stop the paper feeder motor when the paper is just about out of the dispenser.
    digitalWrite(PIN_7, LOW);         // Motor Controller feed motor direction
    digitalWrite(PIN_8, LOW);
    analogWrite(PIN_6, 0);            // PWM Speed signal to pamphlet feed motor
    if (curTime - timeOfLastPam > 5000) {
      stage++;
    }
  }
  else if (stage == WAIT_FOR_USER_INPUT) {        // Wait for an additional button press from the controller to signal to close the door
    if (digitalRead(PIN_5) == HIGH) {
      PDservo.write(0);             // Pamphlet door servo position
      stage++;
      timeOfLastPam = curTime;
    }
  }
  else if (stage == CLOSE_DOOR_END_CYCLE) {      //Wait for the door to close and go back to stage 0: waiting for another pamphlet dispensing command
    PDservo.write(0);               // Pamphlet door servo position
    if (curTime - timeOfLastPam > 1100) {
      stage = DOOR_CLOSED_PD_READY;
      timeOfLastPam = curTime;
    }
  }

}
