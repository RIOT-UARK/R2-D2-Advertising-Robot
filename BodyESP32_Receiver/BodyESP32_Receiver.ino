/*----------------------------------------------------------------------------

    BodyESP32_Receiver.ino

    DESCRIPTION:
      This code recieves PWM channels from the radio receiver and sends PWM
      signals to the motor controllers of the bot. This microcontroller is
      necessary due to having different 'modes' of operation. I.e.: Moving
      forward requires sending a positive value to both the left and right
      drive motors. Turning the bot would then require 'skid steering':
      sending a positive value to one motor, and a negative one to another.

-----------------------------------------------------------------------------*/

/*
TODO: Implement the following modes controlled by Switches 7-10: Drive, Dome rotation, emote, etc. 

ESP32 to Motor Controllers pinout
  PIN 26 --> SIGNAL INPUT FOR L Side of Drive Motor Controller (White Wire)
  PIN 27 --> SIGNAL INPUT FOR R Side of Drive Motor Controller (2.54 dupont labelled 'S')
  RX --> TX ON ARDUINO NANO (HOPEFULLY I WILL CHANGE THIS FROM SERIAL COMM TO smthing else)
  TODO: Better document pinout

*/

#include <esp_now.h>
#include <WiFi.h>
#include "definitions.h"

// Reciever to ESP32 pinout
#define CH1   PIN_13
#define CH2   PIN_4
#define CH3   PIN_16
#define CH4   PIN_17
#define CH5   PIN_5
#define CH6   PIN_18
#define CH7   PIN_19
#define CH8   PIN_21
#define CH9   PIN_22
#define CH10  PIN_23

//Global Variables for interrupt functions for timing PWM signals.
//Sadly, you cant pass arguments in to interrupt functions, so I have to have
//different global variables and interrupt functions for each pin.
volatile unsigned long    CH1PulseBegin = 0;
volatile unsigned long    CH1PulseEnd = 0;
volatile bool             CH1NewPulseDurAvailable = false;

volatile unsigned long    CH2PulseBegin = 0;
volatile unsigned long    CH2PulseEnd = 0;
volatile bool             CH2NewPulseDurAvailable = false;

volatile unsigned long    CH3PulseBegin = 0;
volatile unsigned long    CH3PulseEnd = 0;
volatile bool             CH3NewPulseDurAvailable = false;

volatile unsigned long    CH4PulseBegin = 0;
volatile unsigned long    CH4PulseEnd = 0;
volatile bool             CH4NewPulseDurAvailable = false;

volatile unsigned long    CH5PulseBegin = 0;
volatile unsigned long    CH5PulseEnd = 0;
volatile bool             CH5NewPulseDurAvailable = false;

volatile unsigned long    CH6PulseBegin = 0;
volatile unsigned long    CH6PulseEnd = 0;
volatile bool             CH6NewPulseDurAvailable = false;

volatile unsigned long    CH7PulseBegin = 0;
volatile unsigned long    CH7PulseEnd = 0;
volatile bool             CH7NewPulseDurAvailable = false;

volatile unsigned long    CH8PulseBegin = 0;
volatile unsigned long    CH8PulseEnd = 0;
volatile bool             CH8NewPulseDurAvailable = false;

volatile unsigned long    CH9PulseBegin = 0;
volatile unsigned long    CH9PulseEnd = 0;
volatile bool             CH9NewPulseDurAvailable = false;

volatile unsigned long    CH10PulseBegin = 0;
volatile unsigned long    CH10PulseEnd = 0;
volatile bool             CH10NewPulseDurAvailable = false;

// Ints to represent controller's stick positions, 3 way switch, and potentiometer values
int ch1Value, ch2Value, ch3Value, ch4Value, ch5Value, ch6Value, ch9Value;

//For tracking volume sent to audio board
int lastVolumeSent = 0;

// Bool to represent 2-way switches value
bool ch7Value;
bool ch8Value;
bool ch10Value;

// Ints to be sent to Motor Controllers
int LeftDriveValue, RightDriveValue, DomeMotorValue, Motor4Value;

// Last time an emote was sent to head
unsigned long lastEmoteSent = 0;

// Controls mosfet for projector bulb
bool projectorBulb = false;
unsigned long lastProjectorStatus = 0;

// Control signals for pamphlet dispenser
bool pamphletDispenser = false;
bool turnPDOnFlag = false;
unsigned long lastPamphletDispenserStatus = 0;

// ESP-NOW broadcast address - Broadcast as WAN
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ESP-NOW packet struct
typedef struct esp_pkt {
    String recip;
    int a;
    int vol;
} esp_pkt;

//packet to send
esp_pkt packet;
//packet that is received
esp_pkt incomingReadings;

esp_now_peer_info_t peerInfo;

/*----------------------------------------------------------------------

    OnDataRecv

      Callback function when data is received

----------------------------------------------------------------------*/

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&packet, incomingData, sizeof(packet));

    //This Receiver microcontroller will be receiving data from the computer vision Camera.
    //Todo: add code to receive input packets from the camera and send commands to the Dome motor. 

    if (packet.a ==6) {
      projectorBulb = !projectorBulb;
    }
    if (packet.a == 7) {
      pamphletDispenser = !pamphletDispenser;
      turnPDOnFlag = true;
    }
}
//Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


/*----------------------------------------------------------------------

    Interrupt functions to measure PWM signals

----------------------------------------------------------------------*/

void CH1Interrupt() {
  if (digitalRead(CH1) == HIGH) {
    // start measuring
    CH1PulseBegin = micros();
  }
  else {
    // stop measuring
    CH1PulseEnd = micros();
    CH1NewPulseDurAvailable = true;
  }
}
void CH2Interrupt() {
  if (digitalRead(CH2) == HIGH) {
    // start measuring
    CH2PulseBegin = micros();
  }
  else {
    // stop measuring
    CH2PulseEnd = micros();
    CH2NewPulseDurAvailable = true;
  }
}
void CH3Interrupt() {
  if (digitalRead(CH3) == HIGH) {
    // start measuring
    CH3PulseBegin = micros();
  }
  else {
    // stop measuring
    CH3PulseEnd = micros();
    CH3NewPulseDurAvailable = true;
  }
}
void CH4Interrupt() {
  if (digitalRead(CH4) == HIGH) {
    // start measuring
    CH4PulseBegin = micros();
  }
  else {
    // stop measuring
    CH4PulseEnd = micros();
    CH4NewPulseDurAvailable = true;
  }
}
void CH5Interrupt() {
  if (digitalRead(CH5) == HIGH) {
    // start measuring
    CH5PulseBegin = micros();
  }
  else {
    // stop measuring
    CH5PulseEnd = micros();
    CH5NewPulseDurAvailable = true;
  }
}
void CH6Interrupt() {
  if (digitalRead(CH6) == HIGH) {
    // start measuring
    CH6PulseBegin = micros();
  }
  else {
    // stop measuring
    CH6PulseEnd = micros();
    CH6NewPulseDurAvailable = true;
  }
}
void CH7Interrupt() {
  if (digitalRead(CH7) == HIGH) {
    // start measuring
    CH7PulseBegin = micros();
  }
  else {
    // stop measuring
    CH7PulseEnd = micros();
    CH7NewPulseDurAvailable = true;
  }
}
void CH8Interrupt() {
  if (digitalRead(CH8) == HIGH) {
    // start measuring
    CH8PulseBegin = micros();
  }
  else {
    // stop measuring
    CH8PulseEnd = micros();
    CH8NewPulseDurAvailable = true;
  }
}
void CH9Interrupt() {
  if (digitalRead(CH9) == HIGH) {
    // start measuring
    CH9PulseBegin = micros();
  }
  else {
    // stop measuring
    CH9PulseEnd = micros();
    CH9NewPulseDurAvailable = true;
  }
}
void CH10Interrupt() {
  if (digitalRead(CH10) == HIGH) {
    // start measuring
    CH10PulseBegin = micros();
  }
  else {
    // stop measuring
    CH10PulseEnd = micros();
    CH10NewPulseDurAvailable = true;
  }
}


/*----------------------------------------------------------------------

    Setup function

----------------------------------------------------------------------*/
 
void setup(){
  // Set up serial monitor
  Serial.begin(115200);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }

  //Register callback functions
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  //Register peer network
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  //Add peer network
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //Serial.println("Failed to add peer");
    return;
  }

  // Set all pins as inputs
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
  pinMode(CH7, INPUT);
  pinMode(CH8, INPUT);
  pinMode(CH9, INPUT);
  pinMode(CH10, INPUT);

// As of now, the only recipient of messages from this board will be the Audio board.
  packet.recip = "audi";

// Attach interrupt callback functions to be called on high-low transitions.
  attachInterrupt(digitalPinToInterrupt(CH1), CH1Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2), CH2Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH3), CH3Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH4), CH4Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH5), CH5Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH6), CH6Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH7), CH7Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH8), CH8Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH9), CH9Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH10), CH10Interrupt, CHANGE);

//PWM OUTPUT TO CONTROL LEFT DRIVE MOTOR
  pinMode(PIN_26, OUTPUT);
  ledcAttachPin(PIN_26, 0);      // (pin, channel)
  ledcSetup(CHANNEL_0, 1000, 8); // (channel, frequency, resolution)

//PWM OUTPUT TO CONTROL RIGHT DRIVE MOTOR
  pinMode(PIN_27, OUTPUT);
  ledcAttachPin(PIN_27, 1);      // (pin, channel)
  ledcSetup(CHANNEL_1, 1000, 8); // (channel, frequency, resolution)


 pinMode(PIN_32, OUTPUT); //Projector Bulb MOSFET Control
 pinMode(PIN_33, OUTPUT); //Pamphlet Dispenser Signal
}

/*----------------------------------------------------------------------

    Microcontroller Superloop

----------------------------------------------------------------------*/

void loop() {

  long int time = millis();
  
  /*--------------------------------------------
  If there has been a high-to-low transition
  on a PWM channel, then we have a new pulse
  width available and can map that width to an
  int value
  --------------------------------------------*/
  if (CH1NewPulseDurAvailable) {
    CH1NewPulseDurAvailable = false;
    ch1Value = map((CH1PulseEnd - CH1PulseBegin), 1000, 2000, -100, 100);
    //Serial.println(CH1PulseEnd - CH1PulseBegin);
  }
  if (CH2NewPulseDurAvailable) {
    CH2NewPulseDurAvailable = false;
    if ((CH2PulseEnd - CH2PulseBegin) < 2000) {
    ch2Value = map((CH2PulseEnd - CH2PulseBegin), 0, 3302, 0, 255);
    }
    //Serial.println(CH2PulseEnd - CH2PulseBegin);
  }
  if (CH3NewPulseDurAvailable) {
    CH3NewPulseDurAvailable = false;
    ch3Value = map((CH3PulseEnd - CH3PulseBegin), 1000, 2000, -100, 100);
    //Serial.println(CH3PulseEnd - CH3PulseBegin);
  }
  if (CH4NewPulseDurAvailable) {
    CH4NewPulseDurAvailable = false;
    ch4Value = map((CH4PulseEnd - CH4PulseBegin), 1000, 2000, -100, 100);
    //Serial.println(CH4PulseEnd - CH4PulseBegin);
  }
  if (CH5NewPulseDurAvailable) {
    CH5NewPulseDurAvailable = false;
    ch5Value = map((CH5PulseEnd - CH5PulseBegin), 1000, 2000, 0, 100);
    //Serial.println(CH5PulseEnd - CH5PulseBegin);
  }
  if (CH6NewPulseDurAvailable) {
    CH6NewPulseDurAvailable = false;
    ch6Value = map((CH6PulseEnd - CH6PulseBegin), 1000, 2000, -100, 100);
    //Serial.println(CH6PulseEnd - CH6PulseBegin);
  }
  if (CH7NewPulseDurAvailable) {
    CH7NewPulseDurAvailable = false;
    if ((CH7PulseEnd - CH7PulseBegin) < 1500) {
      ch7Value = 0; //Switch flipped up (off)
    }
    else {
      ch7Value = 1; //Switch flipped down (on)
    }
    //Serial.println(CH7PulseEnd - CH7PulseBegin);
  }
  if (CH8NewPulseDurAvailable) {
    CH8NewPulseDurAvailable = false;
    if ((CH8PulseEnd - CH8PulseBegin) < 1500) {
      ch8Value = 0; //Switch flipped up (off)
    }
    else {
      ch8Value = 1; //Switch flipped down (on)
    }
    //Serial.println(CH8PulseEnd - CH8PulseBegin);
  }
  if (CH9NewPulseDurAvailable) {
    CH9NewPulseDurAvailable = false;
    if ((CH9PulseEnd - CH9PulseBegin) < 1250) {
      ch9Value = 0; //Switch flipped up (off)
    }
    else if ((CH9PulseEnd - CH9PulseBegin) < 1750) {
      ch9Value = 1; //Switch flipped to middle state
    }
    else {
      ch9Value = 2; //Switch flipped down (on)
    }
    //Serial.println(CH9PulseEnd - CH9PulseBegin);
  }
  if (CH10NewPulseDurAvailable) {
    CH10NewPulseDurAvailable = false;
    if ((CH10PulseEnd - CH10PulseBegin) < 1500) {
      ch10Value = 0; //Switch flipped up (off)
    }
    else {
      ch10Value = 1; //Switch flipped down (on)
    }
    //Serial.println(CH10PulseEnd - CH10PulseBegin);
  }
  
  /*--------------------------------------------
  Switch conditions for R2D2 to be in
  DRIVE MODE
  --------------------------------------------*/
  if (ch7Value == 0 && ch8Value == 0 && ch9Value == 0 && ch10Value == 0) {
    //Channel 2 (Right stick) --> Forwards/Backwards (LMotor + RMotor) 
    //Channel 1 (Right stick) --> Turn Left/Right (Either -LMotor/+RMotor or just +1Motor forward )
    
    //DOME MOVEMENT
    //Channel 4 (Left stick)  --> Turn Left/Right (dome motor)
    if (abs(ch4Value) > 15) { // If left stick not being pressed
      Serial.write(ch4Value);
    }
    else {
      // TODO: perhaps enforce a channel value reset???
    }

    if (ch2Value > -1 && ch2Value < 256) {
      int LMotorPulseWidth, RMotorPulseWidth;

      if (abs(ch1Value) > 12) {
        LMotorPulseWidth = ch2Value + (float(ch1Value) * 0.1); //NOTE: Maybe implement dead zones for ch1.
        RMotorPulseWidth = ch2Value - (float(ch1Value) * 0.1);
      }
      else {
        LMotorPulseWidth = ch2Value;
        RMotorPulseWidth = ch2Value;
      }
      ledcWrite(0, LMotorPulseWidth);
      ledcWrite(1, RMotorPulseWidth);
    }
    else {
      ledcWrite(CHANNEL_0, 127);
      ledcWrite(CHANNEL_1, 127);
    }
  }

  /*--------------------------------------------
  Switch conditions for R2D2 to be in
  DOME ROTATION ONLY MODE
  --------------------------------------------*/
  if (ch7Value == 1 && ch8Value == 0 && ch9Value == 0 && ch10Value == 0) {
  //Channel 1 --> Turn Dome Left/Right
  // TODO: IMPLEMENT DOME ROTATION MODE
  }

  /*--------------------------------------------
  Switch conditions for R2D2 to be in
  EMOTE MODE
  --------------------------------------------*/
  if (ch7Value == 0  && ch8Value == 1 && ch9Value == 0 && ch10Value == 0) {

  //Move joysticks all the way to one direction to trigger an emote.
  //Break emote functions out into functions above the main loop

  //if emote 'a' is selected -> packet.a = 2;
  //then -> esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));

    if ( time - lastEmoteSent > 5000 ) {
      //Right stick left
      if (ch1Value < -92) {
        packet.a = 10;
        lastEmoteSent = time;
        esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
      }
      //Right stick right
      else if (ch1Value > 92) {
        packet.a = 11;
        lastEmoteSent = time;
        esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
      }
      //Right stick down
      else if (ch2Value < 25) {
        packet.a = 13;
        lastEmoteSent = time;
        esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
      }
      //Right stick up
      else if (ch2Value > 228) {  // TODO: CHECK TO MAKE SURE THIS VALUE IS ACHIEVABLE
        packet.a = 14;
        lastEmoteSent = time;
        esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
      }
    
    }
  }

  /*--------------------------------------------
  If volume knob has changed >5 since last 
  volume was sent, send a new packet instructing
  the volume to change
  TODO: What are initial conditions???
  --------------------------------------------*/
  if (abs(lastVolumeSent - ch5Value) > 5) {
    lastVolumeSent = ch5Value;
    packet.recip = "audi";
    packet.vol = lastVolumeSent;
    packet.a = -1;
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  //If projectorBulb == true, close MOSFET switch to turn on bulb.
  if (projectorBulb) {
    digitalWrite(32, HIGH);
  }
  else {
    digitalWrite(32, LOW);
  }
  // Sent projector status to soundboard every 5 seconds
  if (millis() - lastProjectorStatus > 5000) {
    if (projectorBulb) {
      packet.a = 21;
    }
    else {
      packet.a = 20;
    }
    packet.recip = "boar";
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
    lastProjectorStatus = millis();
  }

  /*--------------------------------------------
  Control Pamphlet dispenser movement
  TODO: investigate and better document how this works
  --------------------------------------------*/

  if (pamphletDispenser && turnPDOnFlag && (millis() - lastPamphletDispenserStatus > 1000)) {
    digitalWrite(PIN_33, HIGH);
    lastPamphletDispenserStatus  = millis();
    turnPDOnFlag = false;
  }
  else if (pamphletDispenser && !turnPDOnFlag && (millis() - lastPamphletDispenserStatus > 1000)) {
    digitalWrite(PIN_33, LOW);
    pamphletDispenser = false;
  }

  /*--------------------------------------------
  DEBUG SERIAL PRINTS
  --------------------------------------------*/

//Serial.println(projectorBulb);
  //Serial.println(millis() - time);
  //Room for 2 more 'modes' 
  // Print to Serial Monitor
/*  Serial.print("Ch1: ");
  Serial.print(ch1Value);
  Serial.print(" | Ch2: ");
  Serial.print(ch2Value);
  Serial.print(" | Ch3: ");
  Serial.print(ch3Value);
  Serial.print(" | Ch4: ");
  Serial.print(ch4Value);
  Serial.print(" | Ch5: ");
  Serial.print(ch5Value);
  Serial.print(" | Ch6: ");
  Serial.print(ch6Value);
  Serial.print(" | Ch7: ");
  Serial.print(ch7Value);
  Serial.print(" | Ch8: ");
  Serial.print(ch8Value);
  Serial.print(" | Ch9: ");
  Serial.print(ch9Value);
  Serial.print(" | Ch10: ");
  Serial.println(ch10Value); */
  
}
