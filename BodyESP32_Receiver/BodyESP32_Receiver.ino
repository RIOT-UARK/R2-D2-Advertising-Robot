/*----------------------------------------------------------------------------

    BodyESP32_Receiver.ino

    DESCRIPTION:
      This code recieves PWM channels from the radio receiver and sends PWM
      signals to the motor controllers of the bot. This microcontroller is
      necessary due to having different 'modes' of operation. I.e.: Moving
      forward requires sending a positive value to both the left and right
      drive motors. Turning the bot would then require 'skid steering':
      sending a positive value to one motor, and a negative one to another.

    MICROCONTROLLER:
      ESP-32

-----------------------------------------------------------------------------*/

/*
TODO: Implement the following modes controlled by Switches 7-10: Drive, Dome rotation, emote, etc. 

ESP32 to Motor Controllers pinout
  PIN 26 --> SIGNAL INPUT FOR L Side of Drive Motor Controller (White Wire)
  PIN 27 --> SIGNAL INPUT FOR R Side of Drive Motor Controller (2.54 dupont labelled 'S')
      RX --> TX ON ARDUINO NANO (HOPEFULLY I WILL CHANGE THIS FROM SERIAL COMM TO smthing else)
  TODO: Better document pinout

*/

/*-------------------------------------------------------------------
        INCLUDES
--------------------------------------------------------------------*/

#include <esp_now.h>
#include <WiFi.h>
#include <R2D2_LIB.h>

/*-------------------------------------------------------------------
        DEFINES AND CONSTANTS
--------------------------------------------------------------------*/

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

/*-------------------------------------------------------------------
        GLOBAL VARIABLES
--------------------------------------------------------------------*/

// Variables for interrupt functions for timing PWM signals.
// Sadly, you cant pass arguments in to interrupt functions, so I have to have
// different global variables and interrupt functions for each pin.
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

unsigned long curTime = 0;                      /* Current time, in ms */

// Ints to represent controller's stick positions, 3 way switch, and potentiometer values
int ch2Value, ch3Value, ch5Value, ch6Value, ch9Value;

//CAUTION: CH1VALUE AND CH4VALUE USED TO BE INTS. ENSURE THIS DOESN'T CAUSE
//UNFORSEEN CONSEQUENCES
short ch1Value, ch4Value;
unsigned long lastCh1ValueSent = 0;
unsigned long lastCh4ValueSent = 0;
//Outgoing Serial 'packet' for communication with Secondary Motor Driver
byte outgoingSerial[3];

int lastVolumeSent = 0;                         /* For tracking volume sent to audio board */

// Bool to represent 2-way switches value
bool ch7Value;
bool ch8Value;
bool ch10Value;

int LeftDriveValue, RightDriveValue;            /* Ints to be sent to Motor Controllers*/

unsigned long lastEmoteSent = 0;                /* Last time an emote was sent to head */

unsigned int curEmoteMode = NO_EMOTE;           /* Current Emote mode */

unsigned long lastAICamControlPktSent = 0;      /* Last time we sent an AICam Control packet. */

unsigned long lastAICamPacketReceived = 0;      /* Last time we received an AICam Driving packet */

bool projectorBulb = false;                     /* Controls mosfet for projector bulb */
unsigned long lastProjectorStatus = 0;          /* Last time projector was toggled    */

// Control signals for pamphlet dispenser
bool pamphletDispenser = false;
bool turnPDOnFlag = false;
unsigned long lastPamphletDispenserStatus = 0;

bool AICamMode;                                 /* Whether or not we're in AICamMode */

// ESP-NOW broadcast address - Broadcast as WAN
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct_message packet;                          /* ESP NOW packet for outbound wireless communications */

struct_message incomingReadings;                /* packet that is received */

esp_now_peer_info_t peerInfo;

/*----------------------------------------------------------------------

    OnDataRecv

      Callback function when data is received

----------------------------------------------------------------------*/

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&packet, incomingData, sizeof(packet));

    //This Receiver microcontroller will be receiving data from the computer vision Camera.
    //Todo: add code to receive input packets from the camera and send commands to the Dome motor. 

    if (packet.recipient == BODY_ESP32_RECEIVER) {
      switch(packet.role) {
        case TOGGLE_PROJECTOR_BULB:
          projectorBulb = !projectorBulb;
          break;
        case TRIGGER_PAMPHLET_DISPENSER_EVENT:
          pamphletDispenser = !pamphletDispenser;
          turnPDOnFlag = true;
          break;
        case AI_CAM_TURN_DOME_LEFT:
          lastAICamPacketReceived = curTime;
          Serial.write(-30);
          break;
        case AI_CAM_TURN_DOME_RIGHT:
          lastAICamPacketReceived = curTime;
          Serial.write(30);
          break;
        case AI_CAM_NO_DOME_TURN:
          lastAICamPacketReceived = curTime;
          Serial.write(0);
          break;
      }
    }
}


/*----------------------------------------------------------------------

    OnDataSent

      Callback when data is sent

----------------------------------------------------------------------*/

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

    executeEmote1

      Triggers Emote 1

----------------------------------------------------------------------*/

void executeEmote1() {
  long timeSinceEmoteStart = curTime - timeSinceEmoteStart;

  if (timeSinceEmoteStart <= 1000) {
    //Set servos, send commands, etc
  }
  else if (timeSinceEmoteStart <= 2500) {
    
  }
  else if (timeSinceEmoteStart <= 4500) {
    
  }
  else {
    curEmoteMode = NO_EMOTE;
  }
}


/*----------------------------------------------------------------------

    executeEmote2

      Triggers Emote 2

----------------------------------------------------------------------*/

void executeEmote2() {
  long timeSinceEmoteStart = curTime - timeSinceEmoteStart;

  if (timeSinceEmoteStart <= 1000) {
    //Set servos, send commands, etc
  }
  else if (timeSinceEmoteStart <= 2500) {
    
  }
  else if (timeSinceEmoteStart <= 4500) {
    
  }
  else {
    curEmoteMode = NO_EMOTE;
  }
}


/*----------------------------------------------------------------------

    executeEmote3

      Triggers Emote 3

----------------------------------------------------------------------*/

void executeEmote3() {
  long timeSinceEmoteStart = curTime - timeSinceEmoteStart;

  if (timeSinceEmoteStart <= 1000) {
    //Set servos, send commands, etc
  }
  else if (timeSinceEmoteStart <= 2500) {
    
  }
  else if (timeSinceEmoteStart <= 4500) {
    
  }
  else {
    curEmoteMode = NO_EMOTE;
  }
}


/*----------------------------------------------------------------------

    executeEmote4

      Triggers Emote 4

----------------------------------------------------------------------*/

void executeEmote4() {
  long timeSinceEmoteStart = curTime - timeSinceEmoteStart;

  if (timeSinceEmoteStart <= 1000) {
    //Set servos, send commands, etc
  }
  else if (timeSinceEmoteStart <= 2500) {
    
  }
  else if (timeSinceEmoteStart <= 4500) {
    
  }
  else {
    curEmoteMode = NO_EMOTE;
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
  packet.recipient = AUDIOBOARD;

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
  curTime = millis();

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
    // 100Hz update rate for dome movement
    if (curTime - lastCh4ValueSent > 10 ) {
      //Channel 4 (Left stick)  --> Turn Left/Right (dome motor)
      if (abs(ch4Value) > 15) { // If left stick not being pressed
        Serial.write(ch4Value);
      }
      else {
        Serial.write(0);
      }
    lastCh4ValueSent = curTime;
    }

    // DRIVE MOVEMENT
    if (ch2Value > -1 && ch2Value < 256) {      /* if ch2 value in reasonable range     */

      if (abs(ch1Value) > 12) {                 /* if ch1 value is outside of deadzone  */
        LeftDriveValue = ch2Value + (float(ch1Value) * 0.1);
        RightDriveValue = ch2Value - (float(ch1Value) * 0.1);
      }
      else if (abs(ch2Value) > 10 ) {           /* if ch2 is outsize of deadzone        */
        LeftDriveValue = ch2Value;
        RightDriveValue = ch2Value;
      }
      else {                                    /* If both sticks in deadzone           */
        LeftDriveValue = 127;
        RightDriveValue = 127;
      }
      ledcWrite(0, LeftDriveValue);             /* Send pulse width to motors.          */
      ledcWrite(1, RightDriveValue);
    }
    else {                                      /* If ch2 value is bad, send no drive   */
      LeftDriveValue = 127;
      RightDriveValue = 127;
      ledcWrite(0, LeftDriveValue);             /* Send pulse width to motors.           */
      ledcWrite(1, RightDriveValue);
    }
  }
  else if ((LeftDriveValue != 127 || RightDriveValue != 127)) { /* Enforce no driving when not in drive mode. */
    LeftDriveValue = 127;
    RightDriveValue = 127;
    ledcWrite(0, LeftDriveValue);             /* Send pulse width to motors.             */
    ledcWrite(1, RightDriveValue);
  }

  /*--------------------------------------------
  Switch conditions for R2D2 to be in
  DOME ROTATION ONLY MODE
  --------------------------------------------*/
  if (ch7Value == 1 && ch8Value == 0 && ch9Value == 0 && ch10Value == 0) {

    //Channel 4 (Left stick)  --> Turn Left/Right (dome motor)
    if (abs(ch4Value) > 12) { // If left stick is being pressed
      Serial.write(ch4Value);
    }
    //Channel 1 (Right stick) --> Turn Left/Right (dome motor)
    else if (abs(ch1Value) > 12) {
      Serial.write(ch1Value);
    }
    else {
      Serial.write(0);
    }
  }

  /*--------------------------------------------
  Switch conditions for R2D2 to be in
  EMOTE MODE
  --------------------------------------------*/
  if (ch7Value == 0  && ch8Value == 1 && ch9Value == 0 && ch10Value == 0) {

  //Move joysticks all the way to one direction to trigger an emote.
  //Break emote functions out into functions above the main loop

    if ( curTime - lastEmoteSent > 5000 ) {
      //Right stick left
      if (ch1Value < -92) {
        curEmoteMode = EMOTE_1;
        packet.role = TRIGGER_EMOTE_XXXXX1;
        packet.recipient = AUDIOBOARD;
        lastEmoteSent = curTime;
        esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
      }
      //Right stick right
      else if (ch1Value > 92) {
        curEmoteMode = EMOTE_2;
        packet.role = TRIGGER_EMOTE_XXXXX2;
        packet.recipient = AUDIOBOARD;
        lastEmoteSent = curTime;
        esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
      }
      //Right stick down
      else if (ch2Value < 25) {
        curEmoteMode = EMOTE_3;
        packet.role = TRIGGER_EMOTE_XXXXX3;
        packet.recipient = AUDIOBOARD;
        lastEmoteSent = curTime;
        esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
      }
      //Right stick up
      else if (ch2Value > 228) {  // TODO: CHECK TO MAKE SURE THIS VALUE IS ACHIEVABLE
        curEmoteMode = EMOTE_4;
        packet.role = TRIGGER_EMOTE_XXXXX4;
        packet.recipient = AUDIOBOARD;
        lastEmoteSent = curTime;
        esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
      }
    }

    switch (curEmoteMode) {
      case NO_EMOTE:
        break;
      case EMOTE_1:
        executeEmote1();
        break;
      case EMOTE_2:
        executeEmote2();
        break;
      case EMOTE_3:
        executeEmote3();
        break;
      case EMOTE_4:
        executeEmote4();
        break;
      default:
        break;
    }
  }
  // If we change modes in the middle of an emote, immediately stop 
  else {
    curEmoteMode = NO_EMOTE;
  }

  /*--------------------------------------------
  Switch conditions for R2D2 to be in
  AI Cam Mode
  --------------------------------------------*/
  if (ch7Value == 0  && ch8Value == 0 && ch9Value >= 1 && ch10Value == 0) {
    // If transitioning from AICam off to AICam on or 
    // If it's been more than 1.5 seconds since last Control On packet sent
    if (AICamMode == false || (curTime - lastAICamControlPktSent >= 1500)) {
      packet.recipient = AUDIOBOARD;
      packet.role = AI_CAM_CONTROL_ON;
      lastAICamControlPktSent = curTime;
      esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
    }

    AICamMode = true;

    if (curTime - lastAICamPacketReceived >= 650) {
      Serial.write(0);
    }

  }
  else {
    // If transitioning from AICam on to AICam off or
    // If it's been more than 5 seconds since last control Off packet sent
    if (AICamMode == true || (curTime - lastAICamControlPktSent >= 5000)) {
      packet.recipient = AUDIOBOARD;
      packet.role = AI_CAM_CONTROL_OFF;
      lastAICamControlPktSent = curTime;
      esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
      Serial.write(0);
    }

    AICamMode = false;

  }

  /*--------------------------------------------
  Switch conditions for R2D2 to be in
  EMERGENCY DISCONNECT MODE
  --------------------------------------------*/
  if (ch7Value == 1 && ch8Value == 1 && ch9Value == 1 && ch10Value == 1) {

    // Stop Drive Motors
    LeftDriveValue = 127;
    RightDriveValue = 127;
    ledcWrite(0, LeftDriveValue);             /* Send pulse width to motors.             */
    ledcWrite(1, RightDriveValue);
    // Detach DriveValue pins
    ledcDetachPin(PIN_26);
    ledcDetachPin(PIN_27);

    // Stop Dome Motor
    Serial.write(0);

    // Disconnect serial communication to Dome Motor
    Serial.end();

    // Enter an inescapable superloop
    while (1) {
    // DO NOT PUT ANY CODE HERE. THIS LOOP IS MEANT TO STOP ALL EXECUTION
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
    packet.recipient = AUDIOBOARD;
    packet.vol = lastVolumeSent;
    packet.role = VOLUME_HAS_CHANGED;
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  //If projectorBulb == true, close MOSFET switch to turn on bulb.
  if (projectorBulb) {
    digitalWrite(PIN_32, HIGH);
  }
  else {
    digitalWrite(PIN_32, LOW);
  }
  // Send projector status to soundboard every 5 seconds
  // Controls LED status indicator on controller soundboard
  if (curTime - lastProjectorStatus > 5000) {
    if (projectorBulb) {
      packet.role = SET_SOUNDBOARD_LED_HIGH;
    }
    else {
      packet.role = SET_SOUNDBOARD_LED_LOW;
    }
    packet.recipient = CONTROLLER_SOUNDBOARD;
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
    lastProjectorStatus = curTime;
  }

  /*--------------------------------------------
  Control Pamphlet dispenser movement
  Sends signal to secondary motor controller,
  which handles driving PD motor
  --------------------------------------------*/

  if (pamphletDispenser && turnPDOnFlag && (curTime - lastPamphletDispenserStatus > 1000)) {
    digitalWrite(PIN_33, HIGH);
    lastPamphletDispenserStatus = curTime;
    turnPDOnFlag = false;
  }
  else if (pamphletDispenser && !turnPDOnFlag && (curTime - lastPamphletDispenserStatus > 1000)) {
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
