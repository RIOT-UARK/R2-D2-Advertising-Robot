// This code recieves PWM channels from the radio receiver and sends PWM signals to the motor controllers of 
// the bot. This microcontroller is necessary due to having different 'modes' of operation. I.e.: Moving forward requires 
// sending a positive value to both the left and right drive motors. Turning the bot would then require 'skid steering':
// sending a positive value to one motor, and a negative one to another. 

//TODO: Implement the following modes controlled by Switches 7-10: Drive, Dome rotation, emote, etc.

//ESP32 to Motor Controllers pinout

// PIN 26 --> SIGNAL INPUT FOR L Side of Drive Motor Controller (White Wire)
// PIN 27 --> SIGNAL INPUT FOR R Side of Drive Motor Controller (2.54 dupont labelled 'S')
// RX --> TX ON ARDUINO NANO (HOPEFULLY I WILL CHANGE THIS FROM SERIAL COMM TO smthing else)


#include <esp_now.h>
#include <WiFi.h>

// Reciever to ESP32 pinout
#define CH1 13
#define CH2 4
#define CH3 16
#define CH4 17
#define CH5 5
#define CH6 18
#define CH7 19
#define CH8 21
#define CH9 22
#define CH10 23





//Global Variables for interrupt functions for timing PWM signals.
//Sadly, you cant pass arguments in to interrupt functions, so I have to have
//different global variables and interrupt functions for each pin.
volatile unsigned long CH1PulseBegin = 0;
volatile unsigned long CH1PulseEnd = 0;
volatile bool CH1NewPulseDurAvailable = false;
volatile unsigned long CH2PulseBegin = 0;
volatile unsigned long CH2PulseEnd = 0;
volatile bool CH2NewPulseDurAvailable = false;
volatile unsigned long CH3PulseBegin = 0;
volatile unsigned long CH3PulseEnd = 0;
volatile bool CH3NewPulseDurAvailable = false;
volatile unsigned long CH4PulseBegin = 0;
volatile unsigned long CH4PulseEnd = 0;
volatile bool CH4NewPulseDurAvailable = false;
volatile unsigned long CH5PulseBegin = 0;
volatile unsigned long CH5PulseEnd = 0;
volatile bool CH5NewPulseDurAvailable = false;
volatile unsigned long CH6PulseBegin = 0;
volatile unsigned long CH6PulseEnd = 0;
volatile bool CH6NewPulseDurAvailable = false;
volatile unsigned long CH7PulseBegin = 0;
volatile unsigned long CH7PulseEnd = 0;
volatile bool CH7NewPulseDurAvailable = false;
volatile unsigned long CH8PulseBegin = 0;
volatile unsigned long CH8PulseEnd = 0;
volatile bool CH8NewPulseDurAvailable = false;
volatile unsigned long CH9PulseBegin = 0;
volatile unsigned long CH9PulseEnd = 0;
volatile bool CH9NewPulseDurAvailable = false;
volatile unsigned long CH10PulseBegin = 0;
volatile unsigned long CH10PulseEnd = 0;
volatile bool CH10NewPulseDurAvailable = false;

// Ints to represent values from sticks and pots
int ch1Value, ch2Value, ch3Value, ch4Value, ch5Value, ch6Value, ch9Value;

//For tracking volume sent to audio board
int lastVolumeSent = 0;

// Bool to represent switch value
bool ch7Value;
bool ch8Value;

bool ch10Value;
// Ints to be sent to Motor Controllers
int LeftDriveValue, RightDriveValue, DomeMotorValue, Motor4Value;


uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Create a struct that holds the ESP NOW packets
typedef struct struct_message {
    String recip;
    int a;
    int vol;
} struct_message;

//packet to send
struct_message packet;
//packet that is received
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

//Callback function when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&packet, incomingData, sizeof(packet));

    //This Receiver microcontroller will be receiving data from the computer vision Camera.
    //Todo: add code to receive input packets from the camera and send commands to the Dome motor. 
}
//Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


//Interrupt functions to measure PWM signals
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
 /*
// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int pulseWidth = pulseIn(channelInput, HIGH, 30000);                                    //int pulseWidth = pulseiIn(pin, HIGH/LOW, timeout)
  if (pulseWidth < 100) return defaultValue;
  //return map(pulseWidth, 1000, 2000, minLimit, maxLimit);
  return pulseWidth;
}
 
// Read the switch channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int pulseWidth = readChannel(channelInput, 0, 100, intDefaultValue);
  return (pulseWidth > 50);
}*/
 
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
  //Figure out the correct PWM frequencies and set up the output pins with
  // ledcSetup(ledChannel, freq, resolution); and
  // ledcAttachPin(ledPin, ledChannel);

  //As of now, the only recipient of messages from this board will be the Audio board.
  packet.recip = "audi";

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
  pinMode(26, OUTPUT);
  ledcAttachPin(26, 0);
  ledcSetup(0, 1000, 8);


//PWM OUTPUT TO CONTROL RIGHT DRIVE MOTOR
  pinMode(27, OUTPUT);
  ledcAttachPin(27, 1);
  ledcSetup(1, 1000, 8);
  
}


void loop() {

  long int time = millis();
  
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
  
 
  if (ch7Value == 0 && ch8Value == 0 && ch9Value == 0 && ch10Value == 0) {       //DRIVE MODE
    //Channel 2 --> Forwards/Backwards (LMotor + RMotor) 
    //Channel 1 --> Turn Left/Right (Either -LMotor/+RMotor or just +1Motor forward )
    
    //DOME MOVEMENT
    if (abs(ch4Value) > 15) { // If left stick not being pressed
      //ledcWrite(ledChannel, dutyCycle);
      Serial.write(ch4Value);
      //Serial.println(ch4Value);
    }
    else {
    }

    if (ch2Value > -1 && ch2Value < 256) {
      int LMotor, RMotor;
      LMotor = ch2Value + (float(ch1Value) * 0.1); //NOTE: Maybe implement dead zones for ch1.
      RMotor = ch2Value - (float(ch1Value) * 0.1);
      ledcWrite(0, LMotor);
      ledcWrite(1, RMotor);
     /* Serial.print("L: ");
      Serial.print(LMotor);
      Serial.print("  R: ");
      Serial.println(RMotor); */
    }
    else {
      ledcWrite(0, 127);
    }
  }
    if (ch7Value == 1 && ch8Value == 0 && ch9Value == 0 && ch10Value == 0) {       //DOME ROTATION ONLY MODE
    //Channel 1 --> Turn Dome Left/Right
  }
      if (ch7Value == 0  && ch8Value == 1 && ch9Value == 0 && ch10Value == 0) {       //EMOTE MODE
    //Move joysticks all the way to one direction to trigger an emote.
    //Break emote functions out into functions above the main loop

    //if emote 'a' is selected -> packet.a = 2;
    //then -> esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  //If volume knob has changed >5, send a new packet instructing the volume to change
  if (abs(lastVolumeSent - ch5Value) > 5) {
    lastVolumeSent = ch5Value;
    packet.recip = "audi";
    packet.vol = lastVolumeSent;
    packet.a = -1;
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));

  }

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
