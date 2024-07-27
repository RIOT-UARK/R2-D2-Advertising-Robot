/*----------------------------------------------------------------------------

    HeadAudioBoard.ino

    DESCRIPTION:
      This code uses an AIThinker Audio Kit V2.2 Board to recieve signals 
      from either the sounboard 'button box' microcontroller or the reciever 
      microcontroller within R2D2's body, and then parses those commands to 
      play an audio clip stored on its SD card.

    MICROCONTROLLER:
      AIThinker Audio Kit V2.2 Board (ESP-32)

-----------------------------------------------------------------------------*/

// TODO: Receive L/R commands from AMB82 Camera when in facial tracking mode, and send them to reciever microcontroller

// TODO: implement an interrupt / break sequence to parse 'emote' commands from the reciever microcontroller, to give priority to its
// commands, rather than a sounboard input when in emote mode.

/*-------------------------------------------------------------------
        INCLUDES
--------------------------------------------------------------------*/

#include <SD.h>
#include <sd_defines.h>
#include <sd_diskio.h>

#include <esp_now.h>
#include <WiFi.h>
#include "AudioTools.h"
#include "AudioLibs/AudioKit.h"
#include "AudioLibs/AudioSourceSD.h" // or AudioSourceIdxSD.h
#include "AudioCodecs/CodecMP3Helix.h"
#include <R2D2_LIB.h>

/*-------------------------------------------------------------------
        DEFINES AND CONSTANTS
--------------------------------------------------------------------*/

// Constants for mp3 file system handling
const char *startFilePath="/";
const char* ext="mp3";

/*-------------------------------------------------------------------
        GLOBAL VARIABLES
--------------------------------------------------------------------*/

// ESP-NOW broadcast address - Broadcast as WAN
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//ESP NOW wireless communication packets
struct_message packet;
struct_message recPacket;
esp_now_peer_info_t peerInfo;

//counter variables for iterating through sound categories
unsigned int excite = 0;
unsigned int worr = 0;
unsigned int scre = 0;
unsigned int ackn = 0;
unsigned int chat = 0;
float volTmp;

// Whether AI Cam has control over dome movement
bool AICamControl = false;

// Currrent time, time since boot in ms
unsigned long curTime = 0;

// Time of last AICam dome movement packet sent
unsigned long lastAICamPacketTime = 0;

// Time of last received AICamControlPkt
unsigned long lastAICamControlPkt = 0;

// last sent AI Came directed dome movement
int lastSentAICamTurn;

// Time when the LED 'talk' signal will be turned off
unsigned long LEDTalkTimeout = 0;

// Whether or not we are sending LED Talk signal
bool LEDTalkSignal = false;

// Create AudioKit objects
AudioSourceSD source(startFilePath, ext, PIN_AUDIO_KIT_SD_CARD_CS);
AudioKitStream kit;
MP3DecoderHelix decoder;  // or change to MP3DecoderMAD
AudioPlayer player(source, kit, decoder);


/*----------------------------------------------------------------------

    OnDataRecv

      Callback function when data is received. If packet is received
      with AudioBoard marked as recipient, take an action based on 
      packet contents (Either adjust volume or play an audio).

----------------------------------------------------------------------*/

/* TODO: Implement a way for audio board to know when emote mode/ AI mode
 has control of audio. Idea: periodic subscription packets w/ a timeout */

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&recPacket, incomingData, sizeof(recPacket));

  if (recPacket.recipient == AUDIOBOARD) {      /* if HeadAudioBoard is the packet recipient */
    switch (recPacket.role) {
      // Volume change packet.
      case VOLUME_HAS_CHANGED:
        volTmp = float(recPacket.vol) / 100.0;
        player.setVolume(volTmp);
        break;
      // Soundboard packet
      case PLAY_EXCITE_AUDIO:
        if (!LEDTalkSignal) {                   /* Enforces non-overlapping audios             */
          if (excite % 2 == 1) {
            player.setPath("/exci0.mp3");
            //Signal R2D2 psi light to 'talk'
            digitalWrite(PIN_23, HIGH);
            LEDTalkTimeout = curTime + 1250;
            LEDTalkSignal = true;
          }
          else {
            player.setPath("/exci1.mp3");
            //Signal R2D2 psi light to 'talk'
            digitalWrite(PIN_23, HIGH);
            LEDTalkTimeout = curTime + 1940;
            LEDTalkSignal = true;
            }
            excite++;
        }
        break;
      // Soundboard packet
      case PLAY_WORRIED_AUDIO:
        if (!LEDTalkSignal) {                   /* Enforces non-overlapping audios             */
          if (worr % 2 == 1) {
            player.setPath("/worr0.mp3");
            //Signal R2D2 psi light to 'talk'
            digitalWrite(PIN_23, HIGH);
            LEDTalkTimeout = curTime + 1420;
            LEDTalkSignal = true;
          }
          else {
            player.setPath("/worr1.mp3");
            //Signal R2D2 psi light to 'talk'
            digitalWrite(PIN_23, HIGH);
            LEDTalkTimeout = curTime + 1380;
            LEDTalkSignal = true;
          }
            worr++;
        }
        break;
      // Soundboard packet
      case PLAY_SCREAM_AUDIO:
        if (!LEDTalkSignal) {                   /* Enforces non-overlapping audios             */
          if (scre % 2 == 1) {
            player.setPath("/scre0.mp3");
            //Signal R2D2 psi light to 'talk'
            digitalWrite(PIN_23, HIGH);
            LEDTalkTimeout = curTime + 1050;
            LEDTalkSignal = true;
          }
          else {
            player.setPath("/scre1.mp3");
            //Signal R2D2 psi light to 'talk'
            digitalWrite(PIN_23, HIGH);
            LEDTalkTimeout = curTime + 1290;
            LEDTalkSignal = true;
          }
            scre++;
        }
        break;
      // Soundboard packet
      case PLAY_ACKNOWLEDGE_AUDIO:
        if (!LEDTalkSignal) {                   /* Enforces non-overlapping audios             */
          if (ackn % 2 == 1) {
            player.setPath("/ackn0.mp3");
            //Signal R2D2 psi light to 'talk'
            digitalWrite(PIN_23, HIGH);
            LEDTalkTimeout = curTime + 4400;
            LEDTalkSignal = true;
          }
          else {
            player.setPath("/ackn1.mp3");
            //Signal R2D2 psi light to 'talk'
            digitalWrite(PIN_23, HIGH);
            LEDTalkTimeout = curTime + 2120;
            LEDTalkSignal = true;
          }
            ackn++;
        }
        break;
      // Soundboard packet
      case PLAY_CHAT_AUDIO:
        if (!LEDTalkSignal) {                   /* Enforces non-overlapping audios             */
          if (chat % 2 == 1) {
            player.setPath("/chat0.mp3");
            //Signal R2D2 psi light to 'talk'
            digitalWrite(PIN_23, HIGH);
            LEDTalkTimeout = curTime + 1000;
            LEDTalkSignal = true;
          }
          else {
            player.setPath("/chat1.mp3");
            //Signal R2D2 psi light to 'talk'
            digitalWrite(PIN_23, HIGH);
            LEDTalkTimeout = curTime + 1200;
            LEDTalkSignal = true;
          }
            chat++;
        }
        break;
      case AI_CAM_CONTROL_ON:
        AICamControl = true;
        //digitalWrite(PIN_XX, HIGH); TODO: NEED TO ASSIGN THIS PIN
        lastAICamControlPkt = curTime;
        break;
      case AI_CAM_CONTROL_OFF:
        AICamControl = false;
        //digitalWrite(PIN_XX, LOW); TODO: NEED TO ASSIGN THIS PIN
        lastAICamControlPkt = curTime;
        break;
      case TRIGGER_EMOTE_XXXXX1:
        // Play an audio
        // Light up LEDs for a given time
        break;
      case TRIGGER_EMOTE_XXXXX2:
        player.setPath("/scre1.mp3");
          //Signal R2D2 psi light to 'talk'
          digitalWrite(PIN_23, HIGH);
          LEDTalkTimeout = curTime + 1290;
          LEDTalkSignal = true;
        break;
      case TRIGGER_EMOTE_XXXXX3:
        // Play an audio
        // Light up LEDs for a given time
        break;
      case TRIGGER_EMOTE_XXXXX4:
        // Play an audio
        // Light up LEDs for a given time
        break;
      default:
        break;
    }
  
  }
}


/*----------------------------------------------------------------------

    OnDataSent

      Callback function when data is sent

----------------------------------------------------------------------*/

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


/* TODO: Implement an OnDataSent function to send dome motor signals to BodyESP32_Receiver */

/*----------------------------------------------------------------------

    AudioKit functions to control player. Not used right now

----------------------------------------------------------------------*/

void next(bool, int, void*) {
   player.next();
}

void previous(bool, int, void*) {
   player.previous();
}

void startStop(bool, int, void*) {
   player.setActive(!player.isActive());
}


/*----------------------------------------------------------------------

    Setup function

----------------------------------------------------------------------*/

void setup() {
  Serial.begin(115200);
  AudioLogger::instance().begin(Serial, AudioLogger::Info);

  // Signal to HeadLEDController for R2D2 to 'talk'.
  pinMode(PIN_23, OUTPUT);
  // Signal to HeadFacialDetection to permit control of dome.
/*  pinMode(XXXXXX, OUTPUT);                  */
  // Signal from HeadFacialDetection for left Dome movement
/*  pinMode(XXXXXX, INPUT);                   */
  // Signal from HeadFacialDetection for right Dome movement
/*  pinMode(XXXXXX, INPUT);                   */

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);       /* Register send callback function                         */
  esp_now_register_recv_cb(OnDataRecv);       /* Register callback function for when packet is received. */

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // setup output
  auto cfg = kit.defaultConfig(TX_MODE);
  // sd_active is setting up SPI with the right SD pins by calling 
  // SPI.begin(PIN_AUDIO_KIT_SD_CARD_CLK, PIN_AUDIO_KIT_SD_CARD_MISO, PIN_AUDIO_KIT_SD_CARD_MOSI, PIN_AUDIO_KIT_SD_CARD_CS);
  cfg.sd_active = true;
  kit.begin(cfg);

  // setup additional buttons 
  // kit.addAction(PIN_KEY1, startStop);
  //kit.addAction(PIN_KEY4, next);
  //kit.addAction(PIN_KEY3, previous);

  // setup player
  player.setVolume(1.0f);
  player.begin();  

  //Key 5 (pin 18) must be pressed to turn on the amplifier.
  pinMode(PIN_KEY5, OUTPUT);
  digitalWrite(PIN_KEY5, HIGH);
  delay(1);
  digitalWrite(PIN_KEY5, LOW);
  delay(600);
}

/*----------------------------------------------------------------------

    Microcontroller Superloop

----------------------------------------------------------------------*/

void loop() {
  player.copy();
  kit.processActions();
//Serial.println(digitalRead(23));

  curTime = millis();

  // If we are in AI Cam Control mode
  if (AICamControl) {

    // If receiving command to turn left from AICam
    if ((digitalRead(PIN_XX) == HIGH) && (digitalRead(PIN_XX) == LOW)) {    //****TODO***** ASSIGN THESE PIN_XX's
        packet.recipient = BODY_ESP32_RECEIVER;
        packet.role      = AI_CAM_TURN_DOME_LEFT;
    }
    // If receiving command to turn right from AICam
    else if ((digitalRead(PIN_XX) == LOW) && (digitalRead(PIN_XX) == HIGH)) {
        packet.recipient = BODY_ESP32_RECEIVER;
        packet.role      = AI_CAM_TURN_DOME_RIGHT;
    }
    else {
        packet.recipient = BODY_ESP32_RECEIVER;
        packet.role      = AI_CAM_NO_DOME_TURN;
    }

    // If packet has changed or it has been 500ms since last packet sent
    if ((curTime - lastAICamPacketTime > 500) || (packet.role != lastSentAICamTurn)) {
      // last packet role = current packet role
      lastSentAICamTurn = packet.role;
      // Send packet and set last packet sent time to now
      lastAICamPacketTime = curTime;
      esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
    }

    // If we have not received an AICamControlPkt from BodyESP32Receiver in the last 1.75 seconds
    if (curTime - lastAICamControlPkt >= 1750) {
      AICamControl = false;
      digitalWrite(PIN_XX, LOW); //TODO: NEED TO ASSIGN THIS PIN
    }

  }

  // If it is time to turn the 'talk' signal off
  if ( LEDTalkSignal && ( curTime - LEDTalkTimeout > 0 ) ) {
    digitalWrite(PIN_23, LOW);
    LEDTalkSignal = false;
  }
}
