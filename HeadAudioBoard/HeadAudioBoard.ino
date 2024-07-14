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

// Struct that holds the ESP NOW packets
typedef struct struct_message {
    String recip;
    int a;
    int vol;
} struct_message;

struct_message packet;

//counter variables for iterating through sound categories
unsigned int excite = 0;
unsigned int worr = 0;
unsigned int scre = 0;
unsigned int ackn = 0;
unsigned int chat = 0;
float volTmp;

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
  memcpy(&packet, incomingData, sizeof(packet));

  if (packet.recip == "audi") {      /* if AudioBoard is the packet recipient */
    switch (packet.a) {
      // Volume change packet.
      case -1:
        volTmp = float(packet.vol) / 100;
        player.setVolume(volTmp);
        break;
      // Soundboard packet
      case 0:
        if (excite % 2 == 1) {
          player.setPath("/exci0.mp3");
          //Signal R2D2 psi light to 'talk'
          digitalWrite(PIN_23, HIGH);
          delay(1250);
          digitalWrite(PIN_23, LOW);
        }
        else {
          player.setPath("/exci1.mp3");
          //Signal R2D2 psi light to 'talk'
          digitalWrite(PIN_23, HIGH);
          delay(1940);
          digitalWrite(PIN_23, LOW);
          }
          excite++;
        break;
      // Soundboard packet
      case 1:
        if (worr % 2 == 1) {
          player.setPath("/worr0.mp3");
          //Signal R2D2 psi light to 'talk'
          digitalWrite(PIN_23, HIGH);
          delay(1420);
          digitalWrite(PIN_23, LOW);
        }
        else {
          player.setPath("/worr1.mp3");
          //Signal R2D2 psi light to 'talk'
          digitalWrite(PIN_23, HIGH);
          delay(1380);
          digitalWrite(PIN_23, LOW);
        }
          worr++;
        break;
      // Soundboard packet
      case 2:
        if (scre % 2 == 1) {
          player.setPath("/scre0.mp3");
          //Signal R2D2 psi light to 'talk'
          digitalWrite(PIN_23, HIGH);
          delay(1050);
          digitalWrite(PIN_23, LOW);
        }
        else {
          player.setPath("/scre1.mp3");
          //Signal R2D2 psi light to 'talk'
          digitalWrite(PIN_23, HIGH);
          delay(1290);
          digitalWrite(PIN_23, LOW);
        }
          scre++;
        break;
      // Soundboard packet
      case 3:
        if (ackn % 2 == 1) {
          player.setPath("/ackn0.mp3");
          //Signal R2D2 psi light to 'talk'
          digitalWrite(PIN_23, HIGH);
          delay(4400);
          digitalWrite(PIN_23, LOW);
        }
        else {
          player.setPath("/ackn1.mp3");
          //Signal R2D2 psi light to 'talk'
          digitalWrite(PIN_23, HIGH);
          delay(2120);
          digitalWrite(PIN_23, LOW);
        }
          ackn++;
        break;
      // Soundboard packet
      case 4:
        if (chat % 2 == 1) {
          player.setPath("/chat0.mp3");
          //Signal R2D2 psi light to 'talk'
          digitalWrite(PIN_23, HIGH);
          delay(1000);
          digitalWrite(PIN_23, LOW);
        }
        else {
          player.setPath("/chat1.mp3");
          //Signal R2D2 psi light to 'talk'
          digitalWrite(PIN_23, HIGH);
          delay(1200);
          digitalWrite(PIN_23, LOW);
        }
          chat++;
        break;
      case 5:

        break;
      case 6:

        break;
      case 7:

        break;
    }
  
  }
}

/* TODO: Implement an OnDataRecv function to send dome motor signals to BodyESP32_Receiver */

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

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
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

  // Register callback function for when packet is received.
  esp_now_register_recv_cb(OnDataRecv);

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

}
