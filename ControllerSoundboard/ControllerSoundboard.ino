/*----------------------------------------------------------------------------

    ControllerSoundboard.ino

    DESCRIPTION:
      This code controls the Soundboard, attached to the RC Remote used to
      control R2D2. This soundboard uses ESPNOW to send commands to R2's
      audioboard, in order to play audio.

    MICROCONTROLLER:
      ESP-32

-----------------------------------------------------------------------------*/

/*-------------------------------------------------------------------
        INCLUDES
--------------------------------------------------------------------*/

#include <esp_now.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include <R2D2_LIB.h>

/*-------------------------------------------------------------------
        GLOBAL VARIABLES
--------------------------------------------------------------------*/

// ESP-NOW broadcast address - Broadcast as WAN
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Struct for ESPNOW data packet
typedef struct struct_message {
  String recip;
  int a;
  int vol;
} struct_message;

//packet to send
struct_message packet;

esp_now_peer_info_t peerInfo;

unsigned long int curTime = 1;         /* current time, ms since boot */
unsigned long int debounceTime = 0;    /* button debounce time        */
unsigned long int lastSend = 0;        /* time of last packet sent    */
bool isModemSleep = false;             /* flag for modem timeout      */

/*-------------------------------------------------------------------
        Function declarations
--------------------------------------------------------------------*/

void setModemSleep();
void wakeModemSleep();

/*----------------------------------------------------------------------

    OnDataRecv

      Callback function when data is received

----------------------------------------------------------------------*/

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&packet, incomingData, sizeof(packet));

    if (packet.recip == "boar") {
      if (packet.a == 20) {
        //Set LED low
      }
      else if (packet.a == 21) {
        //Set LED High
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


/*----------------------------------------------------------------------

    Setup function

----------------------------------------------------------------------*/

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {                 /* Initialize ESP-NOW connection      */
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);          /* Register send callback function     */
  esp_now_register_recv_cb(OnDataRecv);          /* Register receive callback function  */

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  /*  Button Pins, in physical order from left to right   */
  pinMode(PIN_21, INPUT);
  pinMode(PIN_5, INPUT);
  pinMode(PIN_19, INPUT);
  pinMode(PIN_18, INPUT);
  pinMode(PIN_17, INPUT);
  pinMode(PIN_16, INPUT);
  pinMode(PIN_2, INPUT);
  pinMode(PIN_4, INPUT);

  /*
  CURRENTLY UNUSED
  pinMode(PIN_22, OUTPUT); //LED

  digitalWrite(PIN_22, HIGH);
  */

  //The soundboard will only be sending data to the Audio Player Board
  packet.recip = "audi";
}

/*----------------------------------------------------------------------

    Microcontroller Superloop

----------------------------------------------------------------------*/

void loop() {
  curTime = millis();

  /*-----------------------------------------
  If a button on the soundboad is pressed,
  it will be connected to 3.3v (HIGH). If
  pressed and it has been >750ms since last
  audio was sent to soundboard, the
  controller shall send a packet to signal
  the audioboard to play a sound.
  -----------------------------------------*/

  if ((digitalRead(PIN_21) == HIGH) && ((curTime - debounceTime) > 750)) {
    packet.a = 5;
    debounceTime = curTime;
    lastSend = curTime;
    wakeModemSleep();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  else if ((digitalRead(PIN_5) == HIGH) && ((curTime - debounceTime) > 750)) {
    packet.a = 7;
    debounceTime = curTime;
    lastSend = curTime;
    wakeModemSleep();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  else if ((digitalRead(PIN_19) == HIGH) && ((curTime - debounceTime) > 750)) {
    packet.a = 4;
    debounceTime = curTime;
    lastSend = curTime;
    wakeModemSleep();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  else if ((digitalRead(PIN_18) == HIGH) && ((curTime - debounceTime) > 750)) {
    packet.a = 1;
    debounceTime = curTime;
    lastSend = curTime;
    wakeModemSleep();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  else if ((digitalRead(PIN_17) == HIGH) && ((curTime - debounceTime) > 750)) {
    packet.a = 0;
    debounceTime = curTime;
    lastSend = curTime;
    wakeModemSleep();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet)); 
  }

  else if ((digitalRead(PIN_16) == HIGH) && ((curTime - debounceTime) > 750)) {
    packet.a = 3;
    debounceTime = curTime;
    lastSend = curTime;
    wakeModemSleep();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  else if ((digitalRead(PIN_2) == HIGH) && ((curTime - debounceTime) > 750)) {
    packet.a = 2;
    debounceTime = curTime;
    lastSend = curTime;
    wakeModemSleep();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  else if ((digitalRead(PIN_4) == HIGH) && ((curTime - debounceTime) > 750)) {
    packet.a = 6;
    debounceTime = curTime;
    lastSend = curTime;
    wakeModemSleep();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }
  //Go into sleep mode if 15s of inactivity. Reduces power usage from constantly active WiFi
  else if (curTime - lastSend > 15000) {
    setModemSleep();
  }

  /*
  Serial.print(digitalRead(17));
  Serial.print(digitalRead(18));
  Serial.print(digitalRead(2));  
  Serial.print(digitalRead(16)); 
  Serial.print(digitalRead(19));
  Serial.print(digitalRead(21));
  Serial.print(digitalRead(4));
  Serial.println(digitalRead(5));
  */
  
}


/*----------------------------------------------------------------------

    setModemSleep

      Reduces power usage from constantly active WiFi and high clock
      rate

----------------------------------------------------------------------*/

void setModemSleep() {
    WiFi.setSleep(true);
    isModemSleep = true;
    if (!setCpuFrequencyMhz(40)){
        
    }
    
}
 

/*----------------------------------------------------------------------

    wakeModemSleep

      Wakes modem from sleep and increases CPU frequency

----------------------------------------------------------------------*/

void wakeModemSleep() {
  if (isModemSleep) {
    setCpuFrequencyMhz(240);
    WiFi.setSleep(false);
    isModemSleep = false;
  }
}
