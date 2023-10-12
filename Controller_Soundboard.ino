#include <esp_now.h>
#include <WiFi.h>

//This code controls the Soundboard, attached to the RC Remote used to control R2D2. This soundboard uses ESPNOW to send commands to R2's audioboard,
//in order to play audio.

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

unsigned int debounceTime = 0;

//Struct for ESPNOW data packet
typedef struct struct_message {
  String recip;
  int a;
} struct_message;

struct_message packet;

esp_now_peer_info_t peerInfo;

//ESPNOW func to be called every time data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {


  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  //Button Pins, in order from left to right
  pinMode(21, INPUT);
  pinMode(5, INPUT);
  pinMode(19, INPUT);
  pinMode(18, INPUT);
  pinMode(17, INPUT);
  pinMode(16, INPUT);
  pinMode(2, INPUT);
  pinMode(4, INPUT);

  pinMode(22, OUTPUT); //LED

  digitalWrite(22, HIGH);

  //The soundboard will only be sending data to the Audio Player Board
  packet.recip = "audi";
}



void loop() {

  if ((digitalRead(21) == HIGH) && ((millis() - debounceTime) > 2000)) {
    packet.a = 0;
    debounceTime = millis();

    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
   
  }

  if ((digitalRead(5) == HIGH) && ((millis() - debounceTime) > 2000)) {
    packet.a = 1;
    debounceTime = millis();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  if ((digitalRead(19) == HIGH) && ((millis() - debounceTime) > 2000)) {
    packet.a = 2;
    debounceTime = millis();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  if ((digitalRead(18) == HIGH) && ((millis() - debounceTime) > 2000)) {
    packet.a = 3;
    debounceTime = millis();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  if ((digitalRead(17) == HIGH) && ((millis() - debounceTime) > 2000)) {
    packet.a = 4;
    debounceTime = millis();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  if ((digitalRead(16) == HIGH) && ((millis() - debounceTime) > 2000)) {
    packet.a = 5;
    debounceTime = millis();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  if ((digitalRead(2) == HIGH) && ((millis() - debounceTime) > 2000)) {
    packet.a = 6;
    debounceTime = millis();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  if ((digitalRead(4) == HIGH) && ((millis() - debounceTime) > 2000)) {
    packet.a = 7;
    debounceTime = millis();
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
  }

  /*
  Serial.print(digitalRead(21));
  Serial.print(digitalRead(5));
  Serial.print(digitalRead(19));  
  Serial.print(digitalRead(18)); 
  Serial.print(digitalRead(17));
  Serial.print(digitalRead(16));
  Serial.print(digitalRead(2));
  Serial.println(digitalRead(4));
  */
  
}
