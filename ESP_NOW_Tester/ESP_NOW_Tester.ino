/*----------------------------------------------------------------------------

    ESP_NOW_Tester.ino

    DESCRIPTION:
      This code is not a microcontroller that is used in R2D2. This is a 
	microcontroller that is used to capture packets sent over ESP-NOW 
	and print them over UART to a connected computer.

    MICROCONTROLLER:
      ESP-32

-----------------------------------------------------------------------------*/

/*-------------------------------------------------------------------
        INCLUDES
--------------------------------------------------------------------*/

#include <esp_now.h>
#include <WiFi.h>
#include <R2D2_LIB.h>

/*-------------------------------------------------------------------
        GLOBAL VARIABLES
--------------------------------------------------------------------*/

// ESP-NOW broadcast address - Broadcast as WAN
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//ESP NOW wireless communication packet
struct_message packet;
struct_message incomingPacket;

esp_now_peer_info_t peerInfo;

/*----------------------------------------------------------------------

    OnDataRecv

      Callback function when ESP-NOW packet is received

----------------------------------------------------------------------*/

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingPacket, incomingData, sizeof(incomingPacket));

	String recip;
	String role;

    	switch (incomingPacket.recipient)
    	{
		case AUDIOBOARD:
			recip = "AudioBoard";
			break;
		case CONTROLLER_SOUNDBOARD:
			recip = "Controller Soundboard";
			break;
		case BODY_ESP32_RECEIVER:
			recip = "Body ESP32 Receiver";
			break;
		default:
			recip = "ERROR: UNKNOWN PACKET RECIPIENT";
			break;
    	}
	

	switch (incomingPacket.role)
	{
		case VOLUME_HAS_CHANGED:
			role = "Volume has changed";
			break;
		case PLAY_EXCITE_AUDIO:
			role = "Play Excited Audio";
			break;
		case PLAY_WORRIED_AUDIO:
			role = "Play Worried Audio";
			break;
		case PLAY_SCREAM_AUDIO:
			role = "Play Scream Audio";
			break;
		case PLAY_ACKNOWLEDGE_AUDIO:
			role = "Play Acknowledge Audio";
			break;
		case PLAY_CHAT_AUDIO:
			role = "Play Chat Audio";
			break;
		case PERISCOPE_ACTION:
			role = "UNUSED - Trigger Periscope Action";
			break;
		case TOGGLE_PROJECTOR_BULB:
			role = "Toggle Projector Bulb";
			break;
		case TRIGGER_PAMPHLET_DISPENSER_EVENT:
			role = "Trigger Pamphlet Dispenser Event";
			break;
		case TRIGGER_EMOTE_XXXXX1:
			role = "Trigger Emote 1";
			break;
		case TRIGGER_EMOTE_XXXXX2:
			role = "Trigger Emote 2";
			break;
		case TRIGGER_EMOTE_XXXXX3:
			role = "Trigger Emote 3";
			break;
		case TRIGGER_EMOTE_XXXXX4:
			role = "Trigger Emote 4";
			break;
		case SET_SOUNDBOARD_LED_LOW:
			role = "UNUSED - Soundboard LED Low";
			break;
		case SET_SOUNDBOARD_LED_HIGH:
			role = "UNUSED - Soundboard LED High";
			break;
		case AI_CAM_CONTROL_ON:
			role = "AI CAM - Control ON";
			break;
		case AI_CAM_CONTROL_OFF:
			role = "AI CAM - Control OFF";
			break;
		case AI_CAM_TURN_DOME_LEFT:
			role = "AI CAM - Turn Dome Left";
			break;
		case AI_CAM_TURN_DOME_RIGHT:
			role = "AI CAM - Turn Dome Right";
			break;
		case AI_CAM_NO_DOME_TURN:
			role = "AI CAM - No Dome Turn";
			break;
		default:
			role = "ERROR: UNKNOWN PACKET ROLE";
			break;
	}
	Serial.println("----------------------------------------");
	Serial.println("Packet Recipient: " + recip);
	Serial.println("Packet Role: " + role);
	Serial.println("Packet Volume val: " + incomingPacket.vol);
	Serial.println("----------------------------------------\n");

}

/*----------------------------------------------------------------------

    OnDataSent

      Callback function when ESP-NOW packet is sent

----------------------------------------------------------------------*/

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //We're not sending packets, but we need to register this function anyways
}

/*----------------------------------------------------------------------

    Setup function

----------------------------------------------------------------------*/

void setup() {
	Serial.begin(115200);

	WiFi.mode(WIFI_STA);

	if (esp_now_init() != ESP_OK) {                 /* Initialize ESP-NOW connection      */
    		// Serial.println("Error initializing ESP-NOW");
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
}

void loop() {
	// Do nothing while waiting for packets
}