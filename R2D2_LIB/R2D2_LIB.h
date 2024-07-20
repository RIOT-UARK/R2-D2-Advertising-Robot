
enum HW_PINS {
    PIN_0,
    PIN_1,
    PIN_2,
    PIN_3,
    PIN_4,
    PIN_5,
    PIN_6,
    PIN_7,
    PIN_8,
    PIN_9,
    PIN_10,
    PIN_11,
    PIN_12,
    PIN_13,
    PIN_14,
    PIN_15,
    PIN_16,
    PIN_17,
    PIN_18,
    PIN_19,
    PIN_20,
    PIN_21,
    PIN_22,
    PIN_23,
    PIN_24,
    PIN_25,
    PIN_26,
    PIN_27,
    PIN_28,
    PIN_29,
    PIN_30,
    PIN_31,
    PIN_32,
    PIN_33,
    PIN_34,
    PIN_35,
    PIN_36,
    PIN_37,
    PIN_38,
    PIN_39,
};

enum PWM_DUTY_CYCLE_8_BIT {
    ZERO_DUTY_CYCLE,
    MID_DUTY_CYCLE = 127,
    MAX_DUTY_CYCLE = 255
};

/*---------------------------------------------------
    ESP-32 PWM Channels
---------------------------------------------------*/
enum LEDC_CHANNELS {
    CHANNEL_0,
    CHANNEL_1
};

/*---------------------------------------------------
    Struct that holds the ESP NOW packets. This
    packet is used to communicate wirelessly via
    ESP NOW between ControllerSoundboard, 
    HeadAudioBoard, and BodyESP32_Receiver.
---------------------------------------------------*/
typedef struct struct_message {
    int recipient;          /* Intended recipient               */
    int role;               /* Intended role of packet          */
    int vol;                /* For adjusting AudioBoard volume  */
} struct_message;

/*---------------------------------------------------
    The struct_message packet.role field contains
    the intended purpose of each packet.
---------------------------------------------------*/
enum PACKET_ROLE_VALUES {
    VOLUME_HAS_CHANGED = -1,
    PLAY_EXCITE_AUDIO,
    PLAY_WORRIED_AUDIO,
    PLAY_SCREAM_AUDIO,
    PLAY_ACKNOWLEDGE_AUDIO,
    PLAY_CHAT_AUDIO,

    TOGGLE_PROJECTOR_BULB = 6,
    TRIGGER_PAMPHLET_DISPENSER_EVENT,

    TRIGGER_EMOTE_XXXXX1 = 10,
    TRIGGER_EMOTE_XXXXX2,
    TRIGGER_EMOTE_XXXXX3,
    TRIGGER_EMOTE_XXXXX4,

    SET_SOUNDBOARD_LED_LOW = 20,
    SET_SOUNDBOARD_LED_HIGH = 21,

    AI_CAM_CONTROL_ON = 30,
    AI_CAM_CONTROL_OFF,
    AI_CAM_TURN_DOME_LEFT,
    AI_CAM_TURN_DOME_RIGHT,
    AI_CAM_NO_DOME_TURN


};

/*---------------------------------------------------
    The struct_message packet.recipient field contains
    the intended recipient of each packet
---------------------------------------------------*/
enum PACKET_RECIPIENTS {
    AUDIOBOARD,
    CONTROLLER_SOUNDBOARD,
    BODY_ESP32_RECEIVER
};

/*---------------------------------------------------
    Enums for the custom Serial packet used to 
    communicate between BodyESP32_Receiver and
    Secondary_Motor_Driver
---------------------------------------------------*/
enum SERIAL_PKT_ENUMS {
    SERIAL_PKT_TERMINATOR   = 3,
    SERIAL_PKT_SIZE         = 3,
    SERIAL_PKT_EMERGENCY_DISCONNECT = -8714     /* Random number that should be utterly unachievable */
};

/*---------------------------------------------------
    The Current Emote State in BodyESP32_Receiver
---------------------------------------------------*/
enum CURRENT_EMOTE_MODES {
    NO_EMOTE,
    EMOTE_1,
    EMOTE_2,
    EMOTE_3,
    EMOTE_4
};

/*---------------------------------------------------
    Pamphlet Dispenser Stages
---------------------------------------------------*/
enum PAMPHLET_DISP_STAGES {
    DOOR_CLOSED_PD_READY,
    LOAD_PAMPHLET_OPEN_DOOR,
    FEED_PAPER_OUT,
    PAPER_IS_OUT_WAIT,
    WAIT_FOR_USER_INPUT,
    CLOSE_DOOR_END_CYCLE

};