#ifndef WEASEL_DEBUG_INTERFACE_DEFINES_H
#define WEASEL_DEBUG_INTERFACE_DEFINES_H

    // packet length
    #define NUMBER_OF_BYTES_PER_DATAFRAME   7

    // max used readbuffer
    #define BUFFER_LENGHT   4160 //NUMBER_OF_BYTES_PER_DATAFRAME*100 // FT422H endpoint buffer = 4160 Bytes

    // synchronisation byte
    #define SYNCWORD        0x50

    // paket IDs
    #define PACKET_ID_READ_UNIT_TYPE     0x00
    #define PACKET_ID_READ_VERSION       0x10
    #define PACKET_ID_READ_CONFIG        0x20
    #define PACKET_ID_WRITE_CONFIG       0x30
    #define PACKET_ID_READ_REGISTER      0x40
    #define PACKET_ID_WRITE_REGISTER     0x50
    #define PACKET_ID_LIVE_DATA          0x60
    #define PACKET_ID_START_MEASURE      0x70
    #define PACKET_ID_STOP_MEASURE       0x80
    #define PACKET_ID_READ_AP            0x90
    #define PACKET_ID_WRITE_AP           0xA0
    #define PACKET_ID_READ_IO            0xB0
    #define PACKET_ID_WRITE_IO           0xC0
    #define PACKET_ID_DUMMY              0xF0

    // packet and channel id mask
    #define PACKET_ID_MASK                  0xF0
    #define CHANNEL_ID_MASK                 0x0F

    // number of available live-channels
    #define IC_DEBUGGER_CHANNEL_COUNT       4

    #define CONFIG_PARAM_CH_ENABLE          0x00
    #define CONFIG_PARAM_CH_MOTOR_NR        0x01
    #define CONFIG_PARAM_CH_ADDR            0x02
    #define CONFIG_PARAM_CH_SUBADDR         0x03
    #define CONFIG_PARAM_MODE               0x04
    #define CONFIG_PARAM_TRIGGERCHANNEL     0x05
    #define CONFIG_PARAM_TRIGGERMODE        0x06
    #define CONFIG_PARAM_DOWNSAMPLING       0x07
    #define CONFIG_PARAM_NUMBER_OF_SAMPLES  0x08
    #define CONFIG_PARAM_TRIGGERVALUE       0x09
    #define CONFIG_PARAM_CH_SUBADDR_EN      0x0A
    #define CONFIG_PARAM_CH_SUBADDR_VALUE   0x0B

    // debugger events
    #define DEBUGGER_EVENT_START        0x00
    #define DEBUGGER_EVENT_STOP         0x01
    #define DEBUGGER_EVENT_RESET        0x02
    #define DEBUGGER_EVENT_INITIALIZE   0x03

    // eventIDs
    //#define EVENT_ID_STEP_RESPONSE  0x01
    //#define EVENT_ID_BODE_PLOT      0x01

    // operating modes
    #define IC_DEBUGGER_CONTINUOUS_MODE             0x00
    #define IC_DEBUGGER_SINGLE_SHOT_MODE            0x01

    // trigger
    #define TRIGGER_CONDITION_START_IMMEDIATELY     0x00
    #define TRIGGER_CONDITION_RISING_EDGE           0x01
    #define TRIGGER_CONDITION_FALLING_EDGE          0x02
    #define TRIGGER_CONDITION_BOTH_EDGES            0x03

    // GPIOS to the FPGA
    #define GPIO_EXT_TRIGGER        GPIO_PORT0
    #define GPIO_DATA_AVAILABLE     GPIO_PORT1

//structure for one set of config data

//typedef struct
//{
//    uint8_t Address;
//    uint8_t MotorNr;
//    bool Enable;
//    bool SubAddrEnable;
//    uint8_t SubAddress;
//    uint8_t SubAddressValue;
//}RtmiChannelConfig;

typedef struct
{
    uint16_t    unit_type;
    uint16_t    version_nr;

    uint8_t     channelReg_Address[IC_DEBUGGER_CHANNEL_COUNT];
    uint8_t     channelReg_MotorNr[IC_DEBUGGER_CHANNEL_COUNT];
    bool        channelReg_Enable[IC_DEBUGGER_CHANNEL_COUNT];
    bool        channelReg_SubAddrEnable[IC_DEBUGGER_CHANNEL_COUNT];
    uint8_t     channelReg_SubAddress[IC_DEBUGGER_CHANNEL_COUNT];
    uint8_t     channelReg_SubAddressValue[IC_DEBUGGER_CHANNEL_COUNT];
    uint8_t     channelReg_Shift[IC_DEBUGGER_CHANNEL_COUNT];
    uint32_t    channelReg_Mask[IC_DEBUGGER_CHANNEL_COUNT];

    uint8_t     gcr_liveMode;
    uint8_t     gcr_triggerMode;
    uint8_t     gcr_triggerChannel;
    uint16_t    gcr_downsampling;
    uint8_t     gcr_triggerShift;
    uint32_t    gcr_triggerMask;
    uint8_t     gcr_trigger_Address;
    bool        gcr_trigger_SubAddrEnable;
    uint8_t     gcr_trigger_SubAddress;
    uint8_t     gcr_trigger_SubAddressValue;
    uint8_t     channelCount;
    int32_t     triggerValue;
    uint32_t    valuesPerShot;
    bool        waitingRamDebug;
}RtmiDebuggerDataset;

Q_DECLARE_METATYPE(RtmiDebuggerDataset)

#endif // WEASEL_DEBUG_INTERFACE_DEFINES_H
