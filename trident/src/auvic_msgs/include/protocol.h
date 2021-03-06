#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#define PROTOCOL_MAX_MESSAGE_SIZE  (8U)

/********************************
 * MESSAGE ID DEFINITION
 * ******************************/
// `protocol_MID_<sending_node>_<message_name>`
enum protocol_MID_E {
    protocol_RESERVED = 0U,

    protocol_MID_TRIDENT_deviceName,
    protocol_MID_TRIDENT_motorSetSpeed,
    protocol_MID_TRIDENT_powerEnable,
    protocol_MID_TRIDENT_PBMessageRequest,  // Power Board Message Request
    protocol_MID_TRIDENT_MCMessageRequest, // Motor Controller Message Request
    protocol_MID_TRIDENT_IMUMessageRequest, // IMU Message Request
    protocol_MID_TRIDENT_GrabMessageRequest, // Grabber Message Request
    protocol_MID_TRIDENT_DropMessageRequest, // Dropper Message Request
    protocol_MID_TRIDENT_DVLMessageRequest, // DVL Message Request
    protocol_MID_TRIDENT_TorpedoMessageRequest, // Torpedo Message Request
    protocol_MID_TRIDENT_HPMessageRequest, // Hydrophone Message Request
    protocol_MID_TRIDENT_LBMessageRequest, // LCD_board Message Request
    protocol_MID_TRIDENT_DSMessageRequest, // Depth sensor Message Request

    protocol_MID_TRIDENT_MCISOTP, // ISOTP message sent from TRIDENT to Motor Controller

    protocol_MID_MC_deviceName = 21U,
    protocol_MID_MC_motorRPMLow,
    protocol_MID_MC_motorRPMHigh,
    protocol_MID_MC_ISOTP, // ISOTP message sent from the Motor Controller

    protocol_MID_PB_deviceName = 41U,
    protocol_MID_PB_envData, // Environmental Data
    protocol_MID_PB_battVoltages,
    protocol_MID_PB_battCurrents,

}; // Cannot be higher than 11 bits


/*******************************************
 * MESSAGE DEFINITIONS, ENUMS
 * *****************************************/
enum protocol_node_E {
    PROTOCOL_NODE_TRIDENT,
    PROTOCOL_NODE_POWER_BOARD,
    PROTOCOL_NODE_MOTOR_CONTROLLER,

    PROTOCOL_NODE_COUNT,
};

// protocol_MID_MC_deviceName, protocol_MID_PB_deviceName, protocol_MID_TRIDENT_deviceName
struct protocol_deviceName_S {
    uint8_t name[8U];
};

// protocol_MID_TRIDENT_motorSetSpeed
struct protocol_motorSetSpeed_S {
    int8_t motorSpeed[8U]; // In percent with a base of 127
};

// protocol_MID_MC_motorRPMLow, protocol_MID_MC_motorRPMHigh
struct protocol_motorRPM_S {
    int16_t motorSpeed[4U]; // Motor 0 to 3 got motorRPMLow, Motor 4 to 7 for motorRPMHigh
};

// protocol_MID_TRIDENT_powerEnable
struct protocol_powerEnable_S {
    bool motorPowerEnable;
    bool _5VPowerEnable;
    bool _12V9VPowerEnable;
};

// protocol_MID_TRIDENT_PBMessageRequest
enum protocol_PBMessageRequest_message_E {
    PROTOCOL_PB_MESSAGE_REQUEST_MESSAGE_RID,
    PROTOCOL_PB_MESSAGE_REQUEST_MESSAGE_ENV_DATA,
    PROTOCOL_PB_MESSAGE_REQUEST_MESSAGE_BATT_VOLTAGES,
    PROTOCOL_PB_MESSAGE_REQUEST_MESSAGE_BATT_CURRENTS,

    PROTOCOL_PB_MESSAGE_REQUEST_MESSAGE_COUNT,
};

struct protocol_PBMessageRequest_S {
    protocol_PBMessageRequest_message_E requestedMessage;
};

// protocol_MID_TRIDENT_MCMessageRequest
enum protocol_MCMessageRequest_message_E {
    PROTOCOL_MC_MESSAGE_REQUEST_MESSAGE_RID,
    PROTOCOL_MC_MESSAGE_REQUEST_MESSAGE_RPM_LOW,
    PROTOCOL_MC_MESSAGE_REQUEST_MESSAGE_RPM_HIGH,

    PROTOCOL_MC_MESSAGE_REQUEST_MESSAGE_COUNT,
};

struct protocol_MCMessageRequest_S {
    protocol_MCMessageRequest_message_E requestedMessage;
};

// protocol_MID_PB_envData
struct protocol_PBEnvData_S {
    uint16_t extPressure;
    uint16_t intPressure;
    uint16_t intTemperature;
    uint16_t intHumidity;
};

// protocol_MID_PB_battVoltages
struct protocol_PBBattVoltages_S {
    uint16_t leftBattVoltage; // mV
    uint16_t rightBattVoltage; // mV
};

// protocol_MID_PB_battCurrents
struct protocol_PBBattCurrents_S{
    uint32_t leftBattCurrent; // mA
    uint32_t rightBattCurrent; // mA
};

// protocol_MID_TRIDENT_MCISOTP, protocol_MID_MC_ISOTP
struct protocol_ISOTP_S {
    uint8_t data[8U];
};

/********************************
 * LINK LAYER STUFF
 * ******************************/
struct protocol_allMessages_U {
    protocol_deviceName_S    TRIDENT_deviceName;  // Sent by: TRIDENT, Received by: No One
    protocol_motorSetSpeed_S TRIDENT_motorSetSpeed; // Sent by: TRIDENT, Received by: Motor Controller
    protocol_powerEnable_S   TRIDENT_powerEnable; // Sent by: TRIDENT, Received by: Power Board
    protocol_PBMessageRequest_S TRIDENT_PBMessageRequest; // Sent by: TRIDENT, Received by: Power Board 
    protocol_MCMessageRequest_S TRIDENT_MCMessageRequest; // Sent by TRIDENT, Received by: Motor Controller
    protocol_ISOTP_S            TRIDENT_MCISOTP; // Sent by TRIDENT, Received by: Motor Controller

    protocol_deviceName_S    MC_deviceName; // Sent by Motor Controller, Received by TRIDENT
    protocol_motorRPM_S      MC_motorRPMLow; // Sent by Motor Controller, Received by TRIDENT
    protocol_motorRPM_S      MC_motorRPMHigh; // Sent by Motor Controller, Received by TRIDENT
    protocol_ISOTP_S         MC_ISOTP; // Sent by Motor Controller, Received by TRIDENT

    protocol_deviceName_S    PB_deviceName; // Sent by Power Board, Received by TRIDENT
    protocol_PBEnvData_S     PB_envData; // Sent by Power Board, Received by TRIDENT
    protocol_PBBattVoltages_S PB_battVoltages; // Sent by Power Board, Received by TRIDENT
    protocol_PBBattCurrents_S PB_battCurrents; // Sent by Power Board, Received by TRIDENT
};

struct protocol_message_S {
    protocol_MID_E messageID;
    protocol_allMessages_U  message;
};

#endif // PROTOCOL_HPP