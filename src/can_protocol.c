#include "can_protocol.h"
#include <string.h>

/* Standard CRC-16-CCITT implementation */
uint16_t CAN_CalculateCRC(const uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

bool CAN_PackAdasStatus(CanMessage_t *msg, uint16_t distance_cm, uint8_t state, uint8_t hazard_level) {
    if (!msg) return false;

    memset(msg, 0, sizeof(CanMessage_t));
    msg->id = CAN_ID_ADAS_STATUS;
    msg->dlc = 8;
    
    /* Byte 0-1: Distance in cm (Little Endian) */
    msg->data[0] = (uint8_t)(distance_cm & 0xFF);
    msg->data[1] = (uint8_t)((distance_cm >> 8) & 0xFF);
    
    /* Byte 2: ECU State */
    msg->data[2] = state;
    
    /* Byte 3: Hazard Level */
    msg->data[3] = hazard_level;
    
    /* Byte 4-7: Reserved payload padding */
    msg->data[4] = 0xAA;
    msg->data[5] = 0x55;
    msg->data[6] = 0x00;
    msg->data[7] = 0x00;

    msg->crc = CAN_CalculateCRC(msg->data, msg->dlc);
    return true;
}

bool CAN_UnpackAdasStatus(const CanMessage_t *msg, uint16_t *out_distance_cm, uint8_t *out_state, uint8_t *out_hazard_level) {
    if (!msg || msg->id != CAN_ID_ADAS_STATUS || !CAN_ValidateFrame(msg)) {
        return false;
    }

    if (out_distance_cm) {
        *out_distance_cm = (uint16_t)msg->data[0] | ((uint16_t)msg->data[1] << 8);
    }
    if (out_state) {
        *out_state = msg->data[2];
    }
    if (out_hazard_level) {
        *out_hazard_level = msg->data[3];
    }

    return true;
}

bool CAN_ValidateFrame(const CanMessage_t *msg) {
    if (!msg || msg->dlc > CAN_MAX_DATA_LEN) return false;
    uint16_t expected_crc = CAN_CalculateCRC(msg->data, msg->dlc);
    return (msg->crc == expected_crc);
}
