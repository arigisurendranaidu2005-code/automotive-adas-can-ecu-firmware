#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define CAN_MAX_DATA_LEN 8
#define CAN_ID_ADAS_STATUS 0x120
#define CAN_ID_BRAKE_COMMAND 0x125
#define CAN_ID_DIAGNOSTIC_FAULT 0x190

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[CAN_MAX_DATA_LEN];
    uint16_t crc;
    uint32_t timestamp_ms;
} CanMessage_t;

uint16_t CAN_CalculateCRC(const uint8_t *data, uint8_t length);
bool CAN_PackAdasStatus(CanMessage_t *msg, uint16_t distance_cm, uint8_t state, uint8_t hazard_level);
bool CAN_UnpackAdasStatus(const CanMessage_t *msg, uint16_t *out_distance_cm, uint8_t *out_state, uint8_t *out_hazard_level);
bool CAN_ValidateFrame(const CanMessage_t *msg);

#endif /* CAN_PROTOCOL_H */
