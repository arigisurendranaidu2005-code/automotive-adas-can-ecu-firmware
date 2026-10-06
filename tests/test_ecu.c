#include <stdio.h>
#include <assert.h>
#include "safety_ecu.h"
#include "can_protocol.h"

void test_can_crc_and_packing(void) {
    CanMessage_t msg;
    bool packed = CAN_PackAdasStatus(&msg, 120, ECU_STATE_WARNING, HAZARD_WARNING);
    assert(packed == true);
    assert(msg.id == CAN_ID_ADAS_STATUS);
    assert(msg.dlc == 8);
    assert(CAN_ValidateFrame(&msg) == true);

    uint16_t dist = 0;
    uint8_t state = 0;
    uint8_t hazard = 0;
    bool unpacked = CAN_UnpackAdasStatus(&msg, &dist, &state, &hazard);
    assert(unpacked == true);
    assert(dist == 120);
    assert(state == ECU_STATE_WARNING);
    assert(hazard == HAZARD_WARNING);

    /* Tamper test */
    msg.data[0] ^= 0xFF;
    assert(CAN_ValidateFrame(&msg) == false);
    printf("PASS: test_can_crc_and_packing\n");
}

void test_sensor_fusion_filtering(void) {
    SensorFusion_t sf;
    SensorFusion_Init(&sf);
    assert(sf.hazard_level == HAZARD_CLEAR);

    SensorFusion_AddSample(&sf, 200, 0.05f);
    SensorFusion_AddSample(&sf, 100, 0.05f);
    SensorFusion_AddSample(&sf, 120, 0.05f);
    assert(sf.filtered_distance_cm > 0);
    assert(sf.hazard_level == HAZARD_WARNING);
    printf("PASS: test_sensor_fusion_filtering\n");
}

void test_emergency_braking_state_machine(void) {
    SafetyEcu_t ecu;
    ECU_Init(&ecu);
    ECU_ServiceWatchdog(&ecu);

    /* Approach critical boundary <= 50cm */
    for (int i = 0; i < 5; ++i) {
        ECU_ProcessSensorInput(&ecu, 40, 0.05f);
    }

    assert(ecu.state == ECU_STATE_EMERGENCY_BRAKE);
    assert(ecu.emergency_brake_engaged == true);
    printf("PASS: test_emergency_braking_state_machine\n");
}

void test_watchdog_failure_detection(void) {
    SafetyEcu_t ecu;
    ECU_Init(&ecu);
    ecu.watchdog_counter = 150; // Timeout threshold exceeded
    ECU_ExecuteStateTransition(&ecu);

    assert(ecu.state == ECU_STATE_FAULT);
    assert(ecu.total_faults >= 1);
    printf("PASS: test_watchdog_failure_detection\n");
}

int main(void) {
    printf("=== RUNNING ECU FIRMWARE AUTOMATED UNIT TESTS ===\n");
    test_can_crc_and_packing();
    test_sensor_fusion_filtering();
    test_emergency_braking_state_machine();
    test_watchdog_failure_detection();
    printf("=== ALL 4 UNIT TESTS PASSED SUCCESSFULLY (100%%) ===\n");
    return 0;
}
