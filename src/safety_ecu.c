#include "safety_ecu.h"
#include <string.h>

#define WATCHDOG_TIMEOUT_TICKS 100

void ECU_Init(SafetyEcu_t *ecu) {
    if (!ecu) return;
    memset(ecu, 0, sizeof(SafetyEcu_t));
    ecu->state = ECU_STATE_INIT;
    SensorFusion_Init(&ecu->sensor);
    ecu->watchdog_counter = 0;
    ecu->emergency_brake_engaged = false;
    ecu->state = ECU_STATE_NORMAL;
}

void ECU_ProcessSensorInput(SafetyEcu_t *ecu, uint16_t raw_distance_cm, float dt_sec) {
    if (!ecu) return;
    SensorFusion_AddSample(&ecu->sensor, raw_distance_cm, dt_sec);
    ecu->system_uptime_ms += (uint32_t)(dt_sec * 1000.0f);
    ECU_ExecuteStateTransition(ecu);
}

void ECU_ExecuteStateTransition(SafetyEcu_t *ecu) {
    if (!ecu) return;

    if (!ECU_CheckWatchdogHealth(ecu)) {
        ecu->state = ECU_STATE_FAULT;
        ecu->total_faults++;
        return;
    }

    switch (ecu->sensor.hazard_level) {
        case HAZARD_CRITICAL:
            ecu->state = ECU_STATE_EMERGENCY_BRAKE;
            ecu->emergency_brake_engaged = true;
            break;
        case HAZARD_WARNING:
            ecu->state = ECU_STATE_WARNING;
            ecu->emergency_brake_engaged = false;
            break;
        case HAZARD_CAUTION:
        case HAZARD_CLEAR:
        default:
            ecu->state = ECU_STATE_NORMAL;
            ecu->emergency_brake_engaged = false;
            break;
    }
}

bool ECU_DispatchPeriodicCan(SafetyEcu_t *ecu, CanMessage_t *out_msg) {
    if (!ecu || !out_msg) return false;

    bool success = CAN_PackAdasStatus(
        out_msg,
        ecu->sensor.filtered_distance_cm,
        (uint8_t)ecu->state,
        (uint8_t)ecu->sensor.hazard_level
    );

    if (success) {
        ecu->last_can_tx_time_ms = ecu->system_uptime_ms;
    }
    return success;
}

void ECU_ServiceWatchdog(SafetyEcu_t *ecu) {
    if (ecu) {
        ecu->watchdog_counter = 0;
    }
}

bool ECU_CheckWatchdogHealth(const SafetyEcu_t *ecu) {
    if (!ecu) return false;
    return (ecu->watchdog_counter < WATCHDOG_TIMEOUT_TICKS);
}
