#ifndef SAFETY_ECU_H
#define SAFETY_ECU_H

#include "can_protocol.h"
#include "sensor_fusion.h"

typedef enum {
    ECU_STATE_INIT = 0,
    ECU_STATE_NORMAL = 1,
    ECU_STATE_WARNING = 2,
    ECU_STATE_EMERGENCY_BRAKE = 3,
    ECU_STATE_FAULT = 4
} EcuState_t;

typedef struct {
    EcuState_t state;
    SensorFusion_t sensor;
    uint32_t system_uptime_ms;
    uint32_t last_can_tx_time_ms;
    uint32_t watchdog_counter;
    bool emergency_brake_engaged;
    uint32_t total_faults;
} SafetyEcu_t;

void ECU_Init(SafetyEcu_t *ecu);
void ECU_ProcessSensorInput(SafetyEcu_t *ecu, uint16_t raw_distance_cm, float dt_sec);
void ECU_ExecuteStateTransition(SafetyEcu_t *ecu);
bool ECU_DispatchPeriodicCan(SafetyEcu_t *ecu, CanMessage_t *out_msg);
void ECU_ServiceWatchdog(SafetyEcu_t *ecu);
bool ECU_CheckWatchdogHealth(const SafetyEcu_t *ecu);

#endif /* SAFETY_ECU_H */
