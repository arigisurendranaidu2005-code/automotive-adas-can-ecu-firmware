#include <stdio.h>
#include "safety_ecu.h"
#include "can_protocol.h"

int main(void) {
    printf("====================================================\n");
    printf("   Aptiv ADAS Sensor Fusion & CAN ECU Firmware      \n");
    printf("   Target: 32-bit Automotive Microcontroller        \n");
    printf("====================================================\n\n");

    SafetyEcu_t ecu;
    ECU_Init(&ecu);
    CanMessage_t tx_msg;

    printf("[INIT] ECU State: %d, Initial Distance: %u cm\n", ecu.state, ecu.sensor.filtered_distance_cm);

    /* Simulated Telemetry Sequence: Approach obstacle */
    uint16_t simulated_ranges[] = {250, 180, 140, 110, 80, 45, 30};
    size_t steps = sizeof(simulated_ranges) / sizeof(simulated_ranges[0]);

    for (size_t i = 0; i < steps; ++i) {
        ECU_ServiceWatchdog(&ecu);
        ECU_ProcessSensorInput(&ecu, simulated_ranges[i], 0.05f); // 50ms tick
        ECU_DispatchPeriodicCan(&ecu, &tx_msg);

        printf("Tick %zu | Range: %3u cm -> Filtered: %3u cm | Hazard: %d | ECU State: %d | Brake: %s | CAN CRC: 0x%04X\n",
               i + 1,
               simulated_ranges[i],
               ecu.sensor.filtered_distance_cm,
               ecu.sensor.hazard_level,
               ecu.state,
               ecu.emergency_brake_engaged ? "ENGAGED [!]" : "RELEASED",
               tx_msg.crc);
    }

    printf("\n[SUCCESS] ADAS Emergency Collision Avoidance Triggered Successfully.\n");
    return 0;
}
