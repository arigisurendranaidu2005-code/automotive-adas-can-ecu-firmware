#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <stdint.h>
#include <stdbool.h>

#define ROLLING_WINDOW_SIZE 5
#define COLLISION_THRESHOLD_CM 50
#define WARNING_THRESHOLD_CM 150

typedef enum {
    HAZARD_CLEAR = 0,
    HAZARD_CAUTION = 1,
    HAZARD_WARNING = 2,
    HAZARD_CRITICAL = 3
} HazardLevel_t;

typedef struct {
    uint16_t samples[ROLLING_WINDOW_SIZE];
    uint8_t sample_count;
    uint8_t write_idx;
    uint16_t filtered_distance_cm;
    float relative_velocity_mps;
    HazardLevel_t hazard_level;
} SensorFusion_t;

void SensorFusion_Init(SensorFusion_t *sf);
void SensorFusion_AddSample(SensorFusion_t *sf, uint16_t raw_distance_cm, float dt_sec);
HazardLevel_t SensorFusion_AssessHazard(const SensorFusion_t *sf);

#endif /* SENSOR_FUSION_H */
