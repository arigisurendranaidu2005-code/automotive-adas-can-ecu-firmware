#include "sensor_fusion.h"
#include <string.h>

void SensorFusion_Init(SensorFusion_t *sf) {
    if (!sf) return;
    memset(sf, 0, sizeof(SensorFusion_t));
    sf->hazard_level = HAZARD_CLEAR;
    sf->filtered_distance_cm = 500;
}

void SensorFusion_AddSample(SensorFusion_t *sf, uint16_t raw_distance_cm, float dt_sec) {
    if (!sf) return;

    uint16_t prev_distance = sf->filtered_distance_cm;
    sf->samples[sf->write_idx] = raw_distance_cm;
    sf->write_idx = (sf->write_idx + 1) % ROLLING_WINDOW_SIZE;
    if (sf->sample_count < ROLLING_WINDOW_SIZE) {
        sf->sample_count++;
    }

    /* Compute moving average */
    uint32_t sum = 0;
    for (uint8_t i = 0; i < sf->sample_count; ++i) {
        sum += sf->samples[i];
    }
    sf->filtered_distance_cm = (uint16_t)(sum / sf->sample_count);

    /* Relative velocity in meters/sec */
    if (dt_sec > 0.001f) {
        float delta_m = (float)((int32_t)sf->filtered_distance_cm - (int32_t)prev_distance) / 100.0f;
        sf->relative_velocity_mps = delta_m / dt_sec;
    }

    sf->hazard_level = SensorFusion_AssessHazard(sf);
}

HazardLevel_t SensorFusion_AssessHazard(const SensorFusion_t *sf) {
    if (!sf) return HAZARD_CLEAR;

    if (sf->filtered_distance_cm <= COLLISION_THRESHOLD_CM) {
        return HAZARD_CRITICAL;
    } else if (sf->filtered_distance_cm <= WARNING_THRESHOLD_CM) {
        return HAZARD_WARNING;
    } else if (sf->filtered_distance_cm <= 300) {
        return HAZARD_CAUTION;
    }
    return HAZARD_CLEAR;
}
