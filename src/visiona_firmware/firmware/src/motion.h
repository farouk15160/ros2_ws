#ifndef MOTION_H
#define MOTION_H

#include "config.h"
#include <Adafruit_PWMServoDriver.h>

/**
 * @brief Calculate alpha (smoothing factor) from speed_factor
 * @param speed_factor Movement speed (10=fast, 500=slow)
 * @return Alpha value for IIR filter (0.02 to 0.5)
 */
float calculate_alpha_from_speed(float speed_factor);

/**
 * @brief Motion interpolation task (FreeRTOS)
 * Runs at 100Hz, applies IIR filtering to smooth motion
 */
void task_motion_interpolator(void *pvParameters);

#endif // MOTION_H
