#ifndef SAFETY_H
#define SAFETY_H

#include "config.h"
#include <Adafruit_PWMServoDriver.h>

/**
 * @brief Performs safe shutdown of all motors
 * Sets collision flag, disables hardware, and attempts soft kill on PWM
 */
void perform_safe_shutdown();

/**
 * @brief Emergency stop wrapper
 * Calls perform_safe_shutdown()
 */
void emergency_stop();

#endif // SAFETY_H
