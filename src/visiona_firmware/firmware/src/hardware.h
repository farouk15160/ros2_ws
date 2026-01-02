#ifndef HARDWARE_H
#define HARDWARE_H

#include "config.h"

/**
 * @brief Convert angle to PWM pulse width
 * @param degrees Servo angle (0-180)
 * @param servo_index Index in SERVOS array
 * @return Pulse width in microseconds
 */
int angleToPulse(float degrees, int servo_index);

/**
 * @brief Quintic easing function for smooth motion
 * @param t Time value (0.0 to 1.0)
 * @return Eased value (0.0 to 1.0)
 */
float easeInOutQuintic(float t);

/**
 * @brief Scan I2C bus for devices
 * Prints found device addresses to Serial
 */
void scan_i2c_available_address();

#endif // HARDWARE_H
