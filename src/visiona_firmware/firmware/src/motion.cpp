#include "motion.h"
#include "hardware.h"
#include <esp_task_wdt.h>
#include <cmath>

#define MAX(a,b) ((a) > (b) ? (a) : (b))

extern Adafruit_PWMServoDriver pwm;

float calculate_alpha_from_speed(float speed_factor) {
    // Speed factor ranges: 10 (fast) to 500 (slow)
    // Map to alpha: 0.3 (fast response) to 0.02 (slow, smooth)
    // Formula: alpha = K / speed_factor
    // Using K=6.0 gives good range
    float alpha = 6.0f / MAX(10.0f, speed_factor);
    return constrain(alpha, 0.02f, 0.5f);
}

void task_motion_interpolator(void *pvParameters) {
    esp_task_wdt_add(NULL);
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz Update Rate
    TickType_t xLastWakeTime = xTaskGetTickCount();

    float stepper_current_angle = g_current_angles[0]; // Local tracking for stepper
    float last_speed_factor = 150.0f; // Default mid-speed

    for (;;) {
        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // [SAFETY] Immediate Exit
        if (g_collisionDetected)
            continue;

        xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
        
        // Calculate alpha from current speed setting
        // Extract speed from move_duration_ms (it's stored there by parse_command)
        float max_angle_delta = 0.0f;
        for (int i = 0; i < SERVOS_NUMBER; i++) {
            float delta = fabs(g_target_angles[i] - g_current_angles[i]);
            if (delta > max_angle_delta)
                max_angle_delta = delta;
        }
        
        // Infer speed_factor from duration and delta
        // g_move_duration_ms = max_angle_delta * speed_factor
        float inferred_speed = (max_angle_delta > 0.01f && g_move_duration_ms > 0) ? 
            (float)g_move_duration_ms / max_angle_delta : last_speed_factor;
        
        // Smooth out speed changes
        last_speed_factor = last_speed_factor * 0.9f + inferred_speed * 0.1f;
        
        // Calculate responsive alpha
        float alpha = calculate_alpha_from_speed(last_speed_factor);
        
        bool all_reached = true;

        // --- Servos Interpolation ---
        int pulses[SERVOS_NUMBER];
        for (int i = 0; i < SERVOS_NUMBER; i++) {
            float target = g_target_angles[i];
            float current = g_current_angles[i];
            float diff = target - current;

            if (fabs(diff) > 0.05f) {
                all_reached = false;
                // Apply IIR Filter with calculated alpha
                g_current_angles[i] += diff * alpha;
            } else {
                g_current_angles[i] = target; // Snap to finish
            }
            
            if (i > 0) // Servos start at index 1
                pulses[i] = angleToPulse(g_current_angles[i], i);
        }
        
        // --- Stepper Logic ---
        float step_target = g_target_angles[0];
        float step_diff = step_target - stepper_current_angle;
        if (fabs(step_diff) > 0.05f)
            stepper_current_angle += step_diff * alpha;
        else
            stepper_current_angle = step_target;
            
        g_current_angles[0] = stepper_current_angle;

        long current_hw_steps = g_stepper_current_step_pos;
        long target_hw_steps = (long)(stepper_current_angle * STEPPER_STEPS_PER_DEGREE);
        long steps_to_move = target_hw_steps - current_hw_steps;

        if (steps_to_move != 0 && !g_collisionDetected) {
            digitalWrite(STEPPER_DIR_PIN, (steps_to_move > 0) ? 
                (SERVO_INVERT[0] ? HIGH : LOW) : (SERVO_INVERT[0] ? LOW : HIGH));
            long num_steps = abs(steps_to_move);
            // Limit steps per loop to prevent blocking
            if (num_steps > 5) num_steps = 5; 
            
            for (long k = 0; k < num_steps; k++) {
                digitalWrite(STEPPER_STEP_PIN, HIGH);
                delayMicroseconds(STEPPER_PULSE_WIDTH_US);
                digitalWrite(STEPPER_STEP_PIN, LOW);
                delayMicroseconds(STEPPER_PULSE_DELAY_US);
            }
            g_stepper_current_step_pos += (steps_to_move > 0) ? num_steps : -num_steps;
        }
        
        // --- Gripper Current Force Logic ---
        if (g_grip_target_current_mA > 0 && fabsf(g_gripperCurrent_mA) > g_grip_target_current_mA) {
           // If force exceeded, stop moving gripper
           g_target_angles[GRIPPER_SERVO_INDEX] = g_current_angles[GRIPPER_SERVO_INDEX];
           g_grip_target_current_mA = -1.0f;
        }

        xSemaphoreGive(x_pose_mutex);

        // --- Hardware Write (I2C) ---
        if (g_collisionDetected)
            continue;

        // Try to take I2C Mutex
        if (xSemaphoreTake(x_i2c_mutex, pdMS_TO_TICKS(8)) == pdTRUE) {
            if (!g_collisionDetected) {
                for (int i = 1; i < SERVOS_NUMBER; i++) {
                    pwm.writeMicroseconds(SERVOS[i], pulses[i]);
                    if (i == 1 && JOINT_1_SECOND_SERVO != 99)
                        pwm.writeMicroseconds(JOINT_1_SECOND_SERVO, pulses[i]);
                }
            }
            xSemaphoreGive(x_i2c_mutex);
        }
    }
}
