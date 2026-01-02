#include "monitoring.h"
#include "hardware.h"
#include "serial_protocol.h"
#include <Adafruit_INA219.h>
#include <esp_task_wdt.h>
#include <cmath>

extern Adafruit_INA219 ina219;

void task_monitoring(void *pvParameters) {
    esp_task_wdt_add(NULL);
    long lastStatusSendTime = 0;
    const int nSamples = 250;
    const float sens = 0.066;
    const float vcc = 3.3;
    const int adcMax = 4095;
    static float g_avgCurrent_A = 0.0f;
    const float IIR_ALPHA = 0.1f;

    for (;;) {
        esp_task_wdt_reset();

        // --- 1. Main Current Sensing ---
        long val = 0;
        for (int i = 0; i < nSamples; i++)
            val += analogRead(ACS712_PIN);
        float measured_voltage = ((float)val / nSamples / adcMax) * vcc;
        g_mainCurrent_A = (measured_voltage - g_calibrated_zero_voltage) / sens;
        float current_deviation = fabsf(g_mainCurrent_A - g_avgCurrent_A);
        g_avgCurrent_A = (g_avgCurrent_A * (1.0f - IIR_ALPHA)) + (g_mainCurrent_A * IIR_ALPHA);

        // --- 2. INA219 Gripper Sensing ---
        if (g_ina219_connected) {
            if (!g_collisionDetected) {
                if (xSemaphoreTake(x_i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    float raw_grip_current = ina219.getCurrent_mA();
                    xSemaphoreGive(x_i2c_mutex);
                    g_gripperCurrent_mA = (isnan(raw_grip_current) || isinf(raw_grip_current)) ? 0.0f : raw_grip_current;
                }
            }
        }

        // --- 3. Safety Checks ---
        bool unsafe_to_check = (millis() - g_estop_release_time) < 1000;

        if (g_safety_checks_enabled && !unsafe_to_check && !g_collisionDetected) {
            if (fabsf(g_mainCurrent_A) > g_collision_current_threshold_A && abs(g_mainCurrent_A) < 35) {
                Serial.print("E-STOP: Absolute threshold exceeded! Val: ");
                Serial.println(g_mainCurrent_A);
                // emergency_stop();
                continue;
            }

            xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
            unsigned long current_duration = g_move_duration_ms;
            unsigned long current_start_time = g_move_start_time;
            xSemaphoreGive(x_pose_mutex);

            bool is_moving = (millis() - current_start_time) < current_duration;
            bool movement_grace_over = (millis() - current_start_time) > DEVIATION_GRACE_PERIOD_MS;

            if (is_moving && movement_grace_over && (current_deviation > g_collision_deviation_threshold_A)) {
                Serial.print("E-STOP: Deviation! Dev: ");
                Serial.println(current_deviation);
                // emergency_stop();
                continue;
            }
        }

        // --- 4. Report Status ---
        if (millis() - lastStatusSendTime > 100) {
            StatusPacket status;
            status.header = HEADER_BYTE;
            status.cmd_id = 'S';
            status.main_current_A = g_mainCurrent_A;
            status.gripper_current_mA = g_gripperCurrent_mA;
            status.collision_flag = g_collisionDetected ? 1 : 0;
            
            xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
            unsigned long elapsed_time = millis() - g_move_start_time;
            float fraction = g_move_duration_ms > 0 ? (float)elapsed_time / (float)g_move_duration_ms : 1.0f;
            fraction = constrain(fraction, 0.0f, 1.0f);
            float eased_fraction = easeInOutQuintic(fraction);
            for (int i = 0; i < SERVOS_NUMBER; i++)
                status.angles[i] = (g_current_angles[i] * (1.0f - eased_fraction)) + (g_target_angles[i] * eased_fraction);
            xSemaphoreGive(x_pose_mutex);
            
            status.checksum = calculate_checksum((uint8_t *)&status, sizeof(StatusPacket) - 1);
            Serial.write((uint8_t *)&status, sizeof(StatusPacket));
            lastStatusSendTime = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
