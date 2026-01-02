#include "serial_protocol.h"
#include "hardware.h"
#include "safety.h"
#include <EEPROM.h>
#include <esp_task_wdt.h>
#include <cmath>

extern Adafruit_PWMServoDriver pwm;

uint8_t calculate_checksum(const uint8_t *data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; ++i)
        checksum ^= data[i];
    return checksum;
}

void parse_command(const CommandPacket &cmd) {
    if (g_collisionDetected && cmd.cmd_id != 'E' && cmd.cmd_id != 'K')
        return;

    switch (cmd.cmd_id) {
    case 'M':
    case 'H':
    case 'G': {
        xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
        g_move_start_time = millis();
        float max_angle_delta = 0.0f;
        for (int i = 0; i < SERVOS_NUMBER; i++) {
            float target = (cmd.cmd_id == 'H') ? HOME_POSITION[i] : cmd.angles[i];
            g_target_angles[i] = constrain(target, SERVOS_MIN[i], SERVOS_MAX[i]);
            float delta = std::abs(g_target_angles[i] - g_current_angles[i]);
            if (delta > max_angle_delta)
                max_angle_delta = delta;
        }
        if (cmd.cmd_id == 'G')
            g_grip_target_current_mA = cmd.gripper_current_ma;

        float speed = (cmd.speed_factor > 0.01f) ? cmd.speed_factor : 10.0f;
        g_move_duration_ms = max_angle_delta * speed;
        if (g_move_duration_ms < 50)
            g_move_duration_ms = 50;
        if (cmd.cmd_id == 'H')
            g_move_duration_ms = 3000;

        xSemaphoreGive(x_pose_mutex);
        break;
    }
    case 'S': {
        xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
        EEPROM.write(EEPROM_VALID_FLAG_ADDR, EEPROM_VALID_FLAG);
        EEPROM.put(EEPROM_POSE_ADDR, g_current_angles);
        EEPROM.commit();
        xSemaphoreGive(x_pose_mutex);
        Serial.println("INFO: Current pose saved.");
        break;
    }
    case 'N': {
        xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
        for (int i = 0; i < 6; i++)
            if (cmd.angles[i] < SERVOS_MAX[i])
                SERVOS_MIN[i] = cmd.angles[i];
        xSemaphoreGive(x_pose_mutex);
        Serial.println("INFO: MIN limits updated.");
        break;
    }
    case 'X': {
        xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
        for (int i = 0; i < 6; i++)
            if (cmd.angles[i] > SERVOS_MIN[i])
                SERVOS_MAX[i] = cmd.angles[i];
        xSemaphoreGive(x_pose_mutex);
        Serial.println("INFO: MAX limits updated.");
        break;
    }
    case 'T': {
        float t = cmd.speed_factor;
        if (t > 0.1f && t < 20.0f)
            g_collision_current_threshold_A = t;
        else
            Serial.println("WARN: Invalid Threshold value.");
        break;
    }
    case 'D': {
        float t = cmd.speed_factor;
        if (t > 0.05f && t < 10.0f)
            g_collision_deviation_threshold_A = t;
        else
            Serial.println("WARN: Invalid Deviation value.");
        break;
    }
    case 'O': {
        g_safety_checks_enabled = (cmd.speed_factor > 0.5f);
        Serial.print("INFO: Collision Detection ");
        Serial.println(g_safety_checks_enabled ? "ENABLED" : "DISABLED (Use Caution!)");
        break;
    }
    case 'C': {
        xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
        ArmConfig cfg;
        memcpy(cfg.min_limits, SERVOS_MIN, sizeof(SERVOS_MIN));
        memcpy(cfg.max_limits, SERVOS_MAX, sizeof(SERVOS_MAX));
        cfg.collision_threshold = g_collision_current_threshold_A;
        cfg.deviation_threshold = g_collision_deviation_threshold_A;
        EEPROM.put(EEPROM_CONFIG_ADDR, cfg);
        EEPROM.write(EEPROM_VALID_FLAG_ADDR, EEPROM_VALID_FLAG);
        EEPROM.commit();
        xSemaphoreGive(x_pose_mutex);
        Serial.println("INFO: Config saved.");
        break;
    }
    case 'R': {
        ConfigPacket p;
        p.header = HEADER_BYTE;
        p.cmd_id = 'R';
        xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
        memcpy(p.min_limits, SERVOS_MIN, sizeof(SERVOS_MIN));
        memcpy(p.max_limits, SERVOS_MAX, sizeof(SERVOS_MAX));
        p.collision_threshold = g_collision_current_threshold_A;
        p.deviation_threshold = g_collision_deviation_threshold_A;
        xSemaphoreGive(x_pose_mutex);
        p.checksum = calculate_checksum((uint8_t *)&p, sizeof(ConfigPacket) - 1);
        Serial.write((uint8_t *)&p, sizeof(ConfigPacket));
        break;
    }
    case 'E': {
        g_collisionDetected = false;
        g_estop_release_time = millis();

        digitalWrite(STEPPER_ENABLE_PIN, LOW);
        digitalWrite(OE_PIN, LOW);

        xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
        int pulses[SERVOS_NUMBER];
        for (int i = 1; i < SERVOS_NUMBER; i++)
            pulses[i] = angleToPulse(g_current_angles[i], i);
        xSemaphoreGive(x_pose_mutex);

        if (xSemaphoreTake(x_i2c_mutex, portMAX_DELAY) == pdTRUE) {
            for (int i = 1; i < SERVOS_NUMBER; i++) {
                pwm.writeMicroseconds(SERVOS[i], pulses[i]);
                if (i == 1 && JOINT_1_SECOND_SERVO != 99)
                    pwm.writeMicroseconds(JOINT_1_SECOND_SERVO, pulses[i]);
            }
            xSemaphoreGive(x_i2c_mutex);
        }

        ledcWrite(FAN_PWM_CHANNEL, g_fan_duty_cycle);
        Serial.println("INFO: E-Stop Released (PWM Restored).");
        break;
    }
    case 'K': {
        perform_safe_shutdown();
        Serial.println("INFO: POWER KILLED (Motors Disabled). Send 'E' to restore.");
        break;
    }
    case 'F': {
        g_fan_duty_cycle = constrain((int)cmd.angles[0], 0, 255);
        ledcWrite(FAN_PWM_CHANNEL, g_fan_duty_cycle);
        Serial.println("INFO: Fan set.");
        break;
    }
    }
}

void task_command_parser(void *pvParameters) {
    esp_task_wdt_add(NULL);
    uint8_t serial_buffer[sizeof(CommandPacket)];
    int buffer_pos = 0;
    
    for (;;) {
        esp_task_wdt_reset();
        while (Serial.available() > 0) {
            uint8_t byte_in = Serial.read();
            if (buffer_pos == 0) {
                if (byte_in == HEADER_BYTE)
                    serial_buffer[buffer_pos++] = byte_in;
            } else {
                serial_buffer[buffer_pos++] = byte_in;
                if (buffer_pos >= sizeof(CommandPacket)) {
                    CommandPacket received_cmd;
                    memcpy(&received_cmd, serial_buffer, sizeof(CommandPacket));
                    if (calculate_checksum(serial_buffer, sizeof(CommandPacket) - 1) == received_cmd.checksum)
                        parse_command(received_cmd);
                    else
                        Serial.println("WARN: Checksum failed!");
                    buffer_pos = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
