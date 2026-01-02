/**
 * @file main.cpp
 * @brief ESP32 Robot Arm Firmware - Modular Architecture
 * @author Farouk
 * @date 03 Jan 2026
 * @version 4.0.0 (Restructured + Speed Fix)
 * 
 * @description
 * Fully modular firmware with proper speed control integration.
 * - Speed factor now properly affects motion smoothness
 * - Alpha = 6.0 / speed_factor (10=fast/0.5, 500=slow/0.02)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "hardware.h"
#include "safety.h"
#include "motion.h"
#include "serial_protocol.h"
#include "monitoring.h"

// Hardware instances
Adafruit_INA219 ina219 = Adafruit_INA219(0x41);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setup() {
    // Hardware disable first
    pinMode(OE_PIN, OUTPUT);
    digitalWrite(OE_PIN, HIGH);
    pinMode(STEPPER_ENABLE_PIN, OUTPUT);
    digitalWrite(STEPPER_ENABLE_PIN, HIGH);
    delay(2000);

    Serial.begin(BAUD_RATE);
    Serial.println("\n--- ESP32 Robot Arm Firmware v4.0.0 (Modular) ---");
    EEPROM.begin(EEPROM_SIZE);

    // Stepper pins
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_MS1_PIN, OUTPUT);
    pinMode(STEPPER_MS2_PIN, OUTPUT);
    pinMode(STEPPER_MS3_PIN, OUTPUT);
    digitalWrite(STEPPER_MS1_PIN, HIGH);
    digitalWrite(STEPPER_MS2_PIN, HIGH);
    digitalWrite(STEPPER_MS3_PIN, HIGH);

    // I2C Setup
    Wire.begin();
    Wire.setClock(100000);
    Wire.setTimeOut(3000);
    scan_i2c_available_address();

    // PWM Setup
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(PWM_FREQ);

    // INA219 Setup
    if (!ina219.begin()) {
        Serial.println("WARN: INA219 not found.");
        g_ina219_connected = false;
    } else {
        g_ina219_connected = true;
        Serial.println("INFO: INA219 initialized.");
    }

    // Fan PWM Setup
    ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
    ledcAttachPin(FAN_PWM_PIN, FAN_PWM_CHANNEL);
    ledcWrite(FAN_PWM_CHANNEL, g_fan_duty_cycle);

    // Auto-Calibrate Current Sensor
    Serial.print("INFO: Calibrating Current Sensor...");
    long cal_val = 0;
    for(int i=0; i<100; i++) {
        cal_val += analogRead(ACS712_PIN);
        delay(2);
    }
    g_calibrated_zero_voltage = ((float)cal_val / 100.0f / 4095.0f) * 3.3f;
    Serial.print(" Done. Zero-point: ");
    Serial.println(g_calibrated_zero_voltage, 4);

    // Restore Pose from EEPROM
    if (EEPROM.read(EEPROM_VALID_FLAG_ADDR) == EEPROM_VALID_FLAG) {
        Serial.println("INFO: Restoring pose.");
        float temp_angles[SERVOS_NUMBER];
        EEPROM.get(EEPROM_POSE_ADDR, temp_angles);
        ArmConfig cfg;
        EEPROM.get(EEPROM_CONFIG_ADDR, cfg);
        memcpy(SERVOS_MIN, cfg.min_limits, sizeof(SERVOS_MIN));
        memcpy(SERVOS_MAX, cfg.max_limits, sizeof(SERVOS_MAX));
        g_collision_current_threshold_A = cfg.collision_threshold;
        g_collision_deviation_threshold_A = cfg.deviation_threshold;
        for (int i = 0; i < SERVOS_NUMBER; i++) {
            g_current_angles[i] = constrain(temp_angles[i], SERVOS_MIN[i], SERVOS_MAX[i]);
            g_target_angles[i] = g_current_angles[i];
        }
    } else {
        Serial.println("INFO: Using defaults.");
        for (int i = 0; i < SERVOS_NUMBER; i++) {
            g_current_angles[i] = constrain(HOME_POSITION[i], SERVOS_MIN[i], SERVOS_MAX[i]);
            g_target_angles[i] = g_current_angles[i];
        }
        EEPROM.put(EEPROM_POSE_ADDR, g_current_angles);
        EEPROM.write(EEPROM_VALID_FLAG_ADDR, EEPROM_VALID_FLAG);
        EEPROM.commit();
    }

    g_stepper_current_step_pos = (long)(g_current_angles[0] * STEPPER_STEPS_PER_DEGREE);
    g_move_start_time = millis();
    g_move_duration_ms = 0;

    // Create Mutexes
    x_pose_mutex = xSemaphoreCreateMutex();
    x_i2c_mutex = xSemaphoreCreateMutex();

    // Soft-Start: Pre-load servo positions
    Serial.println("INFO: Pre-loading servo positions (Soft-Start)...");
    for (int i = 1; i < SERVOS_NUMBER; i++) {
        int pulse = angleToPulse(g_current_angles[i], i);
        pwm.writeMicroseconds(SERVOS[i], pulse);
        if (i == 1 && JOINT_1_SECOND_SERVO != 99)
            pwm.writeMicroseconds(JOINT_1_SECOND_SERVO, pulse);
        Serial.print(".");
        delay(150);
    }
    Serial.println("\nINFO: Soft-Start Complete!");

    // Enable Hardware
    digitalWrite(OE_PIN, LOW);
    digitalWrite(STEPPER_ENABLE_PIN, LOW);
    delay(100);

    // Initialize Watchdog
    esp_task_wdt_init(WDT_TIMEOUT, true);

    // Create FreeRTOS Tasks
    xTaskCreatePinnedToCore(task_motion_interpolator, "Motion", 4096, NULL, 2, &h_task_motion_interpolator, 1);
    xTaskCreatePinnedToCore(task_command_parser, "CmdParser", 4096, NULL, 1, &h_task_command_parser, 0);
    xTaskCreatePinnedToCore(task_monitoring, "Monitor", 4096, NULL, 1, &h_task_monitoring, 0);

    Serial.println("INFO: All tasks started. Robot ready!");
}

void loop() {
    vTaskDelete(NULL);  // FreeRTOS handles everything
}
