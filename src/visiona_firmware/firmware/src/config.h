#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================================================
// --- Stepper Motor (Base) Configuration ---
// ==========================================================================
#define STEPPER_DIR_PIN 16
#define STEPPER_STEP_PIN 17
#define STEPPER_MS1_PIN 18
#define STEPPER_MS2_PIN 19
#define STEPPER_MS3_PIN 23
#define STEPPER_ENABLE_PIN 4

#define STEPPER_STEPS_PER_REV 200
#define STEPPER_MICROSTEPS 16
#define STEPPER_GEAR_RATIO 5.0f
#define STEPPER_PULSE_WIDTH_US 10
#define STEPPER_PULSE_DELAY_US 500

const float STEPPER_STEPS_PER_DEGREE = (STEPPER_STEPS_PER_REV * STEPPER_MICROSTEPS * STEPPER_GEAR_RATIO) / 360.0f;

// ==========================================================================
// --- 12V Fan PWM Configuration ---
// ==========================================================================
#define FAN_PWM_PIN 26
#define FAN_PWM_CHANNEL 0
#define FAN_PWM_FREQ 25000
#define FAN_PWM_RESOLUTION 8
#define FAN_DUTY_CYCLE_7_PERCENT 18

// ==========================================================================
// --- Multi-Tasking & Safety Configuration ---
// ==========================================================================
#define WDT_TIMEOUT 3
#define DEVIATION_GRACE_PERIOD_MS 300

// ==========================================================================
// --- Hardware & Servo Configuration ---
// ==========================================================================
#define ACS712_PIN 34
#define OE_PIN 13
#define BAUD_RATE 921600
#define PWM_FREQ 50
#define GRIPPER_SERVO_INDEX 5
#define SERVOS_NUMBER 6
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

// Set to 99 to disable. Set to 11 (or relevant pin) ONLY if you have dual servos on Joint 1.
#define JOINT_1_SECOND_SERVO 99

// ==========================================================================
// --- EEPROM Configuration ---
// ==========================================================================
#define EEPROM_SIZE 128
#define EEPROM_VALID_FLAG 123
#define EEPROM_VALID_FLAG_ADDR 0
#define EEPROM_POSE_ADDR 1
#define EEPROM_CONFIG_ADDR (EEPROM_POSE_ADDR + sizeof(float) * SERVOS_NUMBER)

// ==========================================================================
// --- Communication Protocol ---
// ==========================================================================
#define HEADER_BYTE 0xA5

#pragma pack(push, 1)
struct CommandPacket {
    uint8_t header;
    uint8_t cmd_id;
    float angles[6];
    float speed_factor;
    float gripper_current_ma;
    uint8_t checksum;
};

struct StatusPacket {
    uint8_t header;
    uint8_t cmd_id;
    float main_current_A;
    float gripper_current_mA;
    float angles[6];
    uint8_t collision_flag;
    uint8_t checksum;
};

struct ConfigPacket {
    uint8_t header;
    uint8_t cmd_id;
    float min_limits[6];
    float max_limits[6];
    float collision_threshold;
    float deviation_threshold;
    uint8_t checksum;
};
#pragma pack(pop)

struct ArmConfig {
    float min_limits[SERVOS_NUMBER];
    float max_limits[SERVOS_NUMBER];
    float collision_threshold;
    float deviation_threshold;
};

// ==========================================================================
// --- Servo Configuration Arrays ---
// ==========================================================================
extern const uint8_t SERVOS[SERVOS_NUMBER];
extern float SERVOS_MIN[SERVOS_NUMBER];
extern float SERVOS_MAX[SERVOS_NUMBER];
extern const float HOME_POSITION[SERVOS_NUMBER];
extern const bool SERVO_INVERT[SERVOS_NUMBER];
extern const int16_t TRIM_US[SERVOS_NUMBER];

// ==========================================================================
// --- Global State Variables ---
// ==========================================================================
// RTOS handles
extern TaskHandle_t h_task_command_parser;
extern TaskHandle_t h_task_monitoring;
extern TaskHandle_t h_task_motion_interpolator;
extern SemaphoreHandle_t x_pose_mutex;
extern SemaphoreHandle_t x_i2c_mutex;

// Safety globals
extern volatile float g_collision_current_threshold_A;
extern volatile float g_collision_deviation_threshold_A;
extern volatile bool g_collisionDetected;
extern volatile bool g_safety_checks_enabled;
extern volatile float g_mainCurrent_A;
extern volatile float g_gripperCurrent_mA;
extern float g_calibrated_zero_voltage;
extern bool g_ina219_connected;
extern volatile unsigned long g_estop_release_time;

// Motion control state
extern float g_current_angles[6];
extern float g_target_angles[6];
extern unsigned long g_move_start_time;
extern unsigned long g_move_duration_ms;
extern volatile float g_grip_target_current_mA;
extern volatile long g_stepper_current_step_pos;
extern volatile int g_fan_duty_cycle;

#endif // CONFIG_H
