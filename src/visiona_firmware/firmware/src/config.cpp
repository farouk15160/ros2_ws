#include "config.h"

// Servo configuration arrays
const uint8_t SERVOS[SERVOS_NUMBER] = {0, 7, 9, 11, 13, 15};
float SERVOS_MIN[SERVOS_NUMBER] = {15.0f, 40.0f, 0.0f, 30.0f, 0.0f, 0.0f};
float SERVOS_MAX[SERVOS_NUMBER] = {345.0f, 150.0f, 145.0f, 130.0f, 180.0f, 120.0f};
const float HOME_POSITION[SERVOS_NUMBER] = {133.0f, 100.00, 110.00, 130.00, 15.00, 60.00};
const bool SERVO_INVERT[SERVOS_NUMBER] = {false, true, false, false, false, false};
const int16_t TRIM_US[SERVOS_NUMBER] = {0, 0, 0, 0, 0, 0};

// RTOS handles
TaskHandle_t h_task_command_parser = NULL;
TaskHandle_t h_task_monitoring = NULL;
TaskHandle_t h_task_motion_interpolator = NULL;
SemaphoreHandle_t x_pose_mutex;
SemaphoreHandle_t x_i2c_mutex;

// Safety globals
volatile float g_collision_current_threshold_A = 5.0f;
volatile float g_collision_deviation_threshold_A = 1.0f;
volatile bool g_collisionDetected = false;
volatile bool g_safety_checks_enabled = true;
volatile float g_mainCurrent_A = 0.0;
volatile float g_gripperCurrent_mA = 0.0;
float g_calibrated_zero_voltage = 2.3707f;
bool g_ina219_connected = false;
volatile unsigned long g_estop_release_time = 0;

// Motion control state
float g_current_angles[6];
float g_target_angles[6];
unsigned long g_move_start_time;
unsigned long g_move_duration_ms;
volatile float g_grip_target_current_mA = -1.0f;
volatile long g_stepper_current_step_pos = 0;
volatile int g_fan_duty_cycle = FAN_DUTY_CYCLE_7_PERCENT;
