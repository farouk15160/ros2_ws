/**
 * @file main.cpp
 * @brief Fully interruptible, non-blocking firmware for ROS2 Robot Arm
 * @author Farouk
 * @date 08 Dec 2025
 *
 * @description
 * Version 3.9.16 (The "Safety Toggle" Update):
 * - [NEW] Command 'O' (Override): Allows toggling collision detection ON/OFF via software.
 * Set speed_factor to 0.0 to Disable checks, 1.0 to Enable checks.
 * - [FIX] Kill Command ('K'): Includes previous "Bulldozer" fix for reliable shutdown.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <EEPROM.h>
#include <cmath>
#include <esp_task_wdt.h> // Hardware Watchdog Library

// ==========================================================================
// --- Stepper Motor (Base) Configuration ---
// ==========================================================================
const int STEPPER_DIR_PIN = 16;
const int STEPPER_STEP_PIN = 17;
const int STEPPER_MS1_PIN = 18;
const int STEPPER_MS2_PIN = 19;
const int STEPPER_MS3_PIN = 23;
const int STEPPER_ENABLE_PIN = 4;

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
// Watchdog Timeout in seconds
#define WDT_TIMEOUT 3
#define DEVIATION_GRACE_PERIOD_MS 300 // Time to ignore deviation after move starts

TaskHandle_t h_task_command_parser = NULL;
TaskHandle_t h_task_monitoring = NULL;
TaskHandle_t h_task_motion_interpolator = NULL;
SemaphoreHandle_t x_pose_mutex;
SemaphoreHandle_t x_i2c_mutex;

// SAFETY GLOBALS
volatile float g_collision_current_threshold_A = 5.0f;
volatile float g_collision_deviation_threshold_A = 1.0f;
volatile bool g_collisionDetected = false;
volatile bool g_safety_checks_enabled = true; // [NEW] Master toggle for sensing
volatile float g_mainCurrent_A = 0.0;
volatile float g_gripperCurrent_mA = 0.0;
float g_calibrated_zero_voltage = 2.3707f;
bool g_ina219_connected = false;

// Global to track when E-Stop was released
volatile unsigned long g_estop_release_time = 0;

// --- Motion Control State (Protected by Mutex) ---
float g_current_angles[6];
float g_target_angles[6];
unsigned long g_move_start_time;
unsigned long g_move_duration_ms;
volatile float g_grip_target_current_mA = -1.0f;
volatile long g_stepper_current_step_pos = 0;
volatile int g_fan_duty_cycle = FAN_DUTY_CYCLE_7_PERCENT;

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

Adafruit_INA219 ina219 = Adafruit_INA219(0x41);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t SERVOS[SERVOS_NUMBER] = {0, 7, 9, 11, 13, 15};
float SERVOS_MIN[SERVOS_NUMBER] = {15.0f, 40.0f, 0.0f, 30.0f, 0.0f, 0.0f};
float SERVOS_MAX[SERVOS_NUMBER] = {345.0f, 150.0f, 145.0f, 130.0f, 180.0f, 120.0f};

const float HOME_POSITION[SERVOS_NUMBER] = {133.0f, 100.00, 110.00, 130.00, 15.00, 60.00};
const bool SERVO_INVERT[SERVOS_NUMBER] = {false, true, false, false, false, false};
const int16_t TRIM_US[SERVOS_NUMBER] = {0, 0, 0, 0, 0, 0};

// --- EEPROM Config ---
#define EEPROM_SIZE 128
#define EEPROM_VALID_FLAG 123
#define EEPROM_VALID_FLAG_ADDR 0
#define EEPROM_POSE_ADDR 1
#define EEPROM_CONFIG_ADDR (EEPROM_POSE_ADDR + sizeof(float) * SERVOS_NUMBER)

struct ArmConfig
{
  float min_limits[SERVOS_NUMBER];
  float max_limits[SERVOS_NUMBER];
  float collision_threshold;
  float deviation_threshold;
};

#define HEADER_BYTE 0xA5
#pragma pack(push, 1)
struct CommandPacket
{
  uint8_t header;
  uint8_t cmd_id;
  float angles[6];
  float speed_factor;
  float gripper_current_ma;
  uint8_t checksum;
};
struct StatusPacket
{
  uint8_t header;
  uint8_t cmd_id;
  float main_current_A;
  float gripper_current_mA;
  float angles[6];
  uint8_t collision_flag;
  uint8_t checksum;
};
struct ConfigPacket
{
  uint8_t header;
  uint8_t cmd_id;
  float min_limits[6];
  float max_limits[6];
  float collision_threshold;
  float deviation_threshold;
  uint8_t checksum;
};
#pragma pack(pop)

// --- Helper Functions ---
uint8_t calculate_checksum(const uint8_t *data, size_t len)
{
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; ++i)
    checksum ^= data[i];
  return checksum;
}

int angleToPulse(float degrees, int servo_index)
{
  float min_angle = SERVOS_MIN[servo_index];
  float max_angle = SERVOS_MAX[servo_index];
  float safe_angle = constrain(degrees, min_angle, max_angle);
  if (SERVO_INVERT[servo_index])
    safe_angle = 180.0f - safe_angle;
  int pulse = map(safe_angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  return pulse + TRIM_US[servo_index];
}

static inline float easeInOutQuintic(float t)
{
  return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

// ==========================================================================
// --- Safety: Hard & Soft Kill Functions ---
// ==========================================================================
void perform_safe_shutdown()
{
  // 1. SET FLAG FIRST - This tells other tasks to BACK OFF immediately
  g_collisionDetected = true;

  // 2. Hardware Kill (Try OE Pin just in case)
  digitalWrite(OE_PIN, HIGH);
  delay(10);
  digitalWrite(STEPPER_ENABLE_PIN, HIGH);
  delay(10);

  // ledcWrite(FAN_PWM_CHANNEL, 0);

  // 4. Aggressive Soft Kill Loop
  // We try to take the Mutex multiple times.
  // We do NOT wait forever (deadlock risk), but we retry aggressively.
  bool bus_acquired = false;
  for (int attempt = 0; attempt < 10; attempt++)
  {
    if (xSemaphoreTake(x_i2c_mutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
      bus_acquired = true;
      break;
    }
    // Small delay to let other tasks yield
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  // If we got the bus (or forced it), write the kill commands
  if (bus_acquired)
  {
    for (int i = 1; i < SERVOS_NUMBER; i++)
    {
      pwm.setPWM(SERVOS[i], 0, 4096); // Force OFF
      if (i == 1 && JOINT_1_SECOND_SERVO != 99)
        pwm.setPWM(JOINT_1_SECOND_SERVO, 0, 4096);
      delay(10);
    }
    xSemaphoreGive(x_i2c_mutex);
  }
  else
  {
    Serial.println("ERR: Could not acquire I2C for Soft Kill! (Check Wiring)");
  }
}

void emergency_stop()
{
  perform_safe_shutdown();
}

// ==========================================================================
// --- Task: Motion Interpolator ---
// ==========================================================================
void task_motion_interpolator(void *pvParameters)
{
  esp_task_wdt_add(NULL);
  const TickType_t xFrequency = pdMS_TO_TICKS(20);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    esp_task_wdt_reset();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // [SAFETY] Immediate Exit
    if (g_collisionDetected)
      continue;

    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);

    if (g_move_duration_ms == 0)
    {
      xSemaphoreGive(x_pose_mutex);
      continue;
    }

    unsigned long elapsed_time = millis() - g_move_start_time;
    float fraction = (float)elapsed_time / (float)g_move_duration_ms;

    if (fraction > 1.0f)
      fraction = 1.0f;
    float eased_fraction = easeInOutQuintic(fraction);

    // Gripper Force Logic
    if (g_grip_target_current_mA > 0 && fabsf(g_gripperCurrent_mA) > g_grip_target_current_mA)
    {
      float final_grip_angle = (g_current_angles[GRIPPER_SERVO_INDEX] * (1.0f - eased_fraction)) + (g_target_angles[GRIPPER_SERVO_INDEX] * eased_fraction);
      g_current_angles[GRIPPER_SERVO_INDEX] = final_grip_angle;
      g_target_angles[GRIPPER_SERVO_INDEX] = final_grip_angle;
      g_grip_target_current_mA = -1.0f;
    }

    // Stepper Logic
    long start_step_pos = (long)(g_current_angles[0] * STEPPER_STEPS_PER_DEGREE);
    long end_step_pos = (long)(g_target_angles[0] * STEPPER_STEPS_PER_DEGREE);
    long total_steps_for_move = end_step_pos - start_step_pos;
    long current_target_step_pos = start_step_pos + (long)(total_steps_for_move * eased_fraction);
    long steps_to_move = current_target_step_pos - g_stepper_current_step_pos;

    if (steps_to_move != 0 && !g_collisionDetected)
    {
      digitalWrite(STEPPER_DIR_PIN, (steps_to_move > 0) ? (SERVO_INVERT[0] ? HIGH : LOW) : (SERVO_INVERT[0] ? LOW : HIGH));
      long num_steps_to_pulse = abs(steps_to_move);
      for (long i = 0; i < num_steps_to_pulse; i++)
      {
        if (g_collisionDetected)
          break;
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(STEPPER_PULSE_WIDTH_US);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(STEPPER_PULSE_DELAY_US);
      }
      if (!g_collisionDetected)
      {
        g_stepper_current_step_pos = current_target_step_pos;
      }
    }

    // Prepare I2C Pulses
    int pulses[SERVOS_NUMBER];
    for (int i = 1; i < SERVOS_NUMBER; i++)
    {
      float interpolated_angle = (g_current_angles[i] * (1.0f - eased_fraction)) + (g_target_angles[i] * eased_fraction);
      pulses[i] = angleToPulse(interpolated_angle, i);
    }
    xSemaphoreGive(x_pose_mutex);

    // [SAFETY] Check BEFORE taking I2C Mutex
    if (g_collisionDetected)
      continue;

    // Try to take I2C Mutex
    if (xSemaphoreTake(x_i2c_mutex, portMAX_DELAY) == pdTRUE)
    {
      // [SAFETY] Check AFTER taking I2C Mutex (in case we waited)
      if (!g_collisionDetected)
      {
        for (int i = 1; i < SERVOS_NUMBER; i++)
        {
          pwm.writeMicroseconds(SERVOS[i], pulses[i]);
          if (i == 1 && JOINT_1_SECOND_SERVO != 99)
            pwm.writeMicroseconds(JOINT_1_SECOND_SERVO, pulses[i]);
        }
      }
      xSemaphoreGive(x_i2c_mutex);
    }

    // Finalize Move
    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
    if (fraction >= 1.0f)
    {
      g_grip_target_current_mA = -1.0;
      for (int i = 0; i < SERVOS_NUMBER; i++)
      {
        g_current_angles[i] = g_target_angles[i];
      }
      g_stepper_current_step_pos = end_step_pos;
      g_move_duration_ms = 0;
    }
    xSemaphoreGive(x_pose_mutex);
  }
}

void parse_command(const CommandPacket &cmd)
{
  if (g_collisionDetected && cmd.cmd_id != 'E' && cmd.cmd_id != 'K')
    return;

  switch (cmd.cmd_id)
  {
  case 'M':
  case 'H':
  case 'G':
  {
    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
    g_move_start_time = millis();
    float max_angle_delta = 0.0f;
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
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
  case 'S':
  {
    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
    EEPROM.write(EEPROM_VALID_FLAG_ADDR, EEPROM_VALID_FLAG);
    EEPROM.put(EEPROM_POSE_ADDR, g_current_angles);
    EEPROM.commit();
    xSemaphoreGive(x_pose_mutex);
    Serial.println("INFO: Current pose saved.");
    break;
  }
  case 'N':
  {
    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
    for (int i = 0; i < 6; i++)
      if (cmd.angles[i] < SERVOS_MAX[i])
        SERVOS_MIN[i] = cmd.angles[i];
    xSemaphoreGive(x_pose_mutex);
    Serial.println("INFO: MIN limits updated.");
    break;
  }
  case 'X':
  {
    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
    for (int i = 0; i < 6; i++)
      if (cmd.angles[i] > SERVOS_MIN[i])
        SERVOS_MAX[i] = cmd.angles[i];
    xSemaphoreGive(x_pose_mutex);
    Serial.println("INFO: MAX limits updated.");
    break;
  }
  case 'T':
  {
    float t = cmd.speed_factor;
    if (t > 0.1f && t < 20.0f)
      g_collision_current_threshold_A = t;
    else
      Serial.println("WARN: Invalid Threshold value.");
    break;
  }
  case 'D':
  {
    float t = cmd.speed_factor;
    if (t > 0.05f && t < 10.0f)
      g_collision_deviation_threshold_A = t;
    else
      Serial.println("WARN: Invalid Deviation value.");
    break;
  }
  case 'O': // [NEW] Toggle Collision Detection
  {
    g_safety_checks_enabled = (cmd.speed_factor > 0.5f);
    Serial.print("INFO: Collision Detection ");
    Serial.println(g_safety_checks_enabled ? "ENABLED" : "DISABLED (Use Caution!)");
    break;
  }
  case 'C':
  {
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
  case 'R':
  {
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
  case 'E':
  {
    g_collisionDetected = false;
    g_estop_release_time = millis();

    digitalWrite(STEPPER_ENABLE_PIN, LOW);
    digitalWrite(OE_PIN, LOW);

    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
    int pulses[SERVOS_NUMBER];
    for (int i = 1; i < SERVOS_NUMBER; i++)
    {
      pulses[i] = angleToPulse(g_current_angles[i], i);
    }
    xSemaphoreGive(x_pose_mutex);

    if (xSemaphoreTake(x_i2c_mutex, portMAX_DELAY) == pdTRUE)
    {
      for (int i = 1; i < SERVOS_NUMBER; i++)
      {
        pwm.writeMicroseconds(SERVOS[i], pulses[i]);
        if (i == 1 && JOINT_1_SECOND_SERVO != 99)
        {
          pwm.writeMicroseconds(JOINT_1_SECOND_SERVO, pulses[i]);
        }
      }
      xSemaphoreGive(x_i2c_mutex);
    }

    ledcWrite(FAN_PWM_CHANNEL, g_fan_duty_cycle);
    Serial.println("INFO: E-Stop Released (PWM Restored).");
    break;
  }
  case 'K':
  {
    perform_safe_shutdown();
    Serial.println("INFO: POWER KILLED (Motors Disabled). Send 'E' to restore.");
    break;
  }
  case 'F':
  {
    g_fan_duty_cycle = constrain((int)cmd.angles[0], 0, 255);
    ledcWrite(FAN_PWM_CHANNEL, g_fan_duty_cycle);
    Serial.println("INFO: Fan set.");
    break;
  }
  }
}

void task_command_parser(void *pvParameters)
{
  esp_task_wdt_add(NULL);
  uint8_t serial_buffer[sizeof(CommandPacket)];
  int buffer_pos = 0;
  for (;;)
  {
    esp_task_wdt_reset();
    while (Serial.available() > 0)
    {
      uint8_t byte_in = Serial.read();
      if (buffer_pos == 0)
      {
        if (byte_in == HEADER_BYTE)
          serial_buffer[buffer_pos++] = byte_in;
      }
      else
      {
        serial_buffer[buffer_pos++] = byte_in;
        if (buffer_pos >= sizeof(CommandPacket))
        {
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

void task_monitoring(void *pvParameters)
{
  esp_task_wdt_add(NULL);
  long lastStatusSendTime = 0;
  const int nSamples = 250;
  const float sens = 0.066;
  const float vcc = 3.3;
  const int adcMax = 4095;
  static float g_avgCurrent_A = 0.0f;
  const float IIR_ALPHA = 0.1f;

  for (;;)
  {
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
    if (g_ina219_connected)
    {
      // [SAFETY] Do not attempt I2C read if Collision Flag is set!
      if (!g_collisionDetected)
      {
        if (xSemaphoreTake(x_i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          float raw_grip_current = ina219.getCurrent_mA();
          xSemaphoreGive(x_i2c_mutex);
          g_gripperCurrent_mA = (isnan(raw_grip_current) || isinf(raw_grip_current)) ? 0.0f : raw_grip_current;
        }
      }
    }

    // --- 3. Safety Checks ---
    bool unsafe_to_check = (millis() - g_estop_release_time) < 1000;

    // [NEW] Only check if Global Safety is Enabled ('O' command)
    if (g_safety_checks_enabled && !unsafe_to_check && !g_collisionDetected)
    {
      if (fabsf(g_mainCurrent_A) > g_collision_current_threshold_A && abs(g_mainCurrent_A) < 35)
      {
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

      if (is_moving && movement_grace_over && (current_deviation > g_collision_deviation_threshold_A))
      {
        Serial.print("E-STOP: Deviation! Dev: ");
        Serial.println(current_deviation);
        // emergency_stop();
        continue;
      }
    }

    // --- 4. Report Status ---
    if (millis() - lastStatusSendTime > 100)
    {
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

// I2C-Scanner
void scan_i2c_availble_address()
{
  byte error, address;
  int nDevices = 0;
  Serial.println("Scanning...");
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at 0x");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  Serial.println(nDevices == 0 ? "No I2C devices found" : "done");
}

void setup()
{
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, HIGH);
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, HIGH);
  delay(2000);

  Serial.begin(BAUD_RATE);
  Serial.println("\n--- ESP32 Robot Arm Firmware (v3.9.16 Safety Toggle) ---");
  EEPROM.begin(EEPROM_SIZE);

  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_MS1_PIN, OUTPUT);
  pinMode(STEPPER_MS2_PIN, OUTPUT);
  pinMode(STEPPER_MS3_PIN, OUTPUT);
  digitalWrite(STEPPER_MS1_PIN, HIGH);
  digitalWrite(STEPPER_MS2_PIN, HIGH);
  digitalWrite(STEPPER_MS3_PIN, HIGH);

  // --- I2C SETUP (Optimized for Safety) ---
  Wire.begin();
  Wire.setClock(100000); // Stable speed
  Wire.setTimeOut(3000); // 3ms Timeout to prevent hanging on loose wires
  scan_i2c_availble_address();

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PWM_FREQ);

  if (!ina219.begin())
  {
    Serial.println("WARN: INA219 not found.");
    g_ina219_connected = false;
  }
  else
  {
    g_ina219_connected = true;
    Serial.println("INFO: INA219 initialized.");
  }

  ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  ledcAttachPin(FAN_PWM_PIN, FAN_PWM_CHANNEL);
  ledcWrite(FAN_PWM_CHANNEL, g_fan_duty_cycle);

  // Hardcoded calibration as requested
  Serial.print("INFO: Zero-point: ");
  Serial.println(g_calibrated_zero_voltage, 4);

  // --- Restore Pose from EEPROM ---
  if (EEPROM.read(EEPROM_VALID_FLAG_ADDR) == EEPROM_VALID_FLAG)
  {
    Serial.println("INFO: Restoring pose.");
    float temp_angles[SERVOS_NUMBER];
    EEPROM.get(EEPROM_POSE_ADDR, temp_angles);
    ArmConfig cfg;
    EEPROM.get(EEPROM_CONFIG_ADDR, cfg);
    memcpy(SERVOS_MIN, cfg.min_limits, sizeof(SERVOS_MIN));
    memcpy(SERVOS_MAX, cfg.max_limits, sizeof(SERVOS_MAX));
    g_collision_current_threshold_A = cfg.collision_threshold;
    g_collision_deviation_threshold_A = cfg.deviation_threshold;
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      g_current_angles[i] = constrain(temp_angles[i], SERVOS_MIN[i], SERVOS_MAX[i]);
      g_target_angles[i] = g_current_angles[i];
    }
  }
  else
  {
    Serial.println("INFO: Using defaults.");
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
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

  x_pose_mutex = xSemaphoreCreateMutex();
  x_i2c_mutex = xSemaphoreCreateMutex();

  // --- Soft-Start Pre-Load Loop ---
  Serial.println("INFO: Pre-loading servo positions (Soft-Start)...");
  for (int i = 1; i < SERVOS_NUMBER; i++)
  {
    int pulse = angleToPulse(g_current_angles[i], i);
    pwm.writeMicroseconds(SERVOS[i], pulse);
    if (i == 1 && JOINT_1_SECOND_SERVO != 99)
      pwm.writeMicroseconds(JOINT_1_SECOND_SERVO, pulse);

    Serial.print("."); // Visual feedback
    delay(150);        // 150ms delay to prevent power supply brownout
  }
  Serial.println(""); // New line
  delay(500);

  digitalWrite(STEPPER_ENABLE_PIN, LOW);
  digitalWrite(OE_PIN, LOW);
  Serial.println("Motors enabled.");

  // --- Initialize Watchdog Timer ---
  esp_task_wdt_init(WDT_TIMEOUT, true);

  xTaskCreatePinnedToCore(task_monitoring, "Monitoring", 4096, NULL, 1, &h_task_monitoring, 0);
  xTaskCreatePinnedToCore(task_command_parser, "CmdParser", 4096, NULL, 2, &h_task_command_parser, 1);
  xTaskCreatePinnedToCore(task_motion_interpolator, "Motion", 4096, NULL, 3, &h_task_motion_interpolator, 1);
  Serial.println("Initialization Complete.");
}

void loop() { vTaskDelete(NULL); }