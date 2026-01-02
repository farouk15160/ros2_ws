#include "safety.h"
#include <Adafruit_PWMServoDriver.h>

extern Adafruit_PWMServoDriver pwm;

void perform_safe_shutdown() {
    // 1. SET FLAG FIRST - This tells other tasks to BACK OFF immediately
    g_collisionDetected = true;

    // 2. Hardware Kill (Try OE Pin just in case)
    digitalWrite(OE_PIN, HIGH);
    delay(10);
    digitalWrite(STEPPER_ENABLE_PIN, HIGH);
    delay(10);

    // 3. Aggressive Soft Kill Loop
    // We try to take the Mutex multiple times.
    // We do NOT wait forever (deadlock risk), but we retry aggressively.
    bool bus_acquired = false;
    for (int attempt = 0; attempt < 10; attempt++) {
        if (xSemaphoreTake(x_i2c_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            bus_acquired = true;
            break;
        }
        // Small delay to let other tasks yield
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    // If we got the bus (or forced it), write the kill commands
    if (bus_acquired) {
        for (int i = 1; i < SERVOS_NUMBER; i++) {
            pwm.setPWM(SERVOS[i], 0, 4096); // Force OFF
            if (i == 1 && JOINT_1_SECOND_SERVO != 99)
                pwm.setPWM(JOINT_1_SECOND_SERVO, 0, 4096);
            delay(10);
        }
        xSemaphoreGive(x_i2c_mutex);
    } else {
        Serial.println("ERR: Could not acquire I2C for Soft Kill! (Check Wiring)");
    }
}

void emergency_stop() {
    perform_safe_shutdown();
}
