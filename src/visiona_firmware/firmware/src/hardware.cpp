#include "hardware.h"
#include <Wire.h>

int angleToPulse(float degrees, int servo_index) {
    float min_angle = SERVOS_MIN[servo_index];
    float max_angle = SERVOS_MAX[servo_index];
    float safe_angle = constrain(degrees, min_angle, max_angle);
    if (SERVO_INVERT[servo_index])
        safe_angle = 180.0f - safe_angle;
    int pulse = map(safe_angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
    return pulse + TRIM_US[servo_index];
}

float easeInOutQuintic(float t) {
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

void scan_i2c_available_address() {
    byte error, address;
    int nDevices = 0;
    Serial.println("Scanning...");
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at 0x");
            Serial.println(address, HEX);
            nDevices++;
        }
    }
    Serial.println(nDevices == 0 ? "No I2C devices found" : "done");
}
