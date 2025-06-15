#include "main.h"
#include "mpu.h"
#include "complementry.h"

#include <math.h>

float roll = 0.0f, pitch = 0.0f;

void complementary_filter(float ax, float ay, float az, float gx, float gy, float dt) {


    float accel_roll = atan2f(ay, az) * 180.0f / 3.14159f;
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / 3.14159f;

    // Complementary filter blending
    roll = 0.97f * (roll + gx * dt) + 0.03f * accel_roll;
    pitch = 0.97f * (pitch + gy * dt) + 0.03f * accel_pitch;
}




float get_roll() {
    return roll;
}

float get_pitch() {
    return pitch;
}

