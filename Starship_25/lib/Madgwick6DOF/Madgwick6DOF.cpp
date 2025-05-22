#include "Madgwick6DOF.h"
#include <cmath>

Madgwick6DOF::Madgwick6DOF() {
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    beta = 0.01f;
}

void Madgwick6DOF::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    float norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;
    norm = 1.0f / norm;
    ax *= norm; ay *= norm; az *= norm;

    float d2q0 = 2.0f * q0, d2q1 = 2.0f * q1, d2q2 = 2.0f * q2, d2q3 = 2.0f * q3;
    float d4q0 = 4.0f * q0, d4q1 = 4.0f * q1, d4q2 = 4.0f * q2;
    float d8q1 = 8.0f * q1, d8q2 = 8.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    float s0 = d4q0 * q2q2 + d2q2 * ax + d4q0 * q1q1 - d2q1 * ay;
    float s1 = d4q1 * q3q3 - d2q3 * ax + 4.0f * q0q0 * q1 - d2q0 * ay - d4q1 + d8q1 * q1q1 + d8q1 * q2q2 + d4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + d2q0 * ax + d4q2 * q3q3 - d2q3 * ay - d4q2 + d8q2 * q1q1 + d8q2 * q2q2 + d4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - d2q1 * ax + 4.0f * q2q2 * q3 - d2q2 * ay;

    norm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= norm; s1 *= norm; s2 *= norm; s3 *= norm;

    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q[0] = q0 * norm;
    q[1] = q1 * norm;
    q[2] = q2 * norm;
    q[3] = q3 * norm;
}

float Madgwick6DOF::invSqrt(float x) const {
    return 1.0f / std::sqrt(x);
}

void Madgwick6DOF::getEulerAngles(float& roll, float& pitch, float& yaw) const {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    roll  = std::atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    pitch = std::asin(2.0f * (q0 * q2 - q3 * q1));
    yaw   = std::atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
