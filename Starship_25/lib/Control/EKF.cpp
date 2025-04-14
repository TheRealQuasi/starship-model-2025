#include "EKF.h"
#include "Sensor_fusion.h"
#include "Sensor_handler.h"
#include <cmath>

#define GRAVITY 9.81f
#define DEG2RAD(x) ((x) * M_PI / 180.0f)

EKF::EKF() {
    x.fill(0.0f);
    for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j)
            P[i][j] = (i == j) ? 0.1f : 0.0f;  // Identity * 0.1
}

void EKF::predict(Vector3 acc, Vector3 gyro, float dt) {
    // Gravity compensation (assume world frame = IMU frame)
    acc.z -= GRAVITY;

    // Update velocity
    x[3] += acc.x * dt;
    x[4] += acc.y * dt;
    x[5] += acc.z * dt;

    // Update position
    x[0] += x[3] * dt;
    x[1] += x[4] * dt;
    x[2] += x[5] * dt;

    // Update orientation (roll, pitch)
    x[6] += gyro.x * dt;
    x[7] += gyro.y * dt;

    // TODO: Add covariance prediction here (optional for now)
}

void EKF::updateLidar(float z_meas) {
    std::array<float, 9> H = {0, 0, 1, 0, 0, 0, 0, 0, 0};  // z
    float R = 0.05f; // Measurement noise
    applyEKFUpdate(H, z_meas, R);
}

void EKF::updateOpticalFlow(float vx_meas, float vy_meas) {
    applyEKFUpdate({0,0,0,1,0,0,0,0,0}, vx_meas, 0.1f);  // vx
    applyEKFUpdate({0,0,0,0,1,0,0,0,0}, vy_meas, 0.1f);  // vy
}

void EKF::applyEKFUpdate(const std::array<float, 9>& H, float z_meas, float R) {
    float Hx = 0;
    for (int i = 0; i < 9; ++i) Hx += H[i] * x[i];
    float y = z_meas - Hx;

    float S = R;
    for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j)
            S += H[i] * P[i][j] * H[j];

    std::array<float, 9> K{};
    for (int i = 0; i < 9; ++i) {
        for (int j = 0; j < 9; ++j)
            K[i] += P[i][j] * H[j];
        K[i] /= S;
    }

    // Update state
    for (int i = 0; i < 9; ++i)
        x[i] += K[i] * y;

    // Update covariance (optional: Joseph form)
}

Vector3 EKF::getPosition() const {
    return {x[0], x[1], x[2]};
}

Vector3 EKF::getVelocity() const {
    return {x[3], x[4], x[5]};
}

Vector2 EKF::getOrientation() const {
    return {x[6], x[7]};
}
