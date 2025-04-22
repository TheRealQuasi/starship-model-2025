#include "EKF_C.h"
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

    // === Simple covariance prediction ===
    float q = 0.01f; // Process noise (you can tune this)
    for (int i = 0; i < 9; ++i)
        P[i][i] += q * dt;  // Increase uncertainty
}

void EKF::updateIMU(Vector3 acc_meas, Vector3 gyro_meas) {
    // You can choose to correct based on accel (for gravity) and/or gyro (bias correction)

    // Example: Correct roll and pitch using accelerometer
    // Assume we want roll = atan2(acc_y, acc_z), pitch = atan2(-acc_x, sqrt(acc_y^2 + acc_z^2))

    float roll_meas = atan2f(acc_meas.y, acc_meas.z);
    float pitch_meas = atan2f(-acc_meas.x, sqrtf(acc_meas.y * acc_meas.y + acc_meas.z * acc_meas.z));

    // Apply EKF update for roll
    applyEKFUpdate({0,0,0,0,0,0,1,0,0}, roll_meas, 0.05f);  // roll estimate
    applyEKFUpdate({0,0,0,0,0,0,0,1,0}, pitch_meas, 0.05f); // pitch estimate

    // (Optional) You could also update velocity using accelerometer readings if you trust them
}


void EKF::updateLidar(float z_meas) {
    std::array<float, 9> H = {0, 0, 1, 0, 0, 0, 0, 0, 0};  // z
    float R = 0.1f; // Measurement noise
    applyEKFUpdate(H, z_meas, R);
}

float EKF::clip(float value, float min_value, float max_value) {
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

void EKF::updateOpticalFlow(float vx_meas, float vy_meas) {
    const float max_velocity = 2.0f; // max velocity (positive and negative)

    vx_meas = clip(vx_meas, -max_velocity, max_velocity);
    vy_meas = clip(vy_meas, -max_velocity, max_velocity);

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

    // --- Lightweight Joseph form update ---
    for (int i = 0; i < 9; ++i) {
        for (int j = 0; j < 9; ++j) {
            P[i][j] = P[i][j] - K[i] * H[j] * P[i][j];
        }
    }

    for (int i = 0; i < 9; ++i) {
        for (int j = 0; j < 9; ++j) {
            P[i][j] += K[i] * R * K[j];
        }
    }
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
