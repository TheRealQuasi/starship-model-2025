#include "Dead_reckoning_position.h"
#include <cmath>

PositionEstimator::PositionEstimator(float dt_) : dt(dt_) {}

void PositionEstimator::update(const std::array<float, 3>& acc,
                               float pitch, float roll,
                               float lidar_z,
                               float flow_x,
                               float flow_y)
{
    // --- Rotation matrices ---
    float cr = cos(roll), sr = sin(roll);
    float cp = cos(pitch), sp = sin(pitch);

    float R_x[3][3] = {
        {1, 0, 0},
        {0, cr, -sr},
        {0, sr, cr}
    };

    float R_y[3][3] = {
        {cp, 0, sp},
        {0, 1, 0},
        {-sp, 0, cp}
    };

    float R[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            R[i][j] = 0;
            for (int k = 0; k < 3; ++k)
                R[i][j] += R_x[i][k] * R_y[k][j];
        }

    // g_world = [0; 0; g]
    std::array<float, 3> g_body = {
        R[0][2] * g,
        R[1][2] * g,
        R[2][2] * g
    };

    std::array<float, 3> acc_nogravity = {
        acc[0] - g_body[0],
        acc[1] - g_body[1],
        acc[2] - g_body[2]
    };

    // --- Low-pass filter ---
    if (first_update) {
        acc_filtered = acc_nogravity;
        prev_lidar_z = lidar_z;
        first_update = false;
    } else {
        for (int i = 0; i < 3; ++i)
            acc_filtered[i] = alpha_acc * acc_filtered[i] + (1 - alpha_acc) * acc_nogravity[i];
    }

    // --- Z velocity: IMU + Lidar
    float v_acc_z = velocity[2] + acc_filtered[2] * dt;
    float v_lidar_z = (lidar_z - prev_lidar_z) / dt;
    prev_lidar_z = lidar_z;

    velocity[2] = beta * v_acc_z + (1 - beta) * v_lidar_z;
    position[2] += velocity[2] * dt;

    if (position[2] < 0.0f) position[2] = 0.0f;

    // --- X/Y velocity: IMU + optical flow
    float x_vel_imu = velocity[0] + acc_filtered[0] * dt;
    float y_vel_imu = velocity[1] + acc_filtered[1] * dt;

    float v_of_x = flow_x * flow_scale / dt;
    float v_of_y = flow_y * flow_scale / dt;

    velocity[0] = gamma * x_vel_imu + (1 - gamma) * v_of_x;
    velocity[1] = gamma * y_vel_imu + (1 - gamma) * v_of_y;

    position[0] += velocity[0] * dt;
    position[1] += velocity[1] * dt;
}

std::array<float, 3> PositionEstimator::getPosition() const {
    return position;
}

std::array<float, 3> PositionEstimator::getVelocity() const {
    return velocity;
}
