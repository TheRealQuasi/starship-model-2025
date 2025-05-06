#include "Dead_reckoning_position.h"
#include <cmath>

const float g = 9.81f;
const float alpha_acc = 0.9f;
const float beta = 0.85f;

void updatePosition(float ax, float ay, float az,
                    float roll, float pitch,
                    float dt, float lidar_z,
                    PositionState& state) {
    // --- Remove gravity using roll & pitch ---
    float cr = cos(roll), sr = sin(roll);
    float cp = cos(pitch), sp = sin(pitch);

    float g_x = -g * sp;
    float g_y = g * sr * cp;
    float g_z = g * cr * cp;

    float ax_nog = ax - g_x;
    float ay_nog = ay - g_y;
    float az_nog = az - g_z;

    // Low-pass filter
    state.acc_filtered[0] = alpha_acc * state.acc_filtered[0] + (1 - alpha_acc) * ax_nog;
    state.acc_filtered[1] = alpha_acc * state.acc_filtered[1] + (1 - alpha_acc) * ay_nog;
    state.acc_filtered[2] = alpha_acc * state.acc_filtered[2] + (1 - alpha_acc) * az_nog;

    // --- Z from lidar and acc ---
    float v_acc_z = state.z_vel + state.acc_filtered[2] * dt;
    static float last_lidar = lidar_z;
    float v_lidar_z = (lidar_z - last_lidar) / dt;
    last_lidar = lidar_z;

    state.z_vel = beta * v_acc_z + (1 - beta) * v_lidar_z;
    state.z_pos += state.z_vel * dt;

    // --- X/Y from acc ---
    state.x_vel += state.acc_filtered[0] * dt;
    state.y_vel += state.acc_filtered[1] * dt;

    state.x_pos += state.x_vel * dt;
    state.y_pos += state.y_vel * dt;
}
