#ifndef POSITION_ESTIMATION_H
#define POSITION_ESTIMATION_H

struct PositionState {
    float x_vel = 0, y_vel = 0, z_vel = 0;
    float x_pos = 0, y_pos = 0, z_pos = 0;
    float acc_filtered[3] = {0, 0, 0}; // Exponential moving average
};

void updatePosition(float ax, float ay, float az,
                    float roll, float pitch,
                    float dt, float lidar_z,
                    PositionState& state);

#endif
