#ifndef DEAD_RECKONING_POSITION_H
#define DEAD_RECKONING_POSITION_H

#include <array>

// Convert flow_x and flow_y (in pixels/frame) to velocities (in m/s)
void flowToVelocityXY(float flow_x, float flow_y, float heightInMeters, float& vx, float& vy,
                      float fps = 121.0f, float scalePerMeter = 21.9f);

class PositionEstimator {
public:
    PositionEstimator(float dt);

    void update(const std::array<float, 3>& acc,
                float pitch, float roll,
                float lidar_z,
                float flow_x,
                float flow_y);

    std::array<float, 3> getPosition() const;
    std::array<float, 3> getVelocity() const;

private:
    float dt;
    const float g = 9.81f;

    // Filter constants
    float beta = 0.1f; // 5% IMU, 95% Optical flow
    float epsilon = 0.8f; // 80% IMU, 20% Optical flow
    float alpha_acc = 0.5f;
    float flow_scale = 0.001f;

    // State
    std::array<float, 3> position = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> velocity = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> acc_filtered = {0.0f, 0.0f, 0.0f};

    float prev_lidar_z = 0.0f;
    bool first_update = true;
};

#endif
