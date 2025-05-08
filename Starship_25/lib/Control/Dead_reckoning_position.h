#ifndef DEAD_RECKONING_POSITION_H
#define DEAD_RECKONING_POSITION_H

#include <array>

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
    float beta = 0.2f;
    float gamma = 0.8f;
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
