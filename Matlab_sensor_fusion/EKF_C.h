#ifndef EKF_H
#define EKF_H

#include <array>

struct Vector3 {
    float x, y, z;
};

struct Vector2 {
    float x, y;
};

class EKF {
public:
    EKF();

    void predict(Vector3 acc, Vector3 gyro, float dt);
    void updateIMU(Vector3 acc_meas, Vector3 gyro_meas);
    void updateLidar(float z_meas);
    float clip(float value, float min_value, float max_value);
    void updateOpticalFlow(float vx_meas, float vy_meas);

    Vector3 getPosition() const;
    Vector3 getVelocity() const;
    Vector2 getOrientation() const;

private:
    std::array<float, 9> x;           // State vector
    std::array<std::array<float, 9>, 9> P;  // Covariance

    void applyEKFUpdate(const std::array<float, 9>& H, float z_meas, float R);

    float deg2rad(float deg);
};

#endif
