#ifndef MADGWICK6DOF_H
#define MADGWICK6DOF_H

#include <vector>

class Madgwick6DOF {
public:
    float q[4]; // quaternion
    float beta; // filter gain

    Madgwick6DOF();

    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    void getEulerAngles(float& roll, float& pitch, float& yaw) const;

    private:
        float invSqrt(float x) const;

};

#endif
