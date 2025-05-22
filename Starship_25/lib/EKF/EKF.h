#ifndef EKF_H
#define EKF_H

#include <Arduino.h>

const int STATE_SIZE = 10;
const int MEAS_SIZE = 5;
const int CONTROL_SIZE = 4;

class EKF {
public:
    EKF(float dt);

    void init();
    void predict();
    void update(const float z[MEAS_SIZE]);

    const float* getState() const;

private:
    float dt;

    float x_est[STATE_SIZE];
    float P[STATE_SIZE][STATE_SIZE];
    float A[STATE_SIZE][STATE_SIZE];
    float B[STATE_SIZE][CONTROL_SIZE];
    float H[MEAS_SIZE][STATE_SIZE];
    float Q[STATE_SIZE][STATE_SIZE];
    float R[MEAS_SIZE][MEAS_SIZE];

    void matMul(const float* A, const float* B, float* C, int m, int n, int p);
    void matAdd(const float* A, const float* B, float* C, int m, int n);
    void matSub(const float* A, const float* B, float* C, int m, int n);
    void matTrans(const float* A, float* B, int m, int n);
    void matIdentity(float* A, int n);
    void matCopy(const float* A, float* B, int m, int n);
    bool matInv5x5(const float A[MEAS_SIZE][MEAS_SIZE], float A_inv[MEAS_SIZE][MEAS_SIZE]);  // manual 5x5 inversion
};

#endif
