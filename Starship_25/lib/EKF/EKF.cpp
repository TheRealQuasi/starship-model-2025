#include "EKF.h"
#include <math.h>

EKF::EKF(float timestep) : dt(timestep) {}

void EKF::init() {
    memset(x_est, 0, sizeof(x_est));
    memset(P, 0, sizeof(P));
    memset(A, 0, sizeof(A));
    memset(B, 0, sizeof(B));
    memset(H, 0, sizeof(H));
    memset(Q, 0, sizeof(Q));
    memset(R, 0, sizeof(R));

    // Init P
    for (int i = 0; i < STATE_SIZE; i++) {
        P[i][i] = 0.1f;
        Q[i][i] = 0.01f;
    }

    // Init A
    A[0][1] = 1;
    A[1][7] = 9.81f / 2;
    A[2][3] = 1;
    A[4][9] = 9.81f / 2;
    A[4][5] = 1;
    A[6][7] = 1;
    A[8][9] = 1;

    // Init B
    float m = 2.5;
    float Iy = 0.226;
    float Ix = 0.215;
    float h1 = 0.456;
    float h2 = 0.331;

    B[1][0] = 1.0f / m;
    B[1][1] = 1.0f / m;
    B[4][3] = 1.0f / m;
    B[7][2] = m * 9.81f * h1 / (2 * Iy);
    B[9][3] = m * 9.81f * h2 / (2 * Ix);

    // Init H
    H[0][1] = 1;
    H[1][3] = 1;
    H[2][8] = 1;
    H[3][9] = 1;
    H[4][6] = 1;

    // Init R
    R[0][0] = 0.05f;
    R[1][1] = 0.05f;
    R[2][2] = 0.01f;
    R[3][3] = 0.01f;
    R[4][4] = 0.1f;
}

void EKF::predict() {
    // x_est = A * x_est + B * u
    float Ax[STATE_SIZE];
    for (int i = 0; i < STATE_SIZE; i++) {
        Ax[i] = 0.0f;
        for (int j = 0; j < STATE_SIZE; j++) {
            Ax[i] += A[i][j] * x_est[j];
        }
    }
    for (int i = 0; i < STATE_SIZE; i++) {
        x_est[i] += dt * Ax[i];
    }

    // u = [u1, u2, u3, u4]
    float AP[STATE_SIZE][STATE_SIZE], ATP[STATE_SIZE][STATE_SIZE], P_pred[STATE_SIZE][STATE_SIZE];
    // AP = A * P
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            AP[i][j] = 0;
            for (int k = 0; k < STATE_SIZE; k++) {
                AP[i][j] += A[i][k] * P[k][j];
            }
        }
    }

    // ATP = AP * A'
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            ATP[i][j] = 0;
            for (int k = 0; k < STATE_SIZE; k++) {
                ATP[i][j] += AP[i][k] * A[j][k]; // A'
            }
            P_pred[i][j] = ATP[i][j] + Q[i][j];
        }
    }

    // P = ATP + Q
    memcpy(P, P_pred, sizeof(P));
}

void EKF::update(const float z[MEAS_SIZE]) {
    // z = [z1, z2, z3, z4, z5]
    float Hx[MEAS_SIZE];
    for (int i = 0; i < MEAS_SIZE; i++) {
        Hx[i] = 0;
        for (int j = 0; j < STATE_SIZE; j++) {
            Hx[i] += H[i][j] * x_est[j];
        }
    }

    // y = z - Hx
    float y[MEAS_SIZE];
    for (int i = 0; i < MEAS_SIZE; i++) {
        y[i] = z[i] - Hx[i];
    }

    // S = H * P * H' + R (manual calc)
    float S[MEAS_SIZE][MEAS_SIZE] = {0};
    for (int i = 0; i < MEAS_SIZE; i++) {
        for (int j = 0; j < MEAS_SIZE; j++) {
            for (int k = 0; k < STATE_SIZE; k++) {
                for (int l = 0; l < STATE_SIZE; l++) {
                    S[i][j] += H[i][k] * P[k][l] * H[j][l];
                }
            }
            S[i][j] += R[i][j];
        }
    }

    // S_inv = S^-1 (manual calc)
    float S_inv[MEAS_SIZE][MEAS_SIZE];
    if (!matInv5x5(S, S_inv)) return;

    // K = P * H' * S^-1
    float K[STATE_SIZE][MEAS_SIZE] = {0};
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < MEAS_SIZE; j++) {
            for (int k = 0; k < STATE_SIZE; k++) {
                for (int l = 0; l < MEAS_SIZE; l++) {
                    K[i][j] += P[i][k] * H[l][k] * S_inv[l][j]; // H'
                }
            }
        }
    }

    // x_est = x_est + K * y
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < MEAS_SIZE; j++) {
            x_est[i] += K[i][j] * y[j];
        }
    }

    // P = (I - K*H)*P
    float KH[STATE_SIZE][STATE_SIZE] = {0};
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            for (int k = 0; k < MEAS_SIZE; k++) {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    float I[STATE_SIZE][STATE_SIZE] = {0};
    for (int i = 0; i < STATE_SIZE; i++) I[i][i] = 1;

    float I_KH[STATE_SIZE][STATE_SIZE];
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            I_KH[i][j] = I[i][j] - KH[i][j];
        }
    }

    float P_new[STATE_SIZE][STATE_SIZE] = {0};
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            for (int k = 0; k < STATE_SIZE; k++) {
                P_new[i][j] += I_KH[i][k] * P[k][j];
            }
        }
    }

    memcpy(P, P_new, sizeof(P));
}

const float* EKF::getState() const {
    return x_est;
}

// Dummy inverse for 5x5 matrix (TODO: implement full inversion or use external library)
bool EKF::matInv5x5(const float A[5][5], float A_inv[5][5]) {
    // Placeholder: identity for now
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            A_inv[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    return true;
}
