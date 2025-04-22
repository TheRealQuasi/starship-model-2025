#include "EKF_C.h"
#include <cstring>  // for strcmp
#include "mex.h"

static EKF ekf;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    char command[64];
    mxGetString(prhs[0], command, sizeof(command));

    if (strcmp(command, "predict") == 0) {
        double *acc = mxGetPr(prhs[1]);
        double *gyro = mxGetPr(prhs[2]);
        double dt = mxGetScalar(prhs[3]);

        Vector3 accVec{static_cast<float>(acc[0]), static_cast<float>(acc[1]), static_cast<float>(acc[2])};
        Vector3 gyroVec{static_cast<float>(gyro[0]), static_cast<float>(gyro[1]), static_cast<float>(gyro[2])};
        ekf.predict(accVec, gyroVec, static_cast<float>(dt));
    } 
    else if (strcmp(command, "updateLidar") == 0) {
        double z_meas = mxGetScalar(prhs[1]);
        ekf.updateLidar(static_cast<float>(z_meas));
    } 
    else if (strcmp(command, "updateOpticalFlow") == 0) {
        double vx_meas = mxGetScalar(prhs[1]);
        double vy_meas = mxGetScalar(prhs[2]);
        ekf.updateOpticalFlow(static_cast<float>(vx_meas), static_cast<float>(vy_meas));
    }
    else if (strcmp(command, "updateIMU") == 0) {
        double *acc = mxGetPr(prhs[1]);
        double *gyro = mxGetPr(prhs[2]);
        ekf.updateIMU(
            {static_cast<float>(acc[0]), static_cast<float>(acc[1]), static_cast<float>(acc[2])},
            {static_cast<float>(gyro[0]), static_cast<float>(gyro[1]), static_cast<float>(gyro[2])}
        );
    }
    else if (strcmp(command, "getPosition") == 0) {
        Vector3 pos = ekf.getPosition();
        plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
        double* out = mxGetPr(plhs[0]);
        out[0] = pos.x;
        out[1] = pos.y;
        out[2] = pos.z;
    }
    else if (strcmp(command, "getVelocity") == 0) {
        Vector3 vel = ekf.getVelocity();
        plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
        double* out = mxGetPr(plhs[0]);
        out[0] = vel.x;
        out[1] = vel.y;
        out[2] = vel.z;
    }
    else if (strcmp(command, "getOrientation") == 0) {
        Vector2 ori = ekf.getOrientation();
        plhs[0] = mxCreateDoubleMatrix(2, 1, mxREAL);
        double* out = mxGetPr(plhs[0]);
        out[0] = ori.x;
        out[1] = ori.y;
    }
    else {
        mexErrMsgTxt("Unknown command");
    }
}
