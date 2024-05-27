/*
 * kont_lqr_pid.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "kont_lqr_pid".
 *
 * Model version              : 1.8
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Thu Mar 28 09:59:43 2024
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "kont_lqr_pid.h"
#include "rtwtypes.h"
#include <cstring>
#include <emmintrin.h>
#include "kont_lqr_pid_private.h"

extern "C"
{

#include "rt_nonfinite.h"

}

/* Model step function */
void kont_lqr_pid::step()
{
  /* DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  std::memcpy(&kont_lqr_pid_B.DiscreteTimeIntegrator[0],
              &kont_lqr_pid_DW.DiscreteTimeIntegrator_DSTATE[0], sizeof(real_T) <<
              3U);
  for (int32_T i{0}; i <= 6; i += 2) {
    __m128d tmp;
    __m128d tmp_0;

    /* Constant: '<Root>/Constant' incorporates:
     *  Sum: '<Root>/Sum7'
     */
    tmp = _mm_loadu_pd(&kont_lqr_pid_P.Constant_Value[i]);

    /* Sum: '<Root>/Sum7' */
    tmp_0 = _mm_loadu_pd(&kont_lqr_pid_B.DiscreteTimeIntegrator[i]);
    tmp = _mm_sub_pd(tmp, tmp_0);

    /* Sum: '<Root>/Sum7' */
    _mm_storeu_pd(&kont_lqr_pid_B.Sum7[i], tmp);
  }

  for (int32_T i_0{0}; i_0 < 3; i_0++) {
    /* Gain: '<Root>/Gain' */
    kont_lqr_pid_B.Gain[i_0] = 0.0;
    for (int32_T i{0}; i < 8; i++) {
      kont_lqr_pid_B.Gain[i_0] += kont_lqr_pid_P.Kd[3 * i + i_0] *
        kont_lqr_pid_B.Sum7[i];
    }

    /* End of Gain: '<Root>/Gain' */
  }

  for (int32_T i{0}; i < 8; i++) {
    /* Gain: '<Root>/Gain1' */
    kont_lqr_pid_B.Gain1[i] = 0.0;
    kont_lqr_pid_B.Gain1[i] += kont_lqr_pid_P.B[i] * kont_lqr_pid_B.Gain[0];
    kont_lqr_pid_B.Gain1[i] += kont_lqr_pid_P.B[i + 8] * kont_lqr_pid_B.Gain[1];
    kont_lqr_pid_B.Gain1[i] += kont_lqr_pid_P.B[i + 16] * kont_lqr_pid_B.Gain[2];

    /* Gain: '<Root>/Gain2' */
    kont_lqr_pid_B.Gain2[i] = 0.0;
    for (int32_T i_0{0}; i_0 < 8; i_0++) {
      kont_lqr_pid_B.Gain2[i] += kont_lqr_pid_P.A[(i_0 << 3) + i] *
        kont_lqr_pid_B.DiscreteTimeIntegrator[i_0];
    }

    /* End of Gain: '<Root>/Gain2' */

    /* Sum: '<Root>/Sum1' */
    kont_lqr_pid_B.Sum1[i] = kont_lqr_pid_B.Gain1[i] + kont_lqr_pid_B.Gain2[i];

    /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
    kont_lqr_pid_DW.DiscreteTimeIntegrator_DSTATE[i] +=
      kont_lqr_pid_P.DiscreteTimeIntegrator_gainval * kont_lqr_pid_B.Sum1[i];
  }

  /* Matfile logging */
  rt_UpdateTXYLogVars((&kont_lqr_pid_M)->rtwLogInfo, (&(&kont_lqr_pid_M)
    ->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.01s, 0.0s] */
    if ((rtmGetTFinal((&kont_lqr_pid_M))!=-1) &&
        !((rtmGetTFinal((&kont_lqr_pid_M))-(&kont_lqr_pid_M)->Timing.taskTime0) >
          (&kont_lqr_pid_M)->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus((&kont_lqr_pid_M), "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++(&kont_lqr_pid_M)->Timing.clockTick0)) {
    ++(&kont_lqr_pid_M)->Timing.clockTickH0;
  }

  (&kont_lqr_pid_M)->Timing.taskTime0 = (&kont_lqr_pid_M)->Timing.clockTick0 * (
    &kont_lqr_pid_M)->Timing.stepSize0 + (&kont_lqr_pid_M)->Timing.clockTickH0 *
    (&kont_lqr_pid_M)->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void kont_lqr_pid::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));
  rtmSetTFinal((&kont_lqr_pid_M), 10.0);
  (&kont_lqr_pid_M)->Timing.stepSize0 = 0.01;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (nullptr);
    (&kont_lqr_pid_M)->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo((&kont_lqr_pid_M)->rtwLogInfo, (nullptr));
    rtliSetLogXSignalPtrs((&kont_lqr_pid_M)->rtwLogInfo, (nullptr));
    rtliSetLogT((&kont_lqr_pid_M)->rtwLogInfo, "tout");
    rtliSetLogX((&kont_lqr_pid_M)->rtwLogInfo, "");
    rtliSetLogXFinal((&kont_lqr_pid_M)->rtwLogInfo, "");
    rtliSetLogVarNameModifier((&kont_lqr_pid_M)->rtwLogInfo, "rt_");
    rtliSetLogFormat((&kont_lqr_pid_M)->rtwLogInfo, 4);
    rtliSetLogMaxRows((&kont_lqr_pid_M)->rtwLogInfo, 0);
    rtliSetLogDecimation((&kont_lqr_pid_M)->rtwLogInfo, 1);
    rtliSetLogY((&kont_lqr_pid_M)->rtwLogInfo, "");
    rtliSetLogYSignalInfo((&kont_lqr_pid_M)->rtwLogInfo, (nullptr));
    rtliSetLogYSignalPtrs((&kont_lqr_pid_M)->rtwLogInfo, (nullptr));
  }

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime((&kont_lqr_pid_M)->rtwLogInfo, 0.0,
    rtmGetTFinal((&kont_lqr_pid_M)), (&kont_lqr_pid_M)->Timing.stepSize0,
    (&rtmGetErrorStatus((&kont_lqr_pid_M))));

  /* InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  for (int32_T i{0}; i < 8; i++) {
    kont_lqr_pid_DW.DiscreteTimeIntegrator_DSTATE[i] =
      kont_lqr_pid_P.DiscreteTimeIntegrator_IC;
  }

  /* End of InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
}

/* Model terminate function */
void kont_lqr_pid::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
kont_lqr_pid::kont_lqr_pid() :
  kont_lqr_pid_B(),
  kont_lqr_pid_DW(),
  kont_lqr_pid_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
kont_lqr_pid::~kont_lqr_pid()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_kont_lqr_pid_T * kont_lqr_pid::getRTM()
{
  return (&kont_lqr_pid_M);
}
