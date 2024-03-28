/*
 * kont_lqr_pid.h
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

#ifndef RTW_HEADER_kont_lqr_pid_h_
#define RTW_HEADER_kont_lqr_pid_h_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "kont_lqr_pid_types.h"
#include <cfloat>

extern "C"
{

#include "rt_nonfinite.h"

}

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

/* Block signals (default storage) */
struct B_kont_lqr_pid_T {
  real_T DiscreteTimeIntegrator[8];    /* '<Root>/Discrete-Time Integrator' */
  real_T Sum7[8];                      /* '<Root>/Sum7' */
  real_T Gain[3];                      /* '<Root>/Gain' */
  real_T Gain1[8];                     /* '<Root>/Gain1' */
  real_T Gain2[8];                     /* '<Root>/Gain2' */
  real_T Sum1[8];                      /* '<Root>/Sum1' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_kont_lqr_pid_T {
  real_T DiscreteTimeIntegrator_DSTATE[8];/* '<Root>/Discrete-Time Integrator' */
};

/* Parameters (default storage) */
struct P_kont_lqr_pid_T_ {
  real_T A[64];                        /* Variable: A
                                        * Referenced by: '<Root>/Gain2'
                                        */
  real_T B[24];                        /* Variable: B
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T Kd[24];                       /* Variable: Kd
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T DiscreteTimeIntegrator_gainval;
                           /* Computed Parameter: DiscreteTimeIntegrator_gainval
                            * Referenced by: '<Root>/Discrete-Time Integrator'
                            */
  real_T DiscreteTimeIntegrator_IC;    /* Expression: 0
                                        * Referenced by: '<Root>/Discrete-Time Integrator'
                                        */
  real_T Constant_Value[8];            /* Expression: [0;0;0;0;0;0;5;0]
                                        * Referenced by: '<Root>/Constant'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_kont_lqr_pid_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Class declaration for model kont_lqr_pid */
class kont_lqr_pid final
{
  /* public data and function members */
 public:
  /* Copy Constructor */
  kont_lqr_pid(kont_lqr_pid const&) = delete;

  /* Assignment Operator */
  kont_lqr_pid& operator= (kont_lqr_pid const&) & = delete;

  /* Move Constructor */
  kont_lqr_pid(kont_lqr_pid &&) = delete;

  /* Move Assignment Operator */
  kont_lqr_pid& operator= (kont_lqr_pid &&) = delete;

  /* Real-Time Model get method */
  RT_MODEL_kont_lqr_pid_T * getRTM();

  /* model start function */
  void start();

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  static void terminate();

  /* Constructor */
  kont_lqr_pid();

  /* Destructor */
  ~kont_lqr_pid();

  /* private data and function members */
 private:
  /* Block signals */
  B_kont_lqr_pid_T kont_lqr_pid_B;

  /* Block states */
  DW_kont_lqr_pid_T kont_lqr_pid_DW;

  /* Tunable parameters */
  static P_kont_lqr_pid_T kont_lqr_pid_P;

  /* Real-Time Model */
  RT_MODEL_kont_lqr_pid_T kont_lqr_pid_M;
};

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Zero-Order Hold' : Eliminated since input and output rates are identical
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'kont_lqr_pid'
 */
#endif                                 /* RTW_HEADER_kont_lqr_pid_h_ */
