/*
 * kont_lqr_pid_data.cpp
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

/* Block parameters (default storage) */
P_kont_lqr_pid_T kont_lqr_pid::kont_lqr_pid_P{
  /* Variable: A
   * Referenced by: '<Root>/Gain2'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -9.82, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -9.82, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0 },

  /* Variable: B
   * Referenced by: '<Root>/Gain1'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.6460905349794237, 9.82, 0.0,
    23.548618421052634, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.82, 0.0,
    15.699078947368424, 0.0, 0.0 },

  /* Variable: Kd
   * Referenced by: '<Root>/Gain'
   */
  { -1.8743349756759239E-16, -3.1622776601683915, -1.6441161350788012E-15,
    2.1580862534958093E-15, 14.514644265785588, 1.7624172549411392E-15,
    4.3904337043699538E-16, 3.3116147439717629, -2.7912014937578927E-16,
    -5.0181943488073532E-15, -3.4894359548097505E-15, -3.1622776601683551,
    5.9275307566636005E-15, 1.7177053626012792E-14, 17.125274736465244,
    1.9253667663288691E-15, 2.6251220876600811E-15, 4.6415768967945867,
    2.9246224072111531, -8.0341750942858506E-16, -8.6961018353738024E-15,
    2.1338735259526391, 5.965139274791142E-16, -4.9934899709295091E-15 },

  /* Computed Parameter: DiscreteTimeIntegrator_gainval
   * Referenced by: '<Root>/Discrete-Time Integrator'
   */
  0.01,

  /* Expression: 0
   * Referenced by: '<Root>/Discrete-Time Integrator'
   */
  0.0,

  /* Expression: [0;0;0;0;0;0;5;0]
   * Referenced by: '<Root>/Constant'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0 }
};
