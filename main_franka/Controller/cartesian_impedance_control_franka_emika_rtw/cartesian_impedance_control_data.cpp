/*
 * cartesian_impedance_control_data.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "cartesian_impedance_control".
 *
 * Model version              : 8.158
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Mon Oct  7 11:17:43 2024
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "cartesian_impedance_control.h"

/* Block parameters (default storage) */
P_cartesian_impedance_control_T cartesian_impedance_control_P = {
  /* Variable: q_init
   * Referenced by: '<S3>/Constant'
   */
  { 0.0, -0.78539816339744828, 0.0, -2.3561944901923448, 0.0, 1.5707963267948966,
    0.78539816339744828 },

  /* Expression: 0.001
   * Referenced by: '<S3>/Switch'
   */
  0.001,

  /* Expression: [100 200 500 200 50 50 10]
   * Referenced by: '<Root>/Constant8'
   */
  { 100.0, 200.0, 500.0, 200.0, 50.0, 50.0, 10.0 },

  /* Expression: [0 0 0 0 0 0 0]
   * Referenced by: '<Root>/Constant9'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4]
   * Referenced by: '<Root>/q_d_3'
   */
  { 0.0, -0.78539816339744828, 0.0, -2.3561944901923448, 0.0, 1.5707963267948966,
    0.78539816339744828 },

  /* Expression: 1000
   * Referenced by: '<S3>/Rate Limiter'
   */
  1000.0,

  /* Expression: -1000
   * Referenced by: '<S3>/Rate Limiter'
   */
  -1000.0,

  /* Expression: 0
   * Referenced by: '<S3>/Rate Limiter'
   */
  0.0,

  /* Expression: collision_thresholds
   * Referenced by: '<S3>/Apply Control'
   */
  { 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0 },

  /* Expression: joint_impedance
   * Referenced by: '<S3>/Apply Control'
   */
  { 3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0 },

  /* Expression: cartesian_impedance
   * Referenced by: '<S3>/Apply Control'
   */
  { 3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0 },

  /* Expression: load_inertia
   * Referenced by: '<S3>/Apply Control'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001 },

  /* Expression: EE_T_K
   * Referenced by: '<S3>/Apply Control'
   */
  { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    1.0 },

  /* Expression: init_joint_configuration
   * Referenced by: '<S3>/Apply Control'
   */
  { 0.0, -0.78539816339744828, 0.0, -2.3561944901923448, 0.0, 1.5707963267948966,
    0.78539816339744828 },

  /* Expression: 15
   * Referenced by: '<Root>/Constant10'
   */
  15.0
};
