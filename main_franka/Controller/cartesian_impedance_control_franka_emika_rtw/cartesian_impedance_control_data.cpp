/*
 * cartesian_impedance_control_data.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "cartesian_impedance_control".
 *
 * Model version              : 8.244
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Thu Oct 17 16:20:08 2024
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
  /* Variable: ctrl_param
   * Referenced by:
   *   '<Root>/PD+ controller reduced'
   *   '<Root>/joint space control fixed q3'
   */
  {
    {
      { 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0 },

      { 2.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        2.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        2.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.25 },

      { 0.0, 0.0, 0.0, -1.5969499999999999, 0.0, 2.53075, 0.0 },

      { 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01 },

      { 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2 },

      { 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0 },

      { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 }
    },

    {
      { 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0 },

      { 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0 },

      { 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0 },

      { 2.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.25, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 2.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.25, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 2.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.25, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.25 }
    },

    {
      0.0,
      0.01,

      { 0.00011088900000000002, 9.9856e-5, 6.8062500000000011e-6,
        0.00014745600000000002, 7.743999999999999e-6, 1.1449e-5,
        1.0691560000000002e-5 },

      { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 },
      0.1,
      0.95,
      0.001
    }
  },

  /* Variable: q_init
   * Referenced by: '<S6>/Constant'
   */
  { 0.0, -0.78539816339744828, 0.0, -2.3561944901923448, 0.0, 1.5707963267948966,
    0.78539816339744828 },

  /* Expression: [100 200 500 200 50 50 10]
   * Referenced by: '<Root>/Constant8'
   */
  { 100.0, 200.0, 500.0, 200.0, 50.0, 50.0, 10.0 },

  /* Expression: [10 10 10 10 10 10 10]
   * Referenced by: '<Root>/Constant3'
   */
  { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 },

  /* Expression: zeros(1,7)
   * Referenced by: '<Root>/Constant7'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: 1e-2*[1 1 1 1 1 1 1]
   * Referenced by: '<Root>/Constant6'
   */
  { 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01 },

  /* Expression: 1
   * Referenced by: '<Root>/Constant10'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Constant11'
   */
  0.0,

  /* Computed Parameter: SFunction3_P1_Size
   * Referenced by: '<Root>/S-Function3'
   */
  { 1.0, 16.0 },

  /* Computed Parameter: SFunction3_P1
   * Referenced by: '<Root>/S-Function3'
   */
  { 100.0, 97.0, 116.0, 97.0, 95.0, 102.0, 114.0, 111.0, 109.0, 95.0, 112.0,
    121.0, 116.0, 104.0, 111.0, 110.0 },

  /* Computed Parameter: SFunction3_P2_Size
   * Referenced by: '<Root>/S-Function3'
   */
  { 1.0, 22.0 },

  /* Computed Parameter: SFunction3_P2
   * Referenced by: '<Root>/S-Function3'
   */
  { 100.0, 97.0, 116.0, 97.0, 95.0, 102.0, 114.0, 111.0, 109.0, 95.0, 112.0,
    121.0, 116.0, 104.0, 111.0, 110.0, 95.0, 118.0, 97.0, 108.0, 105.0, 100.0 },

  /* Expression: 0.001
   * Referenced by: '<S6>/Switch'
   */
  0.001,

  /* Computed Parameter: SFunction4_P1_Size
   * Referenced by: '<Root>/S-Function4'
   */
  { 1.0, 18.0 },

  /* Computed Parameter: SFunction4_P1
   * Referenced by: '<Root>/S-Function4'
   */
  { 100.0, 97.0, 116.0, 97.0, 95.0, 102.0, 114.0, 111.0, 109.0, 95.0, 115.0,
    105.0, 109.0, 117.0, 108.0, 105.0, 110.0, 107.0 },

  /* Computed Parameter: SFunction4_P2_Size
   * Referenced by: '<Root>/S-Function4'
   */
  { 1.0, 24.0 },

  /* Computed Parameter: SFunction4_P2
   * Referenced by: '<Root>/S-Function4'
   */
  { 100.0, 97.0, 116.0, 97.0, 95.0, 102.0, 114.0, 111.0, 109.0, 95.0, 115.0,
    105.0, 109.0, 117.0, 108.0, 105.0, 110.0, 107.0, 95.0, 118.0, 97.0, 108.0,
    105.0, 100.0 },

  /* Expression: zeros(7,1)
   * Referenced by: '<Root>/tau_in'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: 0*diag([1 1 1 1 1 1]);
   * Referenced by: '<Root>/cartesian damping'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: 3*diag([1 1 1 1 1 1]);
   * Referenced by: '<Root>/cartesian stiffness'
   */
  { 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 3.0 },

  /* Expression: 5
   * Referenced by: '<Root>/Constant5'
   */
  5.0,

  /* Expression: 1
   * Referenced by: '<Root>/Constant4'
   */
  1.0,

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
   * Referenced by: '<S6>/Rate Limiter'
   */
  1000.0,

  /* Expression: -1000
   * Referenced by: '<S6>/Rate Limiter'
   */
  -1000.0,

  /* Expression: 0
   * Referenced by: '<S6>/Rate Limiter'
   */
  0.0,

  /* Expression: collision_thresholds
   * Referenced by: '<S6>/Apply Control'
   */
  { 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0 },

  /* Expression: joint_impedance
   * Referenced by: '<S6>/Apply Control'
   */
  { 3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0 },

  /* Expression: cartesian_impedance
   * Referenced by: '<S6>/Apply Control'
   */
  { 3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0 },

  /* Expression: load_inertia
   * Referenced by: '<S6>/Apply Control'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001 },

  /* Expression: EE_T_K
   * Referenced by: '<S6>/Apply Control'
   */
  { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    1.0 },

  /* Expression: init_joint_configuration
   * Referenced by: '<S6>/Apply Control'
   */
  { 0.0, -0.78539816339744828, 0.0, -2.3561944901923448, 0.0, 1.5707963267948966,
    0.78539816339744828 },

  /* Computed Parameter: ManualSwitch_CurrentSetting
   * Referenced by: '<Root>/Manual Switch'
   */
  0U,

  /* Computed Parameter: ManualSwitch1_CurrentSetting
   * Referenced by: '<Root>/Manual Switch1'
   */
  1U,

  /* Computed Parameter: ManualSwitch3_CurrentSetting
   * Referenced by: '<Root>/Manual Switch3'
   */
  0U,

  /* Computed Parameter: ManualSwitch2_CurrentSetting
   * Referenced by: '<Root>/Manual Switch2'
   */
  0U
};
