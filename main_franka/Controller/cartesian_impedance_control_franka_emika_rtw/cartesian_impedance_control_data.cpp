/*
 * cartesian_impedance_control_data.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "cartesian_impedance_control".
 *
 * Model version              : 8.68
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Wed Oct  2 17:07:10 2024
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
  /* Variable: ct_ctrl_param
   * Referenced by: '<Root>/CT controller'
   */
  {
    { 64.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 64.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      64.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 64.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      64.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 64.0 },

    { 1024.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1024.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1024.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1024.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1024.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1024.0 },
    3.0,
    0.01,

    { 0.00011088900000000002, 9.9856e-5, 0.00014745600000000002,
      7.743999999999999e-6, 1.1449e-5, 1.0691560000000002e-5 },

    { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0 },
    0.1,
    0.95,
    0.001,

    { 0.0, 0.0, 0.0, -1.5969499999999999, 0.0, 2.53075, 0.0 },

    { 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01 },

    { 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.2 },

    { 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0 },

    { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 }
  },

  /* Variable: q_init
   * Referenced by: '<S7>/Constant'
   */
  { 0.0, -0.78539816339744828, 0.0, -2.3561944901923448, 0.0, 1.5707963267948966,
    0.78539816339744828 },

  /* Mask Parameter: UDPReceivefromc1_localPort
   * Referenced by: '<Root>/UDP Receive from c 1'
   */
  8080,

  /* Mask Parameter: UDPSend_remotePort
   * Referenced by: '<S8>/UDP Send'
   */
  8081,

  /* Expression: 0.001
   * Referenced by: '<S7>/Switch'
   */
  0.001,

  /* Expression: 0
   * Referenced by: '<Root>/Memory'
   */
  0.0,

  /* Expression: zeros(6,1)
   * Referenced by: '<Root>/Constant3'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: 0
   * Referenced by: '<Root>/Memory1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Memory2'
   */
  0.0,

  /* Expression: diag([600, 600, 600, 50.0, 50.0, 50.0])
   * Referenced by: '<S2>/stiffness'
   */
  { 600.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 600.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    600.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0 },

  /* Expression: diag(2*sqrt([150.0, 150.0, 150.0, 50.0, 50.0, 50.0]))
   * Referenced by: '<S2>/damping'
   */
  { 24.494897427831781, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 24.494897427831781, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 24.494897427831781, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    14.142135623730951, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 14.142135623730951, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 14.142135623730951 },

  /* Expression: 5
   * Referenced by: '<Root>/Constant5'
   */
  5.0,

  /* Expression: 1
   * Referenced by: '<Root>/Constant4'
   */
  1.0,

  /* Expression: 1000
   * Referenced by: '<S7>/Rate Limiter'
   */
  1000.0,

  /* Expression: -1000
   * Referenced by: '<S7>/Rate Limiter'
   */
  -1000.0,

  /* Expression: 0
   * Referenced by: '<S7>/Rate Limiter'
   */
  0.0,

  /* Expression: collision_thresholds
   * Referenced by: '<S7>/Apply Control'
   */
  { 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0 },

  /* Expression: joint_impedance
   * Referenced by: '<S7>/Apply Control'
   */
  { 3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0 },

  /* Expression: cartesian_impedance
   * Referenced by: '<S7>/Apply Control'
   */
  { 3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0 },

  /* Expression: load_inertia
   * Referenced by: '<S7>/Apply Control'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001 },

  /* Expression: EE_T_K
   * Referenced by: '<S7>/Apply Control'
   */
  { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    1.0 },

  /* Expression: init_joint_configuration
   * Referenced by: '<S7>/Apply Control'
   */
  { 0.0, -0.78539816339744828, 0.0, -2.3561944901923448, 0.0, 1.5707963267948966,
    0.78539816339744828 }
};
