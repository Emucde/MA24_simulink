/*
 * joint_impedance_control_data.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "joint_impedance_control".
 *
 * Model version              : 1.0
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Fri Nov 22 16:36:31 2024
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "joint_impedance_control.h"

/* Block parameters (default storage) */
P_joint_impedance_control_T joint_impedance_control_P = {
  /* Expression: 0
   * Referenced by: '<S3>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S3>/Constant2'
   */
  0.0,

  /* Expression: 0.05
   * Referenced by: '<S2>/radius'
   */
  0.05,

  /* Expression: 1
   * Referenced by: '<S2>/constant'
   */
  1.0,

  /* Expression: 0.25
   * Referenced by: '<S2>/vel_max'
   */
  0.25,

  /* Expression: 0
   * Referenced by: '<S2>/current_velocity'
   */
  0.0,

  /* Expression: 2.0
   * Referenced by: '<S2>/acceleration_time'
   */
  2.0,

  /* Expression: 0
   * Referenced by: '<S3>/Delay'
   */
  0.0,

  /* Expression: 20.0
   * Referenced by: '<S2>/run_time'
   */
  20.0,

  /* Expression: 0
   * Referenced by: '<S2>/constant0'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S2>/constant1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S2>/angle'
   */
  0.0,

  /* Expression: 2*pi
   * Referenced by: '<S2>/constant2'
   */
  6.2831853071795862,

  /* Expression: [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]
   * Referenced by: '<S1>/k_gains'
   */
  { 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0 },

  /* Expression: [50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0]
   * Referenced by: '<S1>/d_gains'
   */
  { 50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0 },

  /* Expression: collision_thresholds
   * Referenced by: '<Root>/Apply Control'
   */
  { 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0 },

  /* Expression: joint_impedance
   * Referenced by: '<Root>/Apply Control'
   */
  { 3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0 },

  /* Expression: cartesian_impedance
   * Referenced by: '<Root>/Apply Control'
   */
  { 3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0 },

  /* Expression: load_inertia
   * Referenced by: '<Root>/Apply Control'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001 },

  /* Expression: EE_T_K
   * Referenced by: '<Root>/Apply Control'
   */
  { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    1.0 },

  /* Expression: init_joint_configuration
   * Referenced by: '<Root>/Apply Control'
   */
  { 0.0, -0.78539816339744828, 0.0, -2.3561944901923448, 0.0, 1.5707963267948966,
    0.78539816339744828 },

  /* Expression: false
   * Referenced by: '<S2>/velocity_amplitude7'
   */
  false,

  /* Expression: true
   * Referenced by: '<S2>/velocity_amplitude1'
   */
  true
};
