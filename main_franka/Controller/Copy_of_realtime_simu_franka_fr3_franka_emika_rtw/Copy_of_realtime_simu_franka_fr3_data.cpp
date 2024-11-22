/*
 * Copy_of_realtime_simu_franka_fr3_data.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Copy_of_realtime_simu_franka_fr3".
 *
 * Model version              : 8.569
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Fri Nov 22 16:42:30 2024
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "Copy_of_realtime_simu_franka_fr3.h"

/* Block parameters (default storage) */
P_Copy_of_realtime_simu_frank_T Copy_of_realtime_simu_franka__P = {
  /* Expression: zeros(7,1)
   * Referenced by: '<Root>/Constant'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: collision_thresholds
   * Referenced by: '<S1>/Apply Control'
   */
  { 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0 },

  /* Expression: joint_impedance
   * Referenced by: '<S1>/Apply Control'
   */
  { 3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0 },

  /* Expression: cartesian_impedance
   * Referenced by: '<S1>/Apply Control'
   */
  { 3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0 },

  /* Expression: load_inertia
   * Referenced by: '<S1>/Apply Control'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001 },

  /* Expression: EE_T_K
   * Referenced by: '<S1>/Apply Control'
   */
  { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    1.0 },

  /* Expression: init_joint_configuration
   * Referenced by: '<S1>/Apply Control'
   */
  { 0.0, -0.78539816339744828, 0.0, -2.3561944901923448, 0.0, 1.5707963267948966,
    0.78539816339744828 }
};
