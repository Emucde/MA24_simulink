/*
 * realtime_simu_franka_fr3_private.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "realtime_simu_franka_fr3".
 *
 * Model version              : 8.19
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Sat Mar 15 13:36:29 2025
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_realtime_simu_franka_fr3_private_h_
#define RTW_HEADER_realtime_simu_franka_fr3_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "realtime_simu_franka_fr3_types.h"
#include "realtime_simu_franka_fr3.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTFinal
#define rtmSetTFinal(rtm, val)         ((rtm)->Timing.tFinal = (val))
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

extern real_T rt_hypotd_snf(real_T u0, real_T u1);
extern int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator);
extern "C" void s_function_opti_ekf_fun(SimStruct *rts);
extern "C" void s_function_opti_robot_model_bus_fun(SimStruct *rts);
extern "C" void shm_reader_sfun(SimStruct *rts);
extern "C" void shm_writer_sfun(SimStruct *rts);
extern void realtime_simu_getEKFjointvalues(const robot_model *rtu_robot_model,
  const real_T rtu_xk_hat[14], B_getEKFjointvalues_realtime__T *localB);
extern void realtime_simu_fra_Robotmodelbus(const real_T rtu_q[7], const real_T
  rtu_q_p[7], const real_T rtu_q_pp[7], const real_T rtu_H[16], const real_T
  rtu_J[42], const real_T rtu_J_p[42], const real_T rtu_M[49], const real_T
  rtu_C_rnea[7], const real_T rtu_C[49], const real_T rtu_g[7],
  B_Robotmodelbus_realtime_simu_T *localB);
extern void realtime_simu_f_zerofixedstates(const real_T rtu_q[7], const real_T
  rtu_q_p[7], const real_T rtu_q_pp[7], B_zerofixedstates_realtime_si_T *localB);
extern void realtime_simu_franka_fr3_noEKF(boolean_T rtu_Enable, const
  robot_model *rtu_In1, robot_model *rty_Out1, DW_noEKF_realtime_simu_franka_T
  *localDW);

#endif                      /* RTW_HEADER_realtime_simu_franka_fr3_private_h_ */
