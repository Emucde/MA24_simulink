/*
 * realtime_simu_franka_fr3_private.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "realtime_simu_franka_fr3".
 *
 * Model version              : 8.652
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Mon Dec 30 18:11:40 2024
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
#include "realtime_simu_franka_fr3.h"
#include "realtime_simu_franka_fr3_types.h"

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

extern "C" void s_function_opti_ekf_fun(SimStruct *rts);
extern "C" void s_function_opti_robot_model_bus_fun(SimStruct *rts);
extern "C" void shm_reader_sfun(SimStruct *rts);
extern "C" void shm_writer_sfun(SimStruct *rts);
extern void realtime_simu_fra_Robotmodelbus(const real_T rtu_q[7], const real_T
  rtu_q_p[7], const real_T rtu_q_pp[7], const real_T rtu_H[16], const real_T
  rtu_J[42], const real_T rtu_J_p[42], const real_T rtu_M[49], const real_T
  rtu_C_rnea[7], const real_T rtu_C[49], const real_T rtu_g[7],
  B_Robotmodelbus_realtime_simu_T *localB);

#endif                      /* RTW_HEADER_realtime_simu_franka_fr3_private_h_ */
