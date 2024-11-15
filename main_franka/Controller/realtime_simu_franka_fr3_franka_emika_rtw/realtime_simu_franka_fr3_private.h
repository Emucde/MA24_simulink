/*
 * realtime_simu_franka_fr3_private.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "realtime_simu_franka_fr3".
 *
 * Model version              : 8.532
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Fri Nov 15 19:10:54 2024
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

extern real_T rt_hypotd_snf(real_T u0, real_T u1);
extern int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator);
extern "C" void s_function_opti_MPC8(SimStruct *rts);
extern "C" void shm_reader_sfun(SimStruct *rts);
extern "C" void shm_writer_sfun(SimStruct *rts);
extern "C" void s_function_opti_robot_model_bus_fun(SimStruct *rts);
extern void realtime_simu_franka_f_CTRegler(B_CTRegler_realtime_simu_fran_T
  *localB);
extern void realtime__fixedjointfeedforward(B_fixedjointfeedforward_realt_T
  *localB);

#endif                      /* RTW_HEADER_realtime_simu_franka_fr3_private_h_ */
