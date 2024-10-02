/*
 * cartesian_impedance_control_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "cartesian_impedance_control".
 *
 * Model version              : 8.47
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Wed Oct  2 11:53:02 2024
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_cartesian_impedance_control_types_h_
#define RTW_HEADER_cartesian_impedance_control_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_robot_model_
#define DEFINED_TYPEDEF_FOR_robot_model_

struct robot_model
{
  real_T q[6];
  real_T q_p[6];
  real_T q_pp[6];
  real_T H[16];
  real_T J[36];
  real_T J_p[36];
  real_T M[36];
  real_T C_rnea[6];
  real_T g[6];
}

;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_zgJGJvTOOGEq9ehmj4thTC_
#define DEFINED_TYPEDEF_FOR_struct_zgJGJvTOOGEq9ehmj4thTC_

struct struct_zgJGJvTOOGEq9ehmj4thTC
{
  real_T Ta;
};

#endif

#ifndef SS_UINT64
#define SS_UINT64                      19
#endif

#ifndef SS_INT64
#define SS_INT64                       20
#endif

/* Parameters (default storage) */
typedef struct P_cartesian_impedance_control_T_ P_cartesian_impedance_control_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_cartesian_impedance_control_T
  RT_MODEL_cartesian_impedance_control_T;

#endif                     /* RTW_HEADER_cartesian_impedance_control_types_h_ */
