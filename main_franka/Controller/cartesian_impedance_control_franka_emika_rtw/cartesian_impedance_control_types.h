/*
 * cartesian_impedance_control_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "cartesian_impedance_control".
 *
 * Model version              : 8.98
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Thu Oct  3 17:11:54 2024
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
#ifndef DEFINED_TYPEDEF_FOR_struct_KEa48IyRUxLtKgH405LUqB_
#define DEFINED_TYPEDEF_FOR_struct_KEa48IyRUxLtKgH405LUqB_

struct struct_KEa48IyRUxLtKgH405LUqB
{
  real_T T_sim;
  real_T Ta;
  real_T traj_buffer_size;
};

#endif

#ifndef SS_UINT64
#define SS_UINT64                      18
#endif

#ifndef SS_INT64
#define SS_INT64                       19
#endif

/* Parameters (default storage) */
typedef struct P_cartesian_impedance_control_T_ P_cartesian_impedance_control_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_cartesian_impedance_c_T RT_MODEL_cartesian_impedance__T;

#endif                     /* RTW_HEADER_cartesian_impedance_control_types_h_ */
