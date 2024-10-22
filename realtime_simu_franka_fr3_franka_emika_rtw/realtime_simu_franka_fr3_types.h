/*
 * realtime_simu_franka_fr3_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "realtime_simu_franka_fr3".
 *
 * Model version              : 8.276
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Tue Oct 22 15:30:43 2024
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_realtime_simu_franka_fr3_types_h_
#define RTW_HEADER_realtime_simu_franka_fr3_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_robot_model_
#define DEFINED_TYPEDEF_FOR_robot_model_

struct robot_model
{
  real_T q[7];
  real_T q_p[7];
  real_T q_pp[7];
  real_T H[16];
  real_T J[42];
  real_T J_p[42];
  real_T M[49];
  real_T C_rnea[7];
  real_T C[49];
  real_T g[7];
}

;

#endif

#ifndef DEFINED_TYPEDEF_FOR_x_d_
#define DEFINED_TYPEDEF_FOR_x_d_

struct x_d
{
  real_T p_d[3];
  real_T p_d_p[3];
  real_T p_d_pp[3];
  real_T Phi_d[3];
  real_T Phi_d_p[3];
  real_T Phi_d_pp[3];
  real_T R_d[9];
  real_T q_d[4];
  real_T q_d_p[4];
  real_T q_d_pp[4];
  real_T omega_d[3];
  real_T omega_d_p[3];
  real_T alpha_d;
  real_T alpha_d_p;
  real_T alpha_d_pp;
  real_T rot_ax_d[3];
  real_T alpha_d_offset;
  real_T q_d_rel[4];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_OnOf2gXFGCeQ1kokKguxcC_
#define DEFINED_TYPEDEF_FOR_struct_OnOf2gXFGCeQ1kokKguxcC_

struct struct_OnOf2gXFGCeQ1kokKguxcC
{
  real_T Kd1[36];
  real_T Kp1[36];
  real_T q_n[7];
  real_T K_n[49];
  real_T D_n[49];
  real_T k_n_nl[49];
  real_T nl_spring_threshold[7];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_jfblZltmyovPlsjcFJPP5C_
#define DEFINED_TYPEDEF_FOR_struct_jfblZltmyovPlsjcFJPP5C_

struct struct_jfblZltmyovPlsjcFJPP5C
{
  real_T D_d[36];
  real_T K_d[36];
  real_T D_d_jointspace[49];
  real_T K_d_jointspace[49];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_Fkz9JfHurrLZtu6guaEzCB_
#define DEFINED_TYPEDEF_FOR_struct_Fkz9JfHurrLZtu6guaEzCB_

struct struct_Fkz9JfHurrLZtu6guaEzCB
{
  real_T mode;
  real_T k;
  real_T W_bar_N[7];
  real_T W_E[36];
  real_T eps;
  real_T eps_collinear;
  real_T lambda_min;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_RCS9FomuFmdbboJvGAYmbB_
#define DEFINED_TYPEDEF_FOR_struct_RCS9FomuFmdbboJvGAYmbB_

struct struct_RCS9FomuFmdbboJvGAYmbB
{
  struct_OnOf2gXFGCeQ1kokKguxcC ct;
  struct_jfblZltmyovPlsjcFJPP5C pd;
  struct_Fkz9JfHurrLZtu6guaEzCB regularization;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_BbQI3KfNspcpNIEjGA7IuG_
#define DEFINED_TYPEDEF_FOR_struct_BbQI3KfNspcpNIEjGA7IuG_

struct struct_BbQI3KfNspcpNIEjGA7IuG
{
  real_T t[10026];
  real_T p_d[150390];
  real_T p_d_p[150390];
  real_T p_d_pp[150390];
  real_T Phi_d[150390];
  real_T Phi_d_p[150390];
  real_T Phi_d_pp[150390];
  real_T R_d[451170];
  real_T q_d[200520];
  real_T q_d_p[200520];
  real_T q_d_pp[200520];
  real_T omega_d[150390];
  real_T omega_d_p[150390];
  real_T alpha_d[50130];
  real_T alpha_d_p[50130];
  real_T alpha_d_pp[50130];
  real_T rot_ax_d[150390];
  real_T alpha_d_offset[50130];
  real_T q_d_rel[200520];
  real_T N;
};

#endif

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
#define SS_UINT64                      25
#endif

#ifndef SS_INT64
#define SS_INT64                       26
#endif

/* Parameters (default storage) */
typedef struct P_realtime_simu_franka_fr3_T_ P_realtime_simu_franka_fr3_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_realtime_simu_franka_fr3_T
  RT_MODEL_realtime_simu_franka_fr3_T;

#endif                        /* RTW_HEADER_realtime_simu_franka_fr3_types_h_ */
