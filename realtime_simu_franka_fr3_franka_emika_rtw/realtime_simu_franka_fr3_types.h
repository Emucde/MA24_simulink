/*
 * realtime_simu_franka_fr3_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "realtime_simu_franka_fr3".
 *
 * Model version              : 8.405
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Mon Oct 28 11:24:08 2024
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
#ifndef DEFINED_TYPEDEF_FOR_state_traj_bus_
#define DEFINED_TYPEDEF_FOR_state_traj_bus_

struct state_traj_bus
{
  real_T start;
  real_T reset;
  real_T stop;
}

;

#endif

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
};

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

#ifndef DEFINED_TYPEDEF_FOR_struct_bxvYlVJ0lAkCV5jmJsIaMD_
#define DEFINED_TYPEDEF_FOR_struct_bxvYlVJ0lAkCV5jmJsIaMD_

struct struct_bxvYlVJ0lAkCV5jmJsIaMD
{
  real_T N;
  real_T t[10051];
  real_T p_d[180918];
  real_T p_d_p[180918];
  real_T p_d_pp[180918];
  real_T Phi_d[180918];
  real_T Phi_d_p[180918];
  real_T Phi_d_pp[180918];
  real_T R_d[542754];
  real_T q_d[241224];
  real_T q_d_p[241224];
  real_T q_d_pp[241224];
  real_T omega_d[180918];
  real_T omega_d_p[180918];
  real_T alpha_d[60306];
  real_T alpha_d_p[60306];
  real_T alpha_d_pp[60306];
  real_T rot_ax_d[180918];
  real_T alpha_d_offset[60306];
  real_T q_d_rel[241224];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_3L8uQ7Ey3C8cjZ2X06zaaF_
#define DEFINED_TYPEDEF_FOR_struct_3L8uQ7Ey3C8cjZ2X06zaaF_

struct struct_3L8uQ7Ey3C8cjZ2X06zaaF
{
  real_T A[576];
  real_T b[96];
  real_T Ta;
  real_T Phi[576];
  real_T Gamma[96];
  uint8_T p_d_index[4];
  uint8_T p_d_p_index[4];
  uint8_T p_d_pp_index[4];
  uint8_T n_order;
  uint8_T n_input;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_xtY3FIkba7D2gFmP0VadqC_
#define DEFINED_TYPEDEF_FOR_struct_xtY3FIkba7D2gFmP0VadqC_

struct struct_xtY3FIkba7D2gFmP0VadqC
{
  struct_3L8uQ7Ey3C8cjZ2X06zaaF diff_filter;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_ja0GJBLe6lxkTz7h11KlJG_
#define DEFINED_TYPEDEF_FOR_struct_ja0GJBLe6lxkTz7h11KlJG_

struct struct_ja0GJBLe6lxkTz7h11KlJG
{
  real_T A[1764];
  real_T b[294];
  real_T Ta;
  real_T Phi[1764];
  real_T Gamma[294];
  uint8_T p_d_index[7];
  uint8_T p_d_p_index[7];
  uint8_T p_d_pp_index[7];
  uint8_T n_order;
  uint8_T n_input;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_7a8eCPA4nMdHfiggL8ApbC_
#define DEFINED_TYPEDEF_FOR_struct_7a8eCPA4nMdHfiggL8ApbC_

struct struct_7a8eCPA4nMdHfiggL8ApbC
{
  struct_ja0GJBLe6lxkTz7h11KlJG diff_filter_jointspace;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_VPDCMrDfdupDVy28b0hli_
#define DEFINED_TYPEDEF_FOR_struct_VPDCMrDfdupDVy28b0hli_

struct struct_VPDCMrDfdupDVy28b0hli
{
  real_T T;
  real_T omega;
  real_T phi;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_Zz654ZwXwtjsnsQgQLM11G_
#define DEFINED_TYPEDEF_FOR_struct_Zz654ZwXwtjsnsQgQLM11G_

struct struct_Zz654ZwXwtjsnsQgQLM11G
{
  struct_VPDCMrDfdupDVy28b0hli sin_poly;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_7GwfAYRjAWGpyfrsv1fnuH_
#define DEFINED_TYPEDEF_FOR_struct_7GwfAYRjAWGpyfrsv1fnuH_

struct struct_7GwfAYRjAWGpyfrsv1fnuH
{
  real_T start_index[6];
  real_T stop_index[6];
  real_T q_0[42];
  real_T q_0_p[42];
  real_T q_0_pp[42];
  real_T joint_points[126];
  real_T pose[126];
  real_T rotation[162];
  real_T rot_ax[54];
  real_T alpha[18];
  real_T time[18];
  real_T traj_type[6];
  struct_xtY3FIkba7D2gFmP0VadqC diff_filter[6];
  struct_7a8eCPA4nMdHfiggL8ApbC diff_filter_jointspace[6];
  struct_Zz654ZwXwtjsnsQgQLM11G sin_poly[6];
  real_T N_traj;
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
#define SS_UINT64                      29
#endif

#ifndef SS_INT64
#define SS_INT64                       30
#endif

/* Parameters (default storage) */
typedef struct P_realtime_simu_franka_fr3_T_ P_realtime_simu_franka_fr3_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_realtime_simu_franka_fr3_T
  RT_MODEL_realtime_simu_franka_fr3_T;

#endif                        /* RTW_HEADER_realtime_simu_franka_fr3_types_h_ */
