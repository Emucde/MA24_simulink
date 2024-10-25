/*
 * realtime_simu_franka_fr3_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "realtime_simu_franka_fr3".
 *
 * Model version              : 8.398
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Fri Oct 25 15:00:00 2024
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
};

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

#ifndef DEFINED_TYPEDEF_FOR_struct_Yw0vgIQ6yZpeProSkAAsrC_
#define DEFINED_TYPEDEF_FOR_struct_Yw0vgIQ6yZpeProSkAAsrC_

struct struct_Yw0vgIQ6yZpeProSkAAsrC
{
  real_T MPC01[359];
  real_T MPC6[201];
  real_T MPC7[201];
  real_T MPC8[395];
  real_T MPC9[233];
  real_T MPC10[163];
  real_T MPC11[166];
  real_T MPC12[163];
  real_T MPC13[163];
  real_T MPC14[199];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_l1Nsuu0XOOc6U6jDqXw0mE_
#define DEFINED_TYPEDEF_FOR_struct_l1Nsuu0XOOc6U6jDqXw0mE_

struct struct_l1Nsuu0XOOc6U6jDqXw0mE
{
  real_T init_guess[460];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_4K1H1LNAOrvSOTHhKEJOg_
#define DEFINED_TYPEDEF_FOR_struct_4K1H1LNAOrvSOTHhKEJOg_

struct struct_4K1H1LNAOrvSOTHhKEJOg
{
  real_T init_guess[170];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_WOC7dUVd8i8zlETJkESm4F_
#define DEFINED_TYPEDEF_FOR_struct_WOC7dUVd8i8zlETJkESm4F_

struct struct_WOC7dUVd8i8zlETJkESm4F
{
  real_T init_guess[600];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_HynqmHhvVrqpd52uaZuCED_
#define DEFINED_TYPEDEF_FOR_struct_HynqmHhvVrqpd52uaZuCED_

struct struct_HynqmHhvVrqpd52uaZuCED
{
  real_T K_D_q[49];
  real_T K_P_q[49];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_MIVHhJW7aTMZId5MkCQfpG_
#define DEFINED_TYPEDEF_FOR_struct_MIVHhJW7aTMZId5MkCQfpG_

struct struct_MIVHhJW7aTMZId5MkCQfpG
{
  struct_HynqmHhvVrqpd52uaZuCED MPC12;
  struct_HynqmHhvVrqpd52uaZuCED MPC13;
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

#ifndef DEFINED_TYPEDEF_FOR_struct_1rg7ji8rN2aACsEM9DgtxD_
#define DEFINED_TYPEDEF_FOR_struct_1rg7ji8rN2aACsEM9DgtxD_

struct struct_1rg7ji8rN2aACsEM9DgtxD
{
  real_T start_index[5];
  real_T stop_index[5];
  real_T q_0[35];
  real_T q_0_p[35];
  real_T q_0_pp[35];
  real_T joint_points[105];
  real_T pose[105];
  real_T rotation[135];
  real_T rot_ax[45];
  real_T alpha[15];
  real_T time[15];
  real_T traj_type[5];
  struct_xtY3FIkba7D2gFmP0VadqC diff_filter[5];
  struct_7a8eCPA4nMdHfiggL8ApbC diff_filter_jointspace[5];
  struct_Zz654ZwXwtjsnsQgQLM11G sin_poly[5];
  real_T N_traj;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_qIPFwTp4CGXBaltKbUZCzD_
#define DEFINED_TYPEDEF_FOR_struct_qIPFwTp4CGXBaltKbUZCzD_

struct struct_qIPFwTp4CGXBaltKbUZCzD
{
  real_T init_guess[230];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_BA4vVHYs4L8APXB5GH1YyF_
#define DEFINED_TYPEDEF_FOR_struct_BA4vVHYs4L8APXB5GH1YyF_

struct struct_BA4vVHYs4L8APXB5GH1YyF
{
  real_T init_guess[945];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_qBe9UCTbKs0sytBUoosVxE_
#define DEFINED_TYPEDEF_FOR_struct_qBe9UCTbKs0sytBUoosVxE_

struct struct_qBe9UCTbKs0sytBUoosVxE
{
  real_T N;
  real_T t[15001];
  real_T p_d[225015];
  real_T p_d_p[225015];
  real_T p_d_pp[225015];
  real_T Phi_d[225015];
  real_T Phi_d_p[225015];
  real_T Phi_d_pp[225015];
  real_T R_d[675045];
  real_T q_d[300020];
  real_T q_d_p[300020];
  real_T q_d_pp[300020];
  real_T omega_d[225015];
  real_T omega_d_p[225015];
  real_T alpha_d[75005];
  real_T alpha_d_p[75005];
  real_T alpha_d_pp[75005];
  real_T rot_ax_d[225015];
  real_T alpha_d_offset[75005];
  real_T q_d_rel[300020];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_w9gMcpzWoLTxJuKGyF9MrG_
#define DEFINED_TYPEDEF_FOR_struct_w9gMcpzWoLTxJuKGyF9MrG_

struct struct_w9gMcpzWoLTxJuKGyF9MrG
{
  real_T init_guess[640];
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
#define SS_UINT64                      42
#endif

#ifndef SS_INT64
#define SS_INT64                       43
#endif

/* Parameters (default storage) */
typedef struct P_realtime_simu_franka_fr3_T_ P_realtime_simu_franka_fr3_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_realtime_simu_franka_fr3_T
  RT_MODEL_realtime_simu_franka_fr3_T;

#endif                        /* RTW_HEADER_realtime_simu_franka_fr3_types_h_ */
