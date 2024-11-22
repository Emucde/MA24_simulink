/*
 * realtime_simu_franka_fr3_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "realtime_simu_franka_fr3".
 *
 * Model version              : 8.564
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Fri Nov 22 16:33:00 2024
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

#ifndef DEFINED_TYPEDEF_FOR_traj_data_bus_
#define DEFINED_TYPEDEF_FOR_traj_data_bus_

struct traj_data_bus
{
  real_T N;
  real_T t[12001];
  real_T p_d[108009];
  real_T p_d_p[108009];
  real_T p_d_pp[108009];
  real_T Phi_d[108009];
  real_T Phi_d_p[108009];
  real_T Phi_d_pp[108009];
  real_T R_d[324027];
  real_T q_d[144012];
  real_T q_d_p[144012];
  real_T q_d_pp[144012];
  real_T omega_d[108009];
  real_T omega_d_p[108009];
  real_T alpha_d[36003];
  real_T alpha_d_p[36003];
  real_T alpha_d_pp[36003];
  real_T rot_ax_d[108009];
  real_T alpha_d_offset[36003];
  real_T q_d_rel[144012];
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

#ifndef DEFINED_TYPEDEF_FOR_collinearity_bus_
#define DEFINED_TYPEDEF_FOR_collinearity_bus_

struct collinearity_bus
{
  real_T JJ_Y12_B13_R14[3];
  real_T JJ_Y15_B16_R17[3];
  real_T JJ_Y23_B24_R25[3];
  real_T JJ_Y26_B27_R34[3];
  real_T JJ_Y35_B36_R37[3];
  real_T JJ_Y45_B46_R47[3];
  real_T JJ_Y56_B57_R67[3];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_H7i42zUioeRMpvVnQG9wRE_
#define DEFINED_TYPEDEF_FOR_struct_H7i42zUioeRMpvVnQG9wRE_

struct struct_H7i42zUioeRMpvVnQG9wRE
{
  real_T n_DOF;
  real_T q_0_ref[7];
  real_T q_0_p_ref[7];
  real_T q_0_pp_ref[7];
  real_T m_t;
  real_T m_r;
  real_T m;
  real_T g[3];
  real_T g_x;
  real_T g_y;
  real_T g_z;
  real_T g_vis[3];
  real_T q_limit_upper[7];
  real_T q_limit_lower[7];
  real_T q_p_limit_upper[7];
  real_T q_p_limit_lower[7];
  real_T q_pp_limit_upper[7];
  real_T q_pp_limit_lower[7];
  real_T q_n[7];
  real_T torque_limit_upper[7];
  real_T torque_limit_lower[7];
  real_T p_0[3];
  real_T R_0[9];
  real_T sp0_x;
  real_T sp0_y;
  real_T sp0_z;
  real_T m0;
  real_T I0_xx;
  real_T I0_xy;
  real_T I0_xz;
  real_T I0_yy;
  real_T I0_yz;
  real_T I0_zz;
  real_T l1;
  real_T sp1_x;
  real_T sp1_y;
  real_T sp1_z;
  real_T m1;
  real_T I1_xx;
  real_T I1_xy;
  real_T I1_xz;
  real_T I1_yy;
  real_T I1_yz;
  real_T I1_zz;
  real_T l2;
  real_T sp2_x;
  real_T sp2_y;
  real_T sp2_z;
  real_T m2;
  real_T I2_xx;
  real_T I2_xy;
  real_T I2_xz;
  real_T I2_yy;
  real_T I2_yz;
  real_T I2_zz;
  real_T l3;
  real_T sp3_x;
  real_T sp3_y;
  real_T sp3_z;
  real_T m3;
  real_T I3_xx;
  real_T I3_xy;
  real_T I3_xz;
  real_T I3_yy;
  real_T I3_yz;
  real_T I3_zz;
  real_T l4;
  real_T sp4_x;
  real_T sp4_y;
  real_T sp4_z;
  real_T m4;
  real_T I4_xx;
  real_T I4_xy;
  real_T I4_xz;
  real_T I4_yy;
  real_T I4_yz;
  real_T I4_zz;
  real_T l5;
  real_T sp5_x;
  real_T sp5_y;
  real_T sp5_z;
  real_T m5;
  real_T I5_xx;
  real_T I5_xy;
  real_T I5_xz;
  real_T I5_yy;
  real_T I5_yz;
  real_T I5_zz;
  real_T l6;
  real_T sp6_x;
  real_T sp6_y;
  real_T sp6_z;
  real_T m6;
  real_T I6_xx;
  real_T I6_xy;
  real_T I6_xz;
  real_T I6_yy;
  real_T I6_yz;
  real_T I6_zz;
  real_T l7;
  real_T sp7_x;
  real_T sp7_y;
  real_T sp7_z;
  real_T m7;
  real_T I7_xx;
  real_T I7_xy;
  real_T I7_xz;
  real_T I7_yy;
  real_T I7_yz;
  real_T I7_zz;
  real_T sp8_x;
  real_T sp8_y;
  real_T sp8_z;
  real_T m8;
  real_T I8_xx;
  real_T I8_xy;
  real_T I8_xz;
  real_T I8_yy;
  real_T I8_yz;
  real_T I8_zz;
  real_T sp9_x;
  real_T sp9_y;
  real_T sp9_z;
  real_T m9;
  real_T I9_xx;
  real_T I9_xy;
  real_T I9_xz;
  real_T I9_yy;
  real_T I9_yz;
  real_T I9_zz;
  real_T w_finger;
  real_T l_finger;
  real_T sp_finger_x;
  real_T sp_finger_y;
  real_T sp_finger_z;
  real_T m_finger;
  real_T I_finger_xx;
  real_T I_finger_xy;
  real_T I_finger_xz;
  real_T I_finger_yy;
  real_T I_finger_yz;
  real_T I_finger_zz;
  real_T sugihara_limb_vector[7];
  real_T yt_indices[3];
  real_T yr_indices[3];
  real_T n_indices_fixed;
  real_T n_indices[6];
  real_T n_red;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_BAQoxvk5NEutQSlALFTg0D_
#define DEFINED_TYPEDEF_FOR_struct_BAQoxvk5NEutQSlALFTg0D_

struct struct_BAQoxvk5NEutQSlALFTg0D
{
  real_T bT[301];
  real_T N;
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

#ifndef DEFINED_TYPEDEF_FOR_struct_XqwzzZafeCsSqaSmJYqOHC_
#define DEFINED_TYPEDEF_FOR_struct_XqwzzZafeCsSqaSmJYqOHC_

struct struct_XqwzzZafeCsSqaSmJYqOHC
{
  real_T start_index[3];
  real_T stop_index[3];
  real_T q_0[21];
  real_T q_0_p[21];
  real_T q_0_pp[21];
  real_T joint_points[77];
  real_T pose[77];
  real_T rotation[99];
  real_T rot_ax[33];
  real_T alpha[11];
  real_T time[11];
  real_T traj_type[3];
  struct_xtY3FIkba7D2gFmP0VadqC diff_filter[3];
  struct_7a8eCPA4nMdHfiggL8ApbC diff_filter_jointspace[3];
  struct_Zz654ZwXwtjsnsQgQLM11G sin_poly[3];
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

/* Custom Type definition for MATLAB Function: '<S12>/MATLAB Function' */
#include "coder_posix_time.h"
#ifndef struct_sdAmwXbnJnEmimT0NaJRtAD_realt_T
#define struct_sdAmwXbnJnEmimT0NaJRtAD_realt_T

struct sdAmwXbnJnEmimT0NaJRtAD_realt_T
{
  real_T tv_sec;
  real_T tv_nsec;
};

#endif                              /* struct_sdAmwXbnJnEmimT0NaJRtAD_realt_T */

#ifndef SS_UINT64
#define SS_UINT64                      34
#endif

#ifndef SS_INT64
#define SS_INT64                       35
#endif

/* Parameters (default storage) */
typedef struct P_realtime_simu_franka_fr3_T_ P_realtime_simu_franka_fr3_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_realtime_simu_franka_fr3_T
  RT_MODEL_realtime_simu_franka_fr3_T;

#endif                        /* RTW_HEADER_realtime_simu_franka_fr3_types_h_ */
