/*
 * cartesian_impedance_control_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "cartesian_impedance_control".
 *
 * Model version              : 8.68
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Wed Oct  2 17:07:10 2024
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

#ifndef DEFINED_TYPEDEF_FOR_struct_zgJGJvTOOGEq9ehmj4thTC_
#define DEFINED_TYPEDEF_FOR_struct_zgJGJvTOOGEq9ehmj4thTC_

struct struct_zgJGJvTOOGEq9ehmj4thTC
{
  real_T Ta;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_ORyNrVSSEspV4i2F61GoeD_
#define DEFINED_TYPEDEF_FOR_struct_ORyNrVSSEspV4i2F61GoeD_

struct struct_ORyNrVSSEspV4i2F61GoeD
{
  real_T Kd1[36];
  real_T Kp1[36];
  real_T mode;
  real_T k;
  real_T W_bar_N[6];
  real_T W_E[36];
  real_T eps;
  real_T eps_collinear;
  real_T lambda_min;
  real_T q_n[7];
  real_T K_n[36];
  real_T D_n[36];
  real_T k_n_nl[36];
  real_T nl_spring_threshold[7];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_hBU5NKq9CJdVCP1EolJMiG_
#define DEFINED_TYPEDEF_FOR_struct_hBU5NKq9CJdVCP1EolJMiG_

struct struct_hBU5NKq9CJdVCP1EolJMiG
{
  real_T n_DOF;
  real_T n_indices[6];
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
  real_T q_limit_upper[6];
  real_T q_limit_lower[6];
  real_T q_p_limit_upper[6];
  real_T q_p_limit_lower[6];
  real_T q_pp_limit_upper[6];
  real_T q_pp_limit_lower[6];
  real_T q_n[7];
  real_T torque_limit_upper[6];
  real_T torque_limit_lower[6];
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
  real_T sugihara_limb_vector[6];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_MeKvNmqIDTb5jn47KpTJ8F_
#define DEFINED_TYPEDEF_FOR_struct_MeKvNmqIDTb5jn47KpTJ8F_

struct struct_MeKvNmqIDTb5jn47KpTJ8F
{
  real_T t[12501];
  real_T p_d[187515];
  real_T p_d_p[187515];
  real_T p_d_pp[187515];
  real_T Phi_d[187515];
  real_T Phi_d_p[187515];
  real_T Phi_d_pp[187515];
  real_T R_d[562545];
  real_T q_d[250020];
  real_T q_d_p[250020];
  real_T q_d_pp[250020];
  real_T omega_d[187515];
  real_T omega_d_p[187515];
  real_T alpha_d[62505];
  real_T alpha_d_p[62505];
  real_T alpha_d_pp[62505];
  real_T rot_ax_d[187515];
  real_T alpha_d_offset[62505];
  real_T q_d_rel[250020];
  real_T N;
};

#endif

#ifndef SS_UINT64
#define SS_UINT64                      23
#endif

#ifndef SS_INT64
#define SS_INT64                       24
#endif

/* Parameters (default storage) */
typedef struct P_cartesian_impedance_control_T_ P_cartesian_impedance_control_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_cartesian_impedance_control_T
  RT_MODEL_cartesian_impedance_control_T;

#endif                     /* RTW_HEADER_cartesian_impedance_control_types_h_ */
