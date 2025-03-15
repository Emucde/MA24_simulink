/*
 * realtime_simu_franka_fr3.cpp
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

#include "realtime_simu_franka_fr3.h"
#include "realtime_simu_franka_fr3_types.h"
#include "rtwtypes.h"
#include "realtime_simu_franka_fr3_private.h"
#include <string.h>
#include <math.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "coder_posix_time.h"
#include <cstring>

SimulinkPandaRobot simulinkPandaRobot_1721602;

/* Block signals (default storage) */
B_realtime_simu_franka_fr3_T realtime_simu_franka_fr3_B;

/* Block states (default storage) */
DW_realtime_simu_franka_fr3_T realtime_simu_franka_fr3_DW;

/* Real-time model */
RT_MODEL_realtime_simu_franka_fr3_T realtime_simu_franka_fr3_M_ =
  RT_MODEL_realtime_simu_franka_fr3_T();
RT_MODEL_realtime_simu_franka_fr3_T *const realtime_simu_franka_fr3_M =
  &realtime_simu_franka_fr3_M_;

/* Forward declaration for local functions */
static void realtime_simu_franka_timeKeeper(real_T newTime_tv_sec, real_T
  newTime_tv_nsec);
static void realtime_simu_franka_fr3_tic(void);
static void realtime_simu_fran_timeKeeper_l(real_T *outTime_tv_sec, real_T
  *outTime_tv_nsec);
static real_T realtime_simu_franka_fr3_toc(void);
static void realtime_simu_franka_fr_vecnorm(const real_T x[36], real_T y[6]);
static real_T realtime_simu_franka_fr3_xnrm2(int32_T n, const real_T x[36],
  int32_T ix0);
static real_T realtime_simu_franka_fr3_xdotc(int32_T n, const real_T x[36],
  int32_T ix0, const real_T y[36], int32_T iy0);
static void realtime_simu_franka_fr3_xaxpy(int32_T n, real_T a, int32_T ix0,
  real_T y[36], int32_T iy0);
static real_T realtime_simu_franka_fr_xnrm2_l(int32_T n, const real_T x[6],
  int32_T ix0);
static void realtime_simu_franka_fr_xaxpy_b(int32_T n, real_T a, const real_T x
  [36], int32_T ix0, real_T y[6], int32_T iy0);
static void realtime_simu_franka_f_xaxpy_bj(int32_T n, real_T a, const real_T x
  [6], int32_T ix0, real_T y[36], int32_T iy0);
static void realtime_simu_franka_fr3_xswap(real_T x[36], int32_T ix0, int32_T
  iy0);
static void realtime_simu_franka_fr3_xrotg(real_T *a, real_T *b, real_T *c,
  real_T *s);
static void realtime_simu_franka_fr3_xrot(real_T x[36], int32_T ix0, int32_T iy0,
  real_T c, real_T s);
static void realtime_simu_franka_fr3_svd_k(const real_T A[36], real_T U[36],
  real_T s[6], real_T V[36]);
static void realtime_simu_franka_fr3_svd(const real_T A[36], real_T U[36],
  real_T S[36], real_T V[36]);
static real_T realtime_simu_franka_f_xnrm2_lt(int32_T n, const real_T x_data[],
  int32_T ix0);
static void realtime_simu_franka_fr_qrsolve(const real_T A_data[], const int32_T
  A_size[2], const real_T B[6], real_T Y_data[], int32_T *Y_size);
static void realtime_simu_frank_mldivide_oq(const real_T A_data[], const int32_T
  A_size[2], const real_T B[6], real_T Y_data[], int32_T *Y_size);
static void realtime_simu_franka_fr_svd_kyc(const real_T A[36], real_T U[6]);
static void realtime_simu_franka_fr_xzgetrf(real_T A[36], int32_T ipiv[6],
  int32_T *info);
static void realtime_simu_fran_mldivide_oqz(const real_T A[36], real_T B[6]);
static void realtime_simu_franka_f_eml_find(const boolean_T x[36], int32_T
  i_data[], int32_T *i_size, int32_T j_data[], int32_T *j_size);
static void realtime_simu_franka_fr3_svd_ky(const real_T A[36], real_T U[36],
  real_T s[6], real_T V[36]);
static void realtime_simu_franka_fr3_pinv(const real_T A[36], real_T X[36]);
static void realtime_simu_franka_f_mldivide(const real_T A[36], const real_T B
  [36], real_T Y[36]);
static void realtime_simu_franka_fr3_inv(const real_T x[36], real_T y[36]);
static void realtime_simu_franka__qrsolve_l(const real_T A_data[], const int32_T
  A_size[2], real_T Y_data[], int32_T Y_size[2]);
static void realtime_simu_frank_mldivide_d0(const real_T A_data[], const int32_T
  A_size[2], real_T Y_data[], int32_T Y_size[2]);
static void realtime_simu_franka_fr3_mtimes(const real_T A_data[], const int32_T
  A_size[2], const real_T B_data[], const int32_T B_size[2], real_T C[36]);
static int32_T realtime_simu_franka_local_rank(const real_T A[36]);
int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

/*
 * Output and update for atomic system:
 *    '<S16>/get EKF joint values'
 *    '<S29>/get EKF joint values'
 */
void realtime_simu_getEKFjointvalues(const robot_model *rtu_robot_model, const
  real_T rtu_xk_hat[14], B_getEKFjointvalues_realtime__T *localB)
{
  for (int32_T i = 0; i < 7; i++) {
    localB->q[i] = rtu_xk_hat[i];
    localB->q_p[i] = rtu_xk_hat[i + 7];
    localB->q_pp[i] = rtu_robot_model->q_pp[i];
  }
}

/*
 * Output and update for atomic system:
 *    '<S19>/Robot model bus'
 *    '<S19>/Robot model bus1'
 *    '<S32>/Robot model bus'
 *    '<S32>/Robot model bus1'
 *    '<S9>/Robot model bus1'
 */
void realtime_simu_fra_Robotmodelbus(const real_T rtu_q[7], const real_T
  rtu_q_p[7], const real_T rtu_q_pp[7], const real_T rtu_H[16], const real_T
  rtu_J[42], const real_T rtu_J_p[42], const real_T rtu_M[49], const real_T
  rtu_C_rnea[7], const real_T rtu_C[49], const real_T rtu_g[7],
  B_Robotmodelbus_realtime_simu_T *localB)
{
  memcpy(&localB->robot_model_b.H[0], &rtu_H[0], sizeof(real_T) << 4U);
  memcpy(&localB->robot_model_b.J[0], &rtu_J[0], 42U * sizeof(real_T));
  memcpy(&localB->robot_model_b.J_p[0], &rtu_J_p[0], 42U * sizeof(real_T));
  memcpy(&localB->robot_model_b.M[0], &rtu_M[0], 49U * sizeof(real_T));
  memcpy(&localB->robot_model_b.C[0], &rtu_C[0], 49U * sizeof(real_T));
  for (int32_T i = 0; i < 7; i++) {
    localB->robot_model_b.q[i] = rtu_q[i];
    localB->robot_model_b.q_p[i] = rtu_q_p[i];
    localB->robot_model_b.q_pp[i] = rtu_q_pp[i];
    localB->robot_model_b.C_rnea[i] = rtu_C_rnea[i];
    localB->robot_model_b.g[i] = rtu_g[i];
  }
}

/*
 * Output and update for atomic system:
 *    '<S19>/zero fixed states'
 *    '<S32>/zero fixed states'
 */
void realtime_simu_f_zerofixedstates(const real_T rtu_q[7], const real_T
  rtu_q_p[7], const real_T rtu_q_pp[7], B_zerofixedstates_realtime_si_T *localB)
{
  for (int32_T i = 0; i < 7; i++) {
    localB->q_red[i] = rtu_q[i];
    localB->q_p_red[i] = rtu_q_p[i];
    localB->q_pp_red[i] = rtu_q_pp[i];
  }

  localB->q_red[static_cast<int32_T>
    (realtime_simu_franka_fr3_P.param_robot.n_indices_fixed) - 1] = 0.0;
  localB->q_p_red[static_cast<int32_T>
    (realtime_simu_franka_fr3_P.param_robot.n_indices_fixed) - 1] = 0.0;
  localB->q_pp_red[static_cast<int32_T>
    (realtime_simu_franka_fr3_P.param_robot.n_indices_fixed) - 1] = 0.0;
}

/*
 * Output and update for enable system:
 *    '<S13>/no EKF'
 *    '<S6>/no EKF'
 */
void realtime_simu_franka_fr3_noEKF(boolean_T rtu_Enable, const robot_model
  *rtu_In1, robot_model *rty_Out1, DW_noEKF_realtime_simu_franka_T *localDW)
{
  /* Outputs for Enabled SubSystem: '<S13>/no EKF' incorporates:
   *  EnablePort: '<S17>/Enable'
   */
  if (rtu_Enable) {
    /* SignalConversion generated from: '<S17>/In1' */
    *rty_Out1 = *rtu_In1;
    srUpdateBC(localDW->noEKF_SubsysRanBC);
  }

  /* End of Outputs for SubSystem: '<S13>/no EKF' */
}

/* Function for MATLAB Function: '<S26>/MATLAB Function' */
static void realtime_simu_franka_timeKeeper(real_T newTime_tv_sec, real_T
  newTime_tv_nsec)
{
  coderTimespec b_timespec;
  if (!realtime_simu_franka_fr3_DW.savedTime_not_empty) {
    if (!realtime_simu_franka_fr3_DW.freq_not_empty) {
      realtime_simu_franka_fr3_DW.freq_not_empty = true;
      coderInitTimeFunctions(&realtime_simu_franka_fr3_DW.freq);
    }

    coderTimeClockGettimeMonotonic(&b_timespec, realtime_simu_franka_fr3_DW.freq);
    realtime_simu_franka_fr3_DW.savedTime_not_empty = true;
  }

  realtime_simu_franka_fr3_DW.savedTime.tv_sec = newTime_tv_sec;
  realtime_simu_franka_fr3_DW.savedTime.tv_nsec = newTime_tv_nsec;
}

/* Function for MATLAB Function: '<S26>/MATLAB Function' */
static void realtime_simu_franka_fr3_tic(void)
{
  coderTimespec b_timespec;
  if (!realtime_simu_franka_fr3_DW.freq_not_empty) {
    realtime_simu_franka_fr3_DW.freq_not_empty = true;
    coderInitTimeFunctions(&realtime_simu_franka_fr3_DW.freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, realtime_simu_franka_fr3_DW.freq);
  realtime_simu_franka_timeKeeper(b_timespec.tv_sec, b_timespec.tv_nsec);
}

/* Function for MATLAB Function: '<S26>/MATLAB Function' */
static void realtime_simu_fran_timeKeeper_l(real_T *outTime_tv_sec, real_T
  *outTime_tv_nsec)
{
  coderTimespec b_timespec;
  if (!realtime_simu_franka_fr3_DW.savedTime_not_empty) {
    if (!realtime_simu_franka_fr3_DW.freq_not_empty) {
      realtime_simu_franka_fr3_DW.freq_not_empty = true;
      coderInitTimeFunctions(&realtime_simu_franka_fr3_DW.freq);
    }

    coderTimeClockGettimeMonotonic(&b_timespec, realtime_simu_franka_fr3_DW.freq);
    realtime_simu_franka_fr3_DW.savedTime.tv_sec = b_timespec.tv_sec;
    realtime_simu_franka_fr3_DW.savedTime.tv_nsec = b_timespec.tv_nsec;
  }

  *outTime_tv_sec = realtime_simu_franka_fr3_DW.savedTime.tv_sec;
  *outTime_tv_nsec = realtime_simu_franka_fr3_DW.savedTime.tv_nsec;
}

/* Function for MATLAB Function: '<S26>/MATLAB Function' */
static real_T realtime_simu_franka_fr3_toc(void)
{
  coderTimespec b_timespec;
  real_T tstart_tv_nsec;
  real_T tstart_tv_sec;
  realtime_simu_fran_timeKeeper_l(&tstart_tv_sec, &tstart_tv_nsec);
  if (!realtime_simu_franka_fr3_DW.freq_not_empty) {
    realtime_simu_franka_fr3_DW.freq_not_empty = true;
    coderInitTimeFunctions(&realtime_simu_franka_fr3_DW.freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, realtime_simu_franka_fr3_DW.freq);
  return (b_timespec.tv_nsec - tstart_tv_nsec) / 1.0E+9 + (b_timespec.tv_sec -
    tstart_tv_sec);
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_fr_vecnorm(const real_T x[36], real_T y[6])
{
  for (int32_T j = 0; j < 6; j++) {
    real_T b_y;
    real_T scale;
    int32_T ix0;
    ix0 = j * 6 + 1;
    b_y = 0.0;
    scale = 3.3121686421112381E-170;
    for (int32_T k = ix0; k <= ix0 + 5; k++) {
      real_T absxk;
      absxk = fabs(x[k - 1]);
      if (absxk > scale) {
        real_T t;
        t = scale / absxk;
        b_y = b_y * t * t + 1.0;
        scale = absxk;
      } else {
        real_T t;
        t = absxk / scale;
        b_y += t * t;
      }
    }

    y[j] = scale * sqrt(b_y);
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static real_T realtime_simu_franka_fr3_xnrm2(int32_T n, const real_T x[36],
  int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (int32_T k = ix0; k <= kend; k++) {
    real_T absxk;
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static real_T realtime_simu_franka_fr3_xdotc(int32_T n, const real_T x[36],
  int32_T ix0, const real_T y[36], int32_T iy0)
{
  real_T d;
  int32_T b;
  d = 0.0;
  b = static_cast<uint8_T>(n);
  for (int32_T k = 0; k < b; k++) {
    d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
  }

  return d;
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_fr3_xaxpy(int32_T n, real_T a, int32_T ix0,
  real_T y[36], int32_T iy0)
{
  if (!(a == 0.0)) {
    for (int32_T k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += y[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static real_T realtime_simu_franka_fr_xnrm2_l(int32_T n, const real_T x[6],
  int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (int32_T k = ix0; k <= kend; k++) {
    real_T absxk;
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_fr_xaxpy_b(int32_T n, real_T a, const real_T x
  [36], int32_T ix0, real_T y[6], int32_T iy0)
{
  if (!(a == 0.0)) {
    for (int32_T k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_f_xaxpy_bj(int32_T n, real_T a, const real_T x
  [6], int32_T ix0, real_T y[36], int32_T iy0)
{
  if (!(a == 0.0)) {
    for (int32_T k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_fr3_xswap(real_T x[36], int32_T ix0, int32_T
  iy0)
{
  for (int32_T k = 0; k < 6; k++) {
    real_T temp;
    int32_T temp_tmp;
    int32_T tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x[temp_tmp];
    tmp = (iy0 + k) - 1;
    x[temp_tmp] = x[tmp];
    x[tmp] = temp;
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_fr3_xrotg(real_T *a, real_T *b, real_T *c,
  real_T *s)
{
  real_T absa;
  real_T absb;
  real_T roe;
  real_T scale;
  roe = *b;
  absa = fabs(*a);
  absb = fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    *a = 0.0;
    *b = 0.0;
  } else {
    real_T ads;
    real_T bds;
    ads = absa / scale;
    bds = absb / scale;
    scale *= sqrt(ads * ads + bds * bds);
    if (roe < 0.0) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0) {
      *b = 1.0 / *c;
    } else {
      *b = 1.0;
    }

    *a = scale;
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_fr3_xrot(real_T x[36], int32_T ix0, int32_T iy0,
  real_T c, real_T s)
{
  for (int32_T k = 0; k < 6; k++) {
    real_T temp_tmp_0;
    real_T temp_tmp_2;
    int32_T temp_tmp;
    int32_T temp_tmp_1;
    temp_tmp = (iy0 + k) - 1;
    temp_tmp_0 = x[temp_tmp];
    temp_tmp_1 = (ix0 + k) - 1;
    temp_tmp_2 = x[temp_tmp_1];
    x[temp_tmp] = temp_tmp_0 * c - temp_tmp_2 * s;
    x[temp_tmp_1] = temp_tmp_2 * c + temp_tmp_0 * s;
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_fr3_svd_k(const real_T A[36], real_T U[36],
  real_T s[6], real_T V[36])
{
  real_T b_A[36];
  real_T b_s[6];
  real_T e[6];
  real_T work[6];
  real_T nrm;
  real_T r;
  real_T rt;
  real_T smm1;
  real_T ztest0;
  int32_T i;
  int32_T qjj;
  int32_T qp1;
  int32_T qp1jj;
  int32_T qq;
  memcpy(&b_A[0], &A[0], 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    b_s[i] = 0.0;
    e[i] = 0.0;
    work[i] = 0.0;
  }

  memset(&U[0], 0, 36U * sizeof(real_T));
  memset(&V[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 5; i++) {
    boolean_T apply_transform;
    qp1 = i + 2;
    qq = (6 * i + i) + 1;
    apply_transform = false;
    nrm = realtime_simu_franka_fr3_xnrm2(6 - i, b_A, qq);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq - 1] < 0.0) {
        b_s[i] = -nrm;
      } else {
        b_s[i] = nrm;
      }

      if (fabs(b_s[i]) >= 1.0020841800044864E-292) {
        nrm = 1.0 / b_s[i];
        qjj = (qq - i) + 5;
        for (qp1jj = qq; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] *= nrm;
        }
      } else {
        qjj = (qq - i) + 5;
        for (qp1jj = qq; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] /= b_s[i];
        }
      }

      b_A[qq - 1]++;
      b_s[i] = -b_s[i];
    } else {
      b_s[i] = 0.0;
    }

    for (qp1jj = qp1; qp1jj < 7; qp1jj++) {
      qjj = (qp1jj - 1) * 6 + i;
      if (apply_transform) {
        realtime_simu_franka_fr3_xaxpy(6 - i, -(realtime_simu_franka_fr3_xdotc(6
          - i, b_A, qq, b_A, qjj + 1) / b_A[i + 6 * i]), qq, b_A, qjj + 1);
      }

      e[qp1jj - 1] = b_A[qjj];
    }

    for (qq = i + 1; qq < 7; qq++) {
      qjj = (6 * i + qq) - 1;
      U[qjj] = b_A[qjj];
    }

    if (i + 1 <= 4) {
      nrm = realtime_simu_franka_fr_xnrm2_l(5 - i, e, i + 2);
      if (nrm == 0.0) {
        e[i] = 0.0;
      } else {
        if (e[i + 1] < 0.0) {
          e[i] = -nrm;
        } else {
          e[i] = nrm;
        }

        nrm = e[i];
        if (fabs(e[i]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[i];
          for (qq = qp1; qq < 7; qq++) {
            e[qq - 1] *= nrm;
          }
        } else {
          for (qq = qp1; qq < 7; qq++) {
            e[qq - 1] /= nrm;
          }
        }

        e[i + 1]++;
        e[i] = -e[i];
        for (qq = qp1; qq < 7; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 7; qq++) {
          realtime_simu_franka_fr_xaxpy_b(5 - i, e[qq - 1], b_A, (i + 6 * (qq -
            1)) + 2, work, i + 2);
        }

        for (qq = qp1; qq < 7; qq++) {
          realtime_simu_franka_f_xaxpy_bj(5 - i, -e[qq - 1] / e[i + 1], work, i
            + 2, b_A, (i + 6 * (qq - 1)) + 2);
        }
      }

      for (qq = qp1; qq < 7; qq++) {
        V[(qq + 6 * i) - 1] = e[qq - 1];
      }
    }
  }

  i = 4;
  b_s[5] = b_A[35];
  e[4] = b_A[34];
  e[5] = 0.0;
  for (qp1 = 0; qp1 < 6; qp1++) {
    U[qp1 + 30] = 0.0;
  }

  U[35] = 1.0;
  for (qp1 = 4; qp1 >= 0; qp1--) {
    qq = 6 * qp1 + qp1;
    if (b_s[qp1] != 0.0) {
      for (qp1jj = qp1 + 2; qp1jj < 7; qp1jj++) {
        qjj = ((qp1jj - 1) * 6 + qp1) + 1;
        realtime_simu_franka_fr3_xaxpy(6 - qp1, -(realtime_simu_franka_fr3_xdotc
          (6 - qp1, U, qq + 1, U, qjj) / U[qq]), qq + 1, U, qjj);
      }

      for (qp1jj = qp1 + 1; qp1jj < 7; qp1jj++) {
        qjj = (6 * qp1 + qp1jj) - 1;
        U[qjj] = -U[qjj];
      }

      U[qq]++;
      for (qq = 0; qq < qp1; qq++) {
        U[qq + 6 * qp1] = 0.0;
      }
    } else {
      for (qjj = 0; qjj < 6; qjj++) {
        U[qjj + 6 * qp1] = 0.0;
      }

      U[qq] = 1.0;
    }
  }

  for (qp1 = 5; qp1 >= 0; qp1--) {
    if ((qp1 + 1 <= 4) && (e[qp1] != 0.0)) {
      qq = (6 * qp1 + qp1) + 2;
      for (qjj = qp1 + 2; qjj < 7; qjj++) {
        qp1jj = ((qjj - 1) * 6 + qp1) + 2;
        realtime_simu_franka_fr3_xaxpy(5 - qp1, -(realtime_simu_franka_fr3_xdotc
          (5 - qp1, V, qq, V, qp1jj) / V[qq - 1]), qq, V, qp1jj);
      }
    }

    for (qq = 0; qq < 6; qq++) {
      V[qq + 6 * qp1] = 0.0;
    }

    V[qp1 + 6 * qp1] = 1.0;
  }

  qp1 = 0;
  nrm = 0.0;
  for (qq = 0; qq < 6; qq++) {
    ztest0 = e[qq];
    if (b_s[qq] != 0.0) {
      rt = fabs(b_s[qq]);
      r = b_s[qq] / rt;
      b_s[qq] = rt;
      if (qq + 1 < 6) {
        ztest0 /= r;
      }

      qjj = 6 * qq + 1;
      for (qp1jj = qjj; qp1jj <= qjj + 5; qp1jj++) {
        U[qp1jj - 1] *= r;
      }
    }

    if ((qq + 1 < 6) && (ztest0 != 0.0)) {
      rt = fabs(ztest0);
      r = rt / ztest0;
      ztest0 = rt;
      b_s[qq + 1] *= r;
      qjj = (qq + 1) * 6 + 1;
      for (qp1jj = qjj; qp1jj <= qjj + 5; qp1jj++) {
        V[qp1jj - 1] *= r;
      }
    }

    nrm = fmax(nrm, fmax(fabs(b_s[qq]), fabs(ztest0)));
    e[qq] = ztest0;
  }

  while ((i + 2 > 0) && (qp1 < 75)) {
    boolean_T exitg1;
    qq = i + 1;
    exitg1 = false;
    while (!(exitg1 || (qq == 0))) {
      ztest0 = fabs(e[qq - 1]);
      if ((ztest0 <= (fabs(b_s[qq - 1]) + fabs(b_s[qq])) *
           2.2204460492503131E-16) || ((ztest0 <= 1.0020841800044864E-292) ||
           ((qp1 > 20) && (ztest0 <= 2.2204460492503131E-16 * nrm)))) {
        e[qq - 1] = 0.0;
        exitg1 = true;
      } else {
        qq--;
      }
    }

    if (i + 1 == qq) {
      qp1jj = 4;
    } else {
      qjj = i + 2;
      qp1jj = i + 2;
      exitg1 = false;
      while ((!exitg1) && (qp1jj >= qq)) {
        qjj = qp1jj;
        if (qp1jj == qq) {
          exitg1 = true;
        } else {
          ztest0 = 0.0;
          if (qp1jj < i + 2) {
            ztest0 = fabs(e[qp1jj - 1]);
          }

          if (qp1jj > qq + 1) {
            ztest0 += fabs(e[qp1jj - 2]);
          }

          rt = fabs(b_s[qp1jj - 1]);
          if ((rt <= 2.2204460492503131E-16 * ztest0) || (rt <=
               1.0020841800044864E-292)) {
            b_s[qp1jj - 1] = 0.0;
            exitg1 = true;
          } else {
            qp1jj--;
          }
        }
      }

      if (qjj == qq) {
        qp1jj = 3;
      } else if (i + 2 == qjj) {
        qp1jj = 1;
      } else {
        qp1jj = 2;
        qq = qjj;
      }
    }

    switch (qp1jj) {
     case 1:
      ztest0 = e[i];
      e[i] = 0.0;
      for (qjj = i + 1; qjj >= qq + 1; qjj--) {
        realtime_simu_franka_fr3_xrotg(&b_s[qjj - 1], &ztest0, &r, &smm1);
        if (qjj > qq + 1) {
          rt = e[qjj - 2];
          ztest0 = rt * -smm1;
          e[qjj - 2] = rt * r;
        }

        realtime_simu_franka_fr3_xrot(V, 6 * (qjj - 1) + 1, 6 * (i + 1) + 1, r,
          smm1);
      }
      break;

     case 2:
      ztest0 = e[qq - 1];
      e[qq - 1] = 0.0;
      for (qjj = qq + 1; qjj <= i + 2; qjj++) {
        realtime_simu_franka_fr3_xrotg(&b_s[qjj - 1], &ztest0, &r, &smm1);
        rt = e[qjj - 1];
        ztest0 = rt * -smm1;
        e[qjj - 1] = rt * r;
        realtime_simu_franka_fr3_xrot(U, 6 * (qjj - 1) + 1, 6 * (qq - 1) + 1, r,
          smm1);
      }
      break;

     case 3:
      {
        real_T emm1;
        real_T shift;
        ztest0 = b_s[i + 1];
        rt = fmax(fmax(fmax(fmax(fabs(ztest0), fabs(b_s[i])), fabs(e[i])), fabs
                       (b_s[qq])), fabs(e[qq]));
        ztest0 /= rt;
        smm1 = b_s[i] / rt;
        emm1 = e[i] / rt;
        r = b_s[qq] / rt;
        smm1 = ((smm1 + ztest0) * (smm1 - ztest0) + emm1 * emm1) / 2.0;
        emm1 *= ztest0;
        emm1 *= emm1;
        if ((smm1 != 0.0) || (emm1 != 0.0)) {
          shift = sqrt(smm1 * smm1 + emm1);
          if (smm1 < 0.0) {
            shift = -shift;
          }

          shift = emm1 / (smm1 + shift);
        } else {
          shift = 0.0;
        }

        ztest0 = (r + ztest0) * (r - ztest0) + shift;
        rt = e[qq] / rt * r;
        for (qjj = qq + 1; qjj <= i + 1; qjj++) {
          realtime_simu_franka_fr3_xrotg(&ztest0, &rt, &r, &smm1);
          if (qjj > qq + 1) {
            e[qjj - 2] = ztest0;
          }

          rt = e[qjj - 1];
          emm1 = b_s[qjj - 1];
          e[qjj - 1] = rt * r - emm1 * smm1;
          ztest0 = smm1 * b_s[qjj];
          b_s[qjj] *= r;
          realtime_simu_franka_fr3_xrot(V, 6 * (qjj - 1) + 1, 6 * qjj + 1, r,
            smm1);
          b_s[qjj - 1] = emm1 * r + rt * smm1;
          realtime_simu_franka_fr3_xrotg(&b_s[qjj - 1], &ztest0, &r, &smm1);
          ztest0 = e[qjj - 1] * r + smm1 * b_s[qjj];
          b_s[qjj] = e[qjj - 1] * -smm1 + r * b_s[qjj];
          rt = smm1 * e[qjj];
          e[qjj] *= r;
          realtime_simu_franka_fr3_xrot(U, 6 * (qjj - 1) + 1, 6 * qjj + 1, r,
            smm1);
        }

        e[i] = ztest0;
        qp1++;
      }
      break;

     default:
      if (b_s[qq] < 0.0) {
        b_s[qq] = -b_s[qq];
        qp1 = 6 * qq + 1;
        for (qjj = qp1; qjj <= qp1 + 5; qjj++) {
          V[qjj - 1] = -V[qjj - 1];
        }
      }

      qp1 = qq + 1;
      while ((qq + 1 < 6) && (b_s[qq] < b_s[qp1])) {
        rt = b_s[qq];
        b_s[qq] = b_s[qp1];
        b_s[qp1] = rt;
        realtime_simu_franka_fr3_xswap(V, 6 * qq + 1, 6 * (qq + 1) + 1);
        realtime_simu_franka_fr3_xswap(U, 6 * qq + 1, 6 * (qq + 1) + 1);
        qq = qp1;
        qp1++;
      }

      qp1 = 0;
      i--;
      break;
    }
  }

  for (i = 0; i < 6; i++) {
    s[i] = b_s[i];
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_fr3_svd(const real_T A[36], real_T U[36],
  real_T S[36], real_T V[36])
{
  real_T s[6];
  boolean_T p;
  p = true;
  for (int32_T i = 0; i < 36; i++) {
    real_T A_0;
    A_0 = A[i];
    if (p && ((!rtIsInf(A_0)) && (!rtIsNaN(A_0)))) {
    } else {
      p = false;
    }
  }

  if (p) {
    realtime_simu_franka_fr3_svd_k(A, U, s, V);
  } else {
    for (int32_T i = 0; i < 36; i++) {
      U[i] = (rtNaN);
    }

    for (int32_T i = 0; i < 6; i++) {
      s[i] = (rtNaN);
    }

    for (int32_T i = 0; i < 36; i++) {
      V[i] = (rtNaN);
    }
  }

  memset(&S[0], 0, 36U * sizeof(real_T));
  for (int32_T i = 0; i < 6; i++) {
    S[i + 6 * i] = s[i];
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static real_T realtime_simu_franka_f_xnrm2_lt(int32_T n, const real_T x_data[],
  int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n == 1) {
    y = fabs(x_data[ix0 - 1]);
  } else {
    real_T scale;
    int32_T kend;
    scale = 3.3121686421112381E-170;
    kend = (ix0 + n) - 1;
    for (int32_T k = ix0; k <= kend; k++) {
      real_T absxk;
      absxk = fabs(x_data[k - 1]);
      if (absxk > scale) {
        real_T t;
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        real_T t;
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrt(y);
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = sqrt(b * b + 1.0) * a;
  } else if (rtIsNaN(b)) {
    y = (rtNaN);
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_fr_qrsolve(const real_T A_data[], const int32_T
  A_size[2], const real_T B[6], real_T Y_data[], int32_T *Y_size)
{
  real_T b_A_data[30];
  real_T b_B[6];
  real_T tau_data[5];
  real_T vn1_data[5];
  real_T vn2_data[5];
  real_T work_data[5];
  real_T scale;
  real_T smax;
  int32_T b_A_size[2];
  int32_T b_A;
  int32_T i;
  int32_T ii;
  int32_T nmi;
  int32_T rankA;
  int8_T jpvt_data[5];
  rankA = A_size[1];
  b_A_size[1] = A_size[1];
  i = 6 * A_size[1];
  if (i - 1 >= 0) {
    memcpy(&b_A_data[0], &A_data[0], static_cast<uint32_T>(i) * sizeof(real_T));
  }

  i = A_size[1];
  if (i - 1 >= 0) {
    memset(&tau_data[0], 0, static_cast<uint32_T>(i) * sizeof(real_T));
  }

  if (A_size[1] == 0) {
  } else {
    real_T absxk;
    i = A_size[1];
    memset(&jpvt_data[0], 0, static_cast<uint32_T>(i) * sizeof(int8_T));
    i = A_size[1];
    for (b_A = 0; b_A < i; b_A++) {
      jpvt_data[b_A] = static_cast<int8_T>(b_A + 1);
    }

    i = A_size[1];
    memset(&work_data[0], 0, static_cast<uint32_T>(i) * sizeof(real_T));
    i = A_size[1];
    memset(&vn1_data[0], 0, static_cast<uint32_T>(i) * sizeof(real_T));
    i = A_size[1];
    memset(&vn2_data[0], 0, static_cast<uint32_T>(i) * sizeof(real_T));
    i = A_size[1];
    for (b_A = 0; b_A < i; b_A++) {
      ii = b_A * 6 + 1;
      smax = 0.0;
      scale = 3.3121686421112381E-170;
      for (nmi = ii; nmi <= ii + 5; nmi++) {
        absxk = fabs(A_data[nmi - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          smax = smax * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          smax += t * t;
        }
      }

      smax = scale * sqrt(smax);
      vn2_data[b_A] = smax;
      vn1_data[b_A] = smax;
    }

    i = A_size[1];
    for (b_A = 0; b_A < i; b_A++) {
      int32_T b_ix;
      int32_T itemp;
      int32_T iy;
      int32_T pvt;
      ii = b_A * 6 + b_A;
      nmi = rankA - b_A;
      if (nmi < 1) {
        itemp = -1;
      } else {
        itemp = 0;
        if (nmi > 1) {
          smax = fabs(vn1_data[b_A]);
          for (pvt = 2; pvt <= nmi; pvt++) {
            scale = fabs(vn1_data[(b_A + pvt) - 1]);
            if (scale > smax) {
              itemp = pvt - 1;
              smax = scale;
            }
          }
        }
      }

      pvt = b_A + itemp;
      if (pvt != b_A) {
        b_ix = pvt * 6;
        iy = b_A * 6;
        for (int32_T f_k = 0; f_k < 6; f_k++) {
          int32_T temp_tmp;
          temp_tmp = b_ix + f_k;
          smax = b_A_data[temp_tmp];
          itemp = iy + f_k;
          b_A_data[temp_tmp] = b_A_data[itemp];
          b_A_data[itemp] = smax;
        }

        itemp = jpvt_data[pvt];
        jpvt_data[pvt] = jpvt_data[b_A];
        jpvt_data[b_A] = static_cast<int8_T>(itemp);
        vn1_data[pvt] = vn1_data[b_A];
        vn2_data[pvt] = vn2_data[b_A];
      }

      scale = b_A_data[ii];
      itemp = ii + 2;
      tau_data[b_A] = 0.0;
      smax = realtime_simu_franka_f_xnrm2_lt(5 - b_A, b_A_data, ii + 2);
      if (smax != 0.0) {
        smax = rt_hypotd_snf(b_A_data[ii], smax);
        if (b_A_data[ii] >= 0.0) {
          smax = -smax;
        }

        if (fabs(smax) < 1.0020841800044864E-292) {
          pvt = 0;
          do {
            pvt++;
            b_ix = (ii - b_A) + 6;
            for (iy = itemp; iy <= b_ix; iy++) {
              b_A_data[iy - 1] *= 9.9792015476736E+291;
            }

            smax *= 9.9792015476736E+291;
            scale *= 9.9792015476736E+291;
          } while ((fabs(smax) < 1.0020841800044864E-292) && (pvt < 20));

          smax = rt_hypotd_snf(scale, realtime_simu_franka_f_xnrm2_lt(5 - b_A,
            b_A_data, ii + 2));
          if (scale >= 0.0) {
            smax = -smax;
          }

          tau_data[b_A] = (smax - scale) / smax;
          scale = 1.0 / (scale - smax);
          for (iy = itemp; iy <= b_ix; iy++) {
            b_A_data[iy - 1] *= scale;
          }

          for (itemp = 0; itemp < pvt; itemp++) {
            smax *= 1.0020841800044864E-292;
          }

          scale = smax;
        } else {
          tau_data[b_A] = (smax - b_A_data[ii]) / smax;
          scale = 1.0 / (b_A_data[ii] - smax);
          pvt = (ii - b_A) + 6;
          for (b_ix = itemp; b_ix <= pvt; b_ix++) {
            b_A_data[b_ix - 1] *= scale;
          }

          scale = smax;
        }
      }

      b_A_data[ii] = scale;
      if (b_A + 1 < rankA) {
        smax = b_A_data[ii];
        b_A_data[ii] = 1.0;
        if (tau_data[b_A] != 0.0) {
          boolean_T exitg2;
          itemp = 6 - b_A;
          pvt = (ii - b_A) + 5;
          while ((itemp > 0) && (b_A_data[pvt] == 0.0)) {
            itemp--;
            pvt--;
          }

          nmi--;
          exitg2 = false;
          while ((!exitg2) && (nmi > 0)) {
            int32_T exitg1;
            pvt = ((nmi - 1) * 6 + ii) + 6;
            b_ix = pvt;
            do {
              exitg1 = 0;
              if (b_ix + 1 <= pvt + itemp) {
                if (b_A_data[b_ix] != 0.0) {
                  exitg1 = 1;
                } else {
                  b_ix++;
                }
              } else {
                nmi--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }

          nmi--;
        } else {
          itemp = 0;
          nmi = -1;
        }

        if (itemp > 0) {
          if (nmi + 1 != 0) {
            if (nmi >= 0) {
              memset(&work_data[0], 0, static_cast<uint32_T>(nmi + 1) * sizeof
                     (real_T));
            }

            pvt = (6 * nmi + ii) + 7;
            for (b_ix = ii + 7; b_ix <= pvt; b_ix += 6) {
              scale = 0.0;
              iy = (b_ix + itemp) - 1;
              for (int32_T f_k = b_ix; f_k <= iy; f_k++) {
                scale += b_A_data[(ii + f_k) - b_ix] * b_A_data[f_k - 1];
              }

              iy = div_nde_s32_floor((b_ix - ii) - 7, 6);
              work_data[iy] += scale;
            }
          }

          if (!(-tau_data[b_A] == 0.0)) {
            pvt = ii + 7;
            for (b_ix = 0; b_ix <= nmi; b_ix++) {
              scale = work_data[b_ix];
              if (scale != 0.0) {
                scale *= -tau_data[b_A];
                iy = itemp + pvt;
                for (int32_T f_k = pvt; f_k < iy; f_k++) {
                  b_A_data[f_k - 1] += b_A_data[(ii + f_k) - pvt] * scale;
                }
              }

              pvt += 6;
            }
          }
        }

        b_A_data[ii] = smax;
      }

      for (ii = b_A + 2; ii <= rankA; ii++) {
        nmi = (ii - 1) * 6 + b_A;
        smax = vn1_data[ii - 1];
        if (smax != 0.0) {
          scale = fabs(b_A_data[nmi]) / smax;
          scale = 1.0 - scale * scale;
          if (scale < 0.0) {
            scale = 0.0;
          }

          absxk = smax / vn2_data[ii - 1];
          absxk = absxk * absxk * scale;
          if (absxk <= 1.4901161193847656E-8) {
            vn1_data[ii - 1] = realtime_simu_franka_f_xnrm2_lt(5 - b_A, b_A_data,
              nmi + 2);
            vn2_data[ii - 1] = vn1_data[ii - 1];
          } else {
            vn1_data[ii - 1] = smax * sqrt(scale);
          }
        }
      }
    }
  }

  rankA = 0;
  if (A_size[1] > 0) {
    while ((rankA < b_A_size[1]) && (!(fabs(b_A_data[6 * rankA + rankA]) <=
             1.3322676295501878E-14 * fabs(b_A_data[0])))) {
      rankA++;
    }
  }

  for (i = 0; i < 6; i++) {
    b_B[i] = B[i];
  }

  *Y_size = static_cast<int8_T>(A_size[1]);
  i = static_cast<int8_T>(A_size[1]);
  if (i - 1 >= 0) {
    memset(&Y_data[0], 0, static_cast<uint32_T>(i) * sizeof(real_T));
  }

  i = A_size[1];
  for (b_A = 0; b_A < i; b_A++) {
    smax = tau_data[b_A];
    if (smax != 0.0) {
      scale = b_B[b_A];
      for (ii = b_A + 2; ii < 7; ii++) {
        scale += b_A_data[(6 * b_A + ii) - 1] * b_B[ii - 1];
      }

      scale *= smax;
      if (scale != 0.0) {
        b_B[b_A] -= scale;
        for (ii = b_A + 2; ii < 7; ii++) {
          b_B[ii - 1] -= b_A_data[(6 * b_A + ii) - 1] * scale;
        }
      }
    }
  }

  for (i = 0; i < rankA; i++) {
    Y_data[jpvt_data[i] - 1] = b_B[i];
  }

  for (i = rankA; i >= 1; i--) {
    b_A = jpvt_data[i - 1] - 1;
    Y_data[b_A] /= b_A_data[((i - 1) * 6 + i) - 1];
    for (ii = 0; ii <= i - 2; ii++) {
      nmi = jpvt_data[ii] - 1;
      Y_data[nmi] -= b_A_data[(i - 1) * 6 + ii] * Y_data[b_A];
    }
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_frank_mldivide_oq(const real_T A_data[], const int32_T
  A_size[2], const real_T B[6], real_T Y_data[], int32_T *Y_size)
{
  real_T b_A_data[36];
  real_T b_B[6];
  real_T tmp_data[5];
  int32_T i;
  int8_T ipiv[6];
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else if (A_size[1] == 6) {
    int32_T jA;
    int32_T jj;
    memcpy(&b_A_data[0], &A_data[0], 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      b_B[i] = B[i];
      ipiv[i] = static_cast<int8_T>(i + 1);
    }

    for (i = 0; i < 5; i++) {
      real_T smax;
      int32_T a;
      int32_T b_temp_tmp;
      int8_T ipiv_0;
      ipiv_0 = ipiv[i];
      jj = i * 7;
      jA = 6 - i;
      a = 0;
      smax = fabs(b_A_data[jj]);
      for (int32_T c_k = 2; c_k <= jA; c_k++) {
        real_T s;
        s = fabs(b_A_data[(jj + c_k) - 1]);
        if (s > smax) {
          a = c_k - 1;
          smax = s;
        }
      }

      if (b_A_data[jj + a] != 0.0) {
        if (a != 0) {
          a += i;
          ipiv_0 = static_cast<int8_T>(a + 1);
          for (int32_T c_k = 0; c_k < 6; c_k++) {
            b_temp_tmp = c_k * 6 + i;
            smax = b_A_data[b_temp_tmp];
            jA = c_k * 6 + a;
            b_A_data[b_temp_tmp] = b_A_data[jA];
            b_A_data[jA] = smax;
          }
        }

        jA = (jj - i) + 6;
        for (a = jj + 2; a <= jA; a++) {
          b_A_data[a - 1] /= b_A_data[jj];
        }
      }

      jA = jj + 8;
      a = 4 - i;
      for (int32_T c_k = 0; c_k <= a; c_k++) {
        smax = b_A_data[(c_k * 6 + jj) + 6];
        if (smax != 0.0) {
          b_temp_tmp = (jA - i) + 4;
          for (int32_T ijA = jA; ijA <= b_temp_tmp; ijA++) {
            b_A_data[ijA - 1] += b_A_data[((jj + ijA) - jA) + 1] * -smax;
          }
        }

        jA += 6;
      }

      if (i + 1 != ipiv_0) {
        smax = b_B[i];
        b_B[i] = b_B[ipiv_0 - 1];
        b_B[ipiv_0 - 1] = smax;
      }

      ipiv[i] = ipiv_0;
    }

    for (i = 0; i < 6; i++) {
      jj = 6 * i;
      if (b_B[i] != 0.0) {
        for (jA = i + 2; jA < 7; jA++) {
          b_B[jA - 1] -= b_A_data[(jA + jj) - 1] * b_B[i];
        }
      }
    }

    for (i = 5; i >= 0; i--) {
      jj = 6 * i;
      if (b_B[i] != 0.0) {
        b_B[i] /= b_A_data[i + jj];
        for (jA = 0; jA < i; jA++) {
          b_B[jA] -= b_A_data[jA + jj] * b_B[i];
        }
      }
    }

    *Y_size = 6;
    for (jA = 0; jA < 6; jA++) {
      Y_data[jA] = b_B[jA];
    }
  } else {
    realtime_simu_franka_fr_qrsolve(A_data, A_size, B, tmp_data, &i);
    *Y_size = i;
    if (i - 1 >= 0) {
      memcpy(&Y_data[0], &tmp_data[0], static_cast<uint32_T>(i) * sizeof(real_T));
    }
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_fr_svd_kyc(const real_T A[36], real_T U[6])
{
  real_T b_A[36];
  real_T e[6];
  real_T s[6];
  real_T work[6];
  real_T nrm;
  real_T r;
  real_T rt;
  real_T smm1;
  real_T ztest0;
  int32_T i;
  int32_T m;
  int32_T qjj;
  int32_T qp1;
  int32_T qq;
  int32_T qq_tmp;
  memcpy(&b_A[0], &A[0], 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    s[i] = 0.0;
    e[i] = 0.0;
    work[i] = 0.0;
  }

  for (m = 0; m < 5; m++) {
    int32_T g_k;
    boolean_T apply_transform;
    qp1 = m + 2;
    qq_tmp = 6 * m + m;
    qq = qq_tmp + 1;
    apply_transform = false;
    nrm = realtime_simu_franka_fr3_xnrm2(6 - m, b_A, qq_tmp + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq_tmp] < 0.0) {
        s[m] = -nrm;
      } else {
        s[m] = nrm;
      }

      if (fabs(s[m]) >= 1.0020841800044864E-292) {
        nrm = 1.0 / s[m];
        qjj = (qq_tmp - m) + 6;
        for (i = qq; i <= qjj; i++) {
          b_A[i - 1] *= nrm;
        }
      } else {
        qjj = (qq_tmp - m) + 6;
        for (i = qq; i <= qjj; i++) {
          b_A[i - 1] /= s[m];
        }
      }

      b_A[qq_tmp]++;
      s[m] = -s[m];
    } else {
      s[m] = 0.0;
    }

    for (qq = qp1; qq < 7; qq++) {
      qjj = (qq - 1) * 6 + m;
      if (apply_transform) {
        nrm = 0.0;
        i = 6 - m;
        for (g_k = 0; g_k < i; g_k++) {
          nrm += b_A[qq_tmp + g_k] * b_A[qjj + g_k];
        }

        nrm = -(nrm / b_A[qq_tmp]);
        if (!(nrm == 0.0)) {
          for (g_k = 0; g_k < i; g_k++) {
            int32_T b_A_tmp;
            b_A_tmp = qjj + g_k;
            b_A[b_A_tmp] += b_A[qq_tmp + g_k] * nrm;
          }
        }
      }

      e[qq - 1] = b_A[qjj];
    }

    if (m + 1 <= 4) {
      nrm = realtime_simu_franka_fr_xnrm2_l(5 - m, e, m + 2);
      if (nrm == 0.0) {
        e[m] = 0.0;
      } else {
        if (e[m + 1] < 0.0) {
          e[m] = -nrm;
        } else {
          e[m] = nrm;
        }

        nrm = e[m];
        if (fabs(e[m]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[m];
          for (qq_tmp = qp1; qq_tmp < 7; qq_tmp++) {
            e[qq_tmp - 1] *= nrm;
          }
        } else {
          for (qq_tmp = qp1; qq_tmp < 7; qq_tmp++) {
            e[qq_tmp - 1] /= nrm;
          }
        }

        e[m + 1]++;
        e[m] = -e[m];
        for (qq_tmp = qp1; qq_tmp < 7; qq_tmp++) {
          work[qq_tmp - 1] = 0.0;
        }

        for (qq_tmp = qp1; qq_tmp < 7; qq_tmp++) {
          nrm = e[qq_tmp - 1];
          if (!(nrm == 0.0)) {
            qq = ((qq_tmp - 1) * 6 + m) + 1;
            qjj = 5 - m;
            for (i = 0; i < qjj; i++) {
              g_k = (m + i) + 1;
              work[g_k] += b_A[qq + i] * nrm;
            }
          }
        }

        for (qq_tmp = qp1; qq_tmp < 7; qq_tmp++) {
          realtime_simu_franka_f_xaxpy_bj(5 - m, -e[qq_tmp - 1] / e[m + 1], work,
            m + 2, b_A, (m + 6 * (qq_tmp - 1)) + 2);
        }
      }
    }
  }

  m = 4;
  s[5] = b_A[35];
  e[4] = b_A[34];
  e[5] = 0.0;
  qp1 = 0;
  nrm = 0.0;
  for (qq_tmp = 0; qq_tmp < 6; qq_tmp++) {
    ztest0 = e[qq_tmp];
    if (s[qq_tmp] != 0.0) {
      rt = fabs(s[qq_tmp]);
      r = s[qq_tmp] / rt;
      s[qq_tmp] = rt;
      if (qq_tmp + 1 < 6) {
        ztest0 /= r;
      }
    }

    if ((qq_tmp + 1 < 6) && (ztest0 != 0.0)) {
      rt = fabs(ztest0);
      r = rt / ztest0;
      ztest0 = rt;
      s[qq_tmp + 1] *= r;
    }

    nrm = fmax(nrm, fmax(fabs(s[qq_tmp]), fabs(ztest0)));
    e[qq_tmp] = ztest0;
  }

  while ((m + 2 > 0) && (qp1 < 75)) {
    boolean_T exitg1;
    qq_tmp = m + 1;
    exitg1 = false;
    while (!(exitg1 || (qq_tmp == 0))) {
      ztest0 = fabs(e[qq_tmp - 1]);
      if ((ztest0 <= (fabs(s[qq_tmp - 1]) + fabs(s[qq_tmp])) *
           2.2204460492503131E-16) || ((ztest0 <= 1.0020841800044864E-292) ||
           ((qp1 > 20) && (ztest0 <= 2.2204460492503131E-16 * nrm)))) {
        e[qq_tmp - 1] = 0.0;
        exitg1 = true;
      } else {
        qq_tmp--;
      }
    }

    if (m + 1 == qq_tmp) {
      qjj = 4;
    } else {
      qq = m + 2;
      qjj = m + 2;
      exitg1 = false;
      while ((!exitg1) && (qjj >= qq_tmp)) {
        qq = qjj;
        if (qjj == qq_tmp) {
          exitg1 = true;
        } else {
          ztest0 = 0.0;
          if (qjj < m + 2) {
            ztest0 = fabs(e[qjj - 1]);
          }

          if (qjj > qq_tmp + 1) {
            ztest0 += fabs(e[qjj - 2]);
          }

          rt = fabs(s[qjj - 1]);
          if ((rt <= 2.2204460492503131E-16 * ztest0) || (rt <=
               1.0020841800044864E-292)) {
            s[qjj - 1] = 0.0;
            exitg1 = true;
          } else {
            qjj--;
          }
        }
      }

      if (qq == qq_tmp) {
        qjj = 3;
      } else if (m + 2 == qq) {
        qjj = 1;
      } else {
        qjj = 2;
        qq_tmp = qq;
      }
    }

    switch (qjj) {
     case 1:
      ztest0 = e[m];
      e[m] = 0.0;
      for (qq = m + 1; qq >= qq_tmp + 1; qq--) {
        realtime_simu_franka_fr3_xrotg(&s[qq - 1], &ztest0, &r, &smm1);
        if (qq > qq_tmp + 1) {
          rt = e[qq - 2];
          ztest0 = rt * -smm1;
          e[qq - 2] = rt * r;
        }
      }
      break;

     case 2:
      ztest0 = e[qq_tmp - 1];
      e[qq_tmp - 1] = 0.0;
      for (qq = qq_tmp + 1; qq <= m + 2; qq++) {
        realtime_simu_franka_fr3_xrotg(&s[qq - 1], &ztest0, &r, &smm1);
        rt = e[qq - 1];
        ztest0 = rt * -smm1;
        e[qq - 1] = rt * r;
      }
      break;

     case 3:
      {
        real_T emm1;
        real_T shift;
        ztest0 = s[m + 1];
        rt = fmax(fmax(fmax(fmax(fabs(ztest0), fabs(s[m])), fabs(e[m])), fabs
                       (s[qq_tmp])), fabs(e[qq_tmp]));
        ztest0 /= rt;
        smm1 = s[m] / rt;
        emm1 = e[m] / rt;
        r = s[qq_tmp] / rt;
        smm1 = ((smm1 + ztest0) * (smm1 - ztest0) + emm1 * emm1) / 2.0;
        emm1 *= ztest0;
        emm1 *= emm1;
        if ((smm1 != 0.0) || (emm1 != 0.0)) {
          shift = sqrt(smm1 * smm1 + emm1);
          if (smm1 < 0.0) {
            shift = -shift;
          }

          shift = emm1 / (smm1 + shift);
        } else {
          shift = 0.0;
        }

        ztest0 = (r + ztest0) * (r - ztest0) + shift;
        rt = e[qq_tmp] / rt * r;
        for (qq = qq_tmp + 1; qq <= m + 1; qq++) {
          realtime_simu_franka_fr3_xrotg(&ztest0, &rt, &r, &smm1);
          if (qq > qq_tmp + 1) {
            e[qq - 2] = ztest0;
          }

          rt = e[qq - 1];
          emm1 = s[qq - 1];
          e[qq - 1] = rt * r - emm1 * smm1;
          ztest0 = smm1 * s[qq];
          s[qq] *= r;
          s[qq - 1] = emm1 * r + rt * smm1;
          realtime_simu_franka_fr3_xrotg(&s[qq - 1], &ztest0, &r, &rt);
          ztest0 = e[qq - 1] * r + rt * s[qq];
          s[qq] = e[qq - 1] * -rt + r * s[qq];
          rt *= e[qq];
          e[qq] *= r;
        }

        e[m] = ztest0;
        qp1++;
      }
      break;

     default:
      if (s[qq_tmp] < 0.0) {
        s[qq_tmp] = -s[qq_tmp];
      }

      qp1 = qq_tmp + 1;
      while ((qq_tmp + 1 < 6) && (s[qq_tmp] < s[qp1])) {
        rt = s[qq_tmp];
        s[qq_tmp] = s[qp1];
        s[qp1] = rt;
        qq_tmp = qp1;
        qp1++;
      }

      qp1 = 0;
      m--;
      break;
    }
  }

  for (qp1 = 0; qp1 < 6; qp1++) {
    U[qp1] = s[qp1];
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_fr_xzgetrf(real_T A[36], int32_T ipiv[6],
  int32_T *info)
{
  int32_T k;
  for (k = 0; k < 6; k++) {
    ipiv[k] = k + 1;
  }

  *info = 0;
  for (int32_T j = 0; j < 5; j++) {
    real_T smax;
    int32_T a;
    int32_T jA;
    int32_T jj;
    int32_T temp_tmp;
    jj = j * 7;
    jA = 6 - j;
    a = 0;
    smax = fabs(A[jj]);
    for (k = 2; k <= jA; k++) {
      real_T s;
      s = fabs(A[(jj + k) - 1]);
      if (s > smax) {
        a = k - 1;
        smax = s;
      }
    }

    if (A[jj + a] != 0.0) {
      if (a != 0) {
        a += j;
        ipiv[j] = a + 1;
        for (jA = 0; jA < 6; jA++) {
          temp_tmp = jA * 6 + j;
          smax = A[temp_tmp];
          k = jA * 6 + a;
          A[temp_tmp] = A[k];
          A[k] = smax;
        }
      }

      jA = (jj - j) + 6;
      for (k = jj + 2; k <= jA; k++) {
        A[k - 1] /= A[jj];
      }
    } else {
      *info = j + 1;
    }

    jA = jj + 8;
    a = 4 - j;
    for (k = 0; k <= a; k++) {
      temp_tmp = (k * 6 + jj) + 6;
      smax = A[temp_tmp];
      if (A[temp_tmp] != 0.0) {
        int32_T d;
        d = (jA - j) + 4;
        for (temp_tmp = jA; temp_tmp <= d; temp_tmp++) {
          A[temp_tmp - 1] += A[((jj + temp_tmp) - jA) + 1] * -smax;
        }
      }

      jA += 6;
    }
  }

  if ((*info == 0) && (!(A[35] != 0.0))) {
    *info = 6;
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_fran_mldivide_oqz(const real_T A[36], real_T B[6])
{
  real_T b_A[36];
  int32_T ipiv[6];
  int32_T info;
  int32_T ipiv_0;
  int32_T kAcol;
  memcpy(&b_A[0], &A[0], 36U * sizeof(real_T));
  realtime_simu_franka_fr_xzgetrf(b_A, ipiv, &info);
  for (info = 0; info < 5; info++) {
    ipiv_0 = ipiv[info];
    if (info + 1 != ipiv_0) {
      real_T temp;
      temp = B[info];
      B[info] = B[ipiv_0 - 1];
      B[ipiv_0 - 1] = temp;
    }
  }

  for (info = 0; info < 6; info++) {
    kAcol = 6 * info;
    if (B[info] != 0.0) {
      for (ipiv_0 = info + 2; ipiv_0 < 7; ipiv_0++) {
        B[ipiv_0 - 1] -= b_A[(ipiv_0 + kAcol) - 1] * B[info];
      }
    }
  }

  for (info = 5; info >= 0; info--) {
    kAcol = 6 * info;
    if (B[info] != 0.0) {
      B[info] /= b_A[info + kAcol];
      for (ipiv_0 = 0; ipiv_0 < info; ipiv_0++) {
        B[ipiv_0] -= b_A[ipiv_0 + kAcol] * B[info];
      }
    }
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_f_eml_find(const boolean_T x[36], int32_T
  i_data[], int32_T *i_size, int32_T j_data[], int32_T *j_size)
{
  int32_T idx;
  int32_T ii;
  int32_T jj;
  boolean_T exitg1;
  idx = 0;
  ii = 1;
  jj = 1;
  exitg1 = false;
  while ((!exitg1) && (jj <= 6)) {
    boolean_T guard1;
    guard1 = false;
    if (x[((jj - 1) * 6 + ii) - 1]) {
      idx++;
      i_data[idx - 1] = ii;
      j_data[idx - 1] = jj;
      if (idx >= 36) {
        exitg1 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      ii++;
      if (ii > 6) {
        ii = 1;
        jj++;
      }
    }
  }

  if (idx < 1) {
    *i_size = 0;
    *j_size = 0;
  } else {
    *i_size = idx;
    *j_size = idx;
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_fr3_svd_ky(const real_T A[36], real_T U[36],
  real_T s[6], real_T V[36])
{
  real_T Vf[36];
  real_T b_A[36];
  real_T b_s[6];
  real_T e[6];
  real_T work[6];
  real_T nrm;
  real_T r;
  real_T rt;
  real_T smm1;
  real_T ztest0;
  int32_T i;
  int32_T qjj;
  int32_T qp1;
  int32_T qp1jj;
  int32_T qq;
  memcpy(&b_A[0], &A[0], 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    b_s[i] = 0.0;
    e[i] = 0.0;
    work[i] = 0.0;
  }

  memset(&U[0], 0, 36U * sizeof(real_T));
  memset(&Vf[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 5; i++) {
    boolean_T apply_transform;
    qp1 = i + 2;
    qq = (6 * i + i) + 1;
    apply_transform = false;
    nrm = realtime_simu_franka_fr3_xnrm2(6 - i, b_A, qq);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq - 1] < 0.0) {
        b_s[i] = -nrm;
      } else {
        b_s[i] = nrm;
      }

      if (fabs(b_s[i]) >= 1.0020841800044864E-292) {
        nrm = 1.0 / b_s[i];
        qjj = (qq - i) + 5;
        for (qp1jj = qq; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] *= nrm;
        }
      } else {
        qjj = (qq - i) + 5;
        for (qp1jj = qq; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] /= b_s[i];
        }
      }

      b_A[qq - 1]++;
      b_s[i] = -b_s[i];
    } else {
      b_s[i] = 0.0;
    }

    for (qp1jj = qp1; qp1jj < 7; qp1jj++) {
      qjj = (qp1jj - 1) * 6 + i;
      if (apply_transform) {
        realtime_simu_franka_fr3_xaxpy(6 - i, -(realtime_simu_franka_fr3_xdotc(6
          - i, b_A, qq, b_A, qjj + 1) / b_A[i + 6 * i]), qq, b_A, qjj + 1);
      }

      e[qp1jj - 1] = b_A[qjj];
    }

    for (qq = i + 1; qq < 7; qq++) {
      qjj = (6 * i + qq) - 1;
      U[qjj] = b_A[qjj];
    }

    if (i + 1 <= 4) {
      nrm = realtime_simu_franka_fr_xnrm2_l(5 - i, e, i + 2);
      if (nrm == 0.0) {
        e[i] = 0.0;
      } else {
        if (e[i + 1] < 0.0) {
          e[i] = -nrm;
        } else {
          e[i] = nrm;
        }

        nrm = e[i];
        if (fabs(e[i]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[i];
          for (qq = qp1; qq < 7; qq++) {
            e[qq - 1] *= nrm;
          }
        } else {
          for (qq = qp1; qq < 7; qq++) {
            e[qq - 1] /= nrm;
          }
        }

        e[i + 1]++;
        e[i] = -e[i];
        for (qq = qp1; qq < 7; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 7; qq++) {
          realtime_simu_franka_fr_xaxpy_b(5 - i, e[qq - 1], b_A, (i + 6 * (qq -
            1)) + 2, work, i + 2);
        }

        for (qq = qp1; qq < 7; qq++) {
          realtime_simu_franka_f_xaxpy_bj(5 - i, -e[qq - 1] / e[i + 1], work, i
            + 2, b_A, (i + 6 * (qq - 1)) + 2);
        }
      }

      for (qq = qp1; qq < 7; qq++) {
        Vf[(qq + 6 * i) - 1] = e[qq - 1];
      }
    }
  }

  i = 4;
  b_s[5] = b_A[35];
  e[4] = b_A[34];
  e[5] = 0.0;
  for (qp1 = 0; qp1 < 6; qp1++) {
    U[qp1 + 30] = 0.0;
  }

  U[35] = 1.0;
  for (qp1 = 4; qp1 >= 0; qp1--) {
    qq = 6 * qp1 + qp1;
    if (b_s[qp1] != 0.0) {
      for (qp1jj = qp1 + 2; qp1jj < 7; qp1jj++) {
        qjj = ((qp1jj - 1) * 6 + qp1) + 1;
        realtime_simu_franka_fr3_xaxpy(6 - qp1, -(realtime_simu_franka_fr3_xdotc
          (6 - qp1, U, qq + 1, U, qjj) / U[qq]), qq + 1, U, qjj);
      }

      for (qp1jj = qp1 + 1; qp1jj < 7; qp1jj++) {
        qjj = (6 * qp1 + qp1jj) - 1;
        U[qjj] = -U[qjj];
      }

      U[qq]++;
      for (qq = 0; qq < qp1; qq++) {
        U[qq + 6 * qp1] = 0.0;
      }
    } else {
      for (qjj = 0; qjj < 6; qjj++) {
        U[qjj + 6 * qp1] = 0.0;
      }

      U[qq] = 1.0;
    }
  }

  for (qp1 = 5; qp1 >= 0; qp1--) {
    if ((qp1 + 1 <= 4) && (e[qp1] != 0.0)) {
      qq = (6 * qp1 + qp1) + 2;
      for (qjj = qp1 + 2; qjj < 7; qjj++) {
        qp1jj = ((qjj - 1) * 6 + qp1) + 2;
        realtime_simu_franka_fr3_xaxpy(5 - qp1, -(realtime_simu_franka_fr3_xdotc
          (5 - qp1, Vf, qq, Vf, qp1jj) / Vf[qq - 1]), qq, Vf, qp1jj);
      }
    }

    for (qq = 0; qq < 6; qq++) {
      Vf[qq + 6 * qp1] = 0.0;
    }

    Vf[qp1 + 6 * qp1] = 1.0;
  }

  qp1 = 0;
  nrm = 0.0;
  for (qq = 0; qq < 6; qq++) {
    ztest0 = e[qq];
    if (b_s[qq] != 0.0) {
      rt = fabs(b_s[qq]);
      r = b_s[qq] / rt;
      b_s[qq] = rt;
      if (qq + 1 < 6) {
        ztest0 /= r;
      }

      qjj = 6 * qq + 1;
      for (qp1jj = qjj; qp1jj <= qjj + 5; qp1jj++) {
        U[qp1jj - 1] *= r;
      }
    }

    if ((qq + 1 < 6) && (ztest0 != 0.0)) {
      rt = fabs(ztest0);
      r = rt / ztest0;
      ztest0 = rt;
      b_s[qq + 1] *= r;
      qjj = (qq + 1) * 6 + 1;
      for (qp1jj = qjj; qp1jj <= qjj + 5; qp1jj++) {
        Vf[qp1jj - 1] *= r;
      }
    }

    nrm = fmax(nrm, fmax(fabs(b_s[qq]), fabs(ztest0)));
    e[qq] = ztest0;
  }

  while ((i + 2 > 0) && (qp1 < 75)) {
    boolean_T exitg1;
    qq = i + 1;
    exitg1 = false;
    while (!(exitg1 || (qq == 0))) {
      ztest0 = fabs(e[qq - 1]);
      if ((ztest0 <= (fabs(b_s[qq - 1]) + fabs(b_s[qq])) *
           2.2204460492503131E-16) || ((ztest0 <= 1.0020841800044864E-292) ||
           ((qp1 > 20) && (ztest0 <= 2.2204460492503131E-16 * nrm)))) {
        e[qq - 1] = 0.0;
        exitg1 = true;
      } else {
        qq--;
      }
    }

    if (i + 1 == qq) {
      qp1jj = 4;
    } else {
      qjj = i + 2;
      qp1jj = i + 2;
      exitg1 = false;
      while ((!exitg1) && (qp1jj >= qq)) {
        qjj = qp1jj;
        if (qp1jj == qq) {
          exitg1 = true;
        } else {
          ztest0 = 0.0;
          if (qp1jj < i + 2) {
            ztest0 = fabs(e[qp1jj - 1]);
          }

          if (qp1jj > qq + 1) {
            ztest0 += fabs(e[qp1jj - 2]);
          }

          rt = fabs(b_s[qp1jj - 1]);
          if ((rt <= 2.2204460492503131E-16 * ztest0) || (rt <=
               1.0020841800044864E-292)) {
            b_s[qp1jj - 1] = 0.0;
            exitg1 = true;
          } else {
            qp1jj--;
          }
        }
      }

      if (qjj == qq) {
        qp1jj = 3;
      } else if (i + 2 == qjj) {
        qp1jj = 1;
      } else {
        qp1jj = 2;
        qq = qjj;
      }
    }

    switch (qp1jj) {
     case 1:
      ztest0 = e[i];
      e[i] = 0.0;
      for (qjj = i + 1; qjj >= qq + 1; qjj--) {
        realtime_simu_franka_fr3_xrotg(&b_s[qjj - 1], &ztest0, &r, &smm1);
        if (qjj > qq + 1) {
          rt = e[qjj - 2];
          ztest0 = rt * -smm1;
          e[qjj - 2] = rt * r;
        }

        realtime_simu_franka_fr3_xrot(Vf, 6 * (qjj - 1) + 1, 6 * (i + 1) + 1, r,
          smm1);
      }
      break;

     case 2:
      ztest0 = e[qq - 1];
      e[qq - 1] = 0.0;
      for (qjj = qq + 1; qjj <= i + 2; qjj++) {
        realtime_simu_franka_fr3_xrotg(&b_s[qjj - 1], &ztest0, &r, &smm1);
        rt = e[qjj - 1];
        ztest0 = rt * -smm1;
        e[qjj - 1] = rt * r;
        realtime_simu_franka_fr3_xrot(U, 6 * (qjj - 1) + 1, 6 * (qq - 1) + 1, r,
          smm1);
      }
      break;

     case 3:
      {
        real_T emm1;
        real_T shift;
        ztest0 = b_s[i + 1];
        rt = fmax(fmax(fmax(fmax(fabs(ztest0), fabs(b_s[i])), fabs(e[i])), fabs
                       (b_s[qq])), fabs(e[qq]));
        ztest0 /= rt;
        smm1 = b_s[i] / rt;
        emm1 = e[i] / rt;
        r = b_s[qq] / rt;
        smm1 = ((smm1 + ztest0) * (smm1 - ztest0) + emm1 * emm1) / 2.0;
        emm1 *= ztest0;
        emm1 *= emm1;
        if ((smm1 != 0.0) || (emm1 != 0.0)) {
          shift = sqrt(smm1 * smm1 + emm1);
          if (smm1 < 0.0) {
            shift = -shift;
          }

          shift = emm1 / (smm1 + shift);
        } else {
          shift = 0.0;
        }

        ztest0 = (r + ztest0) * (r - ztest0) + shift;
        rt = e[qq] / rt * r;
        for (qjj = qq + 1; qjj <= i + 1; qjj++) {
          realtime_simu_franka_fr3_xrotg(&ztest0, &rt, &r, &smm1);
          if (qjj > qq + 1) {
            e[qjj - 2] = ztest0;
          }

          rt = e[qjj - 1];
          emm1 = b_s[qjj - 1];
          e[qjj - 1] = rt * r - emm1 * smm1;
          ztest0 = smm1 * b_s[qjj];
          b_s[qjj] *= r;
          realtime_simu_franka_fr3_xrot(Vf, 6 * (qjj - 1) + 1, 6 * qjj + 1, r,
            smm1);
          b_s[qjj - 1] = emm1 * r + rt * smm1;
          realtime_simu_franka_fr3_xrotg(&b_s[qjj - 1], &ztest0, &r, &smm1);
          ztest0 = e[qjj - 1] * r + smm1 * b_s[qjj];
          b_s[qjj] = e[qjj - 1] * -smm1 + r * b_s[qjj];
          rt = smm1 * e[qjj];
          e[qjj] *= r;
          realtime_simu_franka_fr3_xrot(U, 6 * (qjj - 1) + 1, 6 * qjj + 1, r,
            smm1);
        }

        e[i] = ztest0;
        qp1++;
      }
      break;

     default:
      if (b_s[qq] < 0.0) {
        b_s[qq] = -b_s[qq];
        qp1 = 6 * qq + 1;
        for (qjj = qp1; qjj <= qp1 + 5; qjj++) {
          Vf[qjj - 1] = -Vf[qjj - 1];
        }
      }

      qp1 = qq + 1;
      while ((qq + 1 < 6) && (b_s[qq] < b_s[qp1])) {
        rt = b_s[qq];
        b_s[qq] = b_s[qp1];
        b_s[qp1] = rt;
        realtime_simu_franka_fr3_xswap(Vf, 6 * qq + 1, 6 * (qq + 1) + 1);
        realtime_simu_franka_fr3_xswap(U, 6 * qq + 1, 6 * (qq + 1) + 1);
        qq = qp1;
        qp1++;
      }

      qp1 = 0;
      i--;
      break;
    }
  }

  for (qp1 = 0; qp1 < 6; qp1++) {
    s[qp1] = b_s[qp1];
    for (i = 0; i < 6; i++) {
      V[i + 6 * qp1] = Vf[6 * qp1 + i];
    }
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_fr3_pinv(const real_T A[36], real_T X[36])
{
  real_T U[36];
  real_T V[36];
  real_T s[6];
  real_T absx;
  int32_T r;
  int32_T vcol;
  boolean_T p;
  p = true;
  for (r = 0; r < 36; r++) {
    absx = A[r];
    X[r] = 0.0;
    if (p && ((!rtIsInf(absx)) && (!rtIsNaN(absx)))) {
    } else {
      p = false;
    }
  }

  if (!p) {
    for (r = 0; r < 36; r++) {
      X[r] = (rtNaN);
    }
  } else {
    realtime_simu_franka_fr3_svd_ky(A, U, s, V);
    absx = fabs(s[0]);
    if (rtIsInf(absx) || rtIsNaN(absx)) {
      absx = (rtNaN);
    } else if (absx < 4.4501477170144028E-308) {
      absx = 4.94065645841247E-324;
    } else {
      frexp(absx, &vcol);
      absx = ldexp(1.0, vcol - 53);
    }

    absx *= 6.0;
    r = -1;
    vcol = 0;
    while ((vcol < 6) && (s[vcol] > absx)) {
      r++;
      vcol++;
    }

    if (r + 1 > 0) {
      int32_T ar;
      vcol = 1;
      for (int32_T j = 0; j <= r; j++) {
        absx = 1.0 / s[j];
        for (ar = vcol; ar <= vcol + 5; ar++) {
          V[ar - 1] *= absx;
        }

        vcol += 6;
      }

      for (vcol = 0; vcol <= 30; vcol += 6) {
        for (int32_T j = vcol + 1; j <= vcol + 6; j++) {
          X[j - 1] = 0.0;
        }
      }

      vcol = 0;
      for (int32_T j = 0; j <= 30; j += 6) {
        int32_T b;
        ar = -1;
        vcol++;
        b = 6 * r + vcol;
        for (int32_T ib = vcol; ib <= b; ib += 6) {
          for (int32_T b_ic = j + 1; b_ic <= j + 6; b_ic++) {
            X[b_ic - 1] += V[(ar + b_ic) - j] * U[ib - 1];
          }

          ar += 6;
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_f_mldivide(const real_T A[36], const real_T B
  [36], real_T Y[36])
{
  real_T b_A[36];
  real_T temp;
  int32_T ipiv[6];
  int32_T i;
  int32_T info;
  int32_T ipiv_0;
  int32_T jBcol;
  int32_T kAcol;
  int32_T temp_tmp;
  memcpy(&Y[0], &B[0], 36U * sizeof(real_T));
  memcpy(&b_A[0], &A[0], 36U * sizeof(real_T));
  realtime_simu_franka_fr_xzgetrf(b_A, ipiv, &info);
  for (i = 0; i < 5; i++) {
    ipiv_0 = ipiv[i];
    if (i + 1 != ipiv_0) {
      for (jBcol = 0; jBcol < 6; jBcol++) {
        temp_tmp = 6 * jBcol + i;
        temp = Y[temp_tmp];
        info = (6 * jBcol + ipiv_0) - 1;
        Y[temp_tmp] = Y[info];
        Y[info] = temp;
      }
    }
  }

  for (ipiv_0 = 0; ipiv_0 < 6; ipiv_0++) {
    jBcol = 6 * ipiv_0;
    for (temp_tmp = 0; temp_tmp < 6; temp_tmp++) {
      kAcol = 6 * temp_tmp;
      i = temp_tmp + jBcol;
      if (Y[i] != 0.0) {
        for (int32_T b_i = temp_tmp + 2; b_i < 7; b_i++) {
          info = (b_i + jBcol) - 1;
          Y[info] -= b_A[(b_i + kAcol) - 1] * Y[i];
        }
      }
    }
  }

  for (ipiv_0 = 0; ipiv_0 < 6; ipiv_0++) {
    jBcol = 6 * ipiv_0;
    for (temp_tmp = 5; temp_tmp >= 0; temp_tmp--) {
      kAcol = 6 * temp_tmp;
      i = temp_tmp + jBcol;
      temp = Y[i];
      if (temp != 0.0) {
        Y[i] = temp / b_A[temp_tmp + kAcol];
        for (int32_T b_i = 0; b_i < temp_tmp; b_i++) {
          info = b_i + jBcol;
          Y[info] -= b_A[b_i + kAcol] * Y[i];
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S11>/CT MATLAB Function' */
static void realtime_simu_franka_fr3_inv(const real_T x[36], real_T y[36])
{
  real_T b_x[36];
  int32_T ipiv[6];
  int32_T info;
  int32_T ipiv_0;
  int32_T jBcol;
  int32_T kAcol;
  int32_T pipk;
  int8_T p[6];
  for (info = 0; info < 36; info++) {
    y[info] = 0.0;
    b_x[info] = x[info];
  }

  realtime_simu_franka_fr_xzgetrf(b_x, ipiv, &info);
  for (info = 0; info < 6; info++) {
    p[info] = static_cast<int8_T>(info + 1);
  }

  for (info = 0; info < 5; info++) {
    ipiv_0 = ipiv[info];
    if (ipiv_0 > info + 1) {
      pipk = p[ipiv_0 - 1];
      p[ipiv_0 - 1] = p[info];
      p[info] = static_cast<int8_T>(pipk);
    }
  }

  for (pipk = 0; pipk < 6; pipk++) {
    ipiv_0 = (p[pipk] - 1) * 6;
    y[pipk + ipiv_0] = 1.0;
    for (jBcol = pipk + 1; jBcol < 7; jBcol++) {
      info = (ipiv_0 + jBcol) - 1;
      if (y[info] != 0.0) {
        for (int32_T i = jBcol + 1; i < 7; i++) {
          kAcol = (ipiv_0 + i) - 1;
          y[kAcol] -= b_x[((jBcol - 1) * 6 + i) - 1] * y[info];
        }
      }
    }
  }

  for (pipk = 0; pipk < 6; pipk++) {
    jBcol = 6 * pipk;
    for (int32_T i = 5; i >= 0; i--) {
      real_T tmp;
      kAcol = 6 * i;
      info = i + jBcol;
      tmp = y[info];
      if (tmp != 0.0) {
        y[info] = tmp / b_x[i + kAcol];
        for (int32_T b_i = 0; b_i < i; b_i++) {
          ipiv_0 = b_i + jBcol;
          y[ipiv_0] -= b_x[b_i + kAcol] * y[info];
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/PD+ MATLAB Function' */
static void realtime_simu_franka__qrsolve_l(const real_T A_data[], const int32_T
  A_size[2], real_T Y_data[], int32_T Y_size[2])
{
  real_T B[36];
  real_T b_A_data[30];
  real_T tau_data[5];
  real_T vn1_data[5];
  real_T vn2_data[5];
  real_T work_data[5];
  real_T scale;
  real_T smax;
  int32_T b_ix;
  int32_T ii;
  int32_T itemp;
  int32_T loop_ub;
  int32_T nmi;
  int32_T pvt;
  int32_T rankA;
  int8_T jpvt_data[5];
  static const int8_T v[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  int32_T b_A_size[2];
  rankA = A_size[1];
  b_A_size[1] = A_size[1];
  loop_ub = 6 * A_size[1];
  if (loop_ub - 1 >= 0) {
    memcpy(&b_A_data[0], &A_data[0], static_cast<uint32_T>(loop_ub) * sizeof
           (real_T));
  }

  loop_ub = A_size[1];
  if (loop_ub - 1 >= 0) {
    memset(&tau_data[0], 0, static_cast<uint32_T>(loop_ub) * sizeof(real_T));
  }

  if (A_size[1] == 0) {
  } else {
    real_T absxk;
    loop_ub = A_size[1];
    memset(&jpvt_data[0], 0, static_cast<uint32_T>(loop_ub) * sizeof(int8_T));
    loop_ub = A_size[1];
    for (int32_T b_A = 0; b_A < loop_ub; b_A++) {
      jpvt_data[b_A] = static_cast<int8_T>(b_A + 1);
    }

    loop_ub = A_size[1];
    memset(&work_data[0], 0, static_cast<uint32_T>(loop_ub) * sizeof(real_T));
    loop_ub = A_size[1];
    memset(&vn1_data[0], 0, static_cast<uint32_T>(loop_ub) * sizeof(real_T));
    loop_ub = A_size[1];
    memset(&vn2_data[0], 0, static_cast<uint32_T>(loop_ub) * sizeof(real_T));
    loop_ub = A_size[1];
    for (int32_T b_A = 0; b_A < loop_ub; b_A++) {
      ii = b_A * 6 + 1;
      smax = 0.0;
      scale = 3.3121686421112381E-170;
      for (nmi = ii; nmi <= ii + 5; nmi++) {
        absxk = fabs(A_data[nmi - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          smax = smax * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          smax += t * t;
        }
      }

      smax = scale * sqrt(smax);
      vn2_data[b_A] = smax;
      vn1_data[b_A] = smax;
    }

    loop_ub = A_size[1];
    for (int32_T b_A = 0; b_A < loop_ub; b_A++) {
      int32_T iy;
      ii = b_A * 6 + b_A;
      nmi = rankA - b_A;
      if (nmi < 1) {
        itemp = -1;
      } else {
        itemp = 0;
        if (nmi > 1) {
          smax = fabs(vn1_data[b_A]);
          for (pvt = 2; pvt <= nmi; pvt++) {
            scale = fabs(vn1_data[(b_A + pvt) - 1]);
            if (scale > smax) {
              itemp = pvt - 1;
              smax = scale;
            }
          }
        }
      }

      pvt = b_A + itemp;
      if (pvt != b_A) {
        b_ix = pvt * 6;
        iy = b_A * 6;
        for (int32_T h_k = 0; h_k < 6; h_k++) {
          int32_T temp_tmp;
          temp_tmp = b_ix + h_k;
          smax = b_A_data[temp_tmp];
          itemp = iy + h_k;
          b_A_data[temp_tmp] = b_A_data[itemp];
          b_A_data[itemp] = smax;
        }

        itemp = jpvt_data[pvt];
        jpvt_data[pvt] = jpvt_data[b_A];
        jpvt_data[b_A] = static_cast<int8_T>(itemp);
        vn1_data[pvt] = vn1_data[b_A];
        vn2_data[pvt] = vn2_data[b_A];
      }

      scale = b_A_data[ii];
      itemp = ii + 2;
      tau_data[b_A] = 0.0;
      smax = realtime_simu_franka_f_xnrm2_lt(5 - b_A, b_A_data, ii + 2);
      if (smax != 0.0) {
        smax = rt_hypotd_snf(b_A_data[ii], smax);
        if (b_A_data[ii] >= 0.0) {
          smax = -smax;
        }

        if (fabs(smax) < 1.0020841800044864E-292) {
          pvt = 0;
          do {
            pvt++;
            b_ix = (ii - b_A) + 6;
            for (iy = itemp; iy <= b_ix; iy++) {
              b_A_data[iy - 1] *= 9.9792015476736E+291;
            }

            smax *= 9.9792015476736E+291;
            scale *= 9.9792015476736E+291;
          } while ((fabs(smax) < 1.0020841800044864E-292) && (pvt < 20));

          smax = rt_hypotd_snf(scale, realtime_simu_franka_f_xnrm2_lt(5 - b_A,
            b_A_data, ii + 2));
          if (scale >= 0.0) {
            smax = -smax;
          }

          tau_data[b_A] = (smax - scale) / smax;
          scale = 1.0 / (scale - smax);
          for (iy = itemp; iy <= b_ix; iy++) {
            b_A_data[iy - 1] *= scale;
          }

          for (itemp = 0; itemp < pvt; itemp++) {
            smax *= 1.0020841800044864E-292;
          }

          scale = smax;
        } else {
          tau_data[b_A] = (smax - b_A_data[ii]) / smax;
          scale = 1.0 / (b_A_data[ii] - smax);
          pvt = (ii - b_A) + 6;
          for (b_ix = itemp; b_ix <= pvt; b_ix++) {
            b_A_data[b_ix - 1] *= scale;
          }

          scale = smax;
        }
      }

      b_A_data[ii] = scale;
      if (b_A + 1 < rankA) {
        smax = b_A_data[ii];
        b_A_data[ii] = 1.0;
        if (tau_data[b_A] != 0.0) {
          boolean_T exitg2;
          itemp = 6 - b_A;
          pvt = (ii - b_A) + 5;
          while ((itemp > 0) && (b_A_data[pvt] == 0.0)) {
            itemp--;
            pvt--;
          }

          nmi--;
          exitg2 = false;
          while ((!exitg2) && (nmi > 0)) {
            int32_T exitg1;
            pvt = ((nmi - 1) * 6 + ii) + 6;
            b_ix = pvt;
            do {
              exitg1 = 0;
              if (b_ix + 1 <= pvt + itemp) {
                if (b_A_data[b_ix] != 0.0) {
                  exitg1 = 1;
                } else {
                  b_ix++;
                }
              } else {
                nmi--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }

          nmi--;
        } else {
          itemp = 0;
          nmi = -1;
        }

        if (itemp > 0) {
          if (nmi + 1 != 0) {
            if (nmi >= 0) {
              memset(&work_data[0], 0, static_cast<uint32_T>(nmi + 1) * sizeof
                     (real_T));
            }

            pvt = (6 * nmi + ii) + 7;
            for (b_ix = ii + 7; b_ix <= pvt; b_ix += 6) {
              scale = 0.0;
              iy = (b_ix + itemp) - 1;
              for (int32_T h_k = b_ix; h_k <= iy; h_k++) {
                scale += b_A_data[(ii + h_k) - b_ix] * b_A_data[h_k - 1];
              }

              iy = div_nde_s32_floor((b_ix - ii) - 7, 6);
              work_data[iy] += scale;
            }
          }

          if (!(-tau_data[b_A] == 0.0)) {
            pvt = ii + 7;
            for (b_ix = 0; b_ix <= nmi; b_ix++) {
              scale = work_data[b_ix];
              if (scale != 0.0) {
                scale *= -tau_data[b_A];
                iy = itemp + pvt;
                for (int32_T h_k = pvt; h_k < iy; h_k++) {
                  b_A_data[h_k - 1] += b_A_data[(ii + h_k) - pvt] * scale;
                }
              }

              pvt += 6;
            }
          }
        }

        b_A_data[ii] = smax;
      }

      for (ii = b_A + 2; ii <= rankA; ii++) {
        nmi = (ii - 1) * 6 + b_A;
        smax = vn1_data[ii - 1];
        if (smax != 0.0) {
          scale = fabs(b_A_data[nmi]) / smax;
          scale = 1.0 - scale * scale;
          if (scale < 0.0) {
            scale = 0.0;
          }

          absxk = smax / vn2_data[ii - 1];
          absxk = absxk * absxk * scale;
          if (absxk <= 1.4901161193847656E-8) {
            vn1_data[ii - 1] = realtime_simu_franka_f_xnrm2_lt(5 - b_A, b_A_data,
              nmi + 2);
            vn2_data[ii - 1] = vn1_data[ii - 1];
          } else {
            vn1_data[ii - 1] = smax * sqrt(scale);
          }
        }
      }
    }
  }

  rankA = 0;
  if (A_size[1] > 0) {
    while ((rankA < b_A_size[1]) && (!(fabs(b_A_data[6 * rankA + rankA]) <=
             1.3322676295501878E-14 * fabs(b_A_data[0])))) {
      rankA++;
    }
  }

  Y_size[0] = static_cast<int8_T>(A_size[1]);
  Y_size[1] = 6;
  loop_ub = static_cast<int8_T>(A_size[1]) * 6;
  if (loop_ub - 1 >= 0) {
    memset(&Y_data[0], 0, static_cast<uint32_T>(loop_ub) * sizeof(real_T));
  }

  for (itemp = 0; itemp < 36; itemp++) {
    B[itemp] = v[itemp];
  }

  loop_ub = A_size[1];
  for (int32_T b_A = 0; b_A < loop_ub; b_A++) {
    if (tau_data[b_A] != 0.0) {
      for (ii = 0; ii < 6; ii++) {
        nmi = 6 * ii + b_A;
        smax = B[nmi];
        scale = smax;
        for (itemp = b_A + 2; itemp < 7; itemp++) {
          scale += B[(6 * ii + itemp) - 1] * b_A_data[(6 * b_A + itemp) - 1];
        }

        scale *= tau_data[b_A];
        if (scale != 0.0) {
          B[nmi] = smax - scale;
          for (nmi = b_A + 2; nmi < 7; nmi++) {
            itemp = (6 * ii + nmi) - 1;
            B[itemp] -= b_A_data[(6 * b_A + nmi) - 1] * scale;
          }
        }
      }
    }
  }

  for (loop_ub = 0; loop_ub < 6; loop_ub++) {
    for (int32_T b_A = 0; b_A < rankA; b_A++) {
      Y_data[(jpvt_data[b_A] + Y_size[0] * loop_ub) - 1] = B[6 * loop_ub + b_A];
    }

    for (int32_T b_A = rankA; b_A >= 1; b_A--) {
      ii = jpvt_data[b_A - 1] - 1;
      itemp = Y_size[0] * loop_ub;
      pvt = ii + itemp;
      b_ix = (b_A - 1) * 6;
      Y_data[pvt] /= b_A_data[(b_A + b_ix) - 1];
      for (nmi = 0; nmi <= b_A - 2; nmi++) {
        pvt = (jpvt_data[nmi] + itemp) - 1;
        Y_data[pvt] -= Y_data[ii + itemp] * b_A_data[nmi + b_ix];
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/PD+ MATLAB Function' */
static void realtime_simu_frank_mldivide_d0(const real_T A_data[], const int32_T
  A_size[2], real_T Y_data[], int32_T Y_size[2])
{
  real_T B[36];
  real_T b_A_data[36];
  real_T tmp_data[30];
  int8_T ipiv[6];
  static const int8_T f[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  int32_T tmp_size[2];
  if (A_size[1] == 0) {
    Y_size[0] = 0;
    Y_size[1] = 6;
  } else if (A_size[1] == 6) {
    real_T smax;
    int32_T jA;
    int32_T jBcol;
    int32_T jj;
    int32_T kAcol;
    int32_T temp;
    for (jA = 0; jA < 36; jA++) {
      b_A_data[jA] = A_data[jA];
      B[jA] = f[jA];
    }

    for (jA = 0; jA < 6; jA++) {
      ipiv[jA] = static_cast<int8_T>(jA + 1);
    }

    for (int32_T Y = 0; Y < 5; Y++) {
      int8_T ipiv_0;
      ipiv_0 = ipiv[Y];
      jj = Y * 7;
      jA = 6 - Y;
      jBcol = 0;
      smax = fabs(b_A_data[jj]);
      for (temp = 2; temp <= jA; temp++) {
        real_T s;
        s = fabs(b_A_data[(jj + temp) - 1]);
        if (s > smax) {
          jBcol = temp - 1;
          smax = s;
        }
      }

      if (b_A_data[jj + jBcol] != 0.0) {
        if (jBcol != 0) {
          jBcol += Y;
          ipiv_0 = static_cast<int8_T>(jBcol + 1);
          for (temp = 0; temp < 6; temp++) {
            kAcol = temp * 6 + Y;
            smax = b_A_data[kAcol];
            jA = temp * 6 + jBcol;
            b_A_data[kAcol] = b_A_data[jA];
            b_A_data[jA] = smax;
          }
        }

        jA = (jj - Y) + 6;
        for (jBcol = jj + 2; jBcol <= jA; jBcol++) {
          b_A_data[jBcol - 1] /= b_A_data[jj];
        }
      }

      jA = jj + 8;
      jBcol = 4 - Y;
      for (temp = 0; temp <= jBcol; temp++) {
        smax = b_A_data[(temp * 6 + jj) + 6];
        if (smax != 0.0) {
          kAcol = (jA - Y) + 4;
          for (int32_T ijA = jA; ijA <= kAcol; ijA++) {
            b_A_data[ijA - 1] += b_A_data[((jj + ijA) - jA) + 1] * -smax;
          }
        }

        jA += 6;
      }

      if (Y + 1 != ipiv_0) {
        for (jA = 0; jA < 6; jA++) {
          jBcol = 6 * jA + Y;
          temp = static_cast<int32_T>(B[jBcol]);
          jj = (6 * jA + ipiv_0) - 1;
          B[jBcol] = B[jj];
          B[jj] = temp;
        }
      }

      ipiv[Y] = ipiv_0;
    }

    for (int32_T Y = 0; Y < 6; Y++) {
      jBcol = 6 * Y;
      for (temp = 0; temp < 6; temp++) {
        kAcol = 6 * temp;
        jA = temp + jBcol;
        if (B[jA] != 0.0) {
          for (int32_T ijA = temp + 2; ijA < 7; ijA++) {
            jj = (ijA + jBcol) - 1;
            B[jj] -= b_A_data[(ijA + kAcol) - 1] * B[jA];
          }
        }
      }
    }

    for (int32_T Y = 0; Y < 6; Y++) {
      jBcol = 6 * Y;
      for (temp = 5; temp >= 0; temp--) {
        kAcol = 6 * temp;
        jA = temp + jBcol;
        smax = B[jA];
        if (smax != 0.0) {
          B[jA] = smax / b_A_data[temp + kAcol];
          for (int32_T ijA = 0; ijA < temp; ijA++) {
            jj = ijA + jBcol;
            B[jj] -= b_A_data[ijA + kAcol] * B[jA];
          }
        }
      }
    }

    Y_size[0] = 6;
    Y_size[1] = 6;
    memcpy(&Y_data[0], &B[0], 36U * sizeof(real_T));
  } else {
    int32_T jj;
    realtime_simu_franka__qrsolve_l(A_data, A_size, tmp_data, tmp_size);
    Y_size[0] = tmp_size[0];
    Y_size[1] = 6;
    jj = tmp_size[0] * 6;
    if (jj - 1 >= 0) {
      memcpy(&Y_data[0], &tmp_data[0], static_cast<uint32_T>(jj) * sizeof(real_T));
    }
  }
}

/* Function for MATLAB Function: '<S12>/PD+ MATLAB Function' */
static void realtime_simu_franka_fr3_mtimes(const real_T A_data[], const int32_T
  A_size[2], const real_T B_data[], const int32_T B_size[2], real_T C[36])
{
  int32_T b;
  b = A_size[1];
  for (int32_T j = 0; j < 6; j++) {
    int32_T boffset;
    int32_T coffset;
    coffset = j * 6;
    boffset = j * B_size[0];
    for (int32_T i = 0; i < 6; i++) {
      C[coffset + i] = 0.0;
    }

    for (int32_T i = 0; i < b; i++) {
      real_T bkj;
      int32_T aoffset;
      aoffset = i * 6;
      bkj = B_data[boffset + i];
      for (int32_T b_i = 0; b_i < 6; b_i++) {
        int32_T C_tmp;
        C_tmp = coffset + b_i;
        C[C_tmp] += A_data[aoffset + b_i] * bkj;
      }
    }
  }
}

/* Function for MATLAB Function: '<S12>/PD+ MATLAB Function' */
static int32_T realtime_simu_franka_local_rank(const real_T A[36])
{
  real_T s[6];
  int32_T i;
  int32_T irank;
  boolean_T p;
  irank = 0;
  p = true;
  for (i = 0; i < 36; i++) {
    real_T A_0;
    A_0 = A[i];
    if (p && ((!rtIsInf(A_0)) && (!rtIsNaN(A_0)))) {
    } else {
      p = false;
    }
  }

  if (p) {
    realtime_simu_franka_fr_svd_kyc(A, s);
  } else {
    for (i = 0; i < 6; i++) {
      s[i] = (rtNaN);
    }
  }

  i = 0;
  while ((i < 6) && (s[i] > 0.1)) {
    irank++;
    i++;
  }

  return irank;
}

/* Model step function */
void realtime_simu_franka_fr3_step(void)
{
  /* local block i/o variables */
  boolean_T rtb_NOT;
  boolean_T rtb_NOT_p;

  {
    real_T x[302];
    real_T f_data[301];
    real_T JJ_colin[49];
    real_T JJ_colin_0[49];
    real_T a[49];
    real_T J_tilde[42];
    real_T J_bar_data[36];
    real_T J_pinv[36];
    real_T J_red[36];
    real_T J_tilde_0[36];
    real_T J_tilde_1[36];
    real_T Kd1[36];
    real_T Kp1[36];
    real_T S_new[36];
    real_T T_bar_data[36];
    real_T U[36];
    real_T b_a[36];
    real_T rtb_en_arr[11];
    real_T R[9];
    real_T rtb_x_d_R_d[9];
    real_T JJ_colin_1[7];
    real_T rtb_q_ref[7];
    real_T tmp[7];
    real_T (*lastU)[7];
    real_T Kp1_0[6];
    real_T b_a_0[6];
    real_T q_p_red[6];
    real_T rtb_ManualSwitch_g[6];
    real_T rtb_ManualSwitch_o[6];
    real_T x_d_pp[6];
    real_T x_d_pp_0[6];
    real_T x_err[6];
    real_T x_err_p[6];
    real_T e_a;
    real_T epsilon;
    real_T h_a;
    real_T i_a;
    real_T rtb_ManualSwitch1;
    real_T rtb_x_d_q_d_idx_3;
    real_T s;
    real_T s2;
    real_T s3;
    real_T s4;
    real_T time_end;
    int32_T ii_data[36];
    int32_T jj_data[36];
    int32_T aoffset;
    int32_T b;
    int32_T coffset_tmp;
    int32_T i;
    int32_T j;
    int32_T jA;
    int32_T jj;
    int8_T b_data[6];
    int8_T c_data[6];
    int8_T c_data_0[6];
    int8_T c[2];
    boolean_T U_0[36];
    boolean_T idx[6];
    boolean_T isodd;
    static const real_T b_0[35] = { 0.0, -0.78539816339744828, 0.0,
      -2.3561944901923448, 0.0, 1.5707963267948966, 0.78539816339744828,
      -4.5750995329742461E-8, 0.78100615355417979, 0.0, -0.46700267477940316,
      3.646713069579081E-7, 1.2480018944761606, 0.78500000720079022, 1.067816,
      0.634589, 0.0, -2.025387, 0.957002, 3.316042, 1.264367, -0.079543,
      -0.811266, 0.0, -1.467184, 1.283216, 1.668681, 0.686408, 1.067816,
      0.634589, 0.0, -2.025387, 0.957002, 3.316042, 1.264367 };

    static const int8_T l[6] = { 0, 1, 3, 4, 5, 6 };

    static const real_T c_0[7] = { 0.0, -0.78539816339744828, 0.0,
      -2.3561944901923448, 0.0, 1.5707963267948966, 0.78539816339744828 };

    static const real_T p_a[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

    static const int8_T m_a[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

    int32_T J_bar_size[2];
    int32_T T_bar_size[2];
    int32_T tmp_size[2];

    /* Reset subsysRan breadcrumbs */
    srClearBC(realtime_simu_franka_fr3_DW.CTControllerSubsystem_SubsysRan);

    /* Reset subsysRan breadcrumbs */
    srClearBC(realtime_simu_franka_fr3_DW.PDControllerSubsystem_SubsysRan);

    /* Reset subsysRan breadcrumbs */
    srClearBC(realtime_simu_franka_fr3_DW.EKF_SubsysRanBC_p);

    /* Reset subsysRan breadcrumbs */
    srClearBC(realtime_simu_franka_fr3_DW.noEKF.noEKF_SubsysRanBC);

    /* Reset subsysRan breadcrumbs */
    srClearBC(realtime_simu_franka_fr3_DW.EKF_SubsysRanBC);

    /* Reset subsysRan breadcrumbs */
    srClearBC(realtime_simu_franka_fr3_DW.jointspacectlsubsys_SubsysRanBC);

    /* Reset subsysRan breadcrumbs */
    srClearBC(realtime_simu_franka_fr3_DW.tau_subsystem_SubsysRanBC);

    /* MATLAB Function: '<S26>/MATLAB Function' incorporates:
     *  Memory: '<S26>/filter window'
     */
    memcpy(&f_data[0], &realtime_simu_franka_fr3_DW.filterwindow_PreviousInput[0],
           301U * sizeof(real_T));
    if (!realtime_simu_franka_fr3_DW.time_start_not_empty) {
      realtime_simu_franka_fr3_tic();
      realtime_simu_franka_fr3_DW.time_start_not_empty = true;
    }

    if (realtime_simu_franka_fr3_DW.cnt_g > 1.0) {
      time_end = realtime_simu_franka_fr3_toc();
      realtime_simu_franka_fr3_B.freq_per_step = 1.0 / (time_end -
        realtime_simu_franka_fr3_DW.time_start);
      realtime_simu_franka_fr3_B.freq_per_step_mean = 0.0;
      for (jj = 0; jj < 301; jj++) {
        realtime_simu_franka_fr3_B.freq_per_step_mean +=
          realtime_simu_franka_fr3_P.param_savgol.bT[jj] *
          realtime_simu_franka_fr3_DW.filterwindow_PreviousInput[jj];
      }

      realtime_simu_franka_fr3_DW.time_start = time_end;
    } else {
      realtime_simu_franka_fr3_B.freq_per_step = 0.0;
      realtime_simu_franka_fr3_B.freq_per_step_mean = 0.0;
    }

    if (realtime_simu_franka_fr3_DW.cnt_g <
        realtime_simu_franka_fr3_P.param_savgol.N) {
      realtime_simu_franka_fr3_DW.cnt_g++;
      if (realtime_simu_franka_fr3_DW.cnt_g > 301.0) {
        aoffset = -1;
        b = -1;
      } else {
        aoffset = static_cast<int32_T>(realtime_simu_franka_fr3_DW.cnt_g) - 2;
        b = 300;
      }

      memcpy(&x[0], &realtime_simu_franka_fr3_DW.filterwindow_PreviousInput[0],
             301U * sizeof(real_T));
      x[301] = realtime_simu_franka_fr3_B.freq_per_step;
      time_end = x[0];
      for (jj = 0; jj < 301; jj++) {
        time_end += x[jj + 1];
      }

      time_end /= 302.0;
      b -= aoffset;
      for (jj = 0; jj < b; jj++) {
        f_data[(aoffset + jj) + 1] = time_end;
      }
    }

    realtime_simu_franka_fr3_B.f_data_o[0] =
      realtime_simu_franka_fr3_B.freq_per_step;
    memcpy(&realtime_simu_franka_fr3_B.f_data_o[1], &f_data[0], 300U * sizeof
           (real_T));

    /* End of MATLAB Function: '<S26>/MATLAB Function' */

    /* S-Function (get_robot_state): '<S5>/Get Robot State2' */
    {
      // Wait for the control thread signal
      if ((bool)realtime_simu_franka_fr3_DW.GetRobotState2_DWORK1 &&
          simulinkPandaRobot_1721602.getControlThreadHasBeenSpawned()) {
        simulinkPandaRobot_1721602.waitForControlThreadStep();
      }

      // If control loop threw exeption terminate execution
      simulinkPandaRobot_1721602.checkIfAndHandleException();
      if (!simulinkPandaRobot_1721602.getCurrentlyInFirstControlStep()) {
        std::vector<std::string> output_signals;
        signalsStringToSignalsVector(output_signals,"q dq O_F_ext_hat_K");

        //copy outputs
        simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 0,
          &realtime_simu_franka_fr3_B.GetRobotState2_o1[0]);
        simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 1,
          &realtime_simu_franka_fr3_B.GetRobotState2_o2[0]);
        simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 2,
          &realtime_simu_franka_fr3_B.GetRobotState2_o3[0]);
      }
    }

    /* Outputs for Enabled SubSystem: '<Root>/jointspace ctl subsys' incorporates:
     *  EnablePort: '<S8>/Enable'
     */
    /* Clock: '<S5>/Clock' incorporates:
     *  Clock: '<S8>/Clock'
     *  Derivative: '<Root>/Derivative'
     */
    epsilon = realtime_simu_franka_fr3_M->Timing.t[0];

    /* End of Outputs for SubSystem: '<Root>/jointspace ctl subsys' */
    for (i = 0; i < 7; i++) {
      /* Switch: '<S5>/Switch' incorporates:
       *  Clock: '<S5>/Clock'
       */
      if (epsilon > realtime_simu_franka_fr3_P.Switch_Threshold) {
        /* Switch: '<S5>/Switch' */
        realtime_simu_franka_fr3_B.Switch[i] =
          realtime_simu_franka_fr3_B.GetRobotState2_o1[i];
      } else {
        /* Switch: '<S5>/Switch' incorporates:
         *  Constant: '<S5>/Constant'
         */
        realtime_simu_franka_fr3_B.Switch[i] =
          realtime_simu_franka_fr3_P.q_init[i];
      }

      /* End of Switch: '<S5>/Switch' */
    }

    /* S-Function (s_function_opti_robot_model_bus_fun): '<S9>/robot model s-function2' */

    /* Level2 S-Function Block: '<S9>/robot model s-function2' (s_function_opti_robot_model_bus_fun) */
    {
      SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[8];
      sfcnOutputs(rts,0);
    }

    /* Derivative: '<Root>/Derivative' */
    if ((realtime_simu_franka_fr3_DW.TimeStampA >= epsilon) &&
        (realtime_simu_franka_fr3_DW.TimeStampB >= epsilon)) {
      /* Derivative: '<Root>/Derivative' */
      for (i = 0; i < 7; i++) {
        realtime_simu_franka_fr3_B.Derivative[i] = 0.0;
      }
    } else {
      time_end = realtime_simu_franka_fr3_DW.TimeStampA;
      lastU = &realtime_simu_franka_fr3_DW.LastUAtTimeA;
      if (realtime_simu_franka_fr3_DW.TimeStampA <
          realtime_simu_franka_fr3_DW.TimeStampB) {
        if (realtime_simu_franka_fr3_DW.TimeStampB < epsilon) {
          time_end = realtime_simu_franka_fr3_DW.TimeStampB;
          lastU = &realtime_simu_franka_fr3_DW.LastUAtTimeB;
        }
      } else if (realtime_simu_franka_fr3_DW.TimeStampA >= epsilon) {
        time_end = realtime_simu_franka_fr3_DW.TimeStampB;
        lastU = &realtime_simu_franka_fr3_DW.LastUAtTimeB;
      }

      time_end = epsilon - time_end;

      /* Derivative: '<Root>/Derivative' */
      for (i = 0; i < 7; i++) {
        realtime_simu_franka_fr3_B.Derivative[i] =
          (realtime_simu_franka_fr3_B.GetRobotState2_o2[i] - (*lastU)[i]) /
          time_end;
      }
    }

    /* MATLAB Function: '<S9>/Robot model bus' incorporates:
     *  S-Function (s_function_opti_robot_model_bus_fun): '<S9>/robot model s-function2'
     */
    memcpy(&realtime_simu_franka_fr3_B.robot_model_a.H[0],
           &realtime_simu_franka_fr3_B.robotmodelsfunction2_o1[0], sizeof(real_T)
           << 4U);
    memcpy(&realtime_simu_franka_fr3_B.robot_model_a.J[0],
           &realtime_simu_franka_fr3_B.robotmodelsfunction2_o2[0], 42U * sizeof
           (real_T));
    memcpy(&realtime_simu_franka_fr3_B.robot_model_a.J_p[0],
           &realtime_simu_franka_fr3_B.robotmodelsfunction2_o3[0], 42U * sizeof
           (real_T));
    memcpy(&realtime_simu_franka_fr3_B.robot_model_a.M[0],
           &realtime_simu_franka_fr3_B.robotmodelsfunction2_o4[0], 49U * sizeof
           (real_T));
    memcpy(&realtime_simu_franka_fr3_B.robot_model_a.C[0],
           &realtime_simu_franka_fr3_B.robotmodelsfunction2_o6[0], 49U * sizeof
           (real_T));
    for (i = 0; i < 7; i++) {
      realtime_simu_franka_fr3_B.robot_model_a.q[i] =
        realtime_simu_franka_fr3_B.Switch[i];
      realtime_simu_franka_fr3_B.robot_model_a.q_p[i] =
        realtime_simu_franka_fr3_B.GetRobotState2_o2[i];
      realtime_simu_franka_fr3_B.robot_model_a.q_pp[i] =
        realtime_simu_franka_fr3_B.Derivative[i];
      realtime_simu_franka_fr3_B.robot_model_a.C_rnea[i] =
        realtime_simu_franka_fr3_B.robotmodelsfunction2_o5[i];
      realtime_simu_franka_fr3_B.robot_model_a.g[i] =
        realtime_simu_franka_fr3_B.robotmodelsfunction2_o7[i];

      /* Delay: '<S6>/uk_prev' incorporates:
       *  S-Function (s_function_opti_robot_model_bus_fun): '<S9>/robot model s-function2'
       */
      if (realtime_simu_franka_fr3_DW.icLoad) {
        realtime_simu_franka_fr3_DW.uk_prev_DSTATE[i] =
          realtime_simu_franka_fr3_B.robot_model_a.g[i];
      }

      /* Delay: '<S6>/uk_prev' */
      realtime_simu_franka_fr3_B.uk_prev[i] =
        realtime_simu_franka_fr3_DW.uk_prev_DSTATE[i];
    }

    /* End of MATLAB Function: '<S9>/Robot model bus' */

    /* ManualSwitch: '<Root>/Manual Switch2' incorporates:
     *  Constant: '<Root>/off'
     *  Constant: '<Root>/on'
     */
    if (realtime_simu_franka_fr3_P.ManualSwitch2_CurrentSetting == 1) {
      time_end = realtime_simu_franka_fr3_P.on_Value_a;
    } else {
      time_end = realtime_simu_franka_fr3_P.off_Value_l;
    }

    /* End of ManualSwitch: '<Root>/Manual Switch2' */

    /* Outputs for Enabled SubSystem: '<S6>/EKF' incorporates:
     *  EnablePort: '<S29>/Enable'
     */
    if (time_end > 0.0) {
      /* Constant: '<S29>/Constant' */
      memcpy(&realtime_simu_franka_fr3_B.Constant[0],
             &realtime_simu_franka_fr3_P.Constant_Value[0], 196U * sizeof(real_T));
      for (i = 0; i < 7; i++) {
        /* SignalConversion generated from: '<S29>/Reduced System sfun casadi solve' */
        realtime_simu_franka_fr3_B.y_kxk_measured[i] =
          realtime_simu_franka_fr3_B.robot_model_a.q[i];
        realtime_simu_franka_fr3_B.y_kxk_measured[i + 7] =
          realtime_simu_franka_fr3_B.robot_model_a.q_p[i];
      }

      for (i = 0; i < 14; i++) {
        /* Delay: '<S29>/xk_minus' incorporates:
         *  SignalConversion generated from: '<S29>/Reduced System sfun casadi solve'
         */
        if (realtime_simu_franka_fr3_DW.icLoad_d) {
          realtime_simu_franka_fr3_DW.xk_minus_DSTATE[i] =
            realtime_simu_franka_fr3_B.y_kxk_measured[i];
        }

        /* Delay: '<S29>/xk_minus' */
        realtime_simu_franka_fr3_B.xk_minus[i] =
          realtime_simu_franka_fr3_DW.xk_minus_DSTATE[i];
      }

      for (i = 0; i < 196; i++) {
        /* Constant: '<S29>/Constant1' */
        realtime_simu_franka_fr3_B.Constant1[i] =
          realtime_simu_franka_fr3_P.Constant1_Value[i];

        /* Delay: '<S29>/Pk_minus' incorporates:
         *  Constant: '<S29>/Constant2'
         */
        if (realtime_simu_franka_fr3_DW.icLoad_b) {
          realtime_simu_franka_fr3_DW.Pk_minus_DSTATE[i] =
            realtime_simu_franka_fr3_P.param_EKF.P0[i];
        }

        /* Delay: '<S29>/Pk_minus' */
        realtime_simu_franka_fr3_B.Pk_minus[i] =
          realtime_simu_franka_fr3_DW.Pk_minus_DSTATE[i];
      }

      /* S-Function (s_function_opti_ekf_fun): '<S29>/Reduced System sfun casadi solve' */

      /* Level2 S-Function Block: '<S29>/Reduced System sfun casadi solve' (s_function_opti_ekf_fun) */
      {
        SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[3];
        sfcnOutputs(rts,0);
      }

      /* MATLAB Function: '<S29>/get EKF joint values' */
      realtime_simu_getEKFjointvalues(&realtime_simu_franka_fr3_B.robot_model_a,
        realtime_simu_franka_fr3_B.xk_plus,
        &realtime_simu_franka_fr3_B.sf_getEKFjointvalues_g);

      /* S-Function (s_function_opti_robot_model_bus_fun): '<S32>/robot model s-function2' */

      /* Level2 S-Function Block: '<S32>/robot model s-function2' (s_function_opti_robot_model_bus_fun) */
      {
        SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[4];
        sfcnOutputs(rts,0);
      }

      /* MATLAB Function: '<S32>/zero fixed states' */
      realtime_simu_f_zerofixedstates
        (realtime_simu_franka_fr3_B.sf_getEKFjointvalues_g.q,
         realtime_simu_franka_fr3_B.sf_getEKFjointvalues_g.q_p,
         realtime_simu_franka_fr3_B.sf_getEKFjointvalues_g.q_pp,
         &realtime_simu_franka_fr3_B.sf_zerofixedstates_i);

      /* S-Function (s_function_opti_robot_model_bus_fun): '<S32>/robot model s-function1' */

      /* Level2 S-Function Block: '<S32>/robot model s-function1' (s_function_opti_robot_model_bus_fun) */
      {
        SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[5];
        sfcnOutputs(rts,0);
      }

      /* MATLAB Function: '<S32>/Robot model bus1' */
      realtime_simu_fra_Robotmodelbus
        (realtime_simu_franka_fr3_B.sf_zerofixedstates_i.q_red,
         realtime_simu_franka_fr3_B.sf_zerofixedstates_i.q_p_red,
         realtime_simu_franka_fr3_B.sf_zerofixedstates_i.q_pp_red,
         realtime_simu_franka_fr3_B.robotmodelsfunction1_o1_c,
         realtime_simu_franka_fr3_B.robotmodelsfunction1_o2_e,
         realtime_simu_franka_fr3_B.robotmodelsfunction1_o3_c,
         realtime_simu_franka_fr3_B.robotmodelsfunction1_o4_p,
         realtime_simu_franka_fr3_B.robotmodelsfunction1_o5_b,
         realtime_simu_franka_fr3_B.robotmodelsfunction1_o6_d,
         realtime_simu_franka_fr3_B.robotmodelsfunction1_o7_m,
         &realtime_simu_franka_fr3_B.sf_Robotmodelbus1_h);

      /* MATLAB Function: '<S32>/Robot model bus' */
      realtime_simu_fra_Robotmodelbus
        (realtime_simu_franka_fr3_B.sf_getEKFjointvalues_g.q,
         realtime_simu_franka_fr3_B.sf_getEKFjointvalues_g.q_p,
         realtime_simu_franka_fr3_B.sf_getEKFjointvalues_g.q_pp,
         realtime_simu_franka_fr3_B.robotmodelsfunction2_o1_o,
         realtime_simu_franka_fr3_B.robotmodelsfunction2_o2_c,
         realtime_simu_franka_fr3_B.robotmodelsfunction2_o3_p,
         realtime_simu_franka_fr3_B.robotmodelsfunction2_o4_m,
         realtime_simu_franka_fr3_B.robotmodelsfunction2_o5_n,
         realtime_simu_franka_fr3_B.robotmodelsfunction2_o6_g,
         realtime_simu_franka_fr3_B.robotmodelsfunction2_o7_e,
         &realtime_simu_franka_fr3_B.sf_Robotmodelbus_e);

      /* Merge: '<S6>/Merge' incorporates:
       *  SignalConversion generated from: '<S29>/robot_model_EKF'
       */
      realtime_simu_franka_fr3_B.Merge =
        realtime_simu_franka_fr3_B.sf_Robotmodelbus_e.robot_model_b;

      /* Update for Delay: '<S29>/xk_minus' incorporates:
       *  S-Function (s_function_opti_ekf_fun): '<S29>/Reduced System sfun casadi solve'
       */
      realtime_simu_franka_fr3_DW.icLoad_d = false;
      memcpy(&realtime_simu_franka_fr3_DW.xk_minus_DSTATE[0],
             &realtime_simu_franka_fr3_B.xkp1_minus[0], 14U * sizeof(real_T));

      /* Update for Delay: '<S29>/Pk_minus' incorporates:
       *  S-Function (s_function_opti_ekf_fun): '<S29>/Reduced System sfun casadi solve'
       */
      realtime_simu_franka_fr3_DW.icLoad_b = false;
      memcpy(&realtime_simu_franka_fr3_DW.Pk_minus_DSTATE[0],
             &realtime_simu_franka_fr3_B.Pkp1_minus[0], 196U * sizeof(real_T));
      srUpdateBC(realtime_simu_franka_fr3_DW.EKF_SubsysRanBC);
    }

    /* End of Outputs for SubSystem: '<S6>/EKF' */

    /* Logic: '<S6>/NOT' */
    rtb_NOT = !(time_end != 0.0);

    /* Outputs for Enabled SubSystem: '<S6>/no EKF' */
    realtime_simu_franka_fr3_noEKF(rtb_NOT,
      &realtime_simu_franka_fr3_B.robot_model_a,
      &realtime_simu_franka_fr3_B.Merge, &realtime_simu_franka_fr3_DW.noEKF_b);

    /* End of Outputs for SubSystem: '<S6>/no EKF' */
    for (i = 0; i < 7; i++) {
      /* Delay: '<S13>/uk_prev' */
      if (realtime_simu_franka_fr3_DW.icLoad_f) {
        realtime_simu_franka_fr3_DW.uk_prev_DSTATE_b[i] =
          realtime_simu_franka_fr3_B.Merge.g[i];
      }

      /* Delay: '<S13>/uk_prev' */
      realtime_simu_franka_fr3_B.uk_prev_e[i] =
        realtime_simu_franka_fr3_DW.uk_prev_DSTATE_b[i];
    }

    /* ManualSwitch: '<S2>/Manual Switch1' incorporates:
     *  Constant: '<S2>/off'
     *  Constant: '<S2>/on'
     */
    if (realtime_simu_franka_fr3_P.ManualSwitch1_CurrentSetting_m == 1) {
      rtb_ManualSwitch1 = realtime_simu_franka_fr3_P.on_Value;
    } else {
      rtb_ManualSwitch1 = realtime_simu_franka_fr3_P.off_Value;
    }

    /* End of ManualSwitch: '<S2>/Manual Switch1' */

    /* Outputs for Enabled SubSystem: '<S13>/EKF' incorporates:
     *  EnablePort: '<S16>/Enable'
     */
    if (rtb_ManualSwitch1 > 0.0) {
      /* Constant: '<S16>/Constant' */
      memcpy(&realtime_simu_franka_fr3_B.Constant_p[0],
             &realtime_simu_franka_fr3_P.param_EKF.Rk[0], 196U * sizeof(real_T));
      for (i = 0; i < 7; i++) {
        /* SignalConversion generated from: '<S16>/Reduced System sfun casadi solve' */
        realtime_simu_franka_fr3_B.y_kxk_measured_n[i] =
          realtime_simu_franka_fr3_B.Merge.q[i];
        realtime_simu_franka_fr3_B.y_kxk_measured_n[i + 7] =
          realtime_simu_franka_fr3_B.Merge.q_p[i];
      }

      for (i = 0; i < 14; i++) {
        /* Delay: '<S16>/xk_minus' incorporates:
         *  SignalConversion generated from: '<S16>/Reduced System sfun casadi solve'
         */
        if (realtime_simu_franka_fr3_DW.icLoad_c) {
          realtime_simu_franka_fr3_DW.xk_minus_DSTATE_m[i] =
            realtime_simu_franka_fr3_B.y_kxk_measured_n[i];
        }

        /* Delay: '<S16>/xk_minus' */
        realtime_simu_franka_fr3_B.xk_minus_d[i] =
          realtime_simu_franka_fr3_DW.xk_minus_DSTATE_m[i];
      }

      for (i = 0; i < 196; i++) {
        /* Constant: '<S16>/Constant1' */
        realtime_simu_franka_fr3_B.Constant1_a[i] =
          realtime_simu_franka_fr3_P.param_EKF.Qk[i];

        /* Delay: '<S16>/Pk_minus' incorporates:
         *  Constant: '<S16>/Constant2'
         */
        if (realtime_simu_franka_fr3_DW.icLoad_br) {
          realtime_simu_franka_fr3_DW.Pk_minus_DSTATE_b[i] =
            realtime_simu_franka_fr3_P.param_EKF.P0[i];
        }

        /* Delay: '<S16>/Pk_minus' */
        realtime_simu_franka_fr3_B.Pk_minus_d[i] =
          realtime_simu_franka_fr3_DW.Pk_minus_DSTATE_b[i];
      }

      /* S-Function (s_function_opti_ekf_fun): '<S16>/Reduced System sfun casadi solve' */

      /* Level2 S-Function Block: '<S16>/Reduced System sfun casadi solve' (s_function_opti_ekf_fun) */
      {
        SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[0];
        sfcnOutputs(rts,0);
      }

      /* MATLAB Function: '<S16>/get EKF joint values' */
      realtime_simu_getEKFjointvalues(&realtime_simu_franka_fr3_B.Merge,
        realtime_simu_franka_fr3_B.xk_plus_j,
        &realtime_simu_franka_fr3_B.sf_getEKFjointvalues);

      /* S-Function (s_function_opti_robot_model_bus_fun): '<S19>/robot model s-function2' */

      /* Level2 S-Function Block: '<S19>/robot model s-function2' (s_function_opti_robot_model_bus_fun) */
      {
        SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[1];
        sfcnOutputs(rts,0);
      }

      /* MATLAB Function: '<S19>/zero fixed states' */
      realtime_simu_f_zerofixedstates
        (realtime_simu_franka_fr3_B.sf_getEKFjointvalues.q,
         realtime_simu_franka_fr3_B.sf_getEKFjointvalues.q_p,
         realtime_simu_franka_fr3_B.sf_getEKFjointvalues.q_pp,
         &realtime_simu_franka_fr3_B.sf_zerofixedstates_a);

      /* S-Function (s_function_opti_robot_model_bus_fun): '<S19>/robot model s-function1' */

      /* Level2 S-Function Block: '<S19>/robot model s-function1' (s_function_opti_robot_model_bus_fun) */
      {
        SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[2];
        sfcnOutputs(rts,0);
      }

      /* MATLAB Function: '<S19>/Robot model bus1' */
      realtime_simu_fra_Robotmodelbus
        (realtime_simu_franka_fr3_B.sf_zerofixedstates_a.q_red,
         realtime_simu_franka_fr3_B.sf_zerofixedstates_a.q_p_red,
         realtime_simu_franka_fr3_B.sf_zerofixedstates_a.q_pp_red,
         realtime_simu_franka_fr3_B.robotmodelsfunction1_o1_i,
         realtime_simu_franka_fr3_B.robotmodelsfunction1_o2_a,
         realtime_simu_franka_fr3_B.robotmodelsfunction1_o3_p,
         realtime_simu_franka_fr3_B.robotmodelsfunction1_o4_a,
         realtime_simu_franka_fr3_B.robotmodelsfunction1_o5_o,
         realtime_simu_franka_fr3_B.robotmodelsfunction1_o6_h,
         realtime_simu_franka_fr3_B.robotmodelsfunction1_o7_md,
         &realtime_simu_franka_fr3_B.sf_Robotmodelbus1_e);

      /* MATLAB Function: '<S19>/Robot model bus' */
      realtime_simu_fra_Robotmodelbus
        (realtime_simu_franka_fr3_B.sf_getEKFjointvalues.q,
         realtime_simu_franka_fr3_B.sf_getEKFjointvalues.q_p,
         realtime_simu_franka_fr3_B.sf_getEKFjointvalues.q_pp,
         realtime_simu_franka_fr3_B.robotmodelsfunction2_o1_b,
         realtime_simu_franka_fr3_B.robotmodelsfunction2_o2_h,
         realtime_simu_franka_fr3_B.robotmodelsfunction2_o3_o,
         realtime_simu_franka_fr3_B.robotmodelsfunction2_o4_d,
         realtime_simu_franka_fr3_B.robotmodelsfunction2_o5_k,
         realtime_simu_franka_fr3_B.robotmodelsfunction2_o6_b,
         realtime_simu_franka_fr3_B.robotmodelsfunction2_o7_l,
         &realtime_simu_franka_fr3_B.sf_Robotmodelbus_b);

      /* Merge: '<S13>/Merge' incorporates:
       *  SignalConversion generated from: '<S16>/robot_model_EKF'
       */
      realtime_simu_franka_fr3_B.Merge_i =
        realtime_simu_franka_fr3_B.sf_Robotmodelbus_b.robot_model_b;

      /* Update for Delay: '<S16>/xk_minus' incorporates:
       *  S-Function (s_function_opti_ekf_fun): '<S16>/Reduced System sfun casadi solve'
       */
      realtime_simu_franka_fr3_DW.icLoad_c = false;
      memcpy(&realtime_simu_franka_fr3_DW.xk_minus_DSTATE_m[0],
             &realtime_simu_franka_fr3_B.xkp1_minus_a[0], 14U * sizeof(real_T));

      /* Update for Delay: '<S16>/Pk_minus' incorporates:
       *  S-Function (s_function_opti_ekf_fun): '<S16>/Reduced System sfun casadi solve'
       */
      realtime_simu_franka_fr3_DW.icLoad_br = false;
      memcpy(&realtime_simu_franka_fr3_DW.Pk_minus_DSTATE_b[0],
             &realtime_simu_franka_fr3_B.Pkp1_minus_k[0], 196U * sizeof(real_T));
      srUpdateBC(realtime_simu_franka_fr3_DW.EKF_SubsysRanBC_p);
    }

    /* End of Outputs for SubSystem: '<S13>/EKF' */

    /* MATLAB Function: '<S9>/zero fixed states' */
    for (jj = 0; jj < 7; jj++) {
      realtime_simu_franka_fr3_B.q_red[jj] =
        realtime_simu_franka_fr3_B.Switch[jj];
      realtime_simu_franka_fr3_B.q_p_red[jj] =
        realtime_simu_franka_fr3_B.GetRobotState2_o2[jj];
      realtime_simu_franka_fr3_B.q_pp_red[jj] =
        realtime_simu_franka_fr3_B.Derivative[jj];
    }

    realtime_simu_franka_fr3_B.q_red[static_cast<int32_T>
      (realtime_simu_franka_fr3_P.param_robot.n_indices_fixed) - 1] = 0.0;
    realtime_simu_franka_fr3_B.q_p_red[static_cast<int32_T>
      (realtime_simu_franka_fr3_P.param_robot.n_indices_fixed) - 1] = 0.0;
    realtime_simu_franka_fr3_B.q_pp_red[static_cast<int32_T>
      (realtime_simu_franka_fr3_P.param_robot.n_indices_fixed) - 1] = 0.0;

    /* End of MATLAB Function: '<S9>/zero fixed states' */

    /* S-Function (s_function_opti_robot_model_bus_fun): '<S9>/robot model s-function1' */

    /* Level2 S-Function Block: '<S9>/robot model s-function1' (s_function_opti_robot_model_bus_fun) */
    {
      SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[9];
      sfcnOutputs(rts,0);
    }

    /* MATLAB Function: '<Root>/Controller Enable' incorporates:
     *  Constant: '<Root>/controller selector'
     *  Constant: '<Root>/use_casadi_flag'
     */
    for (jj = 0; jj < 11; jj++) {
      rtb_en_arr[jj] = static_cast<real_T>(static_cast<uint32_T>(jj) + 1U ==
        realtime_simu_franka_fr3_P.controllerselector_Value) *
        realtime_simu_franka_fr3_P.use_casadi_flag_Value;
    }

    /* End of MATLAB Function: '<Root>/Controller Enable' */
    for (i = 0; i < 7; i++) {
      /* SignalConversion generated from: '<S4>/Bus Selector1' */
      realtime_simu_franka_fr3_B.q_pp[i] =
        realtime_simu_franka_fr3_B.Merge.q_pp[i];
    }

    /* Outputs for Enabled SubSystem: '<Root>/jointspace ctl subsys' incorporates:
     *  EnablePort: '<S8>/Enable'
     */
    /* Constant: '<Root>/Constant1' */
    realtime_simu_franka_fr3_DW.jointspacectlsubsys_MODE =
      (realtime_simu_franka_fr3_P.Constant1_Value_j > 0.0);
    if (realtime_simu_franka_fr3_DW.jointspacectlsubsys_MODE) {
      /* MATLAB Function: '<S8>/home robot logic' incorporates:
       *  Constant: '<Root>/home'
       *  Constant: '<S8>/K_d_init'
       *  Constant: '<S8>/K_d_t0'
       *  Constant: '<S8>/K_d_t1'
       *  Constant: '<S8>/t0'
       *  Constant: '<S8>/t1'
       *  Constant: '<S8>/tend'
       */
      if (!realtime_simu_franka_fr3_DW.enabled_not_empty) {
        realtime_simu_franka_fr3_DW.enabled_not_empty = true;
        realtime_simu_franka_fr3_DW.t_start = epsilon;
      }

      if (realtime_simu_franka_fr3_P.home_Value == 1.0) {
        realtime_simu_franka_fr3_DW.enabled = 1.0;
        realtime_simu_franka_fr3_DW.t_start = epsilon;
      }

      if (epsilon - realtime_simu_franka_fr3_DW.t_start <
          realtime_simu_franka_fr3_P.t0_Value) {
        time_end = (epsilon - realtime_simu_franka_fr3_DW.t_start) /
          realtime_simu_franka_fr3_P.t0_Value;
        for (i = 0; i < 7; i++) {
          realtime_simu_franka_fr3_B.K_d[i] =
            (realtime_simu_franka_fr3_P.K_d_t0_Value[i] -
             realtime_simu_franka_fr3_P.K_d_init_Value[i]) * time_end +
            realtime_simu_franka_fr3_P.K_d_init_Value[i];
        }
      } else {
        s = epsilon - realtime_simu_franka_fr3_DW.t_start;
        if (s < realtime_simu_franka_fr3_P.t1_Value) {
          s2 = (s - realtime_simu_franka_fr3_P.t0_Value) /
            realtime_simu_franka_fr3_P.t1_Value;
          for (i = 0; i < 7; i++) {
            realtime_simu_franka_fr3_B.K_d[i] =
              (realtime_simu_franka_fr3_P.K_d_t1_Value[i] -
               realtime_simu_franka_fr3_P.K_d_t0_Value[i]) * s2 +
              realtime_simu_franka_fr3_P.K_d_init_Value[i];
          }
        } else {
          for (jj = 0; jj < 7; jj++) {
            realtime_simu_franka_fr3_B.K_d[jj] =
              realtime_simu_franka_fr3_P.K_d_t1_Value[jj];
          }
        }
      }

      for (jj = 0; jj < 7; jj++) {
        realtime_simu_franka_fr3_B.D_d[jj] = 2.0 *
          realtime_simu_franka_fr3_B.K_d[jj];
        realtime_simu_franka_fr3_B.D_d[jj] = sqrt
          (realtime_simu_franka_fr3_B.D_d[jj]);
      }

      if (epsilon - realtime_simu_franka_fr3_DW.t_start >
          realtime_simu_franka_fr3_P.tend_Value) {
        realtime_simu_franka_fr3_DW.enabled = 0.0;
      }

      realtime_simu_franka_fr3_B.home_running =
        realtime_simu_franka_fr3_DW.enabled;

      /* End of MATLAB Function: '<S8>/home robot logic' */

      /* MATLAB Function: '<S8>/get reference pose' incorporates:
       *  Constant: '<Root>/home mode'
       *  Constant: '<Root>/trajectory selector'
       */
      if (realtime_simu_franka_fr3_P.homemode_Value == 0.0) {
        for (jj = 0; jj < 7; jj++) {
          rtb_q_ref[jj] = c_0[jj];
        }
      } else {
        for (jj = 0; jj < 7; jj++) {
          rtb_q_ref[jj] = b_0[(static_cast<int32_T>
                               (realtime_simu_franka_fr3_P.trajectoryselector_Value)
                               - 1) * 7 + jj];
        }
      }

      /* End of MATLAB Function: '<S8>/get reference pose' */

      /* MATLAB Function: '<S8>/Joinspace controller' incorporates:
       *  Merge: '<S6>/Merge'
       */
      memset(&JJ_colin[0], 0, 49U * sizeof(real_T));
      for (j = 0; j < 7; j++) {
        JJ_colin[j + 7 * j] = realtime_simu_franka_fr3_B.D_d[j];
      }

      memset(&a[0], 0, 49U * sizeof(real_T));
      for (i = 0; i < 7; i++) {
        a[i + 7 * i] = realtime_simu_franka_fr3_B.K_d[i];
      }

      for (jj = 0; jj < 49; jj++) {
        JJ_colin_0[jj] = -JJ_colin[jj];
      }

      for (jj = 0; jj < 7; jj++) {
        tmp[jj] = realtime_simu_franka_fr3_B.Merge.q[jj] - rtb_q_ref[jj];
        JJ_colin_1[jj] = 0.0;
        for (i = 0; i < 7; i++) {
          JJ_colin_1[jj] += JJ_colin_0[7 * i + jj] *
            realtime_simu_franka_fr3_B.Merge.q_p[i];
        }
      }

      for (jj = 0; jj < 7; jj++) {
        rtb_q_ref[jj] = 0.0;
        for (i = 0; i < 7; i++) {
          rtb_q_ref[jj] += a[7 * i + jj] * tmp[i];
        }

        realtime_simu_franka_fr3_B.tau_k[jj] = JJ_colin_1[jj] - rtb_q_ref[jj];
      }

      /* End of MATLAB Function: '<S8>/Joinspace controller' */
      srUpdateBC(realtime_simu_franka_fr3_DW.jointspacectlsubsys_SubsysRanBC);
    }

    /* End of Outputs for SubSystem: '<Root>/jointspace ctl subsys' */

    /* Sum: '<Root>/Add' incorporates:
     *  Constant: '<Root>/Reset Trajectory'
     *  Constant: '<Root>/home'
     */
    time_end = realtime_simu_franka_fr3_P.ResetTrajectory_Value +
      realtime_simu_franka_fr3_P.home_Value;

    /* Outputs for Enabled SubSystem: '<Root>/tau_subsystem' incorporates:
     *  EnablePort: '<S10>/Enable'
     */
    /* Constant: '<Root>/use_crocoddyl_flag' */
    realtime_simu_franka_fr3_DW.tau_subsystem_MODE =
      (realtime_simu_franka_fr3_P.use_crocoddyl_flag_Value > 0.0);
    if (realtime_simu_franka_fr3_DW.tau_subsystem_MODE) {
      for (i = 0; i < 7; i++) {
        /* SignalConversion generated from: '<S10>/S-Function4' */
        realtime_simu_franka_fr3_B.TmpSignalConversionAtSFunction4[i] =
          realtime_simu_franka_fr3_B.Merge.q[i];
        realtime_simu_franka_fr3_B.TmpSignalConversionAtSFunction4[i + 7] =
          realtime_simu_franka_fr3_B.Merge.q_p[i];
      }

      /* S-Function (shm_reader_sfun): '<S10>/S-Function3' */

      /* Level2 S-Function Block: '<S10>/S-Function3' (shm_reader_sfun) */
      {
        SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[6];
        sfcnOutputs(rts,0);
      }

      /* DataTypeConversion: '<S10>/Cast To Single4' incorporates:
       *  Constant: '<S10>/state_valid'
       */
      s = floor(realtime_simu_franka_fr3_P.state_valid_Value);
      if (rtIsNaN(s) || rtIsInf(s)) {
        s = 0.0;
      } else {
        s = fmod(s, 256.0);
      }

      /* DataTypeConversion: '<S10>/Cast To Single4' */
      realtime_simu_franka_fr3_B.CastToSingle4 = static_cast<int8_T>(s < 0.0 ?
        static_cast<int32_T>(static_cast<int8_T>(-static_cast<int8_T>(
        static_cast<uint8_T>(-s)))) : static_cast<int32_T>(static_cast<int8_T>(
        static_cast<uint8_T>(s))));

      /* DataTypeConversion: '<S10>/Cast To Single3' incorporates:
       *  Constant: '<Root>/Start Trajectory'
       */
      s = floor(realtime_simu_franka_fr3_P.StartTrajectory_Value);
      if (rtIsNaN(s) || rtIsInf(s)) {
        s = 0.0;
      } else {
        s = fmod(s, 256.0);
      }

      /* DataTypeConversion: '<S10>/Cast To Single3' */
      realtime_simu_franka_fr3_B.CastToSingle3 = static_cast<int8_T>(s < 0.0 ?
        static_cast<int32_T>(static_cast<int8_T>(-static_cast<int8_T>(
        static_cast<uint8_T>(-s)))) : static_cast<int32_T>(static_cast<int8_T>(
        static_cast<uint8_T>(s))));

      /* DataTypeConversion: '<S10>/Cast To Single2' */
      s = floor(time_end);
      if (rtIsNaN(s) || rtIsInf(s)) {
        s = 0.0;
      } else {
        s = fmod(s, 256.0);
      }

      /* DataTypeConversion: '<S10>/Cast To Single2' */
      realtime_simu_franka_fr3_B.CastToSingle2 = static_cast<int8_T>(s < 0.0 ?
        static_cast<int32_T>(static_cast<int8_T>(-static_cast<int8_T>(
        static_cast<uint8_T>(-s)))) : static_cast<int32_T>(static_cast<int8_T>(
        static_cast<uint8_T>(s))));

      /* DataTypeConversion: '<S10>/Cast To Single1' incorporates:
       *  Constant: '<Root>/Stop Trajectory'
       */
      s = floor(realtime_simu_franka_fr3_P.StopTrajectory_Value);
      if (rtIsNaN(s) || rtIsInf(s)) {
        s = 0.0;
      } else {
        s = fmod(s, 256.0);
      }

      /* DataTypeConversion: '<S10>/Cast To Single1' */
      realtime_simu_franka_fr3_B.CastToSingle1 = static_cast<int8_T>(s < 0.0 ?
        static_cast<int32_T>(static_cast<int8_T>(-static_cast<int8_T>(
        static_cast<uint8_T>(-s)))) : static_cast<int32_T>(static_cast<int8_T>(
        static_cast<uint8_T>(s))));

      /* DataTypeConversion: '<S10>/Cast To Single' incorporates:
       *  Constant: '<Root>/trajectory selector'
       */
      realtime_simu_franka_fr3_B.CastToSingle = static_cast<int8_T>
        (realtime_simu_franka_fr3_P.trajectoryselector_Value);

      /* DataTypeConversion: '<S10>/Cast To Single5' incorporates:
       *  Constant: '<S10>/torque_valid'
       */
      s = floor(realtime_simu_franka_fr3_P.torque_valid_Value);
      if (rtIsNaN(s) || rtIsInf(s)) {
        s = 0.0;
      } else {
        s = fmod(s, 256.0);
      }

      /* DataTypeConversion: '<S10>/Cast To Single5' */
      realtime_simu_franka_fr3_B.CastToSingle5 = static_cast<int8_T>(s < 0.0 ?
        static_cast<int32_T>(static_cast<int8_T>(-static_cast<int8_T>(
        static_cast<uint8_T>(-s)))) : static_cast<int32_T>(static_cast<int8_T>(
        static_cast<uint8_T>(s))));

      /* S-Function (shm_writer_sfun): '<S10>/S-Function4' */

      /* Level2 S-Function Block: '<S10>/S-Function4' (shm_writer_sfun) */
      {
        SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[7];
        sfcnOutputs(rts,0);
      }

      /* MATLAB Function: '<S10>/torque safety' */
      if (realtime_simu_franka_fr3_B.SFunction3_o2 == 1) {
        for (i = 0; i < 7; i++) {
          realtime_simu_franka_fr3_DW.tau_prev[i] =
            realtime_simu_franka_fr3_B.SFunction3_o1[i];
          realtime_simu_franka_fr3_B.tau[i] =
            realtime_simu_franka_fr3_B.SFunction3_o1[i];
        }

        realtime_simu_franka_fr3_DW.cnt = 0.0;
      } else if (realtime_simu_franka_fr3_DW.cnt < 100.0) {
        for (jj = 0; jj < 7; jj++) {
          realtime_simu_franka_fr3_B.tau[jj] =
            realtime_simu_franka_fr3_DW.tau_prev[jj];
        }

        realtime_simu_franka_fr3_DW.cnt++;
      } else {
        for (i = 0; i < 7; i++) {
          realtime_simu_franka_fr3_B.tau[i] = 0.0;
        }
      }

      /* End of MATLAB Function: '<S10>/torque safety' */
      srUpdateBC(realtime_simu_franka_fr3_DW.tau_subsystem_SubsysRanBC);
    }

    /* End of Outputs for SubSystem: '<Root>/tau_subsystem' */

    /* MATLAB Function: '<S9>/Robot model bus1' */
    realtime_simu_fra_Robotmodelbus(realtime_simu_franka_fr3_B.q_red,
      realtime_simu_franka_fr3_B.q_p_red, realtime_simu_franka_fr3_B.q_pp_red,
      realtime_simu_franka_fr3_B.robotmodelsfunction1_o1,
      realtime_simu_franka_fr3_B.robotmodelsfunction1_o2,
      realtime_simu_franka_fr3_B.robotmodelsfunction1_o3,
      realtime_simu_franka_fr3_B.robotmodelsfunction1_o4,
      realtime_simu_franka_fr3_B.robotmodelsfunction1_o5,
      realtime_simu_franka_fr3_B.robotmodelsfunction1_o6,
      realtime_simu_franka_fr3_B.robotmodelsfunction1_o7,
      &realtime_simu_franka_fr3_B.sf_Robotmodelbus1);

    /* Logic: '<S13>/NOT' */
    rtb_NOT_p = !(rtb_ManualSwitch1 != 0.0);

    /* Outputs for Enabled SubSystem: '<S13>/no EKF' */
    realtime_simu_franka_fr3_noEKF(rtb_NOT_p, &realtime_simu_franka_fr3_B.Merge,
      &realtime_simu_franka_fr3_B.Merge_i, &realtime_simu_franka_fr3_DW.noEKF);

    /* End of Outputs for SubSystem: '<S13>/no EKF' */

    /* Outputs for Enabled SubSystem: '<S2>/CT Controller Subsystem' incorporates:
     *  EnablePort: '<S11>/Enable'
     */
    /* Selector: '<S2>/Selector1' incorporates:
     *  Constant: '<S2>/Constant1'
     */
    if (rtb_en_arr[static_cast<int32_T>
        (realtime_simu_franka_fr3_P.Constant1_Value_n) - 1] > 0.0) {
      /* ManualSwitch: '<S11>/Manual Switch' incorporates:
       *  Constant: '<S11>/D_d'
       *  Constant: '<S11>/K_p2'
       *  Gain: '<S11>/Gain'
       *  Sqrt: '<S11>/Sqrt'
       */
      for (i = 0; i < 6; i++) {
        if (realtime_simu_franka_fr3_P.ManualSwitch_CurrentSetting == 1) {
          rtb_ManualSwitch_o[i] = realtime_simu_franka_fr3_P.Gain_Gain * sqrt
            (realtime_simu_franka_fr3_P.K_p2_Value[i]);
        } else {
          rtb_ManualSwitch_o[i] = realtime_simu_franka_fr3_P.D_d_Value[i];
        }
      }

      /* End of ManualSwitch: '<S11>/Manual Switch' */

      /* MATLAB Function: '<S11>/CT MATLAB Function' incorporates:
       *  BusCreator generated from: '<S11>/CT MATLAB Function'
       *  Constant: '<Root>/Start Trajectory'
       *  Constant: '<Root>/Stop Trajectory'
       *  Constant: '<Root>/trajectory selector'
       *  Constant: '<S11>/K_p2'
       *  Constant: '<S3>/Constant3'
       *  Merge: '<S13>/Merge'
       */
      for (jj = 0; jj < 6; jj++) {
        q_p_red[jj] =
          realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.q_p[l[jj]];
      }

      for (jj = 0; jj < 6; jj++) {
        for (i = 0; i < 6; i++) {
          J_red[jj + 6 * i] =
            realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 * l[i]
            + jj];
        }
      }

      memset(&Kp1[0], 0, 36U * sizeof(real_T));
      for (j = 0; j < 6; j++) {
        Kp1[j + 6 * j] = realtime_simu_franka_fr3_P.K_p2_Value[j];
      }

      memset(&Kd1[0], 0, 36U * sizeof(real_T));
      for (i = 0; i < 6; i++) {
        Kd1[i + 6 * i] = rtb_ManualSwitch_o[i];
      }

      for (jj = 0; jj < 6; jj++) {
        rtb_ManualSwitch_o[jj] = 0.0;
        for (i = 0; i < 6; i++) {
          rtb_ManualSwitch_o[jj] +=
            realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 * l[i]
            + jj] * q_p_red[i];
        }
      }

      for (jj = 0; jj < 3; jj++) {
        for (i = 0; i < 3; i++) {
          j = 3 * i + jj;
          R[j] = 0.0;
          aoffset = ((static_cast<int32_T>(realtime_simu_franka_fr3_DW.cnt_d) -
                      1) * 9 + i) + (static_cast<int32_T>
            (realtime_simu_franka_fr3_P.trajectoryselector_Value) - 1) * 108009;
          R[j] += realtime_simu_franka_fr3_P.traj_data_bus_init.R_d[aoffset] *
            realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.H[jj];
          R[j] += realtime_simu_franka_fr3_P.traj_data_bus_init.R_d[aoffset + 3]
            * realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.H[jj +
            4];
          R[j] += realtime_simu_franka_fr3_P.traj_data_bus_init.R_d[aoffset + 6]
            * realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.H[jj +
            8];
        }
      }

      s2 = (R[0] - R[4]) - R[8];
      s3 = (-R[0] + R[4]) - R[8];
      s4 = (-R[0] - R[4]) + R[8];
      if (s2 > 0.0) {
        s2 = sqrt(s2 + 1.0) / 2.0;
      } else {
        s2 = R[5] - R[7];
        e_a = R[1] + R[3];
        i_a = R[2] + R[6];
        s2 = sqrt(((s2 * s2 + e_a * e_a) + i_a * i_a) / (((3.0 - R[0]) + R[4]) +
                   R[8])) / 2.0;
      }

      if (s3 > 0.0) {
        s3 = sqrt(s3 + 1.0) / 2.0;
      } else {
        i_a = R[6] - R[2];
        h_a = R[1] + R[3];
        s3 = R[5] + R[7];
        s3 = sqrt(((i_a * i_a + h_a * h_a) + s3 * s3) / (((R[0] + 3.0) - R[4]) +
                   R[8])) / 2.0;
      }

      if (s4 > 0.0) {
        s4 = sqrt(s4 + 1.0) / 2.0;
      } else {
        s4 = R[1] - R[3];
        h_a = R[2] + R[6];
        i_a = R[5] + R[7];
        s4 = sqrt(((s4 * s4 + h_a * h_a) + i_a * i_a) / (((R[0] + 3.0) + R[4]) -
                   R[8])) / 2.0;
      }

      j = (static_cast<int32_T>(realtime_simu_franka_fr3_DW.cnt_d) - 1) * 3 + (
        static_cast<int32_T>(realtime_simu_franka_fr3_P.trajectoryselector_Value)
        - 1) * 36003;
      x_err[0] = realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.H[12]
        - realtime_simu_franka_fr3_P.traj_data_bus_init.p_d[j];
      x_err[1] = realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.H[13]
        - realtime_simu_franka_fr3_P.traj_data_bus_init.p_d[j + 1];
      x_err[2] = realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.H[14]
        - realtime_simu_franka_fr3_P.traj_data_bus_init.p_d[j + 2];
      x_err[3] = (static_cast<real_T>(R[5] - R[7] >= 0.0) * 2.0 - 1.0) * s2;
      x_err_p[0] = rtb_ManualSwitch_o[0] -
        realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_p[j];
      x_err_p[3] = rtb_ManualSwitch_o[3] -
        realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d[j];
      x_d_pp[0] = realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_pp[j];
      x_d_pp[3] = realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d_p[j];
      x_err[4] = (static_cast<real_T>(R[6] - R[2] >= 0.0) * 2.0 - 1.0) * s3;
      x_err_p[1] = rtb_ManualSwitch_o[1] -
        realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_p[j + 1];
      x_err_p[4] = rtb_ManualSwitch_o[4] -
        realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d[j + 1];
      x_d_pp[1] = realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_pp[j + 1];
      x_d_pp[4] = realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d_p[j + 1];
      x_err[5] = (static_cast<real_T>(R[1] - R[3] >= 0.0) * 2.0 - 1.0) * s4;
      x_err_p[2] = rtb_ManualSwitch_o[2] -
        realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_p[j + 2];
      x_err_p[5] = rtb_ManualSwitch_o[5] -
        realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d[j + 2];
      x_d_pp[2] = realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_pp[j + 2];
      x_d_pp[5] = realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d_p[j + 2];
      if (realtime_simu_franka_fr3_P.ctrl_param.regularization.mode == 0.0) {
        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            aoffset = 6 * i + jj;
            J_pinv[aoffset] = 0.0;
            for (b = 0; b < 6; b++) {
              J_pinv[aoffset] +=
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
                l[jj] + b] *
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
                l[i] + b];
            }
          }
        }

        realtime_simu_franka_fr3_inv(J_pinv, U);
        for (jj = 0; jj < 6; jj++) {
          s = 0.0;
          Kp1_0[jj] = 0.0;
          for (i = 0; i < 6; i++) {
            aoffset = 6 * i + jj;
            s += Kd1[aoffset] * x_err_p[i];
            J_tilde_0[aoffset] = 0.0;
            for (b = 0; b < 6; b++) {
              J_tilde_0[aoffset] += U[6 * b + jj] * J_red[6 * b + i];
            }

            Kp1_0[jj] += Kp1[aoffset] * x_err[i];
          }

          x_d_pp_0[jj] = x_d_pp[jj] - s;
        }

        for (jj = 0; jj < 6; jj++) {
          s = 0.0;
          for (i = 0; i < 6; i++) {
            s += realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J_p
              [6 * l[i] + jj] * q_p_red[i];
          }

          x_d_pp[jj] = (x_d_pp_0[jj] - Kp1_0[jj]) - s;
        }

        for (jj = 0; jj < 6; jj++) {
          rtb_ManualSwitch_o[jj] = 0.0;
          for (i = 0; i < 6; i++) {
            rtb_ManualSwitch_o[jj] += J_tilde_0[6 * i + jj] * x_d_pp[i];
          }
        }
      } else if (realtime_simu_franka_fr3_P.ctrl_param.regularization.mode ==
                 1.0) {
        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            s = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              s += realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J
                [6 * l[jj] + aoffset] *
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
                l[i] + aoffset];
            }

            aoffset = 6 * i + jj;
            J_pinv[aoffset] = static_cast<real_T>(m_a[aoffset]) *
              realtime_simu_franka_fr3_P.ctrl_param.regularization.k + s;
          }
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            T_bar_data[i + 6 * jj] =
              realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
              l[i] + jj];
          }
        }

        realtime_simu_franka_f_mldivide(J_pinv, T_bar_data, J_tilde_0);
        for (jj = 0; jj < 6; jj++) {
          s = 0.0;
          Kp1_0[jj] = 0.0;
          for (i = 0; i < 6; i++) {
            aoffset = 6 * i + jj;
            s += Kd1[aoffset] * x_err_p[i];
            Kp1_0[jj] += Kp1[aoffset] * x_err[i];
          }

          x_d_pp_0[jj] = x_d_pp[jj] - s;
        }

        for (jj = 0; jj < 6; jj++) {
          s = 0.0;
          for (i = 0; i < 6; i++) {
            s += realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J_p
              [6 * l[i] + jj] * q_p_red[i];
          }

          x_d_pp[jj] = (x_d_pp_0[jj] - Kp1_0[jj]) - s;
        }

        for (jj = 0; jj < 6; jj++) {
          rtb_ManualSwitch_o[jj] = 0.0;
          for (i = 0; i < 6; i++) {
            rtb_ManualSwitch_o[jj] += J_tilde_0[6 * i + jj] * x_d_pp[i];
          }
        }
      } else if (realtime_simu_franka_fr3_P.ctrl_param.regularization.mode ==
                 2.0) {
        s = 0.0;
        for (jj = 0; jj < 6; jj++) {
          x_d_pp_0[jj] = 0.0;
          for (i = 0; i < 6; i++) {
            x_d_pp_0[jj] +=
              realtime_simu_franka_fr3_P.ctrl_param.regularization.W_E[6 * jj +
              i] * (0.5 * x_err[i]);
          }

          s += x_d_pp_0[jj] * x_err[jj];
        }

        for (jj = 0; jj < 36; jj++) {
          J_tilde_0[jj] = sqrt
            (realtime_simu_franka_fr3_P.ctrl_param.regularization.W_E[jj]);
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            b = 6 * i + jj;
            T_bar_data[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              T_bar_data[b] += J_tilde_0[6 * aoffset + jj] *
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
                l[i] + aoffset];
            }
          }
        }

        realtime_simu_franka_fr3_svd(T_bar_data, U, S_new, J_red);
        for (i = 0; i < 6; i++) {
          rtb_ManualSwitch_o[i] = S_new[6 * i + i];
        }

        memset(&J_tilde_0[0], 0, 36U * sizeof(real_T));
        for (jj = 0; jj < 6; jj++) {
          J_tilde_0[jj + 6 * jj] = rtb_ManualSwitch_o[jj];
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            b = 6 * jj + i;
            T_bar_data[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              T_bar_data[b] += J_tilde_0[6 * aoffset + i] * J_tilde_0[6 * jj +
                aoffset];
            }
          }
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            b = 6 * i + jj;
            U[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              U[b] += J_red[6 * aoffset + jj] * T_bar_data[6 * i + aoffset];
            }
          }

          for (i = 0; i < 6; i++) {
            b = 6 * i + jj;
            J_tilde_0[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              J_tilde_0[b] += U[6 * aoffset + jj] * J_red[6 * aoffset + i];
            }
          }
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            b = 6 * jj + i;
            J_red[b] = (static_cast<real_T>(m_a[b]) * s +
                        realtime_simu_franka_fr3_P.ctrl_param.regularization.W_bar_N
                        [l[i]]) + J_tilde_0[b];
          }
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            J_pinv[i + 6 * jj] =
              realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
              l[i] + jj];
          }
        }

        realtime_simu_franka_f_mldivide(J_red, J_pinv, J_tilde_0);
        for (jj = 0; jj < 6; jj++) {
          s = 0.0;
          Kp1_0[jj] = 0.0;
          for (i = 0; i < 6; i++) {
            aoffset = 6 * i + jj;
            s += Kd1[aoffset] * x_err_p[i];
            Kp1_0[jj] += Kp1[aoffset] * x_err[i];
          }

          x_d_pp_0[jj] = x_d_pp[jj] - s;
        }

        for (jj = 0; jj < 6; jj++) {
          s = 0.0;
          for (i = 0; i < 6; i++) {
            s += realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J_p
              [6 * l[i] + jj] * q_p_red[i];
          }

          x_d_pp[jj] = (x_d_pp_0[jj] - Kp1_0[jj]) - s;
        }

        for (jj = 0; jj < 6; jj++) {
          rtb_ManualSwitch_o[jj] = 0.0;
          for (i = 0; i < 6; i++) {
            rtb_ManualSwitch_o[jj] += J_tilde_0[6 * i + jj] * x_d_pp[i];
          }
        }
      } else if ((realtime_simu_franka_fr3_P.ctrl_param.regularization.mode ==
                  3.0) ||
                 (realtime_simu_franka_fr3_P.ctrl_param.regularization.mode ==
                  4.0)) {
        epsilon = realtime_simu_franka_fr3_P.ctrl_param.regularization.eps;
        realtime_simu_franka_fr3_svd(J_red, J_tilde_0, S_new, U);
        if (realtime_simu_franka_fr3_P.ctrl_param.regularization.mode == 3.0) {
          for (i = 0; i < 6; i++) {
            rtb_ManualSwitch_o[i] = fmax(S_new[6 * i + i], epsilon);
          }

          memset(&S_new[0], 0, 36U * sizeof(real_T));
          for (i = 0; i < 6; i++) {
            S_new[i + 6 * i] = rtb_ManualSwitch_o[i];
          }
        } else {
          s = realtime_simu_franka_fr3_P.ctrl_param.regularization.eps *
            realtime_simu_franka_fr3_P.ctrl_param.regularization.eps;
          for (i = 0; i < 6; i++) {
            j = 6 * i + i;
            rtb_ManualSwitch1 = S_new[j];
            if (rtb_ManualSwitch1 < epsilon) {
              rtb_ManualSwitch1 = sqrt(rtb_ManualSwitch1 * rtb_ManualSwitch1 + s);
            }

            S_new[j] = rtb_ManualSwitch1;
          }
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            b = 6 * i + jj;
            T_bar_data[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              T_bar_data[b] += J_tilde_0[6 * aoffset + jj] * S_new[6 * i +
                aoffset];
            }
          }

          for (i = 0; i < 6; i++) {
            b = 6 * i + jj;
            J_tilde_1[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              J_tilde_1[b] += T_bar_data[6 * aoffset + jj] * U[6 * aoffset + i];
            }
          }
        }

        realtime_simu_franka_fr3_pinv(J_tilde_1, J_tilde_0);
        for (jj = 0; jj < 6; jj++) {
          s = 0.0;
          Kp1_0[jj] = 0.0;
          for (i = 0; i < 6; i++) {
            aoffset = 6 * i + jj;
            s += Kd1[aoffset] * x_err_p[i];
            Kp1_0[jj] += Kp1[aoffset] * x_err[i];
          }

          x_d_pp_0[jj] = x_d_pp[jj] - s;
        }

        for (jj = 0; jj < 6; jj++) {
          s = 0.0;
          for (i = 0; i < 6; i++) {
            s += realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J_p
              [6 * l[i] + jj] * q_p_red[i];
          }

          x_d_pp[jj] = (x_d_pp_0[jj] - Kp1_0[jj]) - s;
        }

        for (jj = 0; jj < 6; jj++) {
          rtb_ManualSwitch_o[jj] = 0.0;
          for (i = 0; i < 6; i++) {
            rtb_ManualSwitch_o[jj] += J_tilde_0[6 * i + jj] * x_d_pp[i];
          }
        }
      } else if (realtime_simu_franka_fr3_P.ctrl_param.regularization.mode ==
                 6.0) {
        aoffset = 0;
        isodd = true;
        for (j = 0; j < 36; j++) {
          if (isodd) {
            s = realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[l[j
              / 6] * 6 + j % 6];
            if ((!rtIsInf(s)) && (!rtIsNaN(s))) {
            } else {
              isodd = false;
            }
          } else {
            isodd = false;
          }
        }

        if (isodd) {
          realtime_simu_franka_fr_svd_kyc(J_red, rtb_ManualSwitch_o);
        } else {
          for (i = 0; i < 6; i++) {
            rtb_ManualSwitch_o[i] = (rtNaN);
          }
        }

        jj = 0;
        while ((jj < 6) && (rtb_ManualSwitch_o[jj] > 0.1)) {
          aoffset++;
          jj++;
        }

        if (aoffset < 6) {
          realtime_simu_franka_fr_vecnorm(J_red, rtb_ManualSwitch_o);
          for (jj = 0; jj < 6; jj++) {
            for (i = 0; i < 6; i++) {
              J_tilde_0[jj + 6 * i] =
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
                l[i] + jj] / rtb_ManualSwitch_o[i];
            }
          }

          for (jj = 0; jj < 6; jj++) {
            for (i = 0; i < 6; i++) {
              b = 6 * i + jj;
              S_new[b] = 0.0;
              for (aoffset = 0; aoffset < 6; aoffset++) {
                S_new[b] += J_tilde_0[6 * jj + aoffset] * J_tilde_0[6 * i +
                  aoffset];
              }
            }
          }

          aoffset = 0;
          for (jj = 0; jj < 5; jj++) {
            memset(&S_new[jj * 6 + 6], 0, static_cast<uint32_T>(aoffset + 1) *
                   sizeof(real_T));
            if (aoffset + 1 < 6) {
              aoffset++;
            }
          }

          memset(&U[0], 0, 36U * sizeof(real_T));
          for (jj = 0; jj < 6; jj++) {
            U[jj + 6 * jj] = 1.0;
          }

          for (jj = 0; jj < 36; jj++) {
            s = S_new[jj] - U[jj];
            epsilon = fabs(s);
            U_0[jj] = (epsilon >
                       realtime_simu_franka_fr3_P.ctrl_param.regularization.eps_collinear);
            S_new[jj] = s;
            U[jj] = epsilon;
          }

          realtime_simu_franka_f_eml_find(U_0, ii_data, &jj, jj_data, &b);
          for (jj = 0; jj < b; jj++) {
            for (i = 0; i < 6; i++) {
              J_red[i + 6 * (jj_data[jj] - 1)] = 0.0;
            }
          }

          realtime_simu_franka_fr3_pinv(J_red, J_tilde_0);
          for (jj = 0; jj < 6; jj++) {
            s = 0.0;
            Kp1_0[jj] = 0.0;
            for (i = 0; i < 6; i++) {
              aoffset = 6 * i + jj;
              s += Kd1[aoffset] * x_err_p[i];
              Kp1_0[jj] += Kp1[aoffset] * x_err[i];
            }

            x_d_pp_0[jj] = x_d_pp[jj] - s;
          }

          for (jj = 0; jj < 6; jj++) {
            s = 0.0;
            for (i = 0; i < 6; i++) {
              s +=
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J_p[6
                * l[i] + jj] * q_p_red[i];
            }

            x_d_pp[jj] = (x_d_pp_0[jj] - Kp1_0[jj]) - s;
          }

          for (jj = 0; jj < 6; jj++) {
            rtb_ManualSwitch_o[jj] = 0.0;
            for (i = 0; i < 6; i++) {
              rtb_ManualSwitch_o[jj] += J_tilde_0[6 * i + jj] * x_d_pp[i];
            }
          }

          for (jj = 0; jj < b; jj++) {
            rtb_ManualSwitch_o[jj_data[jj] - 1] = 0.0;
          }
        } else {
          for (jj = 0; jj < 6; jj++) {
            s = 0.0;
            Kp1_0[jj] = 0.0;
            for (i = 0; i < 6; i++) {
              aoffset = 6 * i + jj;
              s += Kd1[aoffset] * x_err_p[i];
              Kp1_0[jj] += Kp1[aoffset] * x_err[i];
            }

            x_d_pp_0[jj] = x_d_pp[jj] - s;
          }

          for (jj = 0; jj < 6; jj++) {
            s = 0.0;
            for (i = 0; i < 6; i++) {
              s +=
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J_p[6
                * l[i] + jj] * q_p_red[i];
            }

            rtb_ManualSwitch_o[jj] = (x_d_pp_0[jj] - Kp1_0[jj]) - s;
          }

          realtime_simu_fran_mldivide_oqz(J_red, rtb_ManualSwitch_o);
        }
      } else {
        realtime_simu_franka_fr_vecnorm(J_red, rtb_ManualSwitch_o);
        for (jj = 0; jj < 6; jj++) {
          rtb_ManualSwitch_o[jj] = 1.0 / rtb_ManualSwitch_o[jj];
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            J_tilde_0[jj + 6 * i] =
              realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
              l[i] + jj] * rtb_ManualSwitch_o[i];
          }
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            b = 6 * i + jj;
            T_bar_data[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              T_bar_data[b] += J_tilde_0[6 * jj + aoffset] * J_tilde_0[6 * i +
                aoffset];
            }
          }
        }

        realtime_simu_franka_fr3_svd(T_bar_data, U, S_new, J_red);
        i = 0;
        for (jj = 0; jj < 6; jj++) {
          isodd = (S_new[6 * jj + jj] >
                   realtime_simu_franka_fr3_P.ctrl_param.regularization.lambda_min);
          if (isodd) {
            i++;
          }

          idx[jj] = isodd;
        }

        jA = i;
        i = 0;
        for (jj = 0; jj < 6; jj++) {
          if (idx[jj]) {
            c_data[i] = static_cast<int8_T>(jj + 1);
            i++;
          }
        }

        for (jj = 0; jj < jA; jj++) {
          for (i = 0; i < 6; i++) {
            T_bar_data[i + 6 * jj] = J_red[(c_data[jj] - 1) * 6 + i] *
              rtb_ManualSwitch_o[i];
          }
        }

        J_bar_size[0] = 6;
        J_bar_size[1] = jA;
        for (jj = 0; jj < jA; jj++) {
          coffset_tmp = jj * 6;
          for (i = 0; i < 6; i++) {
            s = 0.0;
            for (j = 0; j < 6; j++) {
              aoffset = j * 6 + i;
              s +=
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[l[aoffset
                / 6] * 6 + aoffset % 6] * T_bar_data[coffset_tmp + j];
            }

            J_bar_data[coffset_tmp + i] = s;
          }
        }

        for (jj = 0; jj < 6; jj++) {
          s = 0.0;
          Kp1_0[jj] = 0.0;
          for (i = 0; i < 6; i++) {
            aoffset = 6 * i + jj;
            s += Kd1[aoffset] * x_err_p[i];
            Kp1_0[jj] += Kp1[aoffset] * x_err[i];
          }

          x_d_pp_0[jj] = x_d_pp[jj] - s;
        }

        for (jj = 0; jj < 6; jj++) {
          s = 0.0;
          for (i = 0; i < 6; i++) {
            s += realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J_p
              [6 * l[i] + jj] * q_p_red[i];
          }

          x_d_pp[jj] = (x_d_pp_0[jj] - Kp1_0[jj]) - s;
        }

        realtime_simu_frank_mldivide_oq(J_bar_data, J_bar_size, x_d_pp, x_d_pp_0,
          &jj);
        for (j = 0; j < 6; j++) {
          rtb_ManualSwitch_o[j] = 0.0;
        }

        for (j = 0; j < jA; j++) {
          aoffset = j * 6;
          for (jj = 0; jj < 6; jj++) {
            rtb_ManualSwitch_o[jj] += T_bar_data[aoffset + jj] * x_d_pp_0[j];
          }
        }
      }

      for (i = 0; i < 7; i++) {
        rtb_q_ref[i] = 0.0;
      }

      for (jj = 0; jj < 6; jj++) {
        rtb_q_ref[l[jj]] = rtb_ManualSwitch_o[jj];
      }

      if (realtime_simu_franka_fr3_DW.run_flag_d == 0.0) {
        if ((realtime_simu_franka_fr3_P.StartTrajectory_Value == 1.0) &&
            (time_end == 0.0) &&
            (realtime_simu_franka_fr3_P.StopTrajectory_Value == 0.0)) {
          realtime_simu_franka_fr3_DW.run_flag_d = 1.0;
          if (realtime_simu_franka_fr3_DW.cnt_d <
              realtime_simu_franka_fr3_P.traj_data_bus_init.N) {
            realtime_simu_franka_fr3_DW.cnt_d++;
          }
        } else if ((time_end == 1.0) && (realtime_simu_franka_fr3_DW.cnt_d ==
                    realtime_simu_franka_fr3_P.traj_data_bus_init.N)) {
          realtime_simu_franka_fr3_DW.cnt_d = 1.0;
        }
      } else if (realtime_simu_franka_fr3_P.StopTrajectory_Value == 1.0) {
        realtime_simu_franka_fr3_DW.run_flag_d = 0.0;
      } else if (realtime_simu_franka_fr3_DW.cnt_d <
                 realtime_simu_franka_fr3_P.traj_data_bus_init.N) {
        realtime_simu_franka_fr3_DW.cnt_d++;
      } else {
        realtime_simu_franka_fr3_DW.run_flag_d = 0.0;
      }

      for (jj = 0; jj < 7; jj++) {
        s = 0.0;
        for (i = 0; i < 7; i++) {
          s += realtime_simu_franka_fr3_B.Merge_i.M[7 * i + jj] * rtb_q_ref[i];
        }

        /* Merge: '<S2>/Merge1' incorporates:
         *  Merge: '<S13>/Merge'
         *  SignalConversion generated from: '<S11>/tau'
         */
        realtime_simu_franka_fr3_B.Merge1[jj] = (s +
          realtime_simu_franka_fr3_B.Merge_i.C_rnea[jj]) -
          realtime_simu_franka_fr3_B.Merge_i.g[jj];
      }

      /* End of MATLAB Function: '<S11>/CT MATLAB Function' */
      srUpdateBC(realtime_simu_franka_fr3_DW.CTControllerSubsystem_SubsysRan);
    }

    /* End of Selector: '<S2>/Selector1' */
    /* End of Outputs for SubSystem: '<S2>/CT Controller Subsystem' */

    /* Outputs for Enabled SubSystem: '<S2>/PD+ Controller Subsystem' incorporates:
     *  EnablePort: '<S12>/Enable'
     */
    /* Selector: '<S2>/Selector11' incorporates:
     *  Constant: '<S2>/Constant2'
     */
    if (rtb_en_arr[static_cast<int32_T>
        (realtime_simu_franka_fr3_P.Constant2_Value) - 1] > 0.0) {
      /* ManualSwitch: '<S12>/Manual Switch' incorporates:
       *  Constant: '<S12>/D_d'
       *  Constant: '<S12>/K_d2'
       *  Gain: '<S12>/Gain'
       *  Sqrt: '<S12>/Sqrt'
       */
      for (i = 0; i < 6; i++) {
        if (realtime_simu_franka_fr3_P.ManualSwitch_CurrentSetting_h == 1) {
          rtb_ManualSwitch_o[i] = realtime_simu_franka_fr3_P.Gain_Gain_d * sqrt
            (realtime_simu_franka_fr3_P.K_d2_Value[i]);
        } else {
          rtb_ManualSwitch_o[i] = realtime_simu_franka_fr3_P.D_d_Value_k[i];
        }
      }

      /* End of ManualSwitch: '<S12>/Manual Switch' */

      /* MATLAB Function: '<S12>/PD+ MATLAB Function' incorporates:
       *  Constant: '<Root>/trajectory selector'
       *  Constant: '<S12>/K_d2'
       *  Constant: '<S3>/Constant3'
       */
      for (jj = 0; jj < 6; jj++) {
        q_p_red[jj] =
          realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.q_p[l[jj]];
      }

      for (jj = 0; jj < 6; jj++) {
        for (i = 0; i < 6; i++) {
          Kp1[i + 6 * jj] =
            realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.M[7 *
            l[jj] + l[i]];
        }
      }

      for (jj = 0; jj < 6; jj++) {
        for (i = 0; i < 6; i++) {
          Kd1[i + 6 * jj] =
            realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.C[7 *
            l[jj] + l[i]];
        }
      }

      for (jj = 0; jj < 6; jj++) {
        for (i = 0; i < 6; i++) {
          J_red[jj + 6 * i] =
            realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 * l[i]
            + jj];
        }
      }

      memset(&U[0], 0, 36U * sizeof(real_T));
      for (j = 0; j < 6; j++) {
        U[j + 6 * j] = rtb_ManualSwitch_o[j];
      }

      memset(&b_a[0], 0, 36U * sizeof(real_T));
      for (i = 0; i < 6; i++) {
        b_a[i + 6 * i] = realtime_simu_franka_fr3_P.K_d2_Value[i];
      }

      for (jj = 0; jj < 6; jj++) {
        rtb_ManualSwitch_o[jj] = 0.0;
        for (i = 0; i < 6; i++) {
          rtb_ManualSwitch_o[jj] +=
            realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 * l[i]
            + jj] * q_p_red[i];
        }
      }

      for (jj = 0; jj < 3; jj++) {
        for (i = 0; i < 3; i++) {
          j = 3 * i + jj;
          R[j] = 0.0;
          aoffset = ((static_cast<int32_T>(realtime_simu_franka_fr3_DW.cnt_o) -
                      1) * 9 + i) + (static_cast<int32_T>
            (realtime_simu_franka_fr3_P.trajectoryselector_Value) - 1) * 108009;
          R[j] += realtime_simu_franka_fr3_P.traj_data_bus_init.R_d[aoffset] *
            realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.H[jj];
          R[j] += realtime_simu_franka_fr3_P.traj_data_bus_init.R_d[aoffset + 3]
            * realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.H[jj +
            4];
          R[j] += realtime_simu_franka_fr3_P.traj_data_bus_init.R_d[aoffset + 6]
            * realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.H[jj +
            8];
        }
      }

      s2 = (R[0] - R[4]) - R[8];
      s3 = (-R[0] + R[4]) - R[8];
      s4 = (-R[0] - R[4]) + R[8];
      if (s2 > 0.0) {
        s2 = sqrt(s2 + 1.0) / 2.0;
      } else {
        e_a = R[5] - R[7];
        h_a = R[1] + R[3];
        i_a = R[2] + R[6];
        s2 = sqrt(((e_a * e_a + h_a * h_a) + i_a * i_a) / (((3.0 - R[0]) + R[4])
                   + R[8])) / 2.0;
      }

      if (s3 > 0.0) {
        s3 = sqrt(s3 + 1.0) / 2.0;
      } else {
        i_a = R[6] - R[2];
        s3 = R[1] + R[3];
        s = R[5] + R[7];
        s3 = sqrt(((i_a * i_a + s3 * s3) + s * s) / (((R[0] + 3.0) - R[4]) + R[8]))
          / 2.0;
      }

      if (s4 > 0.0) {
        s4 = sqrt(s4 + 1.0) / 2.0;
      } else {
        i_a = R[1] - R[3];
        s = R[2] + R[6];
        epsilon = R[5] + R[7];
        s4 = sqrt(((i_a * i_a + s * s) + epsilon * epsilon) / (((R[0] + 3.0) +
                    R[4]) - R[8])) / 2.0;
      }

      j = (static_cast<int32_T>(realtime_simu_franka_fr3_DW.cnt_o) - 1) * 3 + (
        static_cast<int32_T>(realtime_simu_franka_fr3_P.trajectoryselector_Value)
        - 1) * 36003;
      x_err[0] = realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.H[12]
        - realtime_simu_franka_fr3_P.traj_data_bus_init.p_d[j];
      x_err[1] = realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.H[13]
        - realtime_simu_franka_fr3_P.traj_data_bus_init.p_d[j + 1];
      x_err[2] = realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.H[14]
        - realtime_simu_franka_fr3_P.traj_data_bus_init.p_d[j + 2];
      x_err[3] = (static_cast<real_T>(R[5] - R[7] >= 0.0) * 2.0 - 1.0) * s2;
      x_err[4] = (static_cast<real_T>(R[6] - R[2] >= 0.0) * 2.0 - 1.0) * s3;
      x_err[5] = (static_cast<real_T>(R[1] - R[3] >= 0.0) * 2.0 - 1.0) * s4;
      if (realtime_simu_franka_fr3_P.ctrl_param.regularization.mode == 0.0) {
        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            aoffset = 6 * i + jj;
            J_pinv[aoffset] = 0.0;
            for (b = 0; b < 6; b++) {
              J_pinv[aoffset] +=
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
                l[jj] + b] *
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
                l[i] + b];
            }
          }
        }

        realtime_simu_franka_fr3_inv(J_pinv, J_tilde_0);
        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            b = 6 * i + jj;
            J_bar_data[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              J_bar_data[b] += J_tilde_0[6 * aoffset + jj] *
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
                l[aoffset] + i];
            }
          }
        }
      } else if (realtime_simu_franka_fr3_P.ctrl_param.regularization.mode ==
                 1.0) {
        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            s = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              s += realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J
                [6 * l[jj] + aoffset] *
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
                l[i] + aoffset];
            }

            aoffset = 6 * i + jj;
            J_pinv[aoffset] = p_a[aoffset] *
              realtime_simu_franka_fr3_P.ctrl_param.regularization.k + s;
          }
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            T_bar_data[i + 6 * jj] =
              realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
              l[i] + jj];
          }
        }

        realtime_simu_franka_f_mldivide(J_pinv, T_bar_data, J_bar_data);
      } else if (realtime_simu_franka_fr3_P.ctrl_param.regularization.mode ==
                 2.0) {
        s = 0.0;
        for (jj = 0; jj < 6; jj++) {
          x_d_pp_0[jj] = 0.0;
          for (i = 0; i < 6; i++) {
            x_d_pp_0[jj] +=
              realtime_simu_franka_fr3_P.ctrl_param.regularization.W_E[6 * jj +
              i] * (0.5 * x_err[i]);
          }

          s += x_d_pp_0[jj] * x_err[jj];
        }

        for (jj = 0; jj < 36; jj++) {
          J_bar_data[jj] = sqrt
            (realtime_simu_franka_fr3_P.ctrl_param.regularization.W_E[jj]);
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            b = 6 * i + jj;
            J_pinv[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              J_pinv[b] += J_bar_data[6 * aoffset + jj] *
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
                l[i] + aoffset];
            }
          }
        }

        realtime_simu_franka_fr3_svd(J_pinv, S_new, J_bar_data, J_tilde_0);
        for (jj = 0; jj < 6; jj++) {
          x_d_pp[jj] = J_bar_data[6 * jj + jj];
        }

        memset(&J_bar_data[0], 0, 36U * sizeof(real_T));
        for (jj = 0; jj < 6; jj++) {
          J_bar_data[jj + 6 * jj] = x_d_pp[jj];
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            b = 6 * jj + i;
            J_pinv[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              J_pinv[b] += J_bar_data[6 * aoffset + i] * J_bar_data[6 * jj +
                aoffset];
            }
          }
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            b = 6 * i + jj;
            T_bar_data[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              T_bar_data[b] += J_tilde_0[6 * aoffset + jj] * J_pinv[6 * i +
                aoffset];
            }
          }

          for (i = 0; i < 6; i++) {
            b = 6 * i + jj;
            J_tilde_1[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              J_tilde_1[b] += T_bar_data[6 * aoffset + jj] * J_tilde_0[6 *
                aoffset + i];
            }
          }
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            b = 6 * jj + i;
            J_tilde_0[b] = (p_a[b] * s +
                            realtime_simu_franka_fr3_P.ctrl_param.regularization.W_bar_N
                            [l[i]]) + J_tilde_1[b];
          }
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            J_pinv[i + 6 * jj] =
              realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
              l[i] + jj];
          }
        }

        realtime_simu_franka_f_mldivide(J_tilde_0, J_pinv, J_bar_data);
      } else if ((realtime_simu_franka_fr3_P.ctrl_param.regularization.mode ==
                  3.0) ||
                 (realtime_simu_franka_fr3_P.ctrl_param.regularization.mode ==
                  4.0) ||
                 (realtime_simu_franka_fr3_P.ctrl_param.regularization.mode ==
                  5.0)) {
        realtime_simu_franka_fr3_svd(J_red, J_bar_data, S_new, J_tilde_0);
        epsilon = realtime_simu_franka_fr3_P.ctrl_param.regularization.eps;
        if (realtime_simu_franka_fr3_P.ctrl_param.regularization.mode == 3.0) {
          for (i = 0; i < 6; i++) {
            x_err_p[i] = fmax(S_new[6 * i + i], epsilon);
          }

          memset(&S_new[0], 0, 36U * sizeof(real_T));
          for (i = 0; i < 6; i++) {
            S_new[i + 6 * i] = x_err_p[i];
          }
        } else if (realtime_simu_franka_fr3_P.ctrl_param.regularization.mode ==
                   4.0) {
          s = realtime_simu_franka_fr3_P.ctrl_param.regularization.eps *
            realtime_simu_franka_fr3_P.ctrl_param.regularization.eps;
          for (i = 0; i < 6; i++) {
            jj = 6 * i + i;
            rtb_ManualSwitch1 = S_new[jj];
            if (rtb_ManualSwitch1 < epsilon) {
              rtb_ManualSwitch1 = sqrt(rtb_ManualSwitch1 * rtb_ManualSwitch1 + s);
            }

            S_new[jj] = rtb_ManualSwitch1;
          }
        } else {
          int32_T b_size_idx_0;
          i = 0;
          for (jj = 0; jj < 6; jj++) {
            s = S_new[6 * jj + jj];
            isodd = (s < epsilon);
            if (isodd) {
              i++;
            }

            idx[jj] = isodd;
            x_err_p[jj] = s;
          }

          b_size_idx_0 = i;
          jj = 0;
          aoffset = 0;
          for (i = 0; i < 6; i++) {
            if (idx[i]) {
              b_data[jj] = static_cast<int8_T>(i + 1);
              jj++;
              aoffset++;
            }
          }

          jA = aoffset;
          i = 0;
          for (jj = 0; jj < 6; jj++) {
            if (idx[jj]) {
              c_data_0[i] = static_cast<int8_T>(jj + 1);
              i++;
            }
          }

          for (jj = 0; jj < b_size_idx_0; jj++) {
            x_d_pp_0[jj] = (static_cast<real_T>(x_err_p[b_data[jj] - 1] >= 0.0) *
                            2.0 - 1.0) / 4.2094;
          }

          c[0] = static_cast<int8_T>(aoffset);
          aoffset = static_cast<int8_T>(aoffset);
          for (jj = 0; jj < aoffset; jj++) {
            b = static_cast<int8_T>(jA);
            for (i = 0; i < b; i++) {
              S_new[(c_data_0[i] + 6 * (c_data_0[jj] - 1)) - 1] = x_d_pp_0[c[0] *
                jj + i];
            }
          }
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            b = 6 * i + jj;
            J_pinv[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              J_pinv[b] += J_bar_data[6 * aoffset + jj] * S_new[6 * i + aoffset];
            }
          }

          for (i = 0; i < 6; i++) {
            b = 6 * i + jj;
            T_bar_data[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              T_bar_data[b] += J_pinv[6 * aoffset + jj] * J_tilde_0[6 * aoffset
                + i];
            }
          }
        }

        realtime_simu_franka_fr3_pinv(T_bar_data, J_bar_data);
      } else if (realtime_simu_franka_fr3_P.ctrl_param.regularization.mode ==
                 6.0) {
        if (realtime_simu_franka_local_rank(J_red) < 6) {
          realtime_simu_franka_fr_vecnorm(J_red, x_err_p);
          for (jj = 0; jj < 6; jj++) {
            for (i = 0; i < 6; i++) {
              J_tilde_0[jj + 6 * i] =
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
                l[i] + jj] / x_err_p[i];
            }
          }

          for (jj = 0; jj < 6; jj++) {
            for (i = 0; i < 6; i++) {
              b = 6 * i + jj;
              S_new[b] = 0.0;
              for (aoffset = 0; aoffset < 6; aoffset++) {
                S_new[b] += J_tilde_0[6 * jj + aoffset] * J_tilde_0[6 * i +
                  aoffset];
              }
            }
          }

          aoffset = 0;
          for (jj = 0; jj < 5; jj++) {
            memset(&S_new[jj * 6 + 6], 0, static_cast<uint32_T>(aoffset + 1) *
                   sizeof(real_T));
            if (aoffset + 1 < 6) {
              aoffset++;
            }
          }

          memset(&J_tilde_0[0], 0, 36U * sizeof(real_T));
          for (jj = 0; jj < 6; jj++) {
            J_tilde_0[jj + 6 * jj] = 1.0;
          }

          for (jj = 0; jj < 36; jj++) {
            s = S_new[jj] - J_tilde_0[jj];
            epsilon = fabs(s);
            U_0[jj] = (epsilon >
                       realtime_simu_franka_fr3_P.ctrl_param.regularization.eps_collinear);
            S_new[jj] = s;
            J_tilde_0[jj] = epsilon;
          }

          realtime_simu_franka_f_eml_find(U_0, ii_data, &jj, jj_data, &b);
          for (jj = 0; jj < b; jj++) {
            for (i = 0; i < 6; i++) {
              J_red[i + 6 * (jj_data[jj] - 1)] = 0.0;
            }
          }

          realtime_simu_franka_fr3_pinv(J_red, J_bar_data);
        } else {
          realtime_simu_franka_f_mldivide(J_red, p_a, J_bar_data);
        }
      } else {
        int32_T b_size_idx_0;
        realtime_simu_franka_fr_vecnorm(J_red, x_err_p);
        for (jj = 0; jj < 6; jj++) {
          x_err_p[jj] = 1.0 / x_err_p[jj];
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            J_tilde_0[jj + 6 * i] =
              realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[6 *
              l[i] + jj] * x_err_p[i];
          }
        }

        for (jj = 0; jj < 6; jj++) {
          for (i = 0; i < 6; i++) {
            b = 6 * i + jj;
            T_bar_data[b] = 0.0;
            for (aoffset = 0; aoffset < 6; aoffset++) {
              T_bar_data[b] += J_tilde_0[6 * jj + aoffset] * J_tilde_0[6 * i +
                aoffset];
            }
          }
        }

        realtime_simu_franka_fr3_svd(T_bar_data, S_new, J_bar_data, J_tilde_0);
        i = 0;
        for (jj = 0; jj < 6; jj++) {
          isodd = (J_bar_data[6 * jj + jj] >
                   realtime_simu_franka_fr3_P.ctrl_param.regularization.lambda_min);
          if (isodd) {
            i++;
          }

          idx[jj] = isodd;
        }

        b_size_idx_0 = i;
        i = 0;
        for (jj = 0; jj < 6; jj++) {
          if (idx[jj]) {
            b_data[i] = static_cast<int8_T>(jj + 1);
            i++;
          }
        }

        T_bar_size[0] = 6;
        T_bar_size[1] = b_size_idx_0;
        for (jj = 0; jj < b_size_idx_0; jj++) {
          for (i = 0; i < 6; i++) {
            T_bar_data[i + 6 * jj] = J_tilde_0[(b_data[jj] - 1) * 6 + i] *
              x_err_p[i];
          }
        }

        J_bar_size[0] = 6;
        J_bar_size[1] = b_size_idx_0;
        for (jj = 0; jj < b_size_idx_0; jj++) {
          coffset_tmp = jj * 6;
          for (aoffset = 0; aoffset < 6; aoffset++) {
            s = 0.0;
            for (i = 0; i < 6; i++) {
              b = i * 6 + aoffset;
              s +=
                realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J[l[b
                / 6] * 6 + b % 6] * T_bar_data[coffset_tmp + i];
            }

            J_bar_data[coffset_tmp + aoffset] = s;
          }
        }

        realtime_simu_frank_mldivide_d0(J_bar_data, J_bar_size, J_tilde_0,
          tmp_size);
        realtime_simu_franka_fr3_mtimes(T_bar_data, T_bar_size, J_tilde_0,
          tmp_size, J_bar_data);
      }

      for (jj = 0; jj < 6; jj++) {
        for (i = 0; i < 6; i++) {
          J_tilde_0[i + 6 * jj] = J_bar_data[6 * i + jj];
        }
      }

      for (jj = 0; jj < 6; jj++) {
        for (i = 0; i < 6; i++) {
          b = 6 * i + jj;
          T_bar_data[b] = 0.0;
          for (aoffset = 0; aoffset < 6; aoffset++) {
            T_bar_data[b] += J_tilde_0[6 * aoffset + jj] * Kp1[6 * i + aoffset];
          }
        }

        for (i = 0; i < 6; i++) {
          b = 6 * i + jj;
          S_new[b] = 0.0;
          for (aoffset = 0; aoffset < 6; aoffset++) {
            S_new[b] += T_bar_data[6 * aoffset + jj] * J_bar_data[6 * i +
              aoffset];
          }
        }
      }

      x_d_pp_0[0] = realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_pp[j];
      x_d_pp_0[3] = realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d_p[j];
      x_d_pp_0[1] = realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_pp[j + 1];
      x_d_pp_0[4] = realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d_p[j +
        1];
      x_d_pp_0[2] = realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_pp[j + 2];
      x_d_pp_0[5] = realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d_p[j +
        2];
      for (jj = 0; jj < 6; jj++) {
        for (i = 0; i < 6; i++) {
          aoffset = 6 * jj + i;
          J_pinv[aoffset] = (S_new[6 * i + jj] + S_new[aoffset]) * 0.5;
          T_bar_data[aoffset] = 0.0;
          for (b = 0; b < 6; b++) {
            T_bar_data[aoffset] += Kp1[6 * b + i] * J_bar_data[6 * jj + b];
          }
        }
      }

      for (jj = 0; jj < 6; jj++) {
        for (i = 0; i < 6; i++) {
          s = 0.0;
          for (aoffset = 0; aoffset < 6; aoffset++) {
            s += T_bar_data[6 * aoffset + jj] *
              realtime_simu_franka_fr3_B.sf_Robotmodelbus1.robot_model_b.J_p[6 *
              l[i] + aoffset];
          }

          aoffset = 6 * i + jj;
          S_new[aoffset] = Kd1[aoffset] - s;
        }
      }

      rtb_ManualSwitch_g[0] = rtb_ManualSwitch_o[0] -
        realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_p[j];
      rtb_ManualSwitch_g[3] = rtb_ManualSwitch_o[3] -
        realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d[j];
      rtb_ManualSwitch_g[1] = rtb_ManualSwitch_o[1] -
        realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_p[j + 1];
      rtb_ManualSwitch_g[4] = rtb_ManualSwitch_o[4] -
        realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d[j + 1];
      rtb_ManualSwitch_g[2] = rtb_ManualSwitch_o[2] -
        realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_p[j + 2];
      rtb_ManualSwitch_g[5] = rtb_ManualSwitch_o[5] -
        realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d[j + 2];
      for (jj = 0; jj < 6; jj++) {
        for (i = 0; i < 6; i++) {
          b = 6 * i + jj;
          T_bar_data[b] = 0.0;
          for (aoffset = 0; aoffset < 6; aoffset++) {
            T_bar_data[b] += J_tilde_0[6 * aoffset + jj] * S_new[6 * i + aoffset];
          }
        }

        x_err_p[jj] = 0.0;
        x_d_pp[jj] = 0.0;
        s = 0.0;
        b_a_0[jj] = 0.0;
        for (i = 0; i < 6; i++) {
          b = 6 * i + jj;
          J_tilde_1[b] = 0.0;
          for (aoffset = 0; aoffset < 6; aoffset++) {
            J_tilde_1[b] += T_bar_data[6 * aoffset + jj] * J_bar_data[6 * i +
              aoffset];
          }

          s += U[b] * rtb_ManualSwitch_g[i];
          x_err_p[jj] += J_pinv[b] * x_d_pp_0[i];
          x_d_pp[jj] += J_tilde_1[b] * rtb_ManualSwitch_o[i];
          b_a_0[jj] += b_a[b] * x_err[i];
        }

        Kp1_0[jj] = ((x_err_p[jj] + x_d_pp[jj]) - s) - b_a_0[jj];
      }

      for (jj = 0; jj < 6; jj++) {
        x_d_pp[jj] = 0.0;
        s = 0.0;
        for (i = 0; i < 6; i++) {
          s += Kd1[6 * i + jj] * q_p_red[i];
          x_d_pp[jj] += J_red[6 * jj + i] * Kp1_0[i];
        }

        x_err_p[jj] = x_d_pp[jj] - s;
      }

      realtime_simu_fran_mldivide_oqz(Kp1, x_err_p);
      for (i = 0; i < 7; i++) {
        rtb_q_ref[i] = 0.0;
      }

      for (jj = 0; jj < 6; jj++) {
        rtb_q_ref[l[jj]] = x_err_p[jj];
      }

      for (i = 0; i < 7; i++) {
        /* Merge: '<S2>/Merge1' incorporates:
         *  MATLAB Function: '<S12>/PD+ MATLAB Function'
         */
        realtime_simu_franka_fr3_B.Merge1[i] = 0.0;
      }

      /* MATLAB Function: '<S12>/PD+ MATLAB Function' incorporates:
       *  BusCreator generated from: '<S12>/PD+ MATLAB Function'
       *  Constant: '<Root>/Start Trajectory'
       *  Constant: '<Root>/Stop Trajectory'
       *  Constant: '<S3>/Constant3'
       *  Merge: '<S13>/Merge'
       *  Merge: '<S2>/Merge1'
       */
      for (jj = 0; jj < 6; jj++) {
        realtime_simu_franka_fr3_B.Merge1[l[jj]] = x_d_pp[jj];
      }

      s = 0.0;
      epsilon = 0.0;
      for (jj = 0; jj < 7; jj++) {
        i = 7 * jj + 2;
        s += realtime_simu_franka_fr3_B.Merge_i.M[i] * rtb_q_ref[jj];
        epsilon += realtime_simu_franka_fr3_B.Merge_i.C[i] *
          realtime_simu_franka_fr3_B.Merge_i.q_p[jj];
      }

      realtime_simu_franka_fr3_B.Merge1[2] = s + epsilon;
      if (realtime_simu_franka_fr3_DW.run_flag_c == 0.0) {
        if ((realtime_simu_franka_fr3_P.StartTrajectory_Value == 1.0) &&
            (time_end == 0.0) &&
            (realtime_simu_franka_fr3_P.StopTrajectory_Value == 0.0)) {
          realtime_simu_franka_fr3_DW.run_flag_c = 1.0;
          if (realtime_simu_franka_fr3_DW.cnt_o <
              realtime_simu_franka_fr3_P.traj_data_bus_init.N) {
            realtime_simu_franka_fr3_DW.cnt_o++;
          }
        } else if ((time_end == 1.0) && (realtime_simu_franka_fr3_DW.cnt_o ==
                    realtime_simu_franka_fr3_P.traj_data_bus_init.N)) {
          realtime_simu_franka_fr3_DW.cnt_o = 1.0;
        }
      } else if (realtime_simu_franka_fr3_P.StopTrajectory_Value == 1.0) {
        realtime_simu_franka_fr3_DW.run_flag_c = 0.0;
      } else if (realtime_simu_franka_fr3_DW.cnt_o <
                 realtime_simu_franka_fr3_P.traj_data_bus_init.N) {
        realtime_simu_franka_fr3_DW.cnt_o++;
      } else {
        realtime_simu_franka_fr3_DW.run_flag_c = 0.0;
      }

      srUpdateBC(realtime_simu_franka_fr3_DW.PDControllerSubsystem_SubsysRan);
    }

    /* End of Selector: '<S2>/Selector11' */
    /* End of Outputs for SubSystem: '<S2>/PD+ Controller Subsystem' */
    for (i = 0; i < 7; i++) {
      /* MATLAB Function: '<S7>/joint space control fixed q3' incorporates:
       *  Merge: '<S2>/Merge1'
       */
      realtime_simu_franka_fr3_B.tau_p[i] = realtime_simu_franka_fr3_B.Merge1[i];

      /* Sqrt: '<S7>/Sqrt' incorporates:
       *  Constant: '<S7>/K_d_jointspace1'
       */
      tmp[i] = sqrt(realtime_simu_franka_fr3_P.K_d_jointspace1_Value[i]);
    }

    /* MATLAB Function: '<S7>/joint space control fixed q3' incorporates:
     *  Constant: '<Root>/trajectory selector'
     *  Constant: '<S7>/K_d_jointspace1'
     *  Gain: '<S7>/Gain'
     *  MATLAB Function: '<S7>/get q_0_ref'
     *  Merge: '<S2>/Merge1'
     *  Merge: '<S6>/Merge'
     *  Sqrt: '<S7>/Sqrt'
     */
    realtime_simu_franka_fr3_B.tau_p[2] = ((realtime_simu_franka_fr3_B.Merge.q[2]
      - b_0[(static_cast<int32_T>
             (realtime_simu_franka_fr3_P.trajectoryselector_Value) - 1) * 7 + 2])
      * (-realtime_simu_franka_fr3_B.Merge.M[16] *
         realtime_simu_franka_fr3_P.K_d_jointspace1_Value[2]) -
      realtime_simu_franka_fr3_P.Gain_Gain_l * tmp[2] *
      realtime_simu_franka_fr3_B.Merge.q_p[2]) +
      realtime_simu_franka_fr3_B.Merge1[2];
    for (i = 0; i < 7; i++) {
      /* Switch: '<Root>/Switch' incorporates:
       *  ManualSwitch: '<Root>/Manual Switch1'
       */
      if (realtime_simu_franka_fr3_B.home_running >
          realtime_simu_franka_fr3_P.Switch_Threshold_n) {
        /* Switch: '<Root>/Switch' */
        realtime_simu_franka_fr3_B.Switch_b[i] =
          realtime_simu_franka_fr3_B.tau_k[i];
      } else if (realtime_simu_franka_fr3_P.ManualSwitch1_CurrentSetting == 1) {
        /* ManualSwitch: '<Root>/Manual Switch' incorporates:
         *  ManualSwitch: '<Root>/Manual Switch1'
         */
        if (realtime_simu_franka_fr3_P.ManualSwitch_CurrentSetting_a == 1) {
          /* Switch: '<Root>/Switch' */
          realtime_simu_franka_fr3_B.Switch_b[i] =
            realtime_simu_franka_fr3_B.tau_p[i];
        } else {
          /* Switch: '<Root>/Switch' */
          realtime_simu_franka_fr3_B.Switch_b[i] =
            realtime_simu_franka_fr3_B.tau[i];
        }

        /* End of ManualSwitch: '<Root>/Manual Switch' */
      } else {
        /* Switch: '<Root>/Switch' incorporates:
         *  Constant: '<Root>/Constant'
         *  ManualSwitch: '<Root>/Manual Switch1'
         */
        realtime_simu_franka_fr3_B.Switch_b[i] =
          realtime_simu_franka_fr3_P.Constant_Value_i[i];
      }

      /* End of Switch: '<Root>/Switch' */

      /* Sum: '<S4>/Add' incorporates:
       *  Switch: '<Root>/Switch'
       */
      realtime_simu_franka_fr3_B.Add[i] = realtime_simu_franka_fr3_B.Switch_b[i]
        - realtime_simu_franka_fr3_B.Merge.g[i];
    }

    /* MATLAB Function: '<S3>/MATLAB Function' incorporates:
     *  BusCreator generated from: '<S3>/MATLAB Function'
     *  Constant: '<Root>/Start Trajectory'
     *  Constant: '<Root>/Stop Trajectory'
     *  Constant: '<Root>/trajectory selector'
     *  Constant: '<S3>/Constant3'
     */
    for (jj = 0; jj < 3; jj++) {
      j = ((static_cast<int32_T>(realtime_simu_franka_fr3_DW.cnt_p) - 1) * 3 +
           jj) + (static_cast<int32_T>
                  (realtime_simu_franka_fr3_P.trajectoryselector_Value) - 1) *
        36003;
      realtime_simu_franka_fr3_B.p_d[jj] =
        realtime_simu_franka_fr3_P.traj_data_bus_init.p_d[j];
      realtime_simu_franka_fr3_B.p_d_p[jj] =
        realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_p[j];
      realtime_simu_franka_fr3_B.p_d_pp[jj] =
        realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_pp[j];
      j = ((static_cast<int32_T>(realtime_simu_franka_fr3_DW.cnt_p) - 1) * 9 + 3
           * jj) + (static_cast<int32_T>
                    (realtime_simu_franka_fr3_P.trajectoryselector_Value) - 1) *
        108009;
      rtb_x_d_R_d[3 * jj] = realtime_simu_franka_fr3_P.traj_data_bus_init.R_d[j];
      rtb_x_d_R_d[3 * jj + 1] =
        realtime_simu_franka_fr3_P.traj_data_bus_init.R_d[j + 1];
      rtb_x_d_R_d[3 * jj + 2] =
        realtime_simu_franka_fr3_P.traj_data_bus_init.R_d[j + 2];
    }

    j = ((static_cast<int32_T>(realtime_simu_franka_fr3_DW.cnt_p) - 1) << 2) + (
      static_cast<int32_T>(realtime_simu_franka_fr3_P.trajectoryselector_Value)
      - 1) * 48004;
    s = realtime_simu_franka_fr3_P.traj_data_bus_init.q_d[j];
    epsilon = realtime_simu_franka_fr3_P.traj_data_bus_init.q_d[j + 1];
    rtb_ManualSwitch1 = realtime_simu_franka_fr3_P.traj_data_bus_init.q_d[j + 2];
    rtb_x_d_q_d_idx_3 = realtime_simu_franka_fr3_P.traj_data_bus_init.q_d[j + 3];
    j = (static_cast<int32_T>(realtime_simu_franka_fr3_DW.cnt_p) - 1) * 3 + (
      static_cast<int32_T>(realtime_simu_franka_fr3_P.trajectoryselector_Value)
      - 1) * 36003;
    realtime_simu_franka_fr3_B.omega_d[0] =
      realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d[j];
    realtime_simu_franka_fr3_B.omega_d_p[0] =
      realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d_p[j];
    realtime_simu_franka_fr3_B.omega_d[1] =
      realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d[j + 1];
    realtime_simu_franka_fr3_B.omega_d_p[1] =
      realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d_p[j + 1];
    realtime_simu_franka_fr3_B.omega_d[2] =
      realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d[j + 2];
    realtime_simu_franka_fr3_B.omega_d_p[2] =
      realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d_p[j + 2];
    if (realtime_simu_franka_fr3_DW.run_flag == 0.0) {
      if ((realtime_simu_franka_fr3_P.StartTrajectory_Value == 1.0) && (time_end
           == 0.0) && (realtime_simu_franka_fr3_P.StopTrajectory_Value == 0.0))
      {
        realtime_simu_franka_fr3_DW.run_flag = 1.0;
        if (realtime_simu_franka_fr3_DW.cnt_p <
            realtime_simu_franka_fr3_P.traj_data_bus_init.N) {
          realtime_simu_franka_fr3_DW.cnt_p++;
        }
      } else if ((time_end == 1.0) && (realtime_simu_franka_fr3_DW.cnt_p ==
                  realtime_simu_franka_fr3_P.traj_data_bus_init.N)) {
        realtime_simu_franka_fr3_DW.cnt_p = 1.0;
      }
    } else if (realtime_simu_franka_fr3_DW.run_flag == 1.0) {
      if (realtime_simu_franka_fr3_P.StopTrajectory_Value == 1.0) {
        realtime_simu_franka_fr3_DW.run_flag = 0.0;
      } else if (realtime_simu_franka_fr3_DW.cnt_p <
                 realtime_simu_franka_fr3_P.traj_data_bus_init.N) {
        realtime_simu_franka_fr3_DW.cnt_p++;
      } else {
        realtime_simu_franka_fr3_DW.run_flag = 0.0;
      }
    }

    /* End of MATLAB Function: '<S3>/MATLAB Function' */

    /* MATLAB Function: '<S4>/calc_errors' incorporates:
     *  Merge: '<S6>/Merge'
     */
    for (jj = 0; jj < 3; jj++) {
      for (i = 0; i < 3; i++) {
        j = 3 * i + jj;
        R[j] = 0.0;
        R[j] += realtime_simu_franka_fr3_B.Merge.H[jj] * rtb_x_d_R_d[i];
        R[j] += realtime_simu_franka_fr3_B.Merge.H[jj + 4] * rtb_x_d_R_d[i + 3];
        R[j] += realtime_simu_franka_fr3_B.Merge.H[jj + 8] * rtb_x_d_R_d[i + 6];
      }
    }

    time_end = (R[0] + R[4]) + R[8];
    s2 = (R[0] - R[4]) - R[8];
    s3 = (-R[0] + R[4]) - R[8];
    s4 = (-R[0] - R[4]) + R[8];
    if (time_end > 0.0) {
      time_end = sqrt(time_end + 1.0) / 2.0;
    } else {
      time_end = R[5] - R[7];
      h_a = R[6] - R[2];
      i_a = R[1] - R[3];
      time_end = sqrt(((time_end * time_end + h_a * h_a) + i_a * i_a) / (((3.0 -
        R[0]) - R[4]) - R[8])) / 2.0;
    }

    if (s2 > 0.0) {
      s2 = sqrt(s2 + 1.0) / 2.0;
    } else {
      s2 = R[5] - R[7];
      e_a = R[1] + R[3];
      i_a = R[2] + R[6];
      s2 = sqrt(((s2 * s2 + e_a * e_a) + i_a * i_a) / (((3.0 - R[0]) + R[4]) +
                 R[8])) / 2.0;
    }

    if (s3 > 0.0) {
      s3 = sqrt(s3 + 1.0) / 2.0;
    } else {
      i_a = R[6] - R[2];
      h_a = R[1] + R[3];
      s3 = R[5] + R[7];
      s3 = sqrt(((i_a * i_a + h_a * h_a) + s3 * s3) / (((R[0] + 3.0) - R[4]) +
                 R[8])) / 2.0;
    }

    if (s4 > 0.0) {
      s4 = sqrt(s4 + 1.0) / 2.0;
    } else {
      s4 = R[1] - R[3];
      h_a = R[2] + R[6];
      i_a = R[5] + R[7];
      s4 = sqrt(((s4 * s4 + h_a * h_a) + i_a * i_a) / (((R[0] + 3.0) + R[4]) -
                 R[8])) / 2.0;
    }

    s2 *= static_cast<real_T>(R[5] - R[7] >= 0.0) * 2.0 - 1.0;
    s3 *= static_cast<real_T>(R[6] - R[2] >= 0.0) * 2.0 - 1.0;
    s4 *= static_cast<real_T>(R[1] - R[3] >= 0.0) * 2.0 - 1.0;
    for (jj = 0; jj < 6; jj++) {
      q_p_red[jj] = 0.0;
      x_d_pp_0[jj] = 0.0;
      x_err_p[jj] = 0.0;
      for (i = 0; i < 7; i++) {
        j = 6 * i + jj;
        i_a = realtime_simu_franka_fr3_B.Merge.J[j];
        q_p_red[jj] += i_a * realtime_simu_franka_fr3_B.Merge.q_p[i];
        x_d_pp_0[jj] += i_a * realtime_simu_franka_fr3_B.Merge.q_pp[i];
        x_err_p[jj] += realtime_simu_franka_fr3_B.Merge.J_p[j] *
          realtime_simu_franka_fr3_B.Merge.q_p[i];
      }

      rtb_ManualSwitch_o[jj] = x_d_pp_0[jj] + x_err_p[jj];
    }

    realtime_simu_franka_fr3_B.p[0] = realtime_simu_franka_fr3_B.Merge.H[12];
    realtime_simu_franka_fr3_B.p_p[0] = q_p_red[0];
    realtime_simu_franka_fr3_B.p_pp[0] = rtb_ManualSwitch_o[0];
    realtime_simu_franka_fr3_B.omega_e[0] = q_p_red[3];
    realtime_simu_franka_fr3_B.omega_e_p[0] = rtb_ManualSwitch_o[3];
    realtime_simu_franka_fr3_B.q_e_o[0] = (s * s2 + time_end * epsilon) +
      (rtb_ManualSwitch1 * s4 - s3 * rtb_x_d_q_d_idx_3);
    realtime_simu_franka_fr3_B.q_d_o[0] = epsilon;
    realtime_simu_franka_fr3_B.q_err_o[0] = s2;
    realtime_simu_franka_fr3_B.omega_d_err[0] =
      realtime_simu_franka_fr3_B.omega_e[0] -
      realtime_simu_franka_fr3_B.omega_d[0];
    realtime_simu_franka_fr3_B.omega_d_p_err[0] =
      realtime_simu_franka_fr3_B.omega_e_p[0] -
      realtime_simu_franka_fr3_B.omega_d_p[0];
    realtime_simu_franka_fr3_B.p[1] = realtime_simu_franka_fr3_B.Merge.H[13];
    realtime_simu_franka_fr3_B.p_p[1] = q_p_red[1];
    realtime_simu_franka_fr3_B.p_pp[1] = rtb_ManualSwitch_o[1];
    realtime_simu_franka_fr3_B.omega_e[1] = q_p_red[4];
    realtime_simu_franka_fr3_B.omega_e_p[1] = rtb_ManualSwitch_o[4];
    realtime_simu_franka_fr3_B.q_e_o[1] = (s * s3 + time_end * rtb_ManualSwitch1)
      + (s2 * rtb_x_d_q_d_idx_3 - epsilon * s4);
    realtime_simu_franka_fr3_B.q_d_o[1] = rtb_ManualSwitch1;
    realtime_simu_franka_fr3_B.q_err_o[1] = s3;
    realtime_simu_franka_fr3_B.omega_d_err[1] =
      realtime_simu_franka_fr3_B.omega_e[1] -
      realtime_simu_franka_fr3_B.omega_d[1];
    realtime_simu_franka_fr3_B.omega_d_p_err[1] =
      realtime_simu_franka_fr3_B.omega_e_p[1] -
      realtime_simu_franka_fr3_B.omega_d_p[1];
    realtime_simu_franka_fr3_B.p[2] = realtime_simu_franka_fr3_B.Merge.H[14];
    realtime_simu_franka_fr3_B.p_p[2] = q_p_red[2];
    realtime_simu_franka_fr3_B.p_pp[2] = rtb_ManualSwitch_o[2];
    realtime_simu_franka_fr3_B.omega_e[2] = q_p_red[5];
    realtime_simu_franka_fr3_B.omega_e_p[2] = rtb_ManualSwitch_o[5];
    realtime_simu_franka_fr3_B.q_e_o[2] = (s * s4 + time_end * rtb_x_d_q_d_idx_3)
      + (epsilon * s3 - s2 * rtb_ManualSwitch1);
    realtime_simu_franka_fr3_B.q_d_o[2] = rtb_x_d_q_d_idx_3;
    realtime_simu_franka_fr3_B.q_err_o[2] = s4;
    realtime_simu_franka_fr3_B.omega_d_err[2] =
      realtime_simu_franka_fr3_B.omega_e[2] -
      realtime_simu_franka_fr3_B.omega_d[2];
    realtime_simu_franka_fr3_B.omega_d_p_err[2] =
      realtime_simu_franka_fr3_B.omega_e_p[2] -
      realtime_simu_franka_fr3_B.omega_d_p[2];
    realtime_simu_franka_fr3_B.e_x = realtime_simu_franka_fr3_B.p_d[0] -
      realtime_simu_franka_fr3_B.p[0];
    realtime_simu_franka_fr3_B.e_x_p = realtime_simu_franka_fr3_B.p_d_p[0] -
      realtime_simu_franka_fr3_B.p_p[0];
    realtime_simu_franka_fr3_B.e_x_pp = realtime_simu_franka_fr3_B.p_d_pp[0] -
      realtime_simu_franka_fr3_B.p_pp[0];
    realtime_simu_franka_fr3_B.e_y = realtime_simu_franka_fr3_B.p_d[1] -
      realtime_simu_franka_fr3_B.p[1];
    realtime_simu_franka_fr3_B.e_y_p = realtime_simu_franka_fr3_B.p_d_p[1] -
      realtime_simu_franka_fr3_B.p_p[1];
    realtime_simu_franka_fr3_B.e_y_pp = realtime_simu_franka_fr3_B.p_d_pp[1] -
      realtime_simu_franka_fr3_B.p_pp[1];
    realtime_simu_franka_fr3_B.e_z = realtime_simu_franka_fr3_B.p_d[2] -
      realtime_simu_franka_fr3_B.p[2];
    realtime_simu_franka_fr3_B.e_z_p = realtime_simu_franka_fr3_B.p_d_p[2] -
      realtime_simu_franka_fr3_B.p_p[2];
    realtime_simu_franka_fr3_B.e_z_pp = realtime_simu_franka_fr3_B.p_d_pp[2] -
      realtime_simu_franka_fr3_B.p_pp[2];

    /* End of MATLAB Function: '<S4>/calc_errors' */
    for (i = 0; i < 7; i++) {
      /* SignalConversion generated from: '<S4>/Bus Selector1' */
      realtime_simu_franka_fr3_B.q[i] = realtime_simu_franka_fr3_B.Merge.q[i];

      /* SignalConversion generated from: '<S4>/Bus Selector1' */
      realtime_simu_franka_fr3_B.q_p[i] = realtime_simu_franka_fr3_B.Merge.q_p[i];
    }

    /* MATLAB Function: '<S24>/manipulability and collinearity 7DOF' incorporates:
     *  Merge: '<S6>/Merge'
     */
    for (jj = 0; jj < 6; jj++) {
      for (i = 0; i < 6; i++) {
        b = 6 * i + jj;
        J_red[b] = 0.0;
        for (aoffset = 0; aoffset < 6; aoffset++) {
          j = 6 * l[aoffset];
          J_red[b] += realtime_simu_franka_fr3_B.Merge.J[j + jj] *
            realtime_simu_franka_fr3_B.Merge.J[j + i];
        }
      }

      c_data[jj] = static_cast<int8_T>(jj + 1);
    }

    for (j = 0; j < 5; j++) {
      jj = j * 7;
      aoffset = 6 - j;
      b = 0;
      time_end = fabs(J_red[jj]);
      for (i = 2; i <= aoffset; i++) {
        s = fabs(J_red[(jj + i) - 1]);
        if (s > time_end) {
          b = i - 1;
          time_end = s;
        }
      }

      if (J_red[jj + b] != 0.0) {
        if (b != 0) {
          aoffset = j + b;
          c_data[j] = static_cast<int8_T>(aoffset + 1);
          for (i = 0; i < 6; i++) {
            coffset_tmp = i * 6 + j;
            time_end = J_red[coffset_tmp];
            b = i * 6 + aoffset;
            J_red[coffset_tmp] = J_red[b];
            J_red[b] = time_end;
          }
        }

        b = (jj - j) + 6;
        for (i = jj + 2; i <= b; i++) {
          J_red[i - 1] /= J_red[jj];
        }
      }

      coffset_tmp = 4 - j;
      jA = jj + 8;
      for (i = 0; i <= coffset_tmp; i++) {
        s = J_red[(i * 6 + jj) + 6];
        if (s != 0.0) {
          aoffset = (jA - j) + 4;
          for (b = jA; b <= aoffset; b++) {
            J_red[b - 1] += J_red[((jj + b) - jA) + 1] * -s;
          }
        }

        jA += 6;
      }
    }

    time_end = J_red[0];
    isodd = false;
    for (jj = 0; jj < 5; jj++) {
      time_end *= J_red[((jj + 1) * 6 + jj) + 1];
      if (c_data[jj] > jj + 1) {
        isodd = !isodd;
      }
    }

    if (isodd) {
      time_end = -time_end;
    }

    realtime_simu_franka_fr3_B.w = fabs(time_end);
    realtime_simu_franka_fr3_B.w = sqrt(realtime_simu_franka_fr3_B.w);
    for (i = 0; i < 7; i++) {
      j = i * 6 + 1;
      time_end = 0.0;
      s = 3.3121686421112381E-170;
      for (jj = j; jj <= j + 5; jj++) {
        epsilon = fabs(realtime_simu_franka_fr3_B.Merge.J[jj - 1]);
        if (epsilon > s) {
          rtb_ManualSwitch1 = s / epsilon;
          time_end = time_end * rtb_ManualSwitch1 * rtb_ManualSwitch1 + 1.0;
          s = epsilon;
        } else {
          rtb_ManualSwitch1 = epsilon / s;
          time_end += rtb_ManualSwitch1 * rtb_ManualSwitch1;
        }
      }

      rtb_q_ref[i] = s * sqrt(time_end);
      for (jj = 0; jj < 6; jj++) {
        b = 6 * i + jj;
        J_tilde[b] = realtime_simu_franka_fr3_B.Merge.J[b] / rtb_q_ref[i];
      }
    }

    for (jj = 0; jj < 7; jj++) {
      for (i = 0; i < 7; i++) {
        j = 7 * i + jj;
        JJ_colin[j] = 0.0;
        for (aoffset = 0; aoffset < 6; aoffset++) {
          JJ_colin[j] += J_tilde[6 * jj + aoffset] * J_tilde[6 * i + aoffset];
        }
      }
    }

    /* SignalConversion generated from: '<S4>/From25' */
    realtime_simu_franka_fr3_B.p_emYxByPz[0] = realtime_simu_franka_fr3_B.p[0];
    realtime_simu_franka_fr3_B.p_emYxByPz[3] = realtime_simu_franka_fr3_B.p_d[0];

    /* SignalConversion generated from: '<S4>/From26' */
    realtime_simu_franka_fr3_B._emsYxByPz[0] = realtime_simu_franka_fr3_B.p_p[0];
    realtime_simu_franka_fr3_B._emsYxByPz[3] = realtime_simu_franka_fr3_B.p_d_p
      [0];

    /* SignalConversion generated from: '<S4>/From38' */
    realtime_simu_franka_fr3_B.quat_e24[0] = realtime_simu_franka_fr3_B.q_e_o[0];
    realtime_simu_franka_fr3_B.quat_e24[3] = realtime_simu_franka_fr3_B.q_d_o[0];

    /* SignalConversion generated from: '<S4>/From39' */
    realtime_simu_franka_fr3_B.p_emsYxByPz[0] = realtime_simu_franka_fr3_B.p_pp
      [0];
    realtime_simu_franka_fr3_B.p_emsYxByPz[3] =
      realtime_simu_franka_fr3_B.p_d_pp[0];

    /* SignalConversion generated from: '<S4>/From40' */
    realtime_simu_franka_fr3_B._erads[0] = realtime_simu_franka_fr3_B.omega_e[0];
    realtime_simu_franka_fr3_B._erads[3] = realtime_simu_franka_fr3_B.omega_d[0];

    /* SignalConversion generated from: '<S4>/From41' */
    realtime_simu_franka_fr3_B._erads_p[0] =
      realtime_simu_franka_fr3_B.omega_e_p[0];
    realtime_simu_franka_fr3_B._erads_p[3] =
      realtime_simu_franka_fr3_B.omega_d_p[0];

    /* SignalConversion generated from: '<S4>/From25' */
    realtime_simu_franka_fr3_B.p_emYxByPz[1] = realtime_simu_franka_fr3_B.p[1];
    realtime_simu_franka_fr3_B.p_emYxByPz[4] = realtime_simu_franka_fr3_B.p_d[1];

    /* SignalConversion generated from: '<S4>/From26' */
    realtime_simu_franka_fr3_B._emsYxByPz[1] = realtime_simu_franka_fr3_B.p_p[1];
    realtime_simu_franka_fr3_B._emsYxByPz[4] = realtime_simu_franka_fr3_B.p_d_p
      [1];

    /* SignalConversion generated from: '<S4>/From38' */
    realtime_simu_franka_fr3_B.quat_e24[1] = realtime_simu_franka_fr3_B.q_e_o[1];
    realtime_simu_franka_fr3_B.quat_e24[4] = realtime_simu_franka_fr3_B.q_d_o[1];

    /* SignalConversion generated from: '<S4>/From39' */
    realtime_simu_franka_fr3_B.p_emsYxByPz[1] = realtime_simu_franka_fr3_B.p_pp
      [1];
    realtime_simu_franka_fr3_B.p_emsYxByPz[4] =
      realtime_simu_franka_fr3_B.p_d_pp[1];

    /* SignalConversion generated from: '<S4>/From40' */
    realtime_simu_franka_fr3_B._erads[1] = realtime_simu_franka_fr3_B.omega_e[1];
    realtime_simu_franka_fr3_B._erads[4] = realtime_simu_franka_fr3_B.omega_d[1];

    /* SignalConversion generated from: '<S4>/From41' */
    realtime_simu_franka_fr3_B._erads_p[1] =
      realtime_simu_franka_fr3_B.omega_e_p[1];
    realtime_simu_franka_fr3_B._erads_p[4] =
      realtime_simu_franka_fr3_B.omega_d_p[1];

    /* SignalConversion generated from: '<S4>/From25' */
    realtime_simu_franka_fr3_B.p_emYxByPz[2] = realtime_simu_franka_fr3_B.p[2];
    realtime_simu_franka_fr3_B.p_emYxByPz[5] = realtime_simu_franka_fr3_B.p_d[2];

    /* SignalConversion generated from: '<S4>/From26' */
    realtime_simu_franka_fr3_B._emsYxByPz[2] = realtime_simu_franka_fr3_B.p_p[2];
    realtime_simu_franka_fr3_B._emsYxByPz[5] = realtime_simu_franka_fr3_B.p_d_p
      [2];

    /* SignalConversion generated from: '<S4>/From38' */
    realtime_simu_franka_fr3_B.quat_e24[2] = realtime_simu_franka_fr3_B.q_e_o[2];
    realtime_simu_franka_fr3_B.quat_e24[5] = realtime_simu_franka_fr3_B.q_d_o[2];

    /* SignalConversion generated from: '<S4>/From39' */
    realtime_simu_franka_fr3_B.p_emsYxByPz[2] = realtime_simu_franka_fr3_B.p_pp
      [2];
    realtime_simu_franka_fr3_B.p_emsYxByPz[5] =
      realtime_simu_franka_fr3_B.p_d_pp[2];

    /* SignalConversion generated from: '<S4>/From40' */
    realtime_simu_franka_fr3_B._erads[2] = realtime_simu_franka_fr3_B.omega_e[2];
    realtime_simu_franka_fr3_B._erads[5] = realtime_simu_franka_fr3_B.omega_d[2];

    /* SignalConversion generated from: '<S4>/From41' */
    realtime_simu_franka_fr3_B._erads_p[2] =
      realtime_simu_franka_fr3_B.omega_e_p[2];
    realtime_simu_franka_fr3_B._erads_p[5] =
      realtime_simu_franka_fr3_B.omega_d_p[2];

    /* SignalConversion generated from: '<S4>/From50' */
    realtime_simu_franka_fr3_B.freqperTastepHz[0] =
      realtime_simu_franka_fr3_B.freq_per_step;
    realtime_simu_franka_fr3_B.freqperTastepHz[1] =
      realtime_simu_franka_fr3_B.freq_per_step_mean;

    /* RateLimiter: '<S5>/Rate Limiter' incorporates:
     *  Switch: '<Root>/Switch'
     */
    if (realtime_simu_franka_fr3_DW.LastMajorTime == (rtInf)) {
      /* RateLimiter: '<S5>/Rate Limiter' incorporates:
       *  Switch: '<Root>/Switch'
       */
      for (i = 0; i < 7; i++) {
        realtime_simu_franka_fr3_B.RateLimiter[i] =
          realtime_simu_franka_fr3_B.Switch_b[i];
      }
    } else {
      time_end = realtime_simu_franka_fr3_M->Timing.t[0] -
        realtime_simu_franka_fr3_DW.LastMajorTime;
      for (i = 0; i < 7; i++) {
        epsilon = realtime_simu_franka_fr3_B.Switch_b[i] -
          realtime_simu_franka_fr3_DW.PrevY[i];
        s = time_end * realtime_simu_franka_fr3_P.RateLimiter_RisingLim;
        if (epsilon > s) {
          realtime_simu_franka_fr3_B.RateLimiter[i] = s +
            realtime_simu_franka_fr3_DW.PrevY[i];
        } else {
          s = time_end * realtime_simu_franka_fr3_P.RateLimiter_FallingLim;
          if (epsilon < s) {
            realtime_simu_franka_fr3_B.RateLimiter[i] =
              realtime_simu_franka_fr3_DW.PrevY[i] + s;
          } else {
            realtime_simu_franka_fr3_B.RateLimiter[i] =
              realtime_simu_franka_fr3_B.Switch_b[i];
          }
        }
      }
    }

    /* End of RateLimiter: '<S5>/Rate Limiter' */

    /* S-Function (apply_control): '<S5>/Apply Control' */
    {
      /* S-Function Block: <S5>/Apply Control */
      if ((bool)realtime_simu_franka_fr3_DW.ApplyControl_DWORK1) {
        // Wait for the control thread signal
        if ((bool)realtime_simu_franka_fr3_DW.ApplyControl_DWORK2) {
          simulinkPandaRobot_1721602.waitForControlThreadStep();
        }

        // If control loop threw exeption terminate execution
        simulinkPandaRobot_1721602.checkIfAndHandleException();

        // copy inputs
        simulinkPandaRobot_1721602.copyInputSignal
          (&realtime_simu_franka_fr3_B.RateLimiter[0], 0);

        // notify control thread that the inputs have been read
        simulinkPandaRobot_1721602.notifyControlThreadToContinue();
      } else if (!(bool)realtime_simu_franka_fr3_DW.ApplyControl_DWORK1) {
        // Its the first time _step() function is called -->
        // Initialize according to settings parsed from the mask
        // and spawn control thread
        simulinkPandaRobot_1721602.applyRobotSettings();
        simulinkPandaRobot_1721602.spawnControlThread();
        realtime_simu_franka_fr3_DW.ApplyControl_DWORK1 = 1;
      }
    }

    /* SignalConversion generated from: '<S24>/Bus Selector' incorporates:
     *  MATLAB Function: '<S24>/manipulability and collinearity 7DOF'
     */
    realtime_simu_franka_fr3_B.JJ_Y12_B13_R14[0] = JJ_colin[7];
    realtime_simu_franka_fr3_B.JJ_Y12_B13_R14[1] = JJ_colin[14];
    realtime_simu_franka_fr3_B.JJ_Y12_B13_R14[2] = JJ_colin[21];

    /* SignalConversion generated from: '<S24>/Bus Selector' incorporates:
     *  MATLAB Function: '<S24>/manipulability and collinearity 7DOF'
     */
    realtime_simu_franka_fr3_B.JJ_Y15_B16_R17[0] = JJ_colin[28];
    realtime_simu_franka_fr3_B.JJ_Y15_B16_R17[1] = JJ_colin[35];
    realtime_simu_franka_fr3_B.JJ_Y15_B16_R17[2] = JJ_colin[42];

    /* SignalConversion generated from: '<S24>/Bus Selector' incorporates:
     *  MATLAB Function: '<S24>/manipulability and collinearity 7DOF'
     */
    realtime_simu_franka_fr3_B.JJ_Y23_B24_R25[0] = JJ_colin[15];
    realtime_simu_franka_fr3_B.JJ_Y23_B24_R25[1] = JJ_colin[22];
    realtime_simu_franka_fr3_B.JJ_Y23_B24_R25[2] = JJ_colin[29];

    /* SignalConversion generated from: '<S24>/Bus Selector' incorporates:
     *  MATLAB Function: '<S24>/manipulability and collinearity 7DOF'
     */
    realtime_simu_franka_fr3_B.JJ_Y26_B27_R34[0] = JJ_colin[36];
    realtime_simu_franka_fr3_B.JJ_Y26_B27_R34[1] = JJ_colin[43];
    realtime_simu_franka_fr3_B.JJ_Y26_B27_R34[2] = JJ_colin[23];

    /* SignalConversion generated from: '<S24>/Bus Selector' incorporates:
     *  MATLAB Function: '<S24>/manipulability and collinearity 7DOF'
     */
    realtime_simu_franka_fr3_B.JJ_Y35_B36_R37[0] = JJ_colin[30];
    realtime_simu_franka_fr3_B.JJ_Y35_B36_R37[1] = JJ_colin[37];
    realtime_simu_franka_fr3_B.JJ_Y35_B36_R37[2] = JJ_colin[44];

    /* SignalConversion generated from: '<S24>/Bus Selector' incorporates:
     *  MATLAB Function: '<S24>/manipulability and collinearity 7DOF'
     */
    realtime_simu_franka_fr3_B.JJ_Y45_B46_R47[0] = JJ_colin[31];
    realtime_simu_franka_fr3_B.JJ_Y45_B46_R47[1] = JJ_colin[38];
    realtime_simu_franka_fr3_B.JJ_Y45_B46_R47[2] = JJ_colin[45];

    /* SignalConversion generated from: '<S24>/Bus Selector' incorporates:
     *  MATLAB Function: '<S24>/manipulability and collinearity 7DOF'
     */
    realtime_simu_franka_fr3_B.JJ_Y56_B57_R67[0] = JJ_colin[39];
    realtime_simu_franka_fr3_B.JJ_Y56_B57_R67[1] = JJ_colin[46];
    realtime_simu_franka_fr3_B.JJ_Y56_B57_R67[2] = JJ_colin[47];
  }

  /* Matfile logging */
  rt_UpdateTXYLogVars(realtime_simu_franka_fr3_M->rtwLogInfo,
                      (realtime_simu_franka_fr3_M->Timing.t));

  {
    real_T (*lastU)[7];

    /* Update for Memory: '<S26>/filter window' */
    memcpy(&realtime_simu_franka_fr3_DW.filterwindow_PreviousInput[0],
           &realtime_simu_franka_fr3_B.f_data_o[0], 301U * sizeof(real_T));

    /* Update for Derivative: '<Root>/Derivative' */
    if (realtime_simu_franka_fr3_DW.TimeStampA == (rtInf)) {
      realtime_simu_franka_fr3_DW.TimeStampA =
        realtime_simu_franka_fr3_M->Timing.t[0];
      lastU = &realtime_simu_franka_fr3_DW.LastUAtTimeA;
    } else if (realtime_simu_franka_fr3_DW.TimeStampB == (rtInf)) {
      realtime_simu_franka_fr3_DW.TimeStampB =
        realtime_simu_franka_fr3_M->Timing.t[0];
      lastU = &realtime_simu_franka_fr3_DW.LastUAtTimeB;
    } else if (realtime_simu_franka_fr3_DW.TimeStampA <
               realtime_simu_franka_fr3_DW.TimeStampB) {
      realtime_simu_franka_fr3_DW.TimeStampA =
        realtime_simu_franka_fr3_M->Timing.t[0];
      lastU = &realtime_simu_franka_fr3_DW.LastUAtTimeA;
    } else {
      realtime_simu_franka_fr3_DW.TimeStampB =
        realtime_simu_franka_fr3_M->Timing.t[0];
      lastU = &realtime_simu_franka_fr3_DW.LastUAtTimeB;
    }

    for (int32_T i = 0; i < 7; i++) {
      (*lastU)[i] = realtime_simu_franka_fr3_B.GetRobotState2_o2[i];
    }

    /* End of Update for Derivative: '<Root>/Derivative' */

    /* Update for Delay: '<S6>/uk_prev' */
    realtime_simu_franka_fr3_DW.icLoad = false;

    /* Update for Delay: '<S13>/uk_prev' */
    realtime_simu_franka_fr3_DW.icLoad_f = false;
    for (int32_T i = 0; i < 7; i++) {
      /* Update for Delay: '<S6>/uk_prev' incorporates:
       *  Switch: '<Root>/Switch'
       */
      realtime_simu_franka_fr3_DW.uk_prev_DSTATE[i] =
        realtime_simu_franka_fr3_B.Switch_b[i];

      /* Update for Delay: '<S13>/uk_prev' incorporates:
       *  Merge: '<S2>/Merge1'
       */
      realtime_simu_franka_fr3_DW.uk_prev_DSTATE_b[i] =
        realtime_simu_franka_fr3_B.Merge1[i];

      /* Update for RateLimiter: '<S5>/Rate Limiter' */
      realtime_simu_franka_fr3_DW.PrevY[i] =
        realtime_simu_franka_fr3_B.RateLimiter[i];
    }

    /* Update for RateLimiter: '<S5>/Rate Limiter' */
    realtime_simu_franka_fr3_DW.LastMajorTime =
      realtime_simu_franka_fr3_M->Timing.t[0];
  }

  /* External mode */
  rtExtModeUploadCheckTrigger(2);

  {                                    /* Sample time: [0.0s, 0.0s] */
    rtExtModeUpload(0, (real_T)realtime_simu_franka_fr3_M->Timing.t[0]);
  }

  {                                    /* Sample time: [0.001s, 0.0s] */
    rtExtModeUpload(1, (real_T)realtime_simu_franka_fr3_M->Timing.t[1]);
  }

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.0s, 0.0s] */
    if ((rtmGetTFinal(realtime_simu_franka_fr3_M)!=-1) &&
        !((rtmGetTFinal(realtime_simu_franka_fr3_M)-
           realtime_simu_franka_fr3_M->Timing.t[0]) >
          realtime_simu_franka_fr3_M->Timing.t[0] * (DBL_EPSILON))) {
      rtmSetErrorStatus(realtime_simu_franka_fr3_M, "Simulation finished");
    }

    if (rtmGetStopRequested(realtime_simu_franka_fr3_M)) {
      rtmSetErrorStatus(realtime_simu_franka_fr3_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++realtime_simu_franka_fr3_M->Timing.clockTick0)) {
    ++realtime_simu_franka_fr3_M->Timing.clockTickH0;
  }

  realtime_simu_franka_fr3_M->Timing.t[0] =
    realtime_simu_franka_fr3_M->Timing.clockTick0 *
    realtime_simu_franka_fr3_M->Timing.stepSize0 +
    realtime_simu_franka_fr3_M->Timing.clockTickH0 *
    realtime_simu_franka_fr3_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.001s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++realtime_simu_franka_fr3_M->Timing.clockTick1)) {
      ++realtime_simu_franka_fr3_M->Timing.clockTickH1;
    }

    realtime_simu_franka_fr3_M->Timing.t[1] =
      realtime_simu_franka_fr3_M->Timing.clockTick1 *
      realtime_simu_franka_fr3_M->Timing.stepSize1 +
      realtime_simu_franka_fr3_M->Timing.clockTickH1 *
      realtime_simu_franka_fr3_M->Timing.stepSize1 * 4294967296.0;
  }
}

/* Model initialize function */
void realtime_simu_franka_fr3_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&realtime_simu_franka_fr3_M->solverInfo,
                          &realtime_simu_franka_fr3_M->Timing.simTimeStep);
    rtsiSetTPtr(&realtime_simu_franka_fr3_M->solverInfo, &rtmGetTPtr
                (realtime_simu_franka_fr3_M));
    rtsiSetStepSizePtr(&realtime_simu_franka_fr3_M->solverInfo,
                       &realtime_simu_franka_fr3_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&realtime_simu_franka_fr3_M->solverInfo,
                          (&rtmGetErrorStatus(realtime_simu_franka_fr3_M)));
    rtsiSetRTModelPtr(&realtime_simu_franka_fr3_M->solverInfo,
                      realtime_simu_franka_fr3_M);
  }

  rtsiSetSimTimeStep(&realtime_simu_franka_fr3_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&realtime_simu_franka_fr3_M->solverInfo,
    false);
  rtsiSetSolverName(&realtime_simu_franka_fr3_M->solverInfo,"FixedStepDiscrete");
  realtime_simu_franka_fr3_M->solverInfoPtr =
    (&realtime_simu_franka_fr3_M->solverInfo);

  /* Initialize timing info */
  {
    int_T *mdlTsMap = realtime_simu_franka_fr3_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;

    /* polyspace +2 MISRA2012:D4.1 [Justified:Low] "realtime_simu_franka_fr3_M points to
       static memory which is guaranteed to be non-NULL" */
    realtime_simu_franka_fr3_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    realtime_simu_franka_fr3_M->Timing.sampleTimes =
      (&realtime_simu_franka_fr3_M->Timing.sampleTimesArray[0]);
    realtime_simu_franka_fr3_M->Timing.offsetTimes =
      (&realtime_simu_franka_fr3_M->Timing.offsetTimesArray[0]);

    /* task periods */
    realtime_simu_franka_fr3_M->Timing.sampleTimes[0] = (0.0);
    realtime_simu_franka_fr3_M->Timing.sampleTimes[1] = (0.001);

    /* task offsets */
    realtime_simu_franka_fr3_M->Timing.offsetTimes[0] = (0.0);
    realtime_simu_franka_fr3_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(realtime_simu_franka_fr3_M,
             &realtime_simu_franka_fr3_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = realtime_simu_franka_fr3_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    realtime_simu_franka_fr3_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(realtime_simu_franka_fr3_M, -1);
  realtime_simu_franka_fr3_M->Timing.stepSize0 = 0.001;
  realtime_simu_franka_fr3_M->Timing.stepSize1 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    realtime_simu_franka_fr3_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(realtime_simu_franka_fr3_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(realtime_simu_franka_fr3_M->rtwLogInfo, (NULL));
    rtliSetLogT(realtime_simu_franka_fr3_M->rtwLogInfo, "tout");
    rtliSetLogX(realtime_simu_franka_fr3_M->rtwLogInfo, "");
    rtliSetLogXFinal(realtime_simu_franka_fr3_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(realtime_simu_franka_fr3_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(realtime_simu_franka_fr3_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(realtime_simu_franka_fr3_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(realtime_simu_franka_fr3_M->rtwLogInfo, 1);
    rtliSetLogY(realtime_simu_franka_fr3_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(realtime_simu_franka_fr3_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(realtime_simu_franka_fr3_M->rtwLogInfo, (NULL));
  }

  /* External mode info */
  realtime_simu_franka_fr3_M->Sizes.checksums[0] = (4001283256U);
  realtime_simu_franka_fr3_M->Sizes.checksums[1] = (1114183430U);
  realtime_simu_franka_fr3_M->Sizes.checksums[2] = (484782768U);
  realtime_simu_franka_fr3_M->Sizes.checksums[3] = (743211365U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[43];
    realtime_simu_franka_fr3_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.CTControllerSubsystem_SubsysRan;
    systemRan[3] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.CTControllerSubsystem_SubsysRan;
    systemRan[4] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.CTControllerSubsystem_SubsysRan;
    systemRan[5] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.CTControllerSubsystem_SubsysRan;
    systemRan[6] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.PDControllerSubsystem_SubsysRan;
    systemRan[7] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.PDControllerSubsystem_SubsysRan;
    systemRan[8] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.PDControllerSubsystem_SubsysRan;
    systemRan[9] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.PDControllerSubsystem_SubsysRan;
    systemRan[10] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.EKF_SubsysRanBC_p;
    systemRan[11] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.EKF_SubsysRanBC_p;
    systemRan[12] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.EKF_SubsysRanBC_p;
    systemRan[13] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.EKF_SubsysRanBC_p;
    systemRan[14] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.EKF_SubsysRanBC_p;
    systemRan[15] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.noEKF.noEKF_SubsysRanBC;
    systemRan[16] = &rtAlwaysEnabled;
    systemRan[17] = &rtAlwaysEnabled;
    systemRan[18] = &rtAlwaysEnabled;
    systemRan[19] = &rtAlwaysEnabled;
    systemRan[20] = &rtAlwaysEnabled;
    systemRan[21] = &rtAlwaysEnabled;
    systemRan[22] = (sysRanDType *)&realtime_simu_franka_fr3_DW.EKF_SubsysRanBC;
    systemRan[23] = (sysRanDType *)&realtime_simu_franka_fr3_DW.EKF_SubsysRanBC;
    systemRan[24] = (sysRanDType *)&realtime_simu_franka_fr3_DW.EKF_SubsysRanBC;
    systemRan[25] = (sysRanDType *)&realtime_simu_franka_fr3_DW.EKF_SubsysRanBC;
    systemRan[26] = (sysRanDType *)&realtime_simu_franka_fr3_DW.EKF_SubsysRanBC;
    systemRan[27] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.noEKF_b.noEKF_SubsysRanBC;
    systemRan[28] = &rtAlwaysEnabled;
    systemRan[29] = &rtAlwaysEnabled;
    systemRan[30] = &rtAlwaysEnabled;
    systemRan[31] = &rtAlwaysEnabled;
    systemRan[32] = &rtAlwaysEnabled;
    systemRan[33] = &rtAlwaysEnabled;
    systemRan[34] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.jointspacectlsubsys_SubsysRanBC;
    systemRan[35] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.jointspacectlsubsys_SubsysRanBC;
    systemRan[36] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.jointspacectlsubsys_SubsysRanBC;
    systemRan[37] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.jointspacectlsubsys_SubsysRanBC;
    systemRan[38] = &rtAlwaysEnabled;
    systemRan[39] = &rtAlwaysEnabled;
    systemRan[40] = &rtAlwaysEnabled;
    systemRan[41] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.tau_subsystem_SubsysRanBC;
    systemRan[42] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.tau_subsystem_SubsysRanBC;
    rteiSetModelMappingInfoPtr(realtime_simu_franka_fr3_M->extModeInfo,
      &realtime_simu_franka_fr3_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(realtime_simu_franka_fr3_M->extModeInfo,
                        realtime_simu_franka_fr3_M->Sizes.checksums);
    rteiSetTPtr(realtime_simu_franka_fr3_M->extModeInfo, rtmGetTPtr
                (realtime_simu_franka_fr3_M));
  }

  realtime_simu_franka_fr3_M->solverInfoPtr =
    (&realtime_simu_franka_fr3_M->solverInfo);
  realtime_simu_franka_fr3_M->Timing.stepSize = (0.001);
  rtsiSetFixedStepSize(&realtime_simu_franka_fr3_M->solverInfo, 0.001);
  rtsiSetSolverMode(&realtime_simu_franka_fr3_M->solverInfo,
                    SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  (void) memset((static_cast<void *>(&realtime_simu_franka_fr3_B)), 0,
                sizeof(B_realtime_simu_franka_fr3_T));

  /* states (dwork) */
  (void) memset(static_cast<void *>(&realtime_simu_franka_fr3_DW), 0,
                sizeof(DW_realtime_simu_franka_fr3_T));

  /* block instance data */
  {
    {
      simulinkPandaRobot_1721602 = SimulinkPandaRobot( "172.16.0.2",
        0,
        0,
        0,
        realtime_simu_franka_fr3_P.ApplyControl_P1,
        realtime_simu_franka_fr3_P.ApplyControl_P2,
        realtime_simu_franka_fr3_P.ApplyControl_P3,
        realtime_simu_franka_fr3_P.ApplyControl_P4,
        realtime_simu_franka_fr3_P.ApplyControl_P5,
        1,
        realtime_simu_franka_fr3_P.ApplyControl_P6,
        1);
    }
  }

  /* child S-Function registration */
  {
    RTWSfcnInfo *sfcnInfo =
      &realtime_simu_franka_fr3_M->NonInlinedSFcns.sfcnInfo;
    realtime_simu_franka_fr3_M->sfcnInfo = (sfcnInfo);
    rtssSetErrorStatusPtr(sfcnInfo, (&rtmGetErrorStatus
      (realtime_simu_franka_fr3_M)));
    realtime_simu_franka_fr3_M->Sizes.numSampTimes = (2);
    rtssSetNumRootSampTimesPtr(sfcnInfo,
      &realtime_simu_franka_fr3_M->Sizes.numSampTimes);
    realtime_simu_franka_fr3_M->NonInlinedSFcns.taskTimePtrs[0] = &(rtmGetTPtr
      (realtime_simu_franka_fr3_M)[0]);
    realtime_simu_franka_fr3_M->NonInlinedSFcns.taskTimePtrs[1] = &(rtmGetTPtr
      (realtime_simu_franka_fr3_M)[1]);
    rtssSetTPtrPtr(sfcnInfo,
                   realtime_simu_franka_fr3_M->NonInlinedSFcns.taskTimePtrs);
    rtssSetTStartPtr(sfcnInfo, &rtmGetTStart(realtime_simu_franka_fr3_M));
    rtssSetTFinalPtr(sfcnInfo, &rtmGetTFinal(realtime_simu_franka_fr3_M));
    rtssSetTimeOfLastOutputPtr(sfcnInfo, &rtmGetTimeOfLastOutput
      (realtime_simu_franka_fr3_M));
    rtssSetStepSizePtr(sfcnInfo, &realtime_simu_franka_fr3_M->Timing.stepSize);
    rtssSetStopRequestedPtr(sfcnInfo, &rtmGetStopRequested
      (realtime_simu_franka_fr3_M));
    rtssSetDerivCacheNeedsResetPtr(sfcnInfo,
      &realtime_simu_franka_fr3_M->derivCacheNeedsReset);
    rtssSetZCCacheNeedsResetPtr(sfcnInfo,
      &realtime_simu_franka_fr3_M->zCCacheNeedsReset);
    rtssSetContTimeOutputInconsistentWithStateAtMajorStepPtr(sfcnInfo,
      &realtime_simu_franka_fr3_M->CTOutputIncnstWithState);
    rtssSetSampleHitsPtr(sfcnInfo,
                         &realtime_simu_franka_fr3_M->Timing.sampleHits);
    rtssSetPerTaskSampleHitsPtr(sfcnInfo,
      &realtime_simu_franka_fr3_M->Timing.perTaskSampleHits);
    rtssSetSimModePtr(sfcnInfo, &realtime_simu_franka_fr3_M->simMode);
    rtssSetSolverInfoPtr(sfcnInfo, &realtime_simu_franka_fr3_M->solverInfoPtr);
  }

  realtime_simu_franka_fr3_M->Sizes.numSFcns = (10);

  /* register each child */
  {
    (void) memset(static_cast<void *>
                  (&realtime_simu_franka_fr3_M->NonInlinedSFcns.childSFunctions
                   [0]), 0,
                  10*sizeof(SimStruct));
    realtime_simu_franka_fr3_M->childSfunctions =
      (&realtime_simu_franka_fr3_M->NonInlinedSFcns.childSFunctionPtrs[0]);

    {
      int_T i;
      for (i = 0; i < 10; i++) {
        realtime_simu_franka_fr3_M->childSfunctions[i] =
          (&realtime_simu_franka_fr3_M->NonInlinedSFcns.childSFunctions[i]);
      }
    }

    /* Level2 S-Function Block: realtime_simu_franka_fr3/<S16>/Reduced System sfun casadi solve (s_function_opti_ekf_fun) */
    {
      SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[0];

      /* timing info */
      time_T *sfcnPeriod =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.sfcnPeriod;
      time_T *sfcnOffset =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.sfcnOffset;
      int_T *sfcnTsMap =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.sfcnTsMap;
      (void) memset(static_cast<void*>(sfcnPeriod), 0,
                    sizeof(time_T)*1);
      (void) memset(static_cast<void*>(sfcnOffset), 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.blkInfo2[0]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &realtime_simu_franka_fr3_M->NonInlinedSFcns.inputOutputPortInfo2[0]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, realtime_simu_franka_fr3_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods2[0]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods3[0]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods4[0]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.statesInfo2
                         [0]);
        ssSetPeriodicStatesInfo(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.periodicStatesInfo[0]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 6);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.inputPortInfo[0]);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.inputPortInfo[0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        ssSetInputPortUnit(rts, 1, 0);
        ssSetInputPortUnit(rts, 2, 0);
        ssSetInputPortUnit(rts, 3, 0);
        ssSetInputPortUnit(rts, 4, 0);
        ssSetInputPortUnit(rts, 5, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);
        ssSetInputPortIsContinuousQuantity(rts, 1, 0);
        ssSetInputPortIsContinuousQuantity(rts, 2, 0);
        ssSetInputPortIsContinuousQuantity(rts, 3, 0);
        ssSetInputPortIsContinuousQuantity(rts, 4, 0);
        ssSetInputPortIsContinuousQuantity(rts, 5, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.iDims0;
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0, realtime_simu_franka_fr3_B.uk_prev_e);
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetInputPortNumDimensions(rts, 0, 2);
          ssSetInputPortWidthAsInt(rts, 0, 7);
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.iDims1;
          ssSetInputPortRequiredContiguous(rts, 1, 1);
          ssSetInputPortSignal(rts, 1,
                               realtime_simu_franka_fr3_B.y_kxk_measured_n);
          dimensions[0] = 14;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetInputPortNumDimensions(rts, 1, 2);
          ssSetInputPortWidthAsInt(rts, 1, 14);
        }

        /* port 2 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.iDims2;
          ssSetInputPortRequiredContiguous(rts, 2, 1);
          ssSetInputPortSignal(rts, 2, realtime_simu_franka_fr3_B.Constant_p);
          dimensions[0] = 14;
          dimensions[1] = 14;
          _ssSetInputPortDimensionsPtrAsInt(rts, 2, dimensions);
          _ssSetInputPortNumDimensions(rts, 2, 2);
          ssSetInputPortWidthAsInt(rts, 2, 196);
        }

        /* port 3 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.iDims3;
          ssSetInputPortRequiredContiguous(rts, 3, 1);
          ssSetInputPortSignal(rts, 3, realtime_simu_franka_fr3_B.Constant1_a);
          dimensions[0] = 14;
          dimensions[1] = 14;
          _ssSetInputPortDimensionsPtrAsInt(rts, 3, dimensions);
          _ssSetInputPortNumDimensions(rts, 3, 2);
          ssSetInputPortWidthAsInt(rts, 3, 196);
        }

        /* port 4 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.iDims4;
          ssSetInputPortRequiredContiguous(rts, 4, 1);
          ssSetInputPortSignal(rts, 4, realtime_simu_franka_fr3_B.xk_minus_d);
          dimensions[0] = 14;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 4, dimensions);
          _ssSetInputPortNumDimensions(rts, 4, 2);
          ssSetInputPortWidthAsInt(rts, 4, 14);
        }

        /* port 5 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.iDims5;
          ssSetInputPortRequiredContiguous(rts, 5, 1);
          ssSetInputPortSignal(rts, 5, realtime_simu_franka_fr3_B.Pk_minus_d);
          dimensions[0] = 14;
          dimensions[1] = 14;
          _ssSetInputPortDimensionsPtrAsInt(rts, 5, dimensions);
          _ssSetInputPortNumDimensions(rts, 5, 2);
          ssSetInputPortWidthAsInt(rts, 5, 196);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.outputPortInfo[0]);
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 3);
        _ssSetPortInfo2ForOutputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        ssSetOutputPortUnit(rts, 2, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 1, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 2, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.oDims0;
          dimensions[0] = 14;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetOutputPortNumDimensions(rts, 0, 2);
          ssSetOutputPortWidthAsInt(rts, 0, 14);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            realtime_simu_franka_fr3_B.xk_plus_j));
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.oDims1;
          dimensions[0] = 14;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetOutputPortNumDimensions(rts, 1, 2);
          ssSetOutputPortWidthAsInt(rts, 1, 14);
          ssSetOutputPortSignal(rts, 1, ((real_T *)
            realtime_simu_franka_fr3_B.xkp1_minus_a));
        }

        /* port 2 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.oDims2;
          dimensions[0] = 14;
          dimensions[1] = 14;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 2, dimensions);
          _ssSetOutputPortNumDimensions(rts, 2, 2);
          ssSetOutputPortWidthAsInt(rts, 2, 196);
          ssSetOutputPortSignal(rts, 2, ((real_T *)
            realtime_simu_franka_fr3_B.Pkp1_minus_k));
        }
      }

      /* path info */
      ssSetModelName(rts, "Reduced System sfun casadi solve");
      ssSetPath(rts,
                "realtime_simu_franka_fr3/Controller Subsystem/Subsystem/EKF/Reduced System sfun casadi solve");
      ssSetRTModel(rts,realtime_simu_franka_fr3_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetPWork(rts, (void **)
                 &realtime_simu_franka_fr3_DW.ReducedSystemsfuncasadisolve__f[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        ssSetNumDWorkAsInt(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 9);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &realtime_simu_franka_fr3_DW.ReducedSystemsfuncasadisolve__f
                   [0]);
      }

      /* registration */
      s_function_opti_ekf_fun(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetInputPortConnected(rts, 2, 1);
      _ssSetInputPortConnected(rts, 3, 1);
      _ssSetInputPortConnected(rts, 4, 1);
      _ssSetInputPortConnected(rts, 5, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 2, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);
      _ssSetOutputPortBeingMerged(rts, 2, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
      ssSetInputPortBufferDstPort(rts, 2, -1);
      ssSetInputPortBufferDstPort(rts, 3, -1);
      ssSetInputPortBufferDstPort(rts, 4, -1);
      ssSetInputPortBufferDstPort(rts, 5, -1);
    }

    /* Level2 S-Function Block: realtime_simu_franka_fr3/<S19>/robot model s-function2 (s_function_opti_robot_model_bus_fun) */
    {
      SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[1];

      /* timing info */
      time_T *sfcnPeriod =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.sfcnPeriod;
      time_T *sfcnOffset =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.sfcnOffset;
      int_T *sfcnTsMap =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.sfcnTsMap;
      (void) memset(static_cast<void*>(sfcnPeriod), 0,
                    sizeof(time_T)*1);
      (void) memset(static_cast<void*>(sfcnOffset), 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.blkInfo2[1]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &realtime_simu_franka_fr3_M->NonInlinedSFcns.inputOutputPortInfo2[1]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, realtime_simu_franka_fr3_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods2[1]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods3[1]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods4[1]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.statesInfo2
                         [1]);
        ssSetPeriodicStatesInfo(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.periodicStatesInfo[1]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 2);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.inputPortInfo[0]);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.inputPortInfo[0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        ssSetInputPortUnit(rts, 1, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);
        ssSetInputPortIsContinuousQuantity(rts, 1, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.iDims0;
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0,
                               realtime_simu_franka_fr3_B.sf_getEKFjointvalues.q);
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetInputPortNumDimensions(rts, 0, 2);
          ssSetInputPortWidthAsInt(rts, 0, 7);
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.iDims1;
          ssSetInputPortRequiredContiguous(rts, 1, 1);
          ssSetInputPortSignal(rts, 1,
                               realtime_simu_franka_fr3_B.sf_getEKFjointvalues.q_p);
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetInputPortNumDimensions(rts, 1, 2);
          ssSetInputPortWidthAsInt(rts, 1, 7);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.outputPortInfo[0]);
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 7);
        _ssSetPortInfo2ForOutputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        ssSetOutputPortUnit(rts, 2, 0);
        ssSetOutputPortUnit(rts, 3, 0);
        ssSetOutputPortUnit(rts, 4, 0);
        ssSetOutputPortUnit(rts, 5, 0);
        ssSetOutputPortUnit(rts, 6, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 1, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 2, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 3, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 4, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 5, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 6, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.oDims0;
          dimensions[0] = 4;
          dimensions[1] = 4;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetOutputPortNumDimensions(rts, 0, 2);
          ssSetOutputPortWidthAsInt(rts, 0, 16);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o1_b));
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.oDims1;
          dimensions[0] = 6;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetOutputPortNumDimensions(rts, 1, 2);
          ssSetOutputPortWidthAsInt(rts, 1, 42);
          ssSetOutputPortSignal(rts, 1, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o2_h));
        }

        /* port 2 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.oDims2;
          dimensions[0] = 6;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 2, dimensions);
          _ssSetOutputPortNumDimensions(rts, 2, 2);
          ssSetOutputPortWidthAsInt(rts, 2, 42);
          ssSetOutputPortSignal(rts, 2, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o3_o));
        }

        /* port 3 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.oDims3;
          dimensions[0] = 7;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 3, dimensions);
          _ssSetOutputPortNumDimensions(rts, 3, 2);
          ssSetOutputPortWidthAsInt(rts, 3, 49);
          ssSetOutputPortSignal(rts, 3, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o4_d));
        }

        /* port 4 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.oDims4;
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 4, dimensions);
          _ssSetOutputPortNumDimensions(rts, 4, 2);
          ssSetOutputPortWidthAsInt(rts, 4, 7);
          ssSetOutputPortSignal(rts, 4, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o5_k));
        }

        /* port 5 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.oDims5;
          dimensions[0] = 7;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 5, dimensions);
          _ssSetOutputPortNumDimensions(rts, 5, 2);
          ssSetOutputPortWidthAsInt(rts, 5, 49);
          ssSetOutputPortSignal(rts, 5, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o6_b));
        }

        /* port 6 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.oDims6;
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 6, dimensions);
          _ssSetOutputPortNumDimensions(rts, 6, 2);
          ssSetOutputPortWidthAsInt(rts, 6, 7);
          ssSetOutputPortSignal(rts, 6, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o7_l));
        }
      }

      /* path info */
      ssSetModelName(rts, "robot model s-function2");
      ssSetPath(rts,
                "realtime_simu_franka_fr3/Controller Subsystem/Subsystem/EKF/robot_model_bus_subsys/robot model s-function2");
      ssSetRTModel(rts,realtime_simu_franka_fr3_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetPWork(rts, (void **)
                 &realtime_simu_franka_fr3_DW.robotmodelsfunction2_PWORK_a[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn1.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        ssSetNumDWorkAsInt(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 9);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &realtime_simu_franka_fr3_DW.robotmodelsfunction2_PWORK_a[0]);
      }

      /* registration */
      s_function_opti_robot_model_bus_fun(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 2, 1);
      _ssSetOutputPortConnected(rts, 3, 1);
      _ssSetOutputPortConnected(rts, 4, 1);
      _ssSetOutputPortConnected(rts, 5, 1);
      _ssSetOutputPortConnected(rts, 6, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);
      _ssSetOutputPortBeingMerged(rts, 2, 0);
      _ssSetOutputPortBeingMerged(rts, 3, 0);
      _ssSetOutputPortBeingMerged(rts, 4, 0);
      _ssSetOutputPortBeingMerged(rts, 5, 0);
      _ssSetOutputPortBeingMerged(rts, 6, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
    }

    /* Level2 S-Function Block: realtime_simu_franka_fr3/<S19>/robot model s-function1 (s_function_opti_robot_model_bus_fun) */
    {
      SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[2];

      /* timing info */
      time_T *sfcnPeriod =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.sfcnPeriod;
      time_T *sfcnOffset =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.sfcnOffset;
      int_T *sfcnTsMap =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.sfcnTsMap;
      (void) memset(static_cast<void*>(sfcnPeriod), 0,
                    sizeof(time_T)*1);
      (void) memset(static_cast<void*>(sfcnOffset), 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.blkInfo2[2]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &realtime_simu_franka_fr3_M->NonInlinedSFcns.inputOutputPortInfo2[2]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, realtime_simu_franka_fr3_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods2[2]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods3[2]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods4[2]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.statesInfo2
                         [2]);
        ssSetPeriodicStatesInfo(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.periodicStatesInfo[2]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 2);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.inputPortInfo[0]);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.inputPortInfo[0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        ssSetInputPortUnit(rts, 1, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);
        ssSetInputPortIsContinuousQuantity(rts, 1, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.iDims0;
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0,
                               realtime_simu_franka_fr3_B.sf_zerofixedstates_a.q_red);
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetInputPortNumDimensions(rts, 0, 2);
          ssSetInputPortWidthAsInt(rts, 0, 7);
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.iDims1;
          ssSetInputPortRequiredContiguous(rts, 1, 1);
          ssSetInputPortSignal(rts, 1,
                               realtime_simu_franka_fr3_B.sf_zerofixedstates_a.q_p_red);
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetInputPortNumDimensions(rts, 1, 2);
          ssSetInputPortWidthAsInt(rts, 1, 7);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.outputPortInfo[0]);
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 7);
        _ssSetPortInfo2ForOutputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        ssSetOutputPortUnit(rts, 2, 0);
        ssSetOutputPortUnit(rts, 3, 0);
        ssSetOutputPortUnit(rts, 4, 0);
        ssSetOutputPortUnit(rts, 5, 0);
        ssSetOutputPortUnit(rts, 6, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 1, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 2, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 3, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 4, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 5, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 6, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.oDims0;
          dimensions[0] = 4;
          dimensions[1] = 4;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetOutputPortNumDimensions(rts, 0, 2);
          ssSetOutputPortWidthAsInt(rts, 0, 16);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o1_i));
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.oDims1;
          dimensions[0] = 6;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetOutputPortNumDimensions(rts, 1, 2);
          ssSetOutputPortWidthAsInt(rts, 1, 42);
          ssSetOutputPortSignal(rts, 1, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o2_a));
        }

        /* port 2 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.oDims2;
          dimensions[0] = 6;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 2, dimensions);
          _ssSetOutputPortNumDimensions(rts, 2, 2);
          ssSetOutputPortWidthAsInt(rts, 2, 42);
          ssSetOutputPortSignal(rts, 2, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o3_p));
        }

        /* port 3 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.oDims3;
          dimensions[0] = 7;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 3, dimensions);
          _ssSetOutputPortNumDimensions(rts, 3, 2);
          ssSetOutputPortWidthAsInt(rts, 3, 49);
          ssSetOutputPortSignal(rts, 3, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o4_a));
        }

        /* port 4 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.oDims4;
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 4, dimensions);
          _ssSetOutputPortNumDimensions(rts, 4, 2);
          ssSetOutputPortWidthAsInt(rts, 4, 7);
          ssSetOutputPortSignal(rts, 4, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o5_o));
        }

        /* port 5 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.oDims5;
          dimensions[0] = 7;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 5, dimensions);
          _ssSetOutputPortNumDimensions(rts, 5, 2);
          ssSetOutputPortWidthAsInt(rts, 5, 49);
          ssSetOutputPortSignal(rts, 5, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o6_h));
        }

        /* port 6 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.oDims6;
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 6, dimensions);
          _ssSetOutputPortNumDimensions(rts, 6, 2);
          ssSetOutputPortWidthAsInt(rts, 6, 7);
          ssSetOutputPortSignal(rts, 6, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o7_md));
        }
      }

      /* path info */
      ssSetModelName(rts, "robot model s-function1");
      ssSetPath(rts,
                "realtime_simu_franka_fr3/Controller Subsystem/Subsystem/EKF/robot_model_bus_subsys/robot model s-function1");
      ssSetRTModel(rts,realtime_simu_franka_fr3_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetPWork(rts, (void **)
                 &realtime_simu_franka_fr3_DW.robotmodelsfunction1_PWORK_l[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        ssSetNumDWorkAsInt(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 9);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &realtime_simu_franka_fr3_DW.robotmodelsfunction1_PWORK_l[0]);
      }

      /* registration */
      s_function_opti_robot_model_bus_fun(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 2, 1);
      _ssSetOutputPortConnected(rts, 3, 1);
      _ssSetOutputPortConnected(rts, 4, 1);
      _ssSetOutputPortConnected(rts, 5, 1);
      _ssSetOutputPortConnected(rts, 6, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);
      _ssSetOutputPortBeingMerged(rts, 2, 0);
      _ssSetOutputPortBeingMerged(rts, 3, 0);
      _ssSetOutputPortBeingMerged(rts, 4, 0);
      _ssSetOutputPortBeingMerged(rts, 5, 0);
      _ssSetOutputPortBeingMerged(rts, 6, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
    }

    /* Level2 S-Function Block: realtime_simu_franka_fr3/<S29>/Reduced System sfun casadi solve (s_function_opti_ekf_fun) */
    {
      SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[3];

      /* timing info */
      time_T *sfcnPeriod =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.sfcnPeriod;
      time_T *sfcnOffset =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.sfcnOffset;
      int_T *sfcnTsMap =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.sfcnTsMap;
      (void) memset(static_cast<void*>(sfcnPeriod), 0,
                    sizeof(time_T)*1);
      (void) memset(static_cast<void*>(sfcnOffset), 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.blkInfo2[3]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &realtime_simu_franka_fr3_M->NonInlinedSFcns.inputOutputPortInfo2[3]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, realtime_simu_franka_fr3_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods2[3]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods3[3]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods4[3]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.statesInfo2
                         [3]);
        ssSetPeriodicStatesInfo(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.periodicStatesInfo[3]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 6);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.inputPortInfo[0]);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.inputPortInfo[0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        ssSetInputPortUnit(rts, 1, 0);
        ssSetInputPortUnit(rts, 2, 0);
        ssSetInputPortUnit(rts, 3, 0);
        ssSetInputPortUnit(rts, 4, 0);
        ssSetInputPortUnit(rts, 5, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);
        ssSetInputPortIsContinuousQuantity(rts, 1, 0);
        ssSetInputPortIsContinuousQuantity(rts, 2, 0);
        ssSetInputPortIsContinuousQuantity(rts, 3, 0);
        ssSetInputPortIsContinuousQuantity(rts, 4, 0);
        ssSetInputPortIsContinuousQuantity(rts, 5, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.iDims0;
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0, realtime_simu_franka_fr3_B.uk_prev);
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetInputPortNumDimensions(rts, 0, 2);
          ssSetInputPortWidthAsInt(rts, 0, 7);
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.iDims1;
          ssSetInputPortRequiredContiguous(rts, 1, 1);
          ssSetInputPortSignal(rts, 1, realtime_simu_franka_fr3_B.y_kxk_measured);
          dimensions[0] = 14;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetInputPortNumDimensions(rts, 1, 2);
          ssSetInputPortWidthAsInt(rts, 1, 14);
        }

        /* port 2 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.iDims2;
          ssSetInputPortRequiredContiguous(rts, 2, 1);
          ssSetInputPortSignal(rts, 2, realtime_simu_franka_fr3_B.Constant);
          dimensions[0] = 14;
          dimensions[1] = 14;
          _ssSetInputPortDimensionsPtrAsInt(rts, 2, dimensions);
          _ssSetInputPortNumDimensions(rts, 2, 2);
          ssSetInputPortWidthAsInt(rts, 2, 196);
        }

        /* port 3 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.iDims3;
          ssSetInputPortRequiredContiguous(rts, 3, 1);
          ssSetInputPortSignal(rts, 3, realtime_simu_franka_fr3_B.Constant1);
          dimensions[0] = 14;
          dimensions[1] = 14;
          _ssSetInputPortDimensionsPtrAsInt(rts, 3, dimensions);
          _ssSetInputPortNumDimensions(rts, 3, 2);
          ssSetInputPortWidthAsInt(rts, 3, 196);
        }

        /* port 4 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.iDims4;
          ssSetInputPortRequiredContiguous(rts, 4, 1);
          ssSetInputPortSignal(rts, 4, realtime_simu_franka_fr3_B.xk_minus);
          dimensions[0] = 14;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 4, dimensions);
          _ssSetInputPortNumDimensions(rts, 4, 2);
          ssSetInputPortWidthAsInt(rts, 4, 14);
        }

        /* port 5 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.iDims5;
          ssSetInputPortRequiredContiguous(rts, 5, 1);
          ssSetInputPortSignal(rts, 5, realtime_simu_franka_fr3_B.Pk_minus);
          dimensions[0] = 14;
          dimensions[1] = 14;
          _ssSetInputPortDimensionsPtrAsInt(rts, 5, dimensions);
          _ssSetInputPortNumDimensions(rts, 5, 2);
          ssSetInputPortWidthAsInt(rts, 5, 196);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.outputPortInfo[0]);
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 3);
        _ssSetPortInfo2ForOutputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        ssSetOutputPortUnit(rts, 2, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 1, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 2, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.oDims0;
          dimensions[0] = 14;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetOutputPortNumDimensions(rts, 0, 2);
          ssSetOutputPortWidthAsInt(rts, 0, 14);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            realtime_simu_franka_fr3_B.xk_plus));
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.oDims1;
          dimensions[0] = 14;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetOutputPortNumDimensions(rts, 1, 2);
          ssSetOutputPortWidthAsInt(rts, 1, 14);
          ssSetOutputPortSignal(rts, 1, ((real_T *)
            realtime_simu_franka_fr3_B.xkp1_minus));
        }

        /* port 2 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.oDims2;
          dimensions[0] = 14;
          dimensions[1] = 14;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 2, dimensions);
          _ssSetOutputPortNumDimensions(rts, 2, 2);
          ssSetOutputPortWidthAsInt(rts, 2, 196);
          ssSetOutputPortSignal(rts, 2, ((real_T *)
            realtime_simu_franka_fr3_B.Pkp1_minus));
        }
      }

      /* path info */
      ssSetModelName(rts, "Reduced System sfun casadi solve");
      ssSetPath(rts,
                "realtime_simu_franka_fr3/Subsystem1/EKF/Reduced System sfun casadi solve");
      ssSetRTModel(rts,realtime_simu_franka_fr3_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetPWork(rts, (void **)
                 &realtime_simu_franka_fr3_DW.ReducedSystemsfuncasadisolve_PW[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        ssSetNumDWorkAsInt(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 9);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &realtime_simu_franka_fr3_DW.ReducedSystemsfuncasadisolve_PW
                   [0]);
      }

      /* registration */
      s_function_opti_ekf_fun(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetInputPortConnected(rts, 2, 1);
      _ssSetInputPortConnected(rts, 3, 1);
      _ssSetInputPortConnected(rts, 4, 1);
      _ssSetInputPortConnected(rts, 5, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 2, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);
      _ssSetOutputPortBeingMerged(rts, 2, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
      ssSetInputPortBufferDstPort(rts, 2, -1);
      ssSetInputPortBufferDstPort(rts, 3, -1);
      ssSetInputPortBufferDstPort(rts, 4, -1);
      ssSetInputPortBufferDstPort(rts, 5, -1);
    }

    /* Level2 S-Function Block: realtime_simu_franka_fr3/<S32>/robot model s-function2 (s_function_opti_robot_model_bus_fun) */
    {
      SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[4];

      /* timing info */
      time_T *sfcnPeriod =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.sfcnPeriod;
      time_T *sfcnOffset =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.sfcnOffset;
      int_T *sfcnTsMap =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.sfcnTsMap;
      (void) memset(static_cast<void*>(sfcnPeriod), 0,
                    sizeof(time_T)*1);
      (void) memset(static_cast<void*>(sfcnOffset), 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.blkInfo2[4]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &realtime_simu_franka_fr3_M->NonInlinedSFcns.inputOutputPortInfo2[4]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, realtime_simu_franka_fr3_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods2[4]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods3[4]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods4[4]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.statesInfo2
                         [4]);
        ssSetPeriodicStatesInfo(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.periodicStatesInfo[4]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 2);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.inputPortInfo[0]);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.inputPortInfo[0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        ssSetInputPortUnit(rts, 1, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);
        ssSetInputPortIsContinuousQuantity(rts, 1, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.iDims0;
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0,
                               realtime_simu_franka_fr3_B.sf_getEKFjointvalues_g.q);
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetInputPortNumDimensions(rts, 0, 2);
          ssSetInputPortWidthAsInt(rts, 0, 7);
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.iDims1;
          ssSetInputPortRequiredContiguous(rts, 1, 1);
          ssSetInputPortSignal(rts, 1,
                               realtime_simu_franka_fr3_B.sf_getEKFjointvalues_g.q_p);
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetInputPortNumDimensions(rts, 1, 2);
          ssSetInputPortWidthAsInt(rts, 1, 7);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.outputPortInfo[0]);
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 7);
        _ssSetPortInfo2ForOutputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        ssSetOutputPortUnit(rts, 2, 0);
        ssSetOutputPortUnit(rts, 3, 0);
        ssSetOutputPortUnit(rts, 4, 0);
        ssSetOutputPortUnit(rts, 5, 0);
        ssSetOutputPortUnit(rts, 6, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 1, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 2, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 3, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 4, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 5, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 6, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.oDims0;
          dimensions[0] = 4;
          dimensions[1] = 4;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetOutputPortNumDimensions(rts, 0, 2);
          ssSetOutputPortWidthAsInt(rts, 0, 16);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o1_o));
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.oDims1;
          dimensions[0] = 6;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetOutputPortNumDimensions(rts, 1, 2);
          ssSetOutputPortWidthAsInt(rts, 1, 42);
          ssSetOutputPortSignal(rts, 1, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o2_c));
        }

        /* port 2 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.oDims2;
          dimensions[0] = 6;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 2, dimensions);
          _ssSetOutputPortNumDimensions(rts, 2, 2);
          ssSetOutputPortWidthAsInt(rts, 2, 42);
          ssSetOutputPortSignal(rts, 2, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o3_p));
        }

        /* port 3 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.oDims3;
          dimensions[0] = 7;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 3, dimensions);
          _ssSetOutputPortNumDimensions(rts, 3, 2);
          ssSetOutputPortWidthAsInt(rts, 3, 49);
          ssSetOutputPortSignal(rts, 3, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o4_m));
        }

        /* port 4 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.oDims4;
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 4, dimensions);
          _ssSetOutputPortNumDimensions(rts, 4, 2);
          ssSetOutputPortWidthAsInt(rts, 4, 7);
          ssSetOutputPortSignal(rts, 4, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o5_n));
        }

        /* port 5 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.oDims5;
          dimensions[0] = 7;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 5, dimensions);
          _ssSetOutputPortNumDimensions(rts, 5, 2);
          ssSetOutputPortWidthAsInt(rts, 5, 49);
          ssSetOutputPortSignal(rts, 5, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o6_g));
        }

        /* port 6 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.oDims6;
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 6, dimensions);
          _ssSetOutputPortNumDimensions(rts, 6, 2);
          ssSetOutputPortWidthAsInt(rts, 6, 7);
          ssSetOutputPortSignal(rts, 6, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o7_e));
        }
      }

      /* path info */
      ssSetModelName(rts, "robot model s-function2");
      ssSetPath(rts,
                "realtime_simu_franka_fr3/Subsystem1/EKF/robot_model_bus_subsys/robot model s-function2");
      ssSetRTModel(rts,realtime_simu_franka_fr3_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetPWork(rts, (void **)
                 &realtime_simu_franka_fr3_DW.robotmodelsfunction2_PWORK_g[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        ssSetNumDWorkAsInt(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 9);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &realtime_simu_franka_fr3_DW.robotmodelsfunction2_PWORK_g[0]);
      }

      /* registration */
      s_function_opti_robot_model_bus_fun(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 2, 1);
      _ssSetOutputPortConnected(rts, 3, 1);
      _ssSetOutputPortConnected(rts, 4, 1);
      _ssSetOutputPortConnected(rts, 5, 1);
      _ssSetOutputPortConnected(rts, 6, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);
      _ssSetOutputPortBeingMerged(rts, 2, 0);
      _ssSetOutputPortBeingMerged(rts, 3, 0);
      _ssSetOutputPortBeingMerged(rts, 4, 0);
      _ssSetOutputPortBeingMerged(rts, 5, 0);
      _ssSetOutputPortBeingMerged(rts, 6, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
    }

    /* Level2 S-Function Block: realtime_simu_franka_fr3/<S32>/robot model s-function1 (s_function_opti_robot_model_bus_fun) */
    {
      SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[5];

      /* timing info */
      time_T *sfcnPeriod =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.sfcnPeriod;
      time_T *sfcnOffset =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.sfcnOffset;
      int_T *sfcnTsMap =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.sfcnTsMap;
      (void) memset(static_cast<void*>(sfcnPeriod), 0,
                    sizeof(time_T)*1);
      (void) memset(static_cast<void*>(sfcnOffset), 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.blkInfo2[5]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &realtime_simu_franka_fr3_M->NonInlinedSFcns.inputOutputPortInfo2[5]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, realtime_simu_franka_fr3_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods2[5]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods3[5]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods4[5]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.statesInfo2
                         [5]);
        ssSetPeriodicStatesInfo(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.periodicStatesInfo[5]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 2);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.inputPortInfo[0]);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.inputPortInfo[0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        ssSetInputPortUnit(rts, 1, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);
        ssSetInputPortIsContinuousQuantity(rts, 1, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.iDims0;
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0,
                               realtime_simu_franka_fr3_B.sf_zerofixedstates_i.q_red);
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetInputPortNumDimensions(rts, 0, 2);
          ssSetInputPortWidthAsInt(rts, 0, 7);
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.iDims1;
          ssSetInputPortRequiredContiguous(rts, 1, 1);
          ssSetInputPortSignal(rts, 1,
                               realtime_simu_franka_fr3_B.sf_zerofixedstates_i.q_p_red);
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetInputPortNumDimensions(rts, 1, 2);
          ssSetInputPortWidthAsInt(rts, 1, 7);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.outputPortInfo[0]);
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 7);
        _ssSetPortInfo2ForOutputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        ssSetOutputPortUnit(rts, 2, 0);
        ssSetOutputPortUnit(rts, 3, 0);
        ssSetOutputPortUnit(rts, 4, 0);
        ssSetOutputPortUnit(rts, 5, 0);
        ssSetOutputPortUnit(rts, 6, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 1, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 2, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 3, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 4, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 5, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 6, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.oDims0;
          dimensions[0] = 4;
          dimensions[1] = 4;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetOutputPortNumDimensions(rts, 0, 2);
          ssSetOutputPortWidthAsInt(rts, 0, 16);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o1_c));
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.oDims1;
          dimensions[0] = 6;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetOutputPortNumDimensions(rts, 1, 2);
          ssSetOutputPortWidthAsInt(rts, 1, 42);
          ssSetOutputPortSignal(rts, 1, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o2_e));
        }

        /* port 2 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.oDims2;
          dimensions[0] = 6;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 2, dimensions);
          _ssSetOutputPortNumDimensions(rts, 2, 2);
          ssSetOutputPortWidthAsInt(rts, 2, 42);
          ssSetOutputPortSignal(rts, 2, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o3_c));
        }

        /* port 3 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.oDims3;
          dimensions[0] = 7;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 3, dimensions);
          _ssSetOutputPortNumDimensions(rts, 3, 2);
          ssSetOutputPortWidthAsInt(rts, 3, 49);
          ssSetOutputPortSignal(rts, 3, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o4_p));
        }

        /* port 4 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.oDims4;
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 4, dimensions);
          _ssSetOutputPortNumDimensions(rts, 4, 2);
          ssSetOutputPortWidthAsInt(rts, 4, 7);
          ssSetOutputPortSignal(rts, 4, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o5_b));
        }

        /* port 5 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.oDims5;
          dimensions[0] = 7;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 5, dimensions);
          _ssSetOutputPortNumDimensions(rts, 5, 2);
          ssSetOutputPortWidthAsInt(rts, 5, 49);
          ssSetOutputPortSignal(rts, 5, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o6_d));
        }

        /* port 6 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.oDims6;
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 6, dimensions);
          _ssSetOutputPortNumDimensions(rts, 6, 2);
          ssSetOutputPortWidthAsInt(rts, 6, 7);
          ssSetOutputPortSignal(rts, 6, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o7_m));
        }
      }

      /* path info */
      ssSetModelName(rts, "robot model s-function1");
      ssSetPath(rts,
                "realtime_simu_franka_fr3/Subsystem1/EKF/robot_model_bus_subsys/robot model s-function1");
      ssSetRTModel(rts,realtime_simu_franka_fr3_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetPWork(rts, (void **)
                 &realtime_simu_franka_fr3_DW.robotmodelsfunction1_PWORK_i[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn5.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        ssSetNumDWorkAsInt(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 9);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &realtime_simu_franka_fr3_DW.robotmodelsfunction1_PWORK_i[0]);
      }

      /* registration */
      s_function_opti_robot_model_bus_fun(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 2, 1);
      _ssSetOutputPortConnected(rts, 3, 1);
      _ssSetOutputPortConnected(rts, 4, 1);
      _ssSetOutputPortConnected(rts, 5, 1);
      _ssSetOutputPortConnected(rts, 6, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);
      _ssSetOutputPortBeingMerged(rts, 2, 0);
      _ssSetOutputPortBeingMerged(rts, 3, 0);
      _ssSetOutputPortBeingMerged(rts, 4, 0);
      _ssSetOutputPortBeingMerged(rts, 5, 0);
      _ssSetOutputPortBeingMerged(rts, 6, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
    }

    /* Level2 S-Function Block: realtime_simu_franka_fr3/<S10>/S-Function3 (shm_reader_sfun) */
    {
      SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[6];

      /* timing info */
      time_T *sfcnPeriod =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn6.sfcnPeriod;
      time_T *sfcnOffset =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn6.sfcnOffset;
      int_T *sfcnTsMap =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn6.sfcnTsMap;
      (void) memset(static_cast<void*>(sfcnPeriod), 0,
                    sizeof(time_T)*1);
      (void) memset(static_cast<void*>(sfcnOffset), 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.blkInfo2[6]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &realtime_simu_franka_fr3_M->NonInlinedSFcns.inputOutputPortInfo2[6]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, realtime_simu_franka_fr3_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods2[6]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods3[6]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods4[6]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.statesInfo2
                         [6]);
        ssSetPeriodicStatesInfo(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.periodicStatesInfo[6]);
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn6.outputPortInfo[0]);
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn6.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 2);
        _ssSetPortInfo2ForOutputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn6.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn6.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 1, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidthAsInt(rts, 0, 7);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            realtime_simu_franka_fr3_B.SFunction3_o1));
        }

        /* port 1 */
        {
          _ssSetOutputPortNumDimensions(rts, 1, 1);
          ssSetOutputPortWidthAsInt(rts, 1, 1);
          ssSetOutputPortSignal(rts, 1, ((int8_T *)
            &realtime_simu_franka_fr3_B.SFunction3_o2));
        }
      }

      /* path info */
      ssSetModelName(rts, "S-Function3");
      ssSetPath(rts, "realtime_simu_franka_fr3/tau_subsystem/S-Function3");
      ssSetRTModel(rts,realtime_simu_franka_fr3_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn6.params;
        ssSetSFcnParamsCount(rts, 2);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       realtime_simu_franka_fr3_P.SFunction3_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       realtime_simu_franka_fr3_P.SFunction3_P2_Size);
      }

      /* work vectors */
      ssSetPWork(rts, (void **) &realtime_simu_franka_fr3_DW.SFunction3_PWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn6.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn6.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        ssSetNumDWorkAsInt(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 4);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &realtime_simu_franka_fr3_DW.SFunction3_PWORK[0]);
      }

      /* registration */
      shm_reader_sfun(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.0);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);

      /* Update the BufferDstPort flags for each input port */
    }

    /* Level2 S-Function Block: realtime_simu_franka_fr3/<S10>/S-Function4 (shm_writer_sfun) */
    {
      SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[7];

      /* timing info */
      time_T *sfcnPeriod =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn7.sfcnPeriod;
      time_T *sfcnOffset =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn7.sfcnOffset;
      int_T *sfcnTsMap =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn7.sfcnTsMap;
      (void) memset(static_cast<void*>(sfcnPeriod), 0,
                    sizeof(time_T)*1);
      (void) memset(static_cast<void*>(sfcnOffset), 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.blkInfo2[7]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &realtime_simu_franka_fr3_M->NonInlinedSFcns.inputOutputPortInfo2[7]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, realtime_simu_franka_fr3_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods2[7]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods3[7]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods4[7]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.statesInfo2
                         [7]);
        ssSetPeriodicStatesInfo(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.periodicStatesInfo[7]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 7);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn7.inputPortInfo[0]);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn7.inputPortInfo[0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn7.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        ssSetInputPortUnit(rts, 1, 0);
        ssSetInputPortUnit(rts, 2, 0);
        ssSetInputPortUnit(rts, 3, 0);
        ssSetInputPortUnit(rts, 4, 0);
        ssSetInputPortUnit(rts, 5, 0);
        ssSetInputPortUnit(rts, 6, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn7.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);
        ssSetInputPortIsContinuousQuantity(rts, 1, 0);
        ssSetInputPortIsContinuousQuantity(rts, 2, 0);
        ssSetInputPortIsContinuousQuantity(rts, 3, 0);
        ssSetInputPortIsContinuousQuantity(rts, 4, 0);
        ssSetInputPortIsContinuousQuantity(rts, 5, 0);
        ssSetInputPortIsContinuousQuantity(rts, 6, 0);

        /* port 0 */
        {
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0,
                               realtime_simu_franka_fr3_B.TmpSignalConversionAtSFunction4);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidthAsInt(rts, 0, 14);
        }

        /* port 1 */
        {
          ssSetInputPortRequiredContiguous(rts, 1, 1);
          ssSetInputPortSignal(rts, 1, &realtime_simu_franka_fr3_B.CastToSingle4);
          _ssSetInputPortNumDimensions(rts, 1, 1);
          ssSetInputPortWidthAsInt(rts, 1, 1);
        }

        /* port 2 */
        {
          ssSetInputPortRequiredContiguous(rts, 2, 1);
          ssSetInputPortSignal(rts, 2, &realtime_simu_franka_fr3_B.CastToSingle3);
          _ssSetInputPortNumDimensions(rts, 2, 1);
          ssSetInputPortWidthAsInt(rts, 2, 1);
        }

        /* port 3 */
        {
          ssSetInputPortRequiredContiguous(rts, 3, 1);
          ssSetInputPortSignal(rts, 3, &realtime_simu_franka_fr3_B.CastToSingle2);
          _ssSetInputPortNumDimensions(rts, 3, 1);
          ssSetInputPortWidthAsInt(rts, 3, 1);
        }

        /* port 4 */
        {
          ssSetInputPortRequiredContiguous(rts, 4, 1);
          ssSetInputPortSignal(rts, 4, &realtime_simu_franka_fr3_B.CastToSingle1);
          _ssSetInputPortNumDimensions(rts, 4, 1);
          ssSetInputPortWidthAsInt(rts, 4, 1);
        }

        /* port 5 */
        {
          ssSetInputPortRequiredContiguous(rts, 5, 1);
          ssSetInputPortSignal(rts, 5, &realtime_simu_franka_fr3_B.CastToSingle);
          _ssSetInputPortNumDimensions(rts, 5, 1);
          ssSetInputPortWidthAsInt(rts, 5, 1);
        }

        /* port 6 */
        {
          ssSetInputPortRequiredContiguous(rts, 6, 1);
          ssSetInputPortSignal(rts, 6, &realtime_simu_franka_fr3_B.CastToSingle5);
          _ssSetInputPortNumDimensions(rts, 6, 1);
          ssSetInputPortWidthAsInt(rts, 6, 1);
        }
      }

      /* path info */
      ssSetModelName(rts, "S-Function4");
      ssSetPath(rts, "realtime_simu_franka_fr3/tau_subsystem/S-Function4");
      ssSetRTModel(rts,realtime_simu_franka_fr3_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn7.params;
        ssSetSFcnParamsCount(rts, 8);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       realtime_simu_franka_fr3_P.SFunction4_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       realtime_simu_franka_fr3_P.SFunction4_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       realtime_simu_franka_fr3_P.SFunction4_P3_Size);
        ssSetSFcnParam(rts, 3, (mxArray*)
                       realtime_simu_franka_fr3_P.SFunction4_P4_Size);
        ssSetSFcnParam(rts, 4, (mxArray*)
                       realtime_simu_franka_fr3_P.SFunction4_P5_Size);
        ssSetSFcnParam(rts, 5, (mxArray*)
                       realtime_simu_franka_fr3_P.SFunction4_P6_Size);
        ssSetSFcnParam(rts, 6, (mxArray*)
                       realtime_simu_franka_fr3_P.SFunction4_P7_Size);
        ssSetSFcnParam(rts, 7, (mxArray*)
                       realtime_simu_franka_fr3_P.SFunction4_P8_Size);
      }

      /* work vectors */
      ssSetPWork(rts, (void **) &realtime_simu_franka_fr3_DW.SFunction4_PWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn7.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn7.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        ssSetNumDWorkAsInt(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 14);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &realtime_simu_franka_fr3_DW.SFunction4_PWORK[0]);
      }

      /* registration */
      shm_writer_sfun(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetInputPortConnected(rts, 2, 1);
      _ssSetInputPortConnected(rts, 3, 1);
      _ssSetInputPortConnected(rts, 4, 1);
      _ssSetInputPortConnected(rts, 5, 1);
      _ssSetInputPortConnected(rts, 6, 1);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
      ssSetInputPortBufferDstPort(rts, 2, -1);
      ssSetInputPortBufferDstPort(rts, 3, -1);
      ssSetInputPortBufferDstPort(rts, 4, -1);
      ssSetInputPortBufferDstPort(rts, 5, -1);
      ssSetInputPortBufferDstPort(rts, 6, -1);
    }

    /* Level2 S-Function Block: realtime_simu_franka_fr3/<S9>/robot model s-function2 (s_function_opti_robot_model_bus_fun) */
    {
      SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[8];

      /* timing info */
      time_T *sfcnPeriod =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.sfcnPeriod;
      time_T *sfcnOffset =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.sfcnOffset;
      int_T *sfcnTsMap =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.sfcnTsMap;
      (void) memset(static_cast<void*>(sfcnPeriod), 0,
                    sizeof(time_T)*1);
      (void) memset(static_cast<void*>(sfcnOffset), 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.blkInfo2[8]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &realtime_simu_franka_fr3_M->NonInlinedSFcns.inputOutputPortInfo2[8]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, realtime_simu_franka_fr3_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods2[8]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods3[8]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods4[8]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.statesInfo2
                         [8]);
        ssSetPeriodicStatesInfo(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.periodicStatesInfo[8]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 2);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.inputPortInfo[0]);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.inputPortInfo[0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        ssSetInputPortUnit(rts, 1, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);
        ssSetInputPortIsContinuousQuantity(rts, 1, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.iDims0;
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0, realtime_simu_franka_fr3_B.Switch);
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetInputPortNumDimensions(rts, 0, 2);
          ssSetInputPortWidthAsInt(rts, 0, 7);
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.iDims1;
          ssSetInputPortRequiredContiguous(rts, 1, 1);
          ssSetInputPortSignal(rts, 1,
                               realtime_simu_franka_fr3_B.GetRobotState2_o2);
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetInputPortNumDimensions(rts, 1, 2);
          ssSetInputPortWidthAsInt(rts, 1, 7);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.outputPortInfo[0]);
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 7);
        _ssSetPortInfo2ForOutputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        ssSetOutputPortUnit(rts, 2, 0);
        ssSetOutputPortUnit(rts, 3, 0);
        ssSetOutputPortUnit(rts, 4, 0);
        ssSetOutputPortUnit(rts, 5, 0);
        ssSetOutputPortUnit(rts, 6, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 1, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 2, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 3, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 4, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 5, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 6, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.oDims0;
          dimensions[0] = 4;
          dimensions[1] = 4;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetOutputPortNumDimensions(rts, 0, 2);
          ssSetOutputPortWidthAsInt(rts, 0, 16);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o1));
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.oDims1;
          dimensions[0] = 6;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetOutputPortNumDimensions(rts, 1, 2);
          ssSetOutputPortWidthAsInt(rts, 1, 42);
          ssSetOutputPortSignal(rts, 1, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o2));
        }

        /* port 2 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.oDims2;
          dimensions[0] = 6;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 2, dimensions);
          _ssSetOutputPortNumDimensions(rts, 2, 2);
          ssSetOutputPortWidthAsInt(rts, 2, 42);
          ssSetOutputPortSignal(rts, 2, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o3));
        }

        /* port 3 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.oDims3;
          dimensions[0] = 7;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 3, dimensions);
          _ssSetOutputPortNumDimensions(rts, 3, 2);
          ssSetOutputPortWidthAsInt(rts, 3, 49);
          ssSetOutputPortSignal(rts, 3, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o4));
        }

        /* port 4 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.oDims4;
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 4, dimensions);
          _ssSetOutputPortNumDimensions(rts, 4, 2);
          ssSetOutputPortWidthAsInt(rts, 4, 7);
          ssSetOutputPortSignal(rts, 4, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o5));
        }

        /* port 5 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.oDims5;
          dimensions[0] = 7;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 5, dimensions);
          _ssSetOutputPortNumDimensions(rts, 5, 2);
          ssSetOutputPortWidthAsInt(rts, 5, 49);
          ssSetOutputPortSignal(rts, 5, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o6));
        }

        /* port 6 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.oDims6;
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 6, dimensions);
          _ssSetOutputPortNumDimensions(rts, 6, 2);
          ssSetOutputPortWidthAsInt(rts, 6, 7);
          ssSetOutputPortSignal(rts, 6, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o7));
        }
      }

      /* path info */
      ssSetModelName(rts, "robot model s-function2");
      ssSetPath(rts,
                "realtime_simu_franka_fr3/robot_model_bus_subsys/robot model s-function2");
      ssSetRTModel(rts,realtime_simu_franka_fr3_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetPWork(rts, (void **)
                 &realtime_simu_franka_fr3_DW.robotmodelsfunction2_PWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn8.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        ssSetNumDWorkAsInt(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 9);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &realtime_simu_franka_fr3_DW.robotmodelsfunction2_PWORK[0]);
      }

      /* registration */
      s_function_opti_robot_model_bus_fun(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.0);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 2, 1);
      _ssSetOutputPortConnected(rts, 3, 1);
      _ssSetOutputPortConnected(rts, 4, 1);
      _ssSetOutputPortConnected(rts, 5, 1);
      _ssSetOutputPortConnected(rts, 6, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);
      _ssSetOutputPortBeingMerged(rts, 2, 0);
      _ssSetOutputPortBeingMerged(rts, 3, 0);
      _ssSetOutputPortBeingMerged(rts, 4, 0);
      _ssSetOutputPortBeingMerged(rts, 5, 0);
      _ssSetOutputPortBeingMerged(rts, 6, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
    }

    /* Level2 S-Function Block: realtime_simu_franka_fr3/<S9>/robot model s-function1 (s_function_opti_robot_model_bus_fun) */
    {
      SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[9];

      /* timing info */
      time_T *sfcnPeriod =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.sfcnPeriod;
      time_T *sfcnOffset =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.sfcnOffset;
      int_T *sfcnTsMap =
        realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.sfcnTsMap;
      (void) memset(static_cast<void*>(sfcnPeriod), 0,
                    sizeof(time_T)*1);
      (void) memset(static_cast<void*>(sfcnOffset), 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.blkInfo2[9]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &realtime_simu_franka_fr3_M->NonInlinedSFcns.inputOutputPortInfo2[9]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, realtime_simu_franka_fr3_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods2[9]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods3[9]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &realtime_simu_franka_fr3_M->
                           NonInlinedSFcns.methods4[9]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &realtime_simu_franka_fr3_M->NonInlinedSFcns.statesInfo2
                         [9]);
        ssSetPeriodicStatesInfo(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.periodicStatesInfo[9]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 2);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.inputPortInfo[0]);
        ssSetPortInfoForInputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.inputPortInfo[0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        ssSetInputPortUnit(rts, 1, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);
        ssSetInputPortIsContinuousQuantity(rts, 1, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.iDims0;
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0, realtime_simu_franka_fr3_B.q_red);
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetInputPortNumDimensions(rts, 0, 2);
          ssSetInputPortWidthAsInt(rts, 0, 7);
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.iDims1;
          ssSetInputPortRequiredContiguous(rts, 1, 1);
          ssSetInputPortSignal(rts, 1, realtime_simu_franka_fr3_B.q_p_red);
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetInputPortNumDimensions(rts, 1, 2);
          ssSetInputPortWidthAsInt(rts, 1, 7);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.outputPortInfo[0]);
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 7);
        _ssSetPortInfo2ForOutputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        ssSetOutputPortUnit(rts, 2, 0);
        ssSetOutputPortUnit(rts, 3, 0);
        ssSetOutputPortUnit(rts, 4, 0);
        ssSetOutputPortUnit(rts, 5, 0);
        ssSetOutputPortUnit(rts, 6, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 1, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 2, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 3, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 4, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 5, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 6, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.oDims0;
          dimensions[0] = 4;
          dimensions[1] = 4;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetOutputPortNumDimensions(rts, 0, 2);
          ssSetOutputPortWidthAsInt(rts, 0, 16);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o1));
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.oDims1;
          dimensions[0] = 6;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetOutputPortNumDimensions(rts, 1, 2);
          ssSetOutputPortWidthAsInt(rts, 1, 42);
          ssSetOutputPortSignal(rts, 1, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o2));
        }

        /* port 2 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.oDims2;
          dimensions[0] = 6;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 2, dimensions);
          _ssSetOutputPortNumDimensions(rts, 2, 2);
          ssSetOutputPortWidthAsInt(rts, 2, 42);
          ssSetOutputPortSignal(rts, 2, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o3));
        }

        /* port 3 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.oDims3;
          dimensions[0] = 7;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 3, dimensions);
          _ssSetOutputPortNumDimensions(rts, 3, 2);
          ssSetOutputPortWidthAsInt(rts, 3, 49);
          ssSetOutputPortSignal(rts, 3, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o4));
        }

        /* port 4 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.oDims4;
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 4, dimensions);
          _ssSetOutputPortNumDimensions(rts, 4, 2);
          ssSetOutputPortWidthAsInt(rts, 4, 7);
          ssSetOutputPortSignal(rts, 4, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o5));
        }

        /* port 5 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.oDims5;
          dimensions[0] = 7;
          dimensions[1] = 7;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 5, dimensions);
          _ssSetOutputPortNumDimensions(rts, 5, 2);
          ssSetOutputPortWidthAsInt(rts, 5, 49);
          ssSetOutputPortSignal(rts, 5, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o6));
        }

        /* port 6 */
        {
          int_T *dimensions = (int_T *)
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.oDims6;
          dimensions[0] = 7;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 6, dimensions);
          _ssSetOutputPortNumDimensions(rts, 6, 2);
          ssSetOutputPortWidthAsInt(rts, 6, 7);
          ssSetOutputPortSignal(rts, 6, ((real_T *)
            realtime_simu_franka_fr3_B.robotmodelsfunction1_o7));
        }
      }

      /* path info */
      ssSetModelName(rts, "robot model s-function1");
      ssSetPath(rts,
                "realtime_simu_franka_fr3/robot_model_bus_subsys/robot model s-function1");
      ssSetRTModel(rts,realtime_simu_franka_fr3_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetPWork(rts, (void **)
                 &realtime_simu_franka_fr3_DW.robotmodelsfunction1_PWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn9.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        ssSetNumDWorkAsInt(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 9);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &realtime_simu_franka_fr3_DW.robotmodelsfunction1_PWORK[0]);
      }

      /* registration */
      s_function_opti_robot_model_bus_fun(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.0);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 2, 1);
      _ssSetOutputPortConnected(rts, 3, 1);
      _ssSetOutputPortConnected(rts, 4, 1);
      _ssSetOutputPortConnected(rts, 5, 1);
      _ssSetOutputPortConnected(rts, 6, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);
      _ssSetOutputPortBeingMerged(rts, 2, 0);
      _ssSetOutputPortBeingMerged(rts, 3, 0);
      _ssSetOutputPortBeingMerged(rts, 4, 0);
      _ssSetOutputPortBeingMerged(rts, 5, 0);
      _ssSetOutputPortBeingMerged(rts, 6, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
    }
  }

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(realtime_simu_franka_fr3_M->rtwLogInfo, 0.0,
    rtmGetTFinal(realtime_simu_franka_fr3_M),
    realtime_simu_franka_fr3_M->Timing.stepSize0, (&rtmGetErrorStatus
    (realtime_simu_franka_fr3_M)));

  /* Start for S-Function (get_robot_state): '<S5>/Get Robot State2' */
  {
    realtime_simu_franka_fr3_DW.GetRobotState2_DWORK1 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }

  /* Start for Enabled SubSystem: '<S6>/EKF' */
  /* Start for Constant: '<S29>/Constant' */
  memcpy(&realtime_simu_franka_fr3_B.Constant[0],
         &realtime_simu_franka_fr3_P.Constant_Value[0], 196U * sizeof(real_T));

  /* Start for Constant: '<S29>/Constant1' */
  memcpy(&realtime_simu_franka_fr3_B.Constant1[0],
         &realtime_simu_franka_fr3_P.Constant1_Value[0], 196U * sizeof(real_T));

  /* Start for Enabled SubSystem: '<S13>/EKF' */
  /* Start for Constant: '<S16>/Constant' */
  memcpy(&realtime_simu_franka_fr3_B.Constant_p[0],
         &realtime_simu_franka_fr3_P.param_EKF.Rk[0], 196U * sizeof(real_T));

  /* Start for Constant: '<S16>/Constant1' */
  memcpy(&realtime_simu_franka_fr3_B.Constant1_a[0],
         &realtime_simu_franka_fr3_P.param_EKF.Qk[0], 196U * sizeof(real_T));

  /* Start for Enabled SubSystem: '<Root>/jointspace ctl subsys' */
  realtime_simu_franka_fr3_DW.jointspacectlsubsys_MODE = false;

  /* Start for Enabled SubSystem: '<Root>/tau_subsystem' */
  realtime_simu_franka_fr3_DW.tau_subsystem_MODE = false;

  /* Start for S-Function (shm_reader_sfun): '<S10>/S-Function3' */
  /* Level2 S-Function Block: '<S10>/S-Function3' (shm_reader_sfun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[6];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (shm_writer_sfun): '<S10>/S-Function4' */
  /* Level2 S-Function Block: '<S10>/S-Function4' (shm_writer_sfun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[7];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (apply_control): '<S5>/Apply Control' */
  {
    //Flag for performing initialization in first run of main _step();
    realtime_simu_franka_fr3_DW.ApplyControl_DWORK1 = 0;
    realtime_simu_franka_fr3_DW.ApplyControl_DWORK2 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }

  /* InitializeConditions for Memory: '<S26>/filter window' */
  memcpy(&realtime_simu_franka_fr3_DW.filterwindow_PreviousInput[0],
         &realtime_simu_franka_fr3_P.filterwindow_InitialCondition[0], 301U *
         sizeof(real_T));

  /* InitializeConditions for Derivative: '<Root>/Derivative' */
  realtime_simu_franka_fr3_DW.TimeStampA = (rtInf);
  realtime_simu_franka_fr3_DW.TimeStampB = (rtInf);

  /* InitializeConditions for Delay: '<S6>/uk_prev' */
  realtime_simu_franka_fr3_DW.icLoad = true;

  /* InitializeConditions for Delay: '<S13>/uk_prev' */
  realtime_simu_franka_fr3_DW.icLoad_f = true;

  /* InitializeConditions for RateLimiter: '<S5>/Rate Limiter' */
  realtime_simu_franka_fr3_DW.LastMajorTime = (rtInf);

  /* SystemInitialize for MATLAB Function: '<S26>/MATLAB Function' */
  realtime_simu_franka_fr3_DW.time_start_not_empty = false;
  realtime_simu_franka_fr3_DW.freq_not_empty = false;
  realtime_simu_franka_fr3_DW.savedTime_not_empty = false;
  realtime_simu_franka_fr3_DW.cnt_g = 0.0;
  realtime_simu_franka_fr3_DW.time_start = 0.0;

  /* SystemInitialize for Enabled SubSystem: '<S6>/EKF' */
  /* InitializeConditions for Delay: '<S29>/xk_minus' */
  realtime_simu_franka_fr3_DW.icLoad_d = true;

  /* InitializeConditions for Delay: '<S29>/Pk_minus' */
  realtime_simu_franka_fr3_DW.icLoad_b = true;

  /* End of SystemInitialize for SubSystem: '<S6>/EKF' */

  /* SystemInitialize for Merge: '<S6>/Merge' */
  realtime_simu_franka_fr3_B.Merge =
    realtime_simu_franka_fr3_P.Merge_InitialOutput;

  /* SystemInitialize for Enabled SubSystem: '<S13>/EKF' */
  /* InitializeConditions for Delay: '<S16>/xk_minus' */
  realtime_simu_franka_fr3_DW.icLoad_c = true;

  /* InitializeConditions for Delay: '<S16>/Pk_minus' */
  realtime_simu_franka_fr3_DW.icLoad_br = true;

  /* End of SystemInitialize for SubSystem: '<S13>/EKF' */

  /* SystemInitialize for Enabled SubSystem: '<Root>/jointspace ctl subsys' */
  /* SystemInitialize for MATLAB Function: '<S8>/home robot logic' */
  realtime_simu_franka_fr3_DW.enabled_not_empty = false;
  realtime_simu_franka_fr3_DW.enabled = 0.0;

  /* SystemInitialize for Outport: '<S8>/home run flag' */
  realtime_simu_franka_fr3_B.home_running =
    realtime_simu_franka_fr3_P.homerunflag_Y0;

  /* End of SystemInitialize for SubSystem: '<Root>/jointspace ctl subsys' */

  /* SystemInitialize for Enabled SubSystem: '<Root>/tau_subsystem' */
  /* SystemInitialize for MATLAB Function: '<S10>/torque safety' */
  realtime_simu_franka_fr3_DW.cnt = 0.0;

  /* End of SystemInitialize for SubSystem: '<Root>/tau_subsystem' */

  /* SystemInitialize for Merge: '<S13>/Merge' */
  realtime_simu_franka_fr3_B.Merge_i =
    realtime_simu_franka_fr3_P.Merge_InitialOutput_p;

  /* SystemInitialize for Enabled SubSystem: '<S2>/CT Controller Subsystem' */
  /* SystemInitialize for MATLAB Function: '<S11>/CT MATLAB Function' */
  realtime_simu_franka_fr3_DW.cnt_d = 1.0;
  realtime_simu_franka_fr3_DW.run_flag_d = 0.0;

  /* End of SystemInitialize for SubSystem: '<S2>/CT Controller Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S2>/PD+ Controller Subsystem' */
  /* SystemInitialize for MATLAB Function: '<S12>/PD+ MATLAB Function' */
  realtime_simu_franka_fr3_DW.cnt_o = 1.0;
  realtime_simu_franka_fr3_DW.run_flag_c = 0.0;

  /* End of SystemInitialize for SubSystem: '<S2>/PD+ Controller Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<Root>/tau_subsystem' */
  /* SystemInitialize for Enabled SubSystem: '<Root>/jointspace ctl subsys' */
  for (int32_T i = 0; i < 7; i++) {
    /* SystemInitialize for Outport: '<S8>/tau' */
    realtime_simu_franka_fr3_B.tau_k[i] = realtime_simu_franka_fr3_P.tau_Y0;

    /* SystemInitialize for MATLAB Function: '<S10>/torque safety' */
    realtime_simu_franka_fr3_DW.tau_prev[i] = 0.0;

    /* SystemInitialize for Outport: '<S10>/tau' */
    realtime_simu_franka_fr3_B.tau[i] = realtime_simu_franka_fr3_P.tau_Y0_j;

    /* SystemInitialize for Merge: '<S2>/Merge1' */
    realtime_simu_franka_fr3_B.Merge1[i] =
      realtime_simu_franka_fr3_P.Merge1_InitialOutput;
  }

  /* End of SystemInitialize for SubSystem: '<Root>/jointspace ctl subsys' */
  /* End of SystemInitialize for SubSystem: '<Root>/tau_subsystem' */

  /* SystemInitialize for MATLAB Function: '<S3>/MATLAB Function' */
  realtime_simu_franka_fr3_DW.cnt_p = 1.0;
  realtime_simu_franka_fr3_DW.run_flag = 0.0;
}

/* Model terminate function */
void realtime_simu_franka_fr3_terminate(void)
{
  /* Terminate for S-Function (s_function_opti_robot_model_bus_fun): '<S9>/robot model s-function2' */
  /* Level2 S-Function Block: '<S9>/robot model s-function2' (s_function_opti_robot_model_bus_fun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[8];
    sfcnTerminate(rts);
  }

  /* Terminate for Enabled SubSystem: '<S6>/EKF' */

  /* Terminate for S-Function (s_function_opti_ekf_fun): '<S29>/Reduced System sfun casadi solve' */
  /* Level2 S-Function Block: '<S29>/Reduced System sfun casadi solve' (s_function_opti_ekf_fun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[3];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (s_function_opti_robot_model_bus_fun): '<S32>/robot model s-function2' */
  /* Level2 S-Function Block: '<S32>/robot model s-function2' (s_function_opti_robot_model_bus_fun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[4];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (s_function_opti_robot_model_bus_fun): '<S32>/robot model s-function1' */
  /* Level2 S-Function Block: '<S32>/robot model s-function1' (s_function_opti_robot_model_bus_fun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[5];
    sfcnTerminate(rts);
  }

  /* End of Terminate for SubSystem: '<S6>/EKF' */

  /* Terminate for Enabled SubSystem: '<S13>/EKF' */

  /* Terminate for S-Function (s_function_opti_ekf_fun): '<S16>/Reduced System sfun casadi solve' */
  /* Level2 S-Function Block: '<S16>/Reduced System sfun casadi solve' (s_function_opti_ekf_fun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (s_function_opti_robot_model_bus_fun): '<S19>/robot model s-function2' */
  /* Level2 S-Function Block: '<S19>/robot model s-function2' (s_function_opti_robot_model_bus_fun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[1];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (s_function_opti_robot_model_bus_fun): '<S19>/robot model s-function1' */
  /* Level2 S-Function Block: '<S19>/robot model s-function1' (s_function_opti_robot_model_bus_fun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[2];
    sfcnTerminate(rts);
  }

  /* End of Terminate for SubSystem: '<S13>/EKF' */

  /* Terminate for S-Function (s_function_opti_robot_model_bus_fun): '<S9>/robot model s-function1' */
  /* Level2 S-Function Block: '<S9>/robot model s-function1' (s_function_opti_robot_model_bus_fun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[9];
    sfcnTerminate(rts);
  }

  /* Terminate for Enabled SubSystem: '<Root>/tau_subsystem' */

  /* Terminate for S-Function (shm_reader_sfun): '<S10>/S-Function3' */
  /* Level2 S-Function Block: '<S10>/S-Function3' (shm_reader_sfun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[6];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (shm_writer_sfun): '<S10>/S-Function4' */
  /* Level2 S-Function Block: '<S10>/S-Function4' (shm_writer_sfun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[7];
    sfcnTerminate(rts);
  }

  /* End of Terminate for SubSystem: '<Root>/tau_subsystem' */

  /* Terminate for S-Function (apply_control): '<S5>/Apply Control' */
  {
    /* S-Function Block: <S5>/Apply Control */
  }
}
