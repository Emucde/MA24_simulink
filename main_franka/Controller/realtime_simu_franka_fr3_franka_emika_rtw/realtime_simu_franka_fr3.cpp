/*
 * realtime_simu_franka_fr3.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "realtime_simu_franka_fr3".
 *
 * Model version              : 8.623
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Wed Dec  4 15:54:14 2024
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "realtime_simu_franka_fr3.h"
#include "rtwtypes.h"
#include <string.h>
#include <math.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "coder_posix_time.h"
#include "realtime_simu_franka_fr3_private.h"
#include <cstring>

SimulinkPandaRobot simulinkPandaRobot_17216102;

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
static void realtime_simu_fran_timeKeeper_o(real_T *outTime_tv_sec, real_T
  *outTime_tv_nsec);
static real_T realtime_simu_franka_fr3_toc(void);

/* Function for MATLAB Function: '<S14>/MATLAB Function' */
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

/* Function for MATLAB Function: '<S14>/MATLAB Function' */
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

/* Function for MATLAB Function: '<S14>/MATLAB Function' */
static void realtime_simu_fran_timeKeeper_o(real_T *outTime_tv_sec, real_T
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

/* Function for MATLAB Function: '<S14>/MATLAB Function' */
static real_T realtime_simu_franka_fr3_toc(void)
{
  coderTimespec b_timespec;
  real_T tstart_tv_nsec;
  real_T tstart_tv_sec;
  realtime_simu_fran_timeKeeper_o(&tstart_tv_sec, &tstart_tv_nsec);
  if (!realtime_simu_franka_fr3_DW.freq_not_empty) {
    realtime_simu_franka_fr3_DW.freq_not_empty = true;
    coderInitTimeFunctions(&realtime_simu_franka_fr3_DW.freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, realtime_simu_franka_fr3_DW.freq);
  return (b_timespec.tv_nsec - tstart_tv_nsec) / 1.0E+9 + (b_timespec.tv_sec -
    tstart_tv_sec);
}

/* Model step function */
void realtime_simu_franka_fr3_step(void)
{
  {
    real_T x[302];
    real_T f_data[301];
    real_T JJ_colin[49];
    real_T JJ_colin_0[49];
    real_T a[49];
    real_T J_tilde[42];
    real_T A[36];
    real_T R[9];
    real_T rtb_x_d_R_d[9];
    real_T JJ_colin_1[7];
    real_T rtb_Sqrt[7];
    real_T tmp[7];
    real_T (*lastU)[7];
    real_T tmp_0[6];
    real_T tmp_1[6];
    real_T y_p[6];
    real_T y_pp[6];
    real_T absxk;
    real_T g_a;
    real_T q2;
    real_T rtb_reset;
    real_T s;
    real_T s2;
    real_T s3;
    real_T s4;
    real_T t;
    real_T time_end;
    int32_T b;
    int32_T enable;
    int32_T f_data_o_tmp;
    int32_T i;
    int32_T jj;
    int32_T k;
    int8_T ipiv[6];
    boolean_T isodd;
    static const real_T b_0[7] = { 0.0, -0.78539816339744828, 0.0,
      -2.3561944901923448, 0.0, 1.5707963267948966, 0.78539816339744828 };

    static const int8_T d[6] = { 0, 1, 3, 4, 5, 6 };

    /* Reset subsysRan breadcrumbs */
    srClearBC(realtime_simu_franka_fr3_DW.EKF_SubsysRanBC);

    /* Reset subsysRan breadcrumbs */
    srClearBC(realtime_simu_franka_fr3_DW.noEKF_SubsysRanBC);

    /* Reset subsysRan breadcrumbs */
    srClearBC(realtime_simu_franka_fr3_DW.jointspacectlsubsys_SubsysRanBC);

    /* Reset subsysRan breadcrumbs */
    srClearBC(realtime_simu_franka_fr3_DW.Subsystem_SubsysRanBC);

    /* Reset subsysRan breadcrumbs */
    srClearBC(realtime_simu_franka_fr3_DW.tau_subsystem_SubsysRanBC);

    /* Outputs for Enabled SubSystem: '<Root>/jointspace ctl subsys' incorporates:
     *  EnablePort: '<S7>/Enable'
     */
    /* Clock: '<S4>/Clock' incorporates:
     *  Clock: '<S7>/Clock'
     *  Derivative: '<Root>/Derivative'
     */
    s = realtime_simu_franka_fr3_M->Timing.t[0];

    /* End of Outputs for SubSystem: '<Root>/jointspace ctl subsys' */

    /* S-Function (get_robot_state): '<S4>/Get Robot State2' */
    {
      // Wait for the control thread signal
      if ((bool)realtime_simu_franka_fr3_DW.GetRobotState2_DWORK1 &&
          simulinkPandaRobot_17216102.getControlThreadHasBeenSpawned()) {
        simulinkPandaRobot_17216102.waitForControlThreadStep();
      }

      // If control loop threw exeption terminate execution
      simulinkPandaRobot_17216102.checkIfAndHandleException();
      if (!simulinkPandaRobot_17216102.getCurrentlyInFirstControlStep()) {
        std::vector<std::string> output_signals;
        signalsStringToSignalsVector(output_signals,"q dq O_F_ext_hat_K");

        //copy outputs
        simulinkPandaRobot_17216102.copyOutputSignal(output_signals, 0,
          &realtime_simu_franka_fr3_B.GetRobotState2_o1[0]);
        simulinkPandaRobot_17216102.copyOutputSignal(output_signals, 1,
          &realtime_simu_franka_fr3_B.GetRobotState2_o2[0]);
        simulinkPandaRobot_17216102.copyOutputSignal(output_signals, 2,
          &realtime_simu_franka_fr3_B.GetRobotState2_o3[0]);
      }
    }

    for (i = 0; i < 7; i++) {
      /* Switch: '<S4>/Switch' incorporates:
       *  Clock: '<S4>/Clock'
       */
      if (s > realtime_simu_franka_fr3_P.Switch_Threshold) {
        /* Switch: '<S4>/Switch' */
        realtime_simu_franka_fr3_B.Switch[i] =
          realtime_simu_franka_fr3_B.GetRobotState2_o1[i];
      } else {
        /* Switch: '<S4>/Switch' incorporates:
         *  Constant: '<S4>/Constant'
         */
        realtime_simu_franka_fr3_B.Switch[i] =
          realtime_simu_franka_fr3_P.q_init[i];
      }

      /* End of Switch: '<S4>/Switch' */
    }

    /* MATLAB Function: '<S8>/MATLAB Function' incorporates:
     *  Memory: '<S8>/filter window'
     *  ZeroOrderHold: '<S8>/Zero-Order Hold'
     */
    if (realtime_simu_franka_fr3_DW.cnt <
        realtime_simu_franka_fr3_P.param_savgol.N_n + 3000.0) {
      realtime_simu_franka_fr3_DW.cnt++;
      enable = 0;
    } else {
      enable = 1;
    }

    for (k = 0; k < 7; k++) {
      tmp[k] = 0.0;
      for (jj = 0; jj < 5000; jj++) {
        tmp[k] += realtime_simu_franka_fr3_DW.filterwindow_PreviousInput[5000 *
          k + jj] * realtime_simu_franka_fr3_P.param_savgol.bT_n[jj];
      }

      realtime_simu_franka_fr3_B.input_mean[k] = tmp[k];
      realtime_simu_franka_fr3_B.f_data_o[5000 * k] =
        realtime_simu_franka_fr3_B.Switch[k];
      memcpy(&realtime_simu_franka_fr3_B.f_data_o[k * 5000 + 1],
             &realtime_simu_franka_fr3_DW.filterwindow_PreviousInput[k * 5000],
             4999U * sizeof(real_T));
    }

    /* End of MATLAB Function: '<S8>/MATLAB Function' */

    /* MATLAB Function: '<S14>/MATLAB Function' incorporates:
     *  Memory: '<S14>/filter window'
     */
    memcpy(&f_data[0],
           &realtime_simu_franka_fr3_DW.filterwindow_PreviousInput_d[0], 301U *
           sizeof(real_T));
    if (!realtime_simu_franka_fr3_DW.time_start_not_empty) {
      realtime_simu_franka_fr3_tic();
      realtime_simu_franka_fr3_DW.time_start_not_empty = true;
    }

    if (realtime_simu_franka_fr3_DW.cnt_j > 1.0) {
      time_end = realtime_simu_franka_fr3_toc();
      realtime_simu_franka_fr3_B.freq_per_step = 1.0 / (time_end -
        realtime_simu_franka_fr3_DW.time_start);
      realtime_simu_franka_fr3_B.freq_per_step_mean = 0.0;
      for (k = 0; k < 301; k++) {
        realtime_simu_franka_fr3_B.freq_per_step_mean +=
          realtime_simu_franka_fr3_P.param_savgol.bT[k] *
          realtime_simu_franka_fr3_DW.filterwindow_PreviousInput_d[k];
      }

      realtime_simu_franka_fr3_DW.time_start = time_end;
    } else {
      realtime_simu_franka_fr3_B.freq_per_step = 0.0;
      realtime_simu_franka_fr3_B.freq_per_step_mean = 0.0;
    }

    if (realtime_simu_franka_fr3_DW.cnt_j <
        realtime_simu_franka_fr3_P.param_savgol.N) {
      realtime_simu_franka_fr3_DW.cnt_j++;
      if (realtime_simu_franka_fr3_DW.cnt_j > 301.0) {
        f_data_o_tmp = -1;
        b = -1;
      } else {
        f_data_o_tmp = static_cast<int32_T>(realtime_simu_franka_fr3_DW.cnt_j) -
          2;
        b = 300;
      }

      memcpy(&x[0], &realtime_simu_franka_fr3_DW.filterwindow_PreviousInput_d[0],
             301U * sizeof(real_T));
      x[301] = realtime_simu_franka_fr3_B.freq_per_step;
      time_end = x[0];
      for (k = 0; k < 301; k++) {
        time_end += x[k + 1];
      }

      time_end /= 302.0;
      jj = b - f_data_o_tmp;
      for (k = 0; k < jj; k++) {
        f_data[(f_data_o_tmp + k) + 1] = time_end;
      }
    }

    realtime_simu_franka_fr3_B.f_data_o_h[0] =
      realtime_simu_franka_fr3_B.freq_per_step;
    memcpy(&realtime_simu_franka_fr3_B.f_data_o_h[1], &f_data[0], 300U * sizeof
           (real_T));

    /* End of MATLAB Function: '<S14>/MATLAB Function' */

    /* S-Function (s_function_opti_robot_model_bus_fun): '<S9>/robot model s-function2' */

    /* Level2 S-Function Block: '<S9>/robot model s-function2' (s_function_opti_robot_model_bus_fun) */
    {
      SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[4];
      sfcnOutputs(rts,0);
    }

    /* Derivative: '<Root>/Derivative' */
    if ((realtime_simu_franka_fr3_DW.TimeStampA >= s) &&
        (realtime_simu_franka_fr3_DW.TimeStampB >= s)) {
      /* Derivative: '<Root>/Derivative' */
      for (i = 0; i < 7; i++) {
        realtime_simu_franka_fr3_B.Derivative[i] = 0.0;
      }
    } else {
      time_end = realtime_simu_franka_fr3_DW.TimeStampA;
      lastU = &realtime_simu_franka_fr3_DW.LastUAtTimeA;
      if (realtime_simu_franka_fr3_DW.TimeStampA <
          realtime_simu_franka_fr3_DW.TimeStampB) {
        if (realtime_simu_franka_fr3_DW.TimeStampB < s) {
          time_end = realtime_simu_franka_fr3_DW.TimeStampB;
          lastU = &realtime_simu_franka_fr3_DW.LastUAtTimeB;
        }
      } else if (realtime_simu_franka_fr3_DW.TimeStampA >= s) {
        time_end = realtime_simu_franka_fr3_DW.TimeStampB;
        lastU = &realtime_simu_franka_fr3_DW.LastUAtTimeB;
      }

      absxk = s - time_end;

      /* Derivative: '<Root>/Derivative' */
      for (i = 0; i < 7; i++) {
        realtime_simu_franka_fr3_B.Derivative[i] =
          (realtime_simu_franka_fr3_B.GetRobotState2_o2[i] - (*lastU)[i]) /
          absxk;
      }
    }

    /* MATLAB Function: '<S9>/Robot model bus' incorporates:
     *  S-Function (s_function_opti_robot_model_bus_fun): '<S9>/robot model s-function2'
     */
    memcpy(&realtime_simu_franka_fr3_B.robot_model_o.H[0],
           &realtime_simu_franka_fr3_B.robotmodelsfunction2_o1[0], sizeof(real_T)
           << 4U);
    memcpy(&realtime_simu_franka_fr3_B.robot_model_o.J[0],
           &realtime_simu_franka_fr3_B.robotmodelsfunction2_o2[0], 42U * sizeof
           (real_T));
    memcpy(&realtime_simu_franka_fr3_B.robot_model_o.J_p[0],
           &realtime_simu_franka_fr3_B.robotmodelsfunction2_o3[0], 42U * sizeof
           (real_T));
    memcpy(&realtime_simu_franka_fr3_B.robot_model_o.M[0],
           &realtime_simu_franka_fr3_B.robotmodelsfunction2_o4[0], 49U * sizeof
           (real_T));
    memcpy(&realtime_simu_franka_fr3_B.robot_model_o.C[0],
           &realtime_simu_franka_fr3_B.robotmodelsfunction2_o6[0], 49U * sizeof
           (real_T));
    for (i = 0; i < 7; i++) {
      realtime_simu_franka_fr3_B.robot_model_o.q[i] =
        realtime_simu_franka_fr3_B.Switch[i];
      realtime_simu_franka_fr3_B.robot_model_o.q_p[i] =
        realtime_simu_franka_fr3_B.GetRobotState2_o2[i];
      realtime_simu_franka_fr3_B.robot_model_o.q_pp[i] =
        realtime_simu_franka_fr3_B.Derivative[i];
      realtime_simu_franka_fr3_B.robot_model_o.C_rnea[i] =
        realtime_simu_franka_fr3_B.robotmodelsfunction2_o5[i];
      realtime_simu_franka_fr3_B.robot_model_o.g[i] =
        realtime_simu_franka_fr3_B.robotmodelsfunction2_o7[i];

      /* Delay: '<S5>/uk_prev' incorporates:
       *  S-Function (s_function_opti_robot_model_bus_fun): '<S9>/robot model s-function2'
       */
      if (realtime_simu_franka_fr3_DW.icLoad) {
        realtime_simu_franka_fr3_DW.uk_prev_DSTATE[i] =
          realtime_simu_franka_fr3_B.robot_model_o.g[i];
      }

      /* Delay: '<S5>/uk_prev' */
      realtime_simu_franka_fr3_B.uk_prev[i] =
        realtime_simu_franka_fr3_DW.uk_prev_DSTATE[i];
    }

    /* End of MATLAB Function: '<S9>/Robot model bus' */

    /* ManualSwitch: '<Root>/Manual Switch2' incorporates:
     *  Constant: '<Root>/off'
     *  Constant: '<Root>/on'
     */
    if (realtime_simu_franka_fr3_P.ManualSwitch2_CurrentSetting == 1) {
      time_end = realtime_simu_franka_fr3_P.on_Value;
    } else {
      time_end = realtime_simu_franka_fr3_P.off_Value;
    }

    /* End of ManualSwitch: '<Root>/Manual Switch2' */

    /* Outputs for Enabled SubSystem: '<S5>/EKF' incorporates:
     *  EnablePort: '<S17>/Enable'
     */
    if (time_end > 0.0) {
      /* Constant: '<S17>/Constant' */
      memcpy(&realtime_simu_franka_fr3_B.Constant[0],
             &realtime_simu_franka_fr3_P.param_EKF.Rk[0], 196U * sizeof(real_T));
      for (i = 0; i < 7; i++) {
        /* SignalConversion generated from: '<S17>/Reduced System sfun casadi solve' */
        realtime_simu_franka_fr3_B.y_kxk_measured[i] =
          realtime_simu_franka_fr3_B.robot_model_o.q[i];
        realtime_simu_franka_fr3_B.y_kxk_measured[i + 7] =
          realtime_simu_franka_fr3_B.robot_model_o.q_p[i];
      }

      for (i = 0; i < 14; i++) {
        /* Delay: '<S17>/xk_minus' incorporates:
         *  SignalConversion generated from: '<S17>/Reduced System sfun casadi solve'
         */
        if (realtime_simu_franka_fr3_DW.icLoad_p) {
          realtime_simu_franka_fr3_DW.xk_minus_DSTATE[i] =
            realtime_simu_franka_fr3_B.y_kxk_measured[i];
        }

        /* Delay: '<S17>/xk_minus' */
        realtime_simu_franka_fr3_B.xk_minus[i] =
          realtime_simu_franka_fr3_DW.xk_minus_DSTATE[i];
      }

      for (i = 0; i < 196; i++) {
        /* Constant: '<S17>/Constant1' */
        realtime_simu_franka_fr3_B.Constant1[i] =
          realtime_simu_franka_fr3_P.param_EKF.Qk[i];

        /* Delay: '<S17>/Pk_minus' incorporates:
         *  Constant: '<S17>/Constant2'
         */
        if (realtime_simu_franka_fr3_DW.icLoad_o) {
          realtime_simu_franka_fr3_DW.Pk_minus_DSTATE[i] =
            realtime_simu_franka_fr3_P.param_EKF.P0[i];
        }

        /* Delay: '<S17>/Pk_minus' */
        realtime_simu_franka_fr3_B.Pk_minus[i] =
          realtime_simu_franka_fr3_DW.Pk_minus_DSTATE[i];
      }

      /* S-Function (s_function_opti_ekf_fun): '<S17>/Reduced System sfun casadi solve' */

      /* Level2 S-Function Block: '<S17>/Reduced System sfun casadi solve' (s_function_opti_ekf_fun) */
      {
        SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[0];
        sfcnOutputs(rts,0);
      }

      /* MATLAB Function: '<S17>/get EKF joint values' incorporates:
       *  S-Function (s_function_opti_ekf_fun): '<S17>/Reduced System sfun casadi solve'
       */
      for (i = 0; i < 7; i++) {
        realtime_simu_franka_fr3_B.q_j[i] = realtime_simu_franka_fr3_B.xk_plus[i];
        realtime_simu_franka_fr3_B.q_p_j[i] =
          realtime_simu_franka_fr3_B.xk_plus[i + 7];
      }

      /* End of MATLAB Function: '<S17>/get EKF joint values' */

      /* S-Function (s_function_opti_robot_model_bus_fun): '<S20>/robot model s-function2' */

      /* Level2 S-Function Block: '<S20>/robot model s-function2' (s_function_opti_robot_model_bus_fun) */
      {
        SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[1];
        sfcnOutputs(rts,0);
      }

      for (k = 0; k < 7; k++) {
        /* Merge: '<S5>/Merge' incorporates:
         *  MATLAB Function: '<S20>/Robot model bus'
         *  SignalConversion generated from: '<S17>/robot_model_EKF'
         */
        realtime_simu_franka_fr3_B.Merge.q[k] = realtime_simu_franka_fr3_B.q_j[k];
        realtime_simu_franka_fr3_B.Merge.q_p[k] =
          realtime_simu_franka_fr3_B.q_p_j[k];
        realtime_simu_franka_fr3_B.Merge.q_pp[k] =
          realtime_simu_franka_fr3_B.robot_model_o.q_pp[k];
      }

      /* Merge: '<S5>/Merge' incorporates:
       *  MATLAB Function: '<S20>/Robot model bus'
       *  S-Function (s_function_opti_robot_model_bus_fun): '<S20>/robot model s-function2'
       *  SignalConversion generated from: '<S17>/robot_model_EKF'
       */
      memcpy(&realtime_simu_franka_fr3_B.Merge.H[0],
             &realtime_simu_franka_fr3_B.robotmodelsfunction2_o1_a[0], sizeof
             (real_T) << 4U);
      memcpy(&realtime_simu_franka_fr3_B.Merge.J[0],
             &realtime_simu_franka_fr3_B.robotmodelsfunction2_o2_h[0], 42U *
             sizeof(real_T));
      memcpy(&realtime_simu_franka_fr3_B.Merge.J_p[0],
             &realtime_simu_franka_fr3_B.robotmodelsfunction2_o3_e[0], 42U *
             sizeof(real_T));
      memcpy(&realtime_simu_franka_fr3_B.Merge.M[0],
             &realtime_simu_franka_fr3_B.robotmodelsfunction2_o4_c[0], 49U *
             sizeof(real_T));
      for (k = 0; k < 7; k++) {
        realtime_simu_franka_fr3_B.Merge.C_rnea[k] =
          realtime_simu_franka_fr3_B.robotmodelsfunction2_o5_l[k];
      }

      memcpy(&realtime_simu_franka_fr3_B.Merge.C[0],
             &realtime_simu_franka_fr3_B.robotmodelsfunction2_o6_n[0], 49U *
             sizeof(real_T));
      for (k = 0; k < 7; k++) {
        realtime_simu_franka_fr3_B.Merge.g[k] =
          realtime_simu_franka_fr3_B.robotmodelsfunction2_o7_i[k];
      }

      /* Update for Delay: '<S17>/xk_minus' incorporates:
       *  S-Function (s_function_opti_ekf_fun): '<S17>/Reduced System sfun casadi solve'
       */
      realtime_simu_franka_fr3_DW.icLoad_p = false;
      memcpy(&realtime_simu_franka_fr3_DW.xk_minus_DSTATE[0],
             &realtime_simu_franka_fr3_B.xkp1_minus[0], 14U * sizeof(real_T));

      /* Update for Delay: '<S17>/Pk_minus' incorporates:
       *  S-Function (s_function_opti_ekf_fun): '<S17>/Reduced System sfun casadi solve'
       */
      realtime_simu_franka_fr3_DW.icLoad_o = false;
      memcpy(&realtime_simu_franka_fr3_DW.Pk_minus_DSTATE[0],
             &realtime_simu_franka_fr3_B.Pkp1_minus[0], 196U * sizeof(real_T));
      srUpdateBC(realtime_simu_franka_fr3_DW.EKF_SubsysRanBC);
    }

    /* End of Outputs for SubSystem: '<S5>/EKF' */
    for (i = 0; i < 7; i++) {
      /* Sum: '<S8>/Add' incorporates:
       *  ZeroOrderHold: '<S8>/Zero-Order Hold'
       */
      realtime_simu_franka_fr3_B.noise[i] = realtime_simu_franka_fr3_B.Switch[i]
        - realtime_simu_franka_fr3_B.input_mean[i];
    }

    /* Sum: '<Root>/Add' incorporates:
     *  Constant: '<Root>/Reset Trajectory'
     *  Constant: '<Root>/home'
     */
    rtb_reset = realtime_simu_franka_fr3_P.ResetTrajectory_Value +
      realtime_simu_franka_fr3_P.home_Value;

    /* Outputs for Enabled SubSystem: '<S5>/no EKF' incorporates:
     *  EnablePort: '<S18>/Enable'
     */
    /* Logic: '<S5>/NOT' */
    if (!(time_end != 0.0)) {
      /* Merge: '<S5>/Merge' incorporates:
       *  SignalConversion generated from: '<S18>/In1'
       */
      realtime_simu_franka_fr3_B.Merge =
        realtime_simu_franka_fr3_B.robot_model_o;
      srUpdateBC(realtime_simu_franka_fr3_DW.noEKF_SubsysRanBC);
    }

    /* End of Logic: '<S5>/NOT' */
    /* End of Outputs for SubSystem: '<S5>/no EKF' */

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
        SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[2];
        sfcnOutputs(rts,0);
      }

      /* DataTypeConversion: '<S10>/Cast To Single4' incorporates:
       *  Constant: '<S10>/Constant'
       */
      time_end = floor(realtime_simu_franka_fr3_P.Constant_Value);
      if (rtIsNaN(time_end) || rtIsInf(time_end)) {
        time_end = 0.0;
      } else {
        time_end = fmod(time_end, 256.0);
      }

      /* DataTypeConversion: '<S10>/Cast To Single4' */
      realtime_simu_franka_fr3_B.CastToSingle4 = static_cast<int8_T>(time_end <
        0.0 ? static_cast<int32_T>(static_cast<int8_T>(-static_cast<int8_T>(
        static_cast<uint8_T>(-time_end)))) : static_cast<int32_T>
        (static_cast<int8_T>(static_cast<uint8_T>(time_end))));

      /* DataTypeConversion: '<S10>/Cast To Single3' incorporates:
       *  Constant: '<Root>/Start Trajectory'
       */
      time_end = floor(realtime_simu_franka_fr3_P.StartTrajectory_Value);
      if (rtIsNaN(time_end) || rtIsInf(time_end)) {
        time_end = 0.0;
      } else {
        time_end = fmod(time_end, 256.0);
      }

      /* DataTypeConversion: '<S10>/Cast To Single3' */
      realtime_simu_franka_fr3_B.CastToSingle3 = static_cast<int8_T>(time_end <
        0.0 ? static_cast<int32_T>(static_cast<int8_T>(-static_cast<int8_T>(
        static_cast<uint8_T>(-time_end)))) : static_cast<int32_T>
        (static_cast<int8_T>(static_cast<uint8_T>(time_end))));

      /* DataTypeConversion: '<S10>/Cast To Single2' */
      time_end = floor(rtb_reset);
      if (rtIsNaN(time_end) || rtIsInf(time_end)) {
        time_end = 0.0;
      } else {
        time_end = fmod(time_end, 256.0);
      }

      /* DataTypeConversion: '<S10>/Cast To Single2' */
      realtime_simu_franka_fr3_B.CastToSingle2 = static_cast<int8_T>(time_end <
        0.0 ? static_cast<int32_T>(static_cast<int8_T>(-static_cast<int8_T>(
        static_cast<uint8_T>(-time_end)))) : static_cast<int32_T>
        (static_cast<int8_T>(static_cast<uint8_T>(time_end))));

      /* DataTypeConversion: '<S10>/Cast To Single1' incorporates:
       *  Constant: '<Root>/Stop Trajectory'
       */
      time_end = floor(realtime_simu_franka_fr3_P.StopTrajectory_Value);
      if (rtIsNaN(time_end) || rtIsInf(time_end)) {
        time_end = 0.0;
      } else {
        time_end = fmod(time_end, 256.0);
      }

      /* DataTypeConversion: '<S10>/Cast To Single1' */
      realtime_simu_franka_fr3_B.CastToSingle1 = static_cast<int8_T>(time_end <
        0.0 ? static_cast<int32_T>(static_cast<int8_T>(-static_cast<int8_T>(
        static_cast<uint8_T>(-time_end)))) : static_cast<int32_T>
        (static_cast<int8_T>(static_cast<uint8_T>(time_end))));

      /* DataTypeConversion: '<S10>/Cast To Single' incorporates:
       *  Constant: '<Root>/trajectory selector'
       */
      realtime_simu_franka_fr3_B.CastToSingle = static_cast<int8_T>
        (realtime_simu_franka_fr3_P.trajectoryselector_Value);

      /* S-Function (shm_writer_sfun): '<S10>/S-Function4' */

      /* Level2 S-Function Block: '<S10>/S-Function4' (shm_writer_sfun) */
      {
        SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[3];
        sfcnOutputs(rts,0);
      }

      srUpdateBC(realtime_simu_franka_fr3_DW.tau_subsystem_SubsysRanBC);
    }

    /* End of Outputs for SubSystem: '<Root>/tau_subsystem' */

    /* Abs: '<S8>/Abs' incorporates:
     *  Sqrt: '<S6>/Sqrt'
     *  Sum: '<S8>/Add'
     */
    for (i = 0; i < 7; i++) {
      rtb_Sqrt[i] = fabs(realtime_simu_franka_fr3_B.noise[i]);
    }

    /* End of Abs: '<S8>/Abs' */

    /* S-Function (sdspstatfcns): '<S8>/Mean' incorporates:
     *  Sqrt: '<S6>/Sqrt'
     */
    jj = 0;
    for (k = 0; k < 7; k++) {
      realtime_simu_franka_fr3_DW.Mean_Iteration++;
      if (realtime_simu_franka_fr3_DW.Mean_Iteration > 1U) {
        realtime_simu_franka_fr3_DW.Mean_AccVal += rtb_Sqrt[jj];
        realtime_simu_franka_fr3_B.noiseamplitude[jj] =
          realtime_simu_franka_fr3_DW.Mean_AccVal / static_cast<real_T>
          (realtime_simu_franka_fr3_DW.Mean_Iteration);
      } else {
        if (realtime_simu_franka_fr3_DW.Mean_Iteration == 0U) {
          realtime_simu_franka_fr3_DW.Mean_Iteration = 1U;
        }

        realtime_simu_franka_fr3_DW.Mean_AccVal = rtb_Sqrt[jj];
        realtime_simu_franka_fr3_B.noiseamplitude[jj] = rtb_Sqrt[jj];
      }

      jj++;
    }

    /* End of S-Function (sdspstatfcns): '<S8>/Mean' */

    /* Outputs for Enabled SubSystem: '<S8>/Subsystem' incorporates:
     *  EnablePort: '<S28>/Enable'
     */
    if (enable > 0) {
      /* S-Function (sdspstatfcns): '<S28>/Variance' incorporates:
       *  Sum: '<S8>/Add'
       */
      jj = 0;
      for (k = 0; k < 7; k++) {
        realtime_simu_franka_fr3_DW.Variance_Iteration++;
        if (realtime_simu_franka_fr3_DW.Variance_Iteration > 1U) {
          time_end = realtime_simu_franka_fr3_DW.Variance_AccVal;
          realtime_simu_franka_fr3_DW.Variance_AccVal = ((static_cast<real_T>
            (realtime_simu_franka_fr3_DW.Variance_Iteration) - 1.0) *
            realtime_simu_franka_fr3_DW.Variance_AccVal +
            realtime_simu_franka_fr3_B.noise[jj]) / static_cast<real_T>
            (realtime_simu_franka_fr3_DW.Variance_Iteration);
          time_end = realtime_simu_franka_fr3_DW.Variance_AccVal - time_end;
          absxk = realtime_simu_franka_fr3_B.noise[jj] -
            realtime_simu_franka_fr3_DW.Variance_AccVal;
          realtime_simu_franka_fr3_DW.Variance_SqData = ((static_cast<real_T>
            (realtime_simu_franka_fr3_DW.Variance_Iteration) - 2.0) / (
            static_cast<real_T>(realtime_simu_franka_fr3_DW.Variance_Iteration)
            - 1.0) * realtime_simu_franka_fr3_DW.Variance_SqData + time_end *
            time_end) + absxk * absxk / (static_cast<real_T>
            (realtime_simu_franka_fr3_DW.Variance_Iteration) - 1.0);
          realtime_simu_franka_fr3_B.noisevarianceq[jj] =
            realtime_simu_franka_fr3_DW.Variance_SqData;
        } else {
          if (realtime_simu_franka_fr3_DW.Variance_Iteration == 0U) {
            realtime_simu_franka_fr3_DW.Variance_Iteration = 1U;
          }

          realtime_simu_franka_fr3_DW.Variance_AccVal =
            realtime_simu_franka_fr3_B.noise[jj];
          realtime_simu_franka_fr3_DW.Variance_SqData =
            realtime_simu_franka_fr3_B.noise[jj] *
            realtime_simu_franka_fr3_B.noise[jj];
          realtime_simu_franka_fr3_B.noisevarianceq[jj] = 0.0;
        }

        jj++;
      }

      /* End of S-Function (sdspstatfcns): '<S28>/Variance' */
      srUpdateBC(realtime_simu_franka_fr3_DW.Subsystem_SubsysRanBC);
    }

    /* End of Outputs for SubSystem: '<S8>/Subsystem' */
    /* Outputs for Enabled SubSystem: '<Root>/jointspace ctl subsys' incorporates:
     *  EnablePort: '<S7>/Enable'
     */
    /* Constant: '<Root>/Constant1' */
    realtime_simu_franka_fr3_DW.jointspacectlsubsys_MODE =
      (realtime_simu_franka_fr3_P.Constant1_Value > 0.0);
    if (realtime_simu_franka_fr3_DW.jointspacectlsubsys_MODE) {
      /* MATLAB Function: '<S7>/home robot logic' incorporates:
       *  Constant: '<Root>/home'
       *  Constant: '<S7>/K_d_init'
       *  Constant: '<S7>/K_d_t0'
       *  Constant: '<S7>/K_d_t1'
       *  Constant: '<S7>/t0'
       *  Constant: '<S7>/t1'
       *  Constant: '<S7>/tend'
       */
      if (!realtime_simu_franka_fr3_DW.enabled_not_empty) {
        realtime_simu_franka_fr3_DW.enabled_not_empty = true;
        realtime_simu_franka_fr3_DW.t_start = s;
      }

      if (realtime_simu_franka_fr3_P.home_Value == 1.0) {
        realtime_simu_franka_fr3_DW.enabled = 1.0;
        realtime_simu_franka_fr3_DW.t_start = s;
      }

      if (s - realtime_simu_franka_fr3_DW.t_start <
          realtime_simu_franka_fr3_P.t0_Value) {
        q2 = (s - realtime_simu_franka_fr3_DW.t_start) /
          realtime_simu_franka_fr3_P.t0_Value;
        for (i = 0; i < 7; i++) {
          realtime_simu_franka_fr3_B.K_d[i] =
            (realtime_simu_franka_fr3_P.K_d_t0_Value[i] -
             realtime_simu_franka_fr3_P.K_d_init_Value[i]) * q2 +
            realtime_simu_franka_fr3_P.K_d_init_Value[i];
        }
      } else {
        time_end = s - realtime_simu_franka_fr3_DW.t_start;
        if (time_end < realtime_simu_franka_fr3_P.t1_Value) {
          q2 = (time_end - realtime_simu_franka_fr3_P.t0_Value) /
            realtime_simu_franka_fr3_P.t1_Value;
          for (i = 0; i < 7; i++) {
            realtime_simu_franka_fr3_B.K_d[i] =
              (realtime_simu_franka_fr3_P.K_d_t1_Value[i] -
               realtime_simu_franka_fr3_P.K_d_t0_Value[i]) * q2 +
              realtime_simu_franka_fr3_P.K_d_init_Value[i];
          }
        } else {
          for (k = 0; k < 7; k++) {
            realtime_simu_franka_fr3_B.K_d[k] =
              realtime_simu_franka_fr3_P.K_d_t1_Value[k];
          }
        }
      }

      for (k = 0; k < 7; k++) {
        realtime_simu_franka_fr3_B.D_d[k] = 2.0 *
          realtime_simu_franka_fr3_B.K_d[k];
        realtime_simu_franka_fr3_B.D_d[k] = sqrt
          (realtime_simu_franka_fr3_B.D_d[k]);
      }

      if (s - realtime_simu_franka_fr3_DW.t_start >
          realtime_simu_franka_fr3_P.tend_Value) {
        realtime_simu_franka_fr3_DW.enabled = 0.0;
      }

      realtime_simu_franka_fr3_B.home_running =
        realtime_simu_franka_fr3_DW.enabled;

      /* End of MATLAB Function: '<S7>/home robot logic' */

      /* MATLAB Function: '<S7>/get reference pose' incorporates:
       *  Constant: '<Root>/home mode'
       *  Constant: '<Root>/trajectory selector'
       */
      if (realtime_simu_franka_fr3_P.homemode_Value == 0.0) {
        for (k = 0; k < 7; k++) {
          rtb_Sqrt[k] = b_0[k];
        }
      } else {
        for (k = 0; k < 7; k++) {
          rtb_Sqrt[k] = realtime_simu_franka_fr3_P.param_traj.q_0
            [(static_cast<int32_T>
              (realtime_simu_franka_fr3_P.trajectoryselector_Value) - 1) * 7 + k];
        }
      }

      /* End of MATLAB Function: '<S7>/get reference pose' */

      /* MATLAB Function: '<S7>/Joinspace controller' incorporates:
       *  Merge: '<S5>/Merge'
       */
      memset(&JJ_colin[0], 0, 49U * sizeof(real_T));
      for (k = 0; k < 7; k++) {
        JJ_colin[k + 7 * k] = realtime_simu_franka_fr3_B.D_d[k];
      }

      memset(&a[0], 0, 49U * sizeof(real_T));
      for (enable = 0; enable < 7; enable++) {
        a[enable + 7 * enable] = realtime_simu_franka_fr3_B.K_d[enable];
      }

      for (k = 0; k < 49; k++) {
        JJ_colin_0[k] = -JJ_colin[k];
      }

      for (k = 0; k < 7; k++) {
        tmp[k] = realtime_simu_franka_fr3_B.Merge.q[k] - rtb_Sqrt[k];
        JJ_colin_1[k] = 0.0;
        for (jj = 0; jj < 7; jj++) {
          JJ_colin_1[k] += JJ_colin_0[7 * jj + k] *
            realtime_simu_franka_fr3_B.Merge.q_p[jj];
        }
      }

      for (k = 0; k < 7; k++) {
        rtb_Sqrt[k] = 0.0;
        for (jj = 0; jj < 7; jj++) {
          rtb_Sqrt[k] += a[7 * jj + k] * tmp[jj];
        }

        realtime_simu_franka_fr3_B.tau[k] = JJ_colin_1[k] - rtb_Sqrt[k];
      }

      /* End of MATLAB Function: '<S7>/Joinspace controller' */
      srUpdateBC(realtime_simu_franka_fr3_DW.jointspacectlsubsys_SubsysRanBC);
    }

    /* End of Outputs for SubSystem: '<Root>/jointspace ctl subsys' */
    for (i = 0; i < 7; i++) {
      /* ManualSwitch: '<Root>/Manual Switch' */
      if (realtime_simu_franka_fr3_P.ManualSwitch_CurrentSetting == 1) {
        rtb_Sqrt[i] = 0.0;
      } else {
        rtb_Sqrt[i] = realtime_simu_franka_fr3_B.SFunction3_o1[i];
      }

      /* End of ManualSwitch: '<Root>/Manual Switch' */

      /* MATLAB Function: '<S6>/get q_0_ref' incorporates:
       *  Constant: '<Root>/trajectory selector'
       */
      realtime_simu_franka_fr3_B.q_0_ref_fixed[i] =
        realtime_simu_franka_fr3_P.param_traj.q_0[(static_cast<int32_T>
        (realtime_simu_franka_fr3_P.trajectoryselector_Value) - 1) * 7 + i];

      /* Gain: '<S6>/Gain' incorporates:
       *  Constant: '<S6>/K_d_jointspace1'
       *  Sqrt: '<S6>/Sqrt'
       */
      realtime_simu_franka_fr3_B.Gain[i] = realtime_simu_franka_fr3_P.Gain_Gain *
        sqrt(realtime_simu_franka_fr3_P.K_d_jointspace1_Value[i]);
    }

    /* MATLAB Function: '<S6>/joint space control fixed q3' incorporates:
     *  Constant: '<S6>/K_d_jointspace1'
     *  Merge: '<S5>/Merge'
     */
    rtb_Sqrt[2] += (realtime_simu_franka_fr3_B.Merge.q[2] -
                    realtime_simu_franka_fr3_B.q_0_ref_fixed[2]) *
      -realtime_simu_franka_fr3_P.K_d_jointspace1_Value[2] -
      realtime_simu_franka_fr3_B.Gain[2] * realtime_simu_franka_fr3_B.Merge.q_p
      [2];

    /* Switch: '<Root>/Switch' */
    if (realtime_simu_franka_fr3_B.home_running >
        realtime_simu_franka_fr3_P.Switch_Threshold_c) {
      /* Switch: '<Root>/Switch' */
      for (i = 0; i < 7; i++) {
        realtime_simu_franka_fr3_B.Switch_n[i] =
          realtime_simu_franka_fr3_B.tau[i];
      }
    } else {
      /* ManualSwitch: '<Root>/Manual Switch1' */
      isodd = (realtime_simu_franka_fr3_P.ManualSwitch1_CurrentSetting == 1);

      /* Switch: '<Root>/Switch' */
      for (k = 0; k < 7; k++) {
        /* ManualSwitch: '<Root>/Manual Switch1' incorporates:
         *  Constant: '<Root>/Constant'
         *  MATLAB Function: '<S6>/joint space control fixed q3'
         */
        if (isodd) {
          realtime_simu_franka_fr3_B.Switch_n[k] = rtb_Sqrt[k];
        } else {
          realtime_simu_franka_fr3_B.Switch_n[k] =
            realtime_simu_franka_fr3_P.Constant_Value_b[k];
        }
      }
    }

    /* End of Switch: '<Root>/Switch' */

    /* RateLimiter: '<S4>/Rate Limiter' incorporates:
     *  Switch: '<Root>/Switch'
     */
    if (realtime_simu_franka_fr3_DW.LastMajorTime == (rtInf)) {
      /* RateLimiter: '<S4>/Rate Limiter' incorporates:
       *  Switch: '<Root>/Switch'
       */
      for (i = 0; i < 7; i++) {
        realtime_simu_franka_fr3_B.RateLimiter[i] =
          realtime_simu_franka_fr3_B.Switch_n[i];
      }
    } else {
      absxk = realtime_simu_franka_fr3_M->Timing.t[0] -
        realtime_simu_franka_fr3_DW.LastMajorTime;
      for (i = 0; i < 7; i++) {
        s = realtime_simu_franka_fr3_B.Switch_n[i] -
          realtime_simu_franka_fr3_DW.PrevY[i];
        time_end = absxk * realtime_simu_franka_fr3_P.RateLimiter_RisingLim;
        if (s > time_end) {
          realtime_simu_franka_fr3_B.RateLimiter[i] = time_end +
            realtime_simu_franka_fr3_DW.PrevY[i];
        } else {
          time_end = absxk * realtime_simu_franka_fr3_P.RateLimiter_FallingLim;
          if (s < time_end) {
            realtime_simu_franka_fr3_B.RateLimiter[i] =
              realtime_simu_franka_fr3_DW.PrevY[i] + time_end;
          } else {
            realtime_simu_franka_fr3_B.RateLimiter[i] =
              realtime_simu_franka_fr3_B.Switch_n[i];
          }
        }
      }
    }

    /* End of RateLimiter: '<S4>/Rate Limiter' */

    /* S-Function (apply_control): '<S4>/Apply Control' */
    {
      /* S-Function Block: <S4>/Apply Control */
      if ((bool)realtime_simu_franka_fr3_DW.ApplyControl_DWORK1) {
        // Wait for the control thread signal
        if ((bool)realtime_simu_franka_fr3_DW.ApplyControl_DWORK2) {
          simulinkPandaRobot_17216102.waitForControlThreadStep();
        }

        // If control loop threw exeption terminate execution
        simulinkPandaRobot_17216102.checkIfAndHandleException();

        // copy inputs
        simulinkPandaRobot_17216102.copyInputSignal
          (&realtime_simu_franka_fr3_B.RateLimiter[0], 0);

        // notify control thread that the inputs have been read
        simulinkPandaRobot_17216102.notifyControlThreadToContinue();
      } else if (!(bool)realtime_simu_franka_fr3_DW.ApplyControl_DWORK1) {
        // Its the first time _step() function is called -->
        // Initialize according to settings parsed from the mask
        // and spawn control thread
        simulinkPandaRobot_17216102.applyRobotSettings();
        simulinkPandaRobot_17216102.spawnControlThread();
        realtime_simu_franka_fr3_DW.ApplyControl_DWORK1 = 1;
      }
    }

    for (i = 0; i < 7; i++) {
      /* Sum: '<S3>/Add' incorporates:
       *  Switch: '<Root>/Switch'
       */
      realtime_simu_franka_fr3_B.Add[i] = realtime_simu_franka_fr3_B.Switch_n[i]
        - realtime_simu_franka_fr3_B.Merge.g[i];
    }

    /* MATLAB Function: '<S2>/MATLAB Function' incorporates:
     *  BusCreator generated from: '<S2>/MATLAB Function'
     *  Constant: '<Root>/Start Trajectory'
     *  Constant: '<Root>/Stop Trajectory'
     *  Constant: '<Root>/trajectory selector'
     *  Constant: '<S2>/Constant3'
     */
    for (k = 0; k < 3; k++) {
      jj = ((static_cast<int32_T>(realtime_simu_franka_fr3_DW.cnt_c) - 1) * 3 +
            k) + (static_cast<int32_T>
                  (realtime_simu_franka_fr3_P.trajectoryselector_Value) - 1) *
        36003;
      realtime_simu_franka_fr3_B.p_d[k] =
        realtime_simu_franka_fr3_P.traj_data_bus_init.p_d[jj];
      realtime_simu_franka_fr3_B.p_d_p[k] =
        realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_p[jj];
      realtime_simu_franka_fr3_B.p_d_pp[k] =
        realtime_simu_franka_fr3_P.traj_data_bus_init.p_d_pp[jj];
      jj = ((static_cast<int32_T>(realtime_simu_franka_fr3_DW.cnt_c) - 1) * 9 +
            3 * k) + (static_cast<int32_T>
                      (realtime_simu_franka_fr3_P.trajectoryselector_Value) - 1)
        * 108009;
      rtb_x_d_R_d[3 * k] = realtime_simu_franka_fr3_P.traj_data_bus_init.R_d[jj];
      rtb_x_d_R_d[3 * k + 1] =
        realtime_simu_franka_fr3_P.traj_data_bus_init.R_d[jj + 1];
      rtb_x_d_R_d[3 * k + 2] =
        realtime_simu_franka_fr3_P.traj_data_bus_init.R_d[jj + 2];
    }

    k = ((static_cast<int32_T>(realtime_simu_franka_fr3_DW.cnt_c) - 1) << 2) + (
      static_cast<int32_T>(realtime_simu_franka_fr3_P.trajectoryselector_Value)
      - 1) * 48004;
    time_end = realtime_simu_franka_fr3_P.traj_data_bus_init.q_d[k];
    s = realtime_simu_franka_fr3_P.traj_data_bus_init.q_d[k + 1];
    absxk = realtime_simu_franka_fr3_P.traj_data_bus_init.q_d[k + 2];
    t = realtime_simu_franka_fr3_P.traj_data_bus_init.q_d[k + 3];
    k = (static_cast<int32_T>(realtime_simu_franka_fr3_DW.cnt_c) - 1) * 3 + (
      static_cast<int32_T>(realtime_simu_franka_fr3_P.trajectoryselector_Value)
      - 1) * 36003;
    realtime_simu_franka_fr3_B.omega_d[0] =
      realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d[k];
    realtime_simu_franka_fr3_B.omega_d_p[0] =
      realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d_p[k];
    realtime_simu_franka_fr3_B.omega_d[1] =
      realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d[k + 1];
    realtime_simu_franka_fr3_B.omega_d_p[1] =
      realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d_p[k + 1];
    realtime_simu_franka_fr3_B.omega_d[2] =
      realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d[k + 2];
    realtime_simu_franka_fr3_B.omega_d_p[2] =
      realtime_simu_franka_fr3_P.traj_data_bus_init.omega_d_p[k + 2];
    if (realtime_simu_franka_fr3_DW.run_flag == 0.0) {
      if ((realtime_simu_franka_fr3_P.StartTrajectory_Value == 1.0) &&
          (rtb_reset == 0.0) && (realtime_simu_franka_fr3_P.StopTrajectory_Value
           == 0.0)) {
        realtime_simu_franka_fr3_DW.run_flag = 1.0;
        if (realtime_simu_franka_fr3_DW.cnt_c <
            realtime_simu_franka_fr3_P.traj_data_bus_init.N) {
          realtime_simu_franka_fr3_DW.cnt_c++;
        }
      } else if ((rtb_reset == 1.0) && (realtime_simu_franka_fr3_DW.cnt_c ==
                  realtime_simu_franka_fr3_P.traj_data_bus_init.N)) {
        realtime_simu_franka_fr3_DW.cnt_c = 1.0;
      }
    } else if (realtime_simu_franka_fr3_DW.run_flag == 1.0) {
      if (realtime_simu_franka_fr3_P.StopTrajectory_Value == 1.0) {
        realtime_simu_franka_fr3_DW.run_flag = 0.0;
      } else if (realtime_simu_franka_fr3_DW.cnt_c <
                 realtime_simu_franka_fr3_P.traj_data_bus_init.N) {
        realtime_simu_franka_fr3_DW.cnt_c++;
      } else {
        realtime_simu_franka_fr3_DW.run_flag = 0.0;
      }
    }

    /* End of MATLAB Function: '<S2>/MATLAB Function' */

    /* MATLAB Function: '<S3>/calc_errors' incorporates:
     *  Merge: '<S5>/Merge'
     */
    for (k = 0; k < 3; k++) {
      for (jj = 0; jj < 3; jj++) {
        f_data_o_tmp = 3 * jj + k;
        R[f_data_o_tmp] = 0.0;
        R[f_data_o_tmp] += realtime_simu_franka_fr3_B.Merge.H[k] *
          rtb_x_d_R_d[jj];
        R[f_data_o_tmp] += realtime_simu_franka_fr3_B.Merge.H[k + 4] *
          rtb_x_d_R_d[jj + 3];
        R[f_data_o_tmp] += realtime_simu_franka_fr3_B.Merge.H[k + 8] *
          rtb_x_d_R_d[jj + 6];
      }
    }

    rtb_reset = (R[0] + R[4]) + R[8];
    s2 = (R[0] - R[4]) - R[8];
    s3 = (-R[0] + R[4]) - R[8];
    s4 = (-R[0] - R[4]) + R[8];
    if (rtb_reset > 0.0) {
      rtb_reset = sqrt(rtb_reset + 1.0) / 2.0;
    } else {
      q2 = R[5] - R[7];
      rtb_reset = R[6] - R[2];
      g_a = R[1] - R[3];
      rtb_reset = sqrt(((q2 * q2 + rtb_reset * rtb_reset) + g_a * g_a) / (((3.0
        - R[0]) - R[4]) - R[8])) / 2.0;
    }

    if (s2 > 0.0) {
      q2 = sqrt(s2 + 1.0) / 2.0;
    } else {
      q2 = R[5] - R[7];
      s2 = R[1] + R[3];
      g_a = R[2] + R[6];
      q2 = sqrt(((q2 * q2 + s2 * s2) + g_a * g_a) / (((3.0 - R[0]) + R[4]) + R[8]))
        / 2.0;
    }

    if (s3 > 0.0) {
      s3 = sqrt(s3 + 1.0) / 2.0;
    } else {
      s3 = R[6] - R[2];
      s2 = R[1] + R[3];
      g_a = R[5] + R[7];
      s3 = sqrt(((s3 * s3 + s2 * s2) + g_a * g_a) / (((R[0] + 3.0) - R[4]) + R[8]))
        / 2.0;
    }

    if (s4 > 0.0) {
      s2 = sqrt(s4 + 1.0) / 2.0;
    } else {
      s4 = R[1] - R[3];
      s2 = R[2] + R[6];
      g_a = R[5] + R[7];
      s2 = sqrt(((s4 * s4 + s2 * s2) + g_a * g_a) / (((R[0] + 3.0) + R[4]) - R[8]))
        / 2.0;
    }

    s4 = (static_cast<real_T>(R[5] - R[7] >= 0.0) * 2.0 - 1.0) * q2;
    s3 *= static_cast<real_T>(R[6] - R[2] >= 0.0) * 2.0 - 1.0;
    q2 = (static_cast<real_T>(R[1] - R[3] >= 0.0) * 2.0 - 1.0) * s2;
    for (k = 0; k < 6; k++) {
      y_p[k] = 0.0;
      tmp_0[k] = 0.0;
      tmp_1[k] = 0.0;
      for (jj = 0; jj < 7; jj++) {
        f_data_o_tmp = 6 * jj + k;
        s2 = realtime_simu_franka_fr3_B.Merge.J[f_data_o_tmp];
        y_p[k] += s2 * realtime_simu_franka_fr3_B.Merge.q_p[jj];
        tmp_0[k] += s2 * realtime_simu_franka_fr3_B.Merge.q_pp[jj];
        tmp_1[k] += realtime_simu_franka_fr3_B.Merge.J_p[f_data_o_tmp] *
          realtime_simu_franka_fr3_B.Merge.q_p[jj];
      }

      y_pp[k] = tmp_0[k] + tmp_1[k];
    }

    realtime_simu_franka_fr3_B.p[0] = realtime_simu_franka_fr3_B.Merge.H[12];
    realtime_simu_franka_fr3_B.p_p[0] = y_p[0];
    realtime_simu_franka_fr3_B.p_pp[0] = y_pp[0];
    realtime_simu_franka_fr3_B.omega_e[0] = y_p[3];
    realtime_simu_franka_fr3_B.omega_e_p[0] = y_pp[3];
    realtime_simu_franka_fr3_B.q_e_o[0] = (time_end * s4 + rtb_reset * s) +
      (absxk * q2 - s3 * t);
    realtime_simu_franka_fr3_B.q_d_o[0] = s;
    realtime_simu_franka_fr3_B.q_err_o[0] = s4;
    realtime_simu_franka_fr3_B.omega_d_err[0] =
      realtime_simu_franka_fr3_B.omega_e[0] -
      realtime_simu_franka_fr3_B.omega_d[0];
    realtime_simu_franka_fr3_B.omega_d_p_err[0] =
      realtime_simu_franka_fr3_B.omega_e_p[0] -
      realtime_simu_franka_fr3_B.omega_d_p[0];
    realtime_simu_franka_fr3_B.p[1] = realtime_simu_franka_fr3_B.Merge.H[13];
    realtime_simu_franka_fr3_B.p_p[1] = y_p[1];
    realtime_simu_franka_fr3_B.p_pp[1] = y_pp[1];
    realtime_simu_franka_fr3_B.omega_e[1] = y_p[4];
    realtime_simu_franka_fr3_B.omega_e_p[1] = y_pp[4];
    realtime_simu_franka_fr3_B.q_e_o[1] = (time_end * s3 + rtb_reset * absxk) +
      (s4 * t - s * q2);
    realtime_simu_franka_fr3_B.q_d_o[1] = absxk;
    realtime_simu_franka_fr3_B.q_err_o[1] = s3;
    realtime_simu_franka_fr3_B.omega_d_err[1] =
      realtime_simu_franka_fr3_B.omega_e[1] -
      realtime_simu_franka_fr3_B.omega_d[1];
    realtime_simu_franka_fr3_B.omega_d_p_err[1] =
      realtime_simu_franka_fr3_B.omega_e_p[1] -
      realtime_simu_franka_fr3_B.omega_d_p[1];
    realtime_simu_franka_fr3_B.p[2] = realtime_simu_franka_fr3_B.Merge.H[14];
    realtime_simu_franka_fr3_B.p_p[2] = y_p[2];
    realtime_simu_franka_fr3_B.p_pp[2] = y_pp[2];
    realtime_simu_franka_fr3_B.omega_e[2] = y_p[5];
    realtime_simu_franka_fr3_B.omega_e_p[2] = y_pp[5];
    realtime_simu_franka_fr3_B.q_e_o[2] = (time_end * q2 + rtb_reset * t) + (s *
      s3 - s4 * absxk);
    realtime_simu_franka_fr3_B.q_d_o[2] = t;
    realtime_simu_franka_fr3_B.q_err_o[2] = q2;
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

    /* End of MATLAB Function: '<S3>/calc_errors' */
    for (i = 0; i < 7; i++) {
      /* SignalConversion generated from: '<S3>/Bus Selector1' */
      realtime_simu_franka_fr3_B.q_pp[i] =
        realtime_simu_franka_fr3_B.Merge.q_pp[i];

      /* SignalConversion generated from: '<S3>/Bus Selector1' */
      realtime_simu_franka_fr3_B.q[i] = realtime_simu_franka_fr3_B.Merge.q[i];

      /* SignalConversion generated from: '<S3>/Bus Selector1' */
      realtime_simu_franka_fr3_B.q_p[i] = realtime_simu_franka_fr3_B.Merge.q_p[i];
    }

    /* MATLAB Function: '<S12>/manipulability and collinearity 7DOF' incorporates:
     *  Merge: '<S5>/Merge'
     */
    for (k = 0; k < 6; k++) {
      for (jj = 0; jj < 6; jj++) {
        f_data_o_tmp = 6 * jj + k;
        A[f_data_o_tmp] = 0.0;
        for (enable = 0; enable < 6; enable++) {
          i = 6 * d[enable];
          A[f_data_o_tmp] += realtime_simu_franka_fr3_B.Merge.J[i + k] *
            realtime_simu_franka_fr3_B.Merge.J[i + jj];
        }
      }

      ipiv[k] = static_cast<int8_T>(k + 1);
    }

    for (k = 0; k < 5; k++) {
      int32_T jA;
      jj = k * 7;
      enable = 6 - k;
      i = 0;
      time_end = fabs(A[jj]);
      for (f_data_o_tmp = 2; f_data_o_tmp <= enable; f_data_o_tmp++) {
        s = fabs(A[(jj + f_data_o_tmp) - 1]);
        if (s > time_end) {
          i = f_data_o_tmp - 1;
          time_end = s;
        }
      }

      if (A[jj + i] != 0.0) {
        if (i != 0) {
          i += k;
          ipiv[k] = static_cast<int8_T>(i + 1);
          for (enable = 0; enable < 6; enable++) {
            b = enable * 6 + k;
            time_end = A[b];
            f_data_o_tmp = enable * 6 + i;
            A[b] = A[f_data_o_tmp];
            A[f_data_o_tmp] = time_end;
          }
        }

        b = (jj - k) + 6;
        for (i = jj + 2; i <= b; i++) {
          A[i - 1] /= A[jj];
        }
      }

      b = 4 - k;
      jA = jj + 8;
      for (enable = 0; enable <= b; enable++) {
        time_end = A[(enable * 6 + jj) + 6];
        if (time_end != 0.0) {
          f_data_o_tmp = (jA - k) + 4;
          for (i = jA; i <= f_data_o_tmp; i++) {
            A[i - 1] += A[((jj + i) - jA) + 1] * -time_end;
          }
        }

        jA += 6;
      }
    }

    time_end = A[0];
    isodd = false;
    for (k = 0; k < 5; k++) {
      time_end *= A[((k + 1) * 6 + k) + 1];
      if (ipiv[k] > k + 1) {
        isodd = !isodd;
      }
    }

    if (isodd) {
      time_end = -time_end;
    }

    realtime_simu_franka_fr3_B.w = fabs(time_end);
    realtime_simu_franka_fr3_B.w = sqrt(realtime_simu_franka_fr3_B.w);
    for (jj = 0; jj < 7; jj++) {
      f_data_o_tmp = jj * 6 + 1;
      time_end = 0.0;
      s = 3.3121686421112381E-170;
      for (k = f_data_o_tmp; k <= f_data_o_tmp + 5; k++) {
        absxk = fabs(realtime_simu_franka_fr3_B.Merge.J[k - 1]);
        if (absxk > s) {
          t = s / absxk;
          time_end = time_end * t * t + 1.0;
          s = absxk;
        } else {
          t = absxk / s;
          time_end += t * t;
        }
      }

      rtb_Sqrt[jj] = s * sqrt(time_end);
      for (k = 0; k < 6; k++) {
        f_data_o_tmp = 6 * jj + k;
        J_tilde[f_data_o_tmp] = realtime_simu_franka_fr3_B.Merge.J[f_data_o_tmp]
          / rtb_Sqrt[jj];
      }
    }

    for (k = 0; k < 7; k++) {
      for (jj = 0; jj < 7; jj++) {
        f_data_o_tmp = 7 * jj + k;
        JJ_colin[f_data_o_tmp] = 0.0;
        for (enable = 0; enable < 6; enable++) {
          JJ_colin[f_data_o_tmp] += J_tilde[6 * k + enable] * J_tilde[6 * jj +
            enable];
        }
      }
    }

    /* SignalConversion generated from: '<S3>/From25' */
    realtime_simu_franka_fr3_B.p_emYxByPz[0] = realtime_simu_franka_fr3_B.p[0];
    realtime_simu_franka_fr3_B.p_emYxByPz[3] = realtime_simu_franka_fr3_B.p_d[0];

    /* SignalConversion generated from: '<S3>/From26' */
    realtime_simu_franka_fr3_B._emsYxByPz[0] = realtime_simu_franka_fr3_B.p_p[0];
    realtime_simu_franka_fr3_B._emsYxByPz[3] = realtime_simu_franka_fr3_B.p_d_p
      [0];

    /* SignalConversion generated from: '<S3>/From38' */
    realtime_simu_franka_fr3_B.quat_e24[0] = realtime_simu_franka_fr3_B.q_e_o[0];
    realtime_simu_franka_fr3_B.quat_e24[3] = realtime_simu_franka_fr3_B.q_d_o[0];

    /* SignalConversion generated from: '<S3>/From39' */
    realtime_simu_franka_fr3_B.p_emsYxByPz[0] = realtime_simu_franka_fr3_B.p_pp
      [0];
    realtime_simu_franka_fr3_B.p_emsYxByPz[3] =
      realtime_simu_franka_fr3_B.p_d_pp[0];

    /* SignalConversion generated from: '<S3>/From40' */
    realtime_simu_franka_fr3_B._erads[0] = realtime_simu_franka_fr3_B.omega_e[0];
    realtime_simu_franka_fr3_B._erads[3] = realtime_simu_franka_fr3_B.omega_d[0];

    /* SignalConversion generated from: '<S3>/From41' */
    realtime_simu_franka_fr3_B._erads_p[0] =
      realtime_simu_franka_fr3_B.omega_e_p[0];
    realtime_simu_franka_fr3_B._erads_p[3] =
      realtime_simu_franka_fr3_B.omega_d_p[0];

    /* SignalConversion generated from: '<S3>/From25' */
    realtime_simu_franka_fr3_B.p_emYxByPz[1] = realtime_simu_franka_fr3_B.p[1];
    realtime_simu_franka_fr3_B.p_emYxByPz[4] = realtime_simu_franka_fr3_B.p_d[1];

    /* SignalConversion generated from: '<S3>/From26' */
    realtime_simu_franka_fr3_B._emsYxByPz[1] = realtime_simu_franka_fr3_B.p_p[1];
    realtime_simu_franka_fr3_B._emsYxByPz[4] = realtime_simu_franka_fr3_B.p_d_p
      [1];

    /* SignalConversion generated from: '<S3>/From38' */
    realtime_simu_franka_fr3_B.quat_e24[1] = realtime_simu_franka_fr3_B.q_e_o[1];
    realtime_simu_franka_fr3_B.quat_e24[4] = realtime_simu_franka_fr3_B.q_d_o[1];

    /* SignalConversion generated from: '<S3>/From39' */
    realtime_simu_franka_fr3_B.p_emsYxByPz[1] = realtime_simu_franka_fr3_B.p_pp
      [1];
    realtime_simu_franka_fr3_B.p_emsYxByPz[4] =
      realtime_simu_franka_fr3_B.p_d_pp[1];

    /* SignalConversion generated from: '<S3>/From40' */
    realtime_simu_franka_fr3_B._erads[1] = realtime_simu_franka_fr3_B.omega_e[1];
    realtime_simu_franka_fr3_B._erads[4] = realtime_simu_franka_fr3_B.omega_d[1];

    /* SignalConversion generated from: '<S3>/From41' */
    realtime_simu_franka_fr3_B._erads_p[1] =
      realtime_simu_franka_fr3_B.omega_e_p[1];
    realtime_simu_franka_fr3_B._erads_p[4] =
      realtime_simu_franka_fr3_B.omega_d_p[1];

    /* SignalConversion generated from: '<S3>/From25' */
    realtime_simu_franka_fr3_B.p_emYxByPz[2] = realtime_simu_franka_fr3_B.p[2];
    realtime_simu_franka_fr3_B.p_emYxByPz[5] = realtime_simu_franka_fr3_B.p_d[2];

    /* SignalConversion generated from: '<S3>/From26' */
    realtime_simu_franka_fr3_B._emsYxByPz[2] = realtime_simu_franka_fr3_B.p_p[2];
    realtime_simu_franka_fr3_B._emsYxByPz[5] = realtime_simu_franka_fr3_B.p_d_p
      [2];

    /* SignalConversion generated from: '<S3>/From38' */
    realtime_simu_franka_fr3_B.quat_e24[2] = realtime_simu_franka_fr3_B.q_e_o[2];
    realtime_simu_franka_fr3_B.quat_e24[5] = realtime_simu_franka_fr3_B.q_d_o[2];

    /* SignalConversion generated from: '<S3>/From39' */
    realtime_simu_franka_fr3_B.p_emsYxByPz[2] = realtime_simu_franka_fr3_B.p_pp
      [2];
    realtime_simu_franka_fr3_B.p_emsYxByPz[5] =
      realtime_simu_franka_fr3_B.p_d_pp[2];

    /* SignalConversion generated from: '<S3>/From40' */
    realtime_simu_franka_fr3_B._erads[2] = realtime_simu_franka_fr3_B.omega_e[2];
    realtime_simu_franka_fr3_B._erads[5] = realtime_simu_franka_fr3_B.omega_d[2];

    /* SignalConversion generated from: '<S3>/From41' */
    realtime_simu_franka_fr3_B._erads_p[2] =
      realtime_simu_franka_fr3_B.omega_e_p[2];
    realtime_simu_franka_fr3_B._erads_p[5] =
      realtime_simu_franka_fr3_B.omega_d_p[2];

    /* SignalConversion generated from: '<S3>/From50' */
    realtime_simu_franka_fr3_B.freqperTastepHz[0] =
      realtime_simu_franka_fr3_B.freq_per_step;
    realtime_simu_franka_fr3_B.freqperTastepHz[1] =
      realtime_simu_franka_fr3_B.freq_per_step_mean;

    /* SignalConversion generated from: '<S12>/Bus Selector' incorporates:
     *  MATLAB Function: '<S12>/manipulability and collinearity 7DOF'
     */
    realtime_simu_franka_fr3_B.JJ_Y12_B13_R14[0] = JJ_colin[7];
    realtime_simu_franka_fr3_B.JJ_Y12_B13_R14[1] = JJ_colin[14];
    realtime_simu_franka_fr3_B.JJ_Y12_B13_R14[2] = JJ_colin[21];

    /* SignalConversion generated from: '<S12>/Bus Selector' incorporates:
     *  MATLAB Function: '<S12>/manipulability and collinearity 7DOF'
     */
    realtime_simu_franka_fr3_B.JJ_Y15_B16_R17[0] = JJ_colin[28];
    realtime_simu_franka_fr3_B.JJ_Y15_B16_R17[1] = JJ_colin[35];
    realtime_simu_franka_fr3_B.JJ_Y15_B16_R17[2] = JJ_colin[42];

    /* SignalConversion generated from: '<S12>/Bus Selector' incorporates:
     *  MATLAB Function: '<S12>/manipulability and collinearity 7DOF'
     */
    realtime_simu_franka_fr3_B.JJ_Y23_B24_R25[0] = JJ_colin[15];
    realtime_simu_franka_fr3_B.JJ_Y23_B24_R25[1] = JJ_colin[22];
    realtime_simu_franka_fr3_B.JJ_Y23_B24_R25[2] = JJ_colin[29];

    /* SignalConversion generated from: '<S12>/Bus Selector' incorporates:
     *  MATLAB Function: '<S12>/manipulability and collinearity 7DOF'
     */
    realtime_simu_franka_fr3_B.JJ_Y26_B27_R34[0] = JJ_colin[36];
    realtime_simu_franka_fr3_B.JJ_Y26_B27_R34[1] = JJ_colin[43];
    realtime_simu_franka_fr3_B.JJ_Y26_B27_R34[2] = JJ_colin[23];

    /* SignalConversion generated from: '<S12>/Bus Selector' incorporates:
     *  MATLAB Function: '<S12>/manipulability and collinearity 7DOF'
     */
    realtime_simu_franka_fr3_B.JJ_Y35_B36_R37[0] = JJ_colin[30];
    realtime_simu_franka_fr3_B.JJ_Y35_B36_R37[1] = JJ_colin[37];
    realtime_simu_franka_fr3_B.JJ_Y35_B36_R37[2] = JJ_colin[44];

    /* SignalConversion generated from: '<S12>/Bus Selector' incorporates:
     *  MATLAB Function: '<S12>/manipulability and collinearity 7DOF'
     */
    realtime_simu_franka_fr3_B.JJ_Y45_B46_R47[0] = JJ_colin[31];
    realtime_simu_franka_fr3_B.JJ_Y45_B46_R47[1] = JJ_colin[38];
    realtime_simu_franka_fr3_B.JJ_Y45_B46_R47[2] = JJ_colin[45];

    /* SignalConversion generated from: '<S12>/Bus Selector' incorporates:
     *  MATLAB Function: '<S12>/manipulability and collinearity 7DOF'
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

    /* Update for Memory: '<S8>/filter window' */
    memcpy(&realtime_simu_franka_fr3_DW.filterwindow_PreviousInput[0],
           &realtime_simu_franka_fr3_B.f_data_o[0], 35000U * sizeof(real_T));

    /* Update for Memory: '<S14>/filter window' */
    memcpy(&realtime_simu_franka_fr3_DW.filterwindow_PreviousInput_d[0],
           &realtime_simu_franka_fr3_B.f_data_o_h[0], 301U * sizeof(real_T));

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

    /* Update for Delay: '<S5>/uk_prev' incorporates:
     *  Switch: '<Root>/Switch'
     */
    realtime_simu_franka_fr3_DW.icLoad = false;
    for (int32_T i = 0; i < 7; i++) {
      realtime_simu_franka_fr3_DW.uk_prev_DSTATE[i] =
        realtime_simu_franka_fr3_B.Switch_n[i];

      /* Update for RateLimiter: '<S4>/Rate Limiter' incorporates:
       *  Switch: '<Root>/Switch'
       */
      realtime_simu_franka_fr3_DW.PrevY[i] =
        realtime_simu_franka_fr3_B.RateLimiter[i];
    }

    /* End of Update for Delay: '<S5>/uk_prev' */

    /* Update for RateLimiter: '<S4>/Rate Limiter' */
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
  realtime_simu_franka_fr3_M->Sizes.checksums[0] = (1500687667U);
  realtime_simu_franka_fr3_M->Sizes.checksums[1] = (2601619809U);
  realtime_simu_franka_fr3_M->Sizes.checksums[2] = (4131611643U);
  realtime_simu_franka_fr3_M->Sizes.checksums[3] = (1105354683U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[23];
    realtime_simu_franka_fr3_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = (sysRanDType *)&realtime_simu_franka_fr3_DW.EKF_SubsysRanBC;
    systemRan[7] = (sysRanDType *)&realtime_simu_franka_fr3_DW.EKF_SubsysRanBC;
    systemRan[8] = (sysRanDType *)&realtime_simu_franka_fr3_DW.EKF_SubsysRanBC;
    systemRan[9] = (sysRanDType *)&realtime_simu_franka_fr3_DW.noEKF_SubsysRanBC;
    systemRan[10] = &rtAlwaysEnabled;
    systemRan[11] = &rtAlwaysEnabled;
    systemRan[12] = &rtAlwaysEnabled;
    systemRan[13] = &rtAlwaysEnabled;
    systemRan[14] = &rtAlwaysEnabled;
    systemRan[15] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.jointspacectlsubsys_SubsysRanBC;
    systemRan[16] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.jointspacectlsubsys_SubsysRanBC;
    systemRan[17] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.jointspacectlsubsys_SubsysRanBC;
    systemRan[18] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.jointspacectlsubsys_SubsysRanBC;
    systemRan[19] = &rtAlwaysEnabled;
    systemRan[20] = (sysRanDType *)
      &realtime_simu_franka_fr3_DW.Subsystem_SubsysRanBC;
    systemRan[21] = &rtAlwaysEnabled;
    systemRan[22] = (sysRanDType *)
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
      simulinkPandaRobot_17216102 = SimulinkPandaRobot( "172.16.10.2",
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

  realtime_simu_franka_fr3_M->Sizes.numSFcns = (5);

  /* register each child */
  {
    (void) memset(static_cast<void *>
                  (&realtime_simu_franka_fr3_M->NonInlinedSFcns.childSFunctions
                   [0]), 0,
                  5*sizeof(SimStruct));
    realtime_simu_franka_fr3_M->childSfunctions =
      (&realtime_simu_franka_fr3_M->NonInlinedSFcns.childSFunctionPtrs[0]);

    {
      int_T i;
      for (i = 0; i < 5; i++) {
        realtime_simu_franka_fr3_M->childSfunctions[i] =
          (&realtime_simu_franka_fr3_M->NonInlinedSFcns.childSFunctions[i]);
      }
    }

    /* Level2 S-Function Block: realtime_simu_franka_fr3/<S17>/Reduced System sfun casadi solve (s_function_opti_ekf_fun) */
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
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.iDims1;
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
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.iDims2;
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
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.iDims3;
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
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.iDims4;
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
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn0.iDims5;
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
            realtime_simu_franka_fr3_B.xk_plus));
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
            realtime_simu_franka_fr3_B.xkp1_minus));
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

    /* Level2 S-Function Block: realtime_simu_franka_fr3/<S20>/robot model s-function2 (s_function_opti_robot_model_bus_fun) */
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
          ssSetInputPortSignal(rts, 0, realtime_simu_franka_fr3_B.q_j);
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
          ssSetInputPortSignal(rts, 1, realtime_simu_franka_fr3_B.q_p_j);
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
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o1_a));
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
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o3_e));
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
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o4_c));
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
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o5_l));
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
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o6_n));
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
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o7_i));
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
                 &realtime_simu_franka_fr3_DW.robotmodelsfunction2_PWORK_k[0]);

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
                   &realtime_simu_franka_fr3_DW.robotmodelsfunction2_PWORK_k[0]);
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

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.outputPortInfo[0]);
        ssSetPortInfoForOutputs(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 2);
        _ssSetPortInfo2ForOutputUnits(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.outputPortCoSimAttribute
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
          ssSetOutputPortSignal(rts, 1, ((real_T *)
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
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.params;
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
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn2.dWorkAux;
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
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.params;
        ssSetSFcnParamsCount(rts, 6);
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
      }

      /* work vectors */
      ssSetPWork(rts, (void **) &realtime_simu_franka_fr3_DW.SFunction4_PWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn3.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        ssSetNumDWorkAsInt(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 12);
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

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
      ssSetInputPortBufferDstPort(rts, 2, -1);
      ssSetInputPortBufferDstPort(rts, 3, -1);
      ssSetInputPortBufferDstPort(rts, 4, -1);
      ssSetInputPortBufferDstPort(rts, 5, -1);
    }

    /* Level2 S-Function Block: realtime_simu_franka_fr3/<S9>/robot model s-function2 (s_function_opti_robot_model_bus_fun) */
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
            &realtime_simu_franka_fr3_M->NonInlinedSFcns.Sfcn4.iDims1;
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
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o1));
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
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o2));
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
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o3));
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
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o4));
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
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o5));
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
            realtime_simu_franka_fr3_B.robotmodelsfunction2_o6));
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
  }

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(realtime_simu_franka_fr3_M->rtwLogInfo, 0.0,
    rtmGetTFinal(realtime_simu_franka_fr3_M),
    realtime_simu_franka_fr3_M->Timing.stepSize0, (&rtmGetErrorStatus
    (realtime_simu_franka_fr3_M)));

  /* Start for S-Function (get_robot_state): '<S4>/Get Robot State2' */
  {
    realtime_simu_franka_fr3_DW.GetRobotState2_DWORK1 = (double)
      simulinkPandaRobot_17216102.establishIfCurrentBlockFirstToBeComputed();
  }

  /* Start for Enabled SubSystem: '<S5>/EKF' */
  /* Start for Constant: '<S17>/Constant' */
  memcpy(&realtime_simu_franka_fr3_B.Constant[0],
         &realtime_simu_franka_fr3_P.param_EKF.Rk[0], 196U * sizeof(real_T));

  /* Start for Constant: '<S17>/Constant1' */
  memcpy(&realtime_simu_franka_fr3_B.Constant1[0],
         &realtime_simu_franka_fr3_P.param_EKF.Qk[0], 196U * sizeof(real_T));

  /* Start for Enabled SubSystem: '<Root>/tau_subsystem' */
  realtime_simu_franka_fr3_DW.tau_subsystem_MODE = false;

  /* Start for S-Function (shm_reader_sfun): '<S10>/S-Function3' */
  /* Level2 S-Function Block: '<S10>/S-Function3' (shm_reader_sfun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[2];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (shm_writer_sfun): '<S10>/S-Function4' */
  /* Level2 S-Function Block: '<S10>/S-Function4' (shm_writer_sfun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[3];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for Enabled SubSystem: '<Root>/jointspace ctl subsys' */
  realtime_simu_franka_fr3_DW.jointspacectlsubsys_MODE = false;

  /* Start for S-Function (apply_control): '<S4>/Apply Control' */
  {
    //Flag for performing initialization in first run of main _step();
    realtime_simu_franka_fr3_DW.ApplyControl_DWORK1 = 0;
    realtime_simu_franka_fr3_DW.ApplyControl_DWORK2 = (double)
      simulinkPandaRobot_17216102.establishIfCurrentBlockFirstToBeComputed();
  }

  /* InitializeConditions for Memory: '<S8>/filter window' */
  memcpy(&realtime_simu_franka_fr3_DW.filterwindow_PreviousInput[0],
         &realtime_simu_franka_fr3_P.filterwindow_InitialCondition[0], 35000U *
         sizeof(real_T));

  /* InitializeConditions for Memory: '<S14>/filter window' */
  memcpy(&realtime_simu_franka_fr3_DW.filterwindow_PreviousInput_d[0],
         &realtime_simu_franka_fr3_P.filterwindow_InitialCondition_i[0], 301U *
         sizeof(real_T));

  /* InitializeConditions for Derivative: '<Root>/Derivative' */
  realtime_simu_franka_fr3_DW.TimeStampA = (rtInf);
  realtime_simu_franka_fr3_DW.TimeStampB = (rtInf);

  /* InitializeConditions for Delay: '<S5>/uk_prev' */
  realtime_simu_franka_fr3_DW.icLoad = true;

  /* InitializeConditions for S-Function (sdspstatfcns): '<S8>/Mean' */
  realtime_simu_franka_fr3_DW.Mean_Iteration = 0U;
  realtime_simu_franka_fr3_DW.Mean_AccVal = 0.0;

  /* InitializeConditions for RateLimiter: '<S4>/Rate Limiter' */
  realtime_simu_franka_fr3_DW.LastMajorTime = (rtInf);

  /* SystemInitialize for MATLAB Function: '<S8>/MATLAB Function' */
  realtime_simu_franka_fr3_DW.cnt = 0.0;

  /* SystemInitialize for MATLAB Function: '<S14>/MATLAB Function' */
  realtime_simu_franka_fr3_DW.time_start_not_empty = false;
  realtime_simu_franka_fr3_DW.freq_not_empty = false;
  realtime_simu_franka_fr3_DW.savedTime_not_empty = false;
  realtime_simu_franka_fr3_DW.cnt_j = 0.0;
  realtime_simu_franka_fr3_DW.time_start = 0.0;

  /* SystemInitialize for Enabled SubSystem: '<S5>/EKF' */
  /* InitializeConditions for Delay: '<S17>/xk_minus' */
  realtime_simu_franka_fr3_DW.icLoad_p = true;

  /* InitializeConditions for Delay: '<S17>/Pk_minus' */
  realtime_simu_franka_fr3_DW.icLoad_o = true;

  /* End of SystemInitialize for SubSystem: '<S5>/EKF' */

  /* SystemInitialize for Merge: '<S5>/Merge' */
  realtime_simu_franka_fr3_B.Merge =
    realtime_simu_franka_fr3_P.Merge_InitialOutput;

  /* SystemInitialize for Enabled SubSystem: '<S8>/Subsystem' */
  /* InitializeConditions for S-Function (sdspstatfcns): '<S28>/Variance' */
  realtime_simu_franka_fr3_DW.Variance_Iteration = 0U;
  realtime_simu_franka_fr3_DW.Variance_AccVal = 0.0;
  realtime_simu_franka_fr3_DW.Variance_SqData = 0.0;

  /* End of SystemInitialize for SubSystem: '<S8>/Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<Root>/jointspace ctl subsys' */
  /* SystemInitialize for MATLAB Function: '<S7>/home robot logic' */
  realtime_simu_franka_fr3_DW.enabled_not_empty = false;
  realtime_simu_franka_fr3_DW.enabled = 0.0;

  /* SystemInitialize for Enabled SubSystem: '<S8>/Subsystem' */
  /* SystemInitialize for Enabled SubSystem: '<Root>/tau_subsystem' */
  for (int32_T i = 0; i < 7; i++) {
    /* SystemInitialize for S-Function (shm_reader_sfun): '<S10>/S-Function3' incorporates:
     *  Outport: '<S10>/tau'
     */
    realtime_simu_franka_fr3_B.SFunction3_o1[i] =
      realtime_simu_franka_fr3_P.tau_Y0_d;

    /* SystemInitialize for S-Function (sdspstatfcns): '<S28>/Variance' incorporates:
     *  Outport: '<S28>/noise variance q'
     */
    realtime_simu_franka_fr3_B.noisevarianceq[i] =
      realtime_simu_franka_fr3_P.noisevarianceq_Y0;

    /* SystemInitialize for Outport: '<S7>/tau' */
    realtime_simu_franka_fr3_B.tau[i] = realtime_simu_franka_fr3_P.tau_Y0;
  }

  /* End of SystemInitialize for SubSystem: '<Root>/tau_subsystem' */
  /* End of SystemInitialize for SubSystem: '<S8>/Subsystem' */

  /* SystemInitialize for Outport: '<S7>/home run flag' */
  realtime_simu_franka_fr3_B.home_running =
    realtime_simu_franka_fr3_P.homerunflag_Y0;

  /* End of SystemInitialize for SubSystem: '<Root>/jointspace ctl subsys' */

  /* SystemInitialize for MATLAB Function: '<S2>/MATLAB Function' */
  realtime_simu_franka_fr3_DW.cnt_c = 1.0;
  realtime_simu_franka_fr3_DW.run_flag = 0.0;
}

/* Model terminate function */
void realtime_simu_franka_fr3_terminate(void)
{
  /* Terminate for S-Function (s_function_opti_robot_model_bus_fun): '<S9>/robot model s-function2' */
  /* Level2 S-Function Block: '<S9>/robot model s-function2' (s_function_opti_robot_model_bus_fun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[4];
    sfcnTerminate(rts);
  }

  /* Terminate for Enabled SubSystem: '<S5>/EKF' */

  /* Terminate for S-Function (s_function_opti_ekf_fun): '<S17>/Reduced System sfun casadi solve' */
  /* Level2 S-Function Block: '<S17>/Reduced System sfun casadi solve' (s_function_opti_ekf_fun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (s_function_opti_robot_model_bus_fun): '<S20>/robot model s-function2' */
  /* Level2 S-Function Block: '<S20>/robot model s-function2' (s_function_opti_robot_model_bus_fun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[1];
    sfcnTerminate(rts);
  }

  /* End of Terminate for SubSystem: '<S5>/EKF' */

  /* Terminate for Enabled SubSystem: '<Root>/tau_subsystem' */

  /* Terminate for S-Function (shm_reader_sfun): '<S10>/S-Function3' */
  /* Level2 S-Function Block: '<S10>/S-Function3' (shm_reader_sfun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[2];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (shm_writer_sfun): '<S10>/S-Function4' */
  /* Level2 S-Function Block: '<S10>/S-Function4' (shm_writer_sfun) */
  {
    SimStruct *rts = realtime_simu_franka_fr3_M->childSfunctions[3];
    sfcnTerminate(rts);
  }

  /* End of Terminate for SubSystem: '<Root>/tau_subsystem' */

  /* Terminate for S-Function (apply_control): '<S4>/Apply Control' */
  {
    /* S-Function Block: <S4>/Apply Control */
  }
}
