/*
 * cartesian_impedance_control.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "cartesian_impedance_control".
 *
 * Model version              : 8.21
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Thu Sep 26 15:01:04 2024
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "cartesian_impedance_control.h"
#include "rtwtypes.h"
#include <math.h>
#include "cartesian_impedance_control_private.h"
#include <cstring>

extern "C"
{

#include "rt_nonfinite.h"

}

const real_T cartesian_impedance_cont_period = 0.001;
SimulinkPandaRobot simulinkPandaRobot_1721602;

/* Block signals (default storage) */
B_cartesian_impedance_control_T cartesian_impedance_control_B;

/* Block states (default storage) */
DW_cartesian_impedance_control_T cartesian_impedance_control_DW;

/* Real-time model */
RT_MODEL_cartesian_impedance_control_T cartesian_impedance_control_M_ =
  RT_MODEL_cartesian_impedance_control_T();
RT_MODEL_cartesian_impedance_control_T *const cartesian_impedance_control_M =
  &cartesian_impedance_control_M_;

/* Model step function */
void cartesian_impedance_control_step(void)
{
  {
    real_T rtb_Product4[9];
    real_T rtb_Subtract[9];
    real_T tmp_1[7];
    real_T rtb_cartesiandampingdesign[6];
    real_T tmp[6];
    real_T tmp_0[6];
    real_T S;
    real_T qy;
    real_T rtb_Clock;
    int32_T i;
    int32_T rtb_Subtract_tmp;
    int32_T samplesRead;
    char_T *sErr;

    /* Reset subsysRan breadcrumbs */
    srClearBC(cartesian_impedance_control_DW.UDPsendtoc_SubsysRanBC);

    /* S-Function (get_robot_state): '<S4>/Get Robot State2' */
    {
      // Wait for the control thread signal
      if ((bool)cartesian_impedance_control_DW.GetRobotState2_DWORK1 &&
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
          &cartesian_impedance_control_B.GetRobotState2_o1[0]);
        simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 1,
          &cartesian_impedance_control_B.GetRobotState2_o2[0]);
        simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 2,
          &cartesian_impedance_control_B.GetRobotState2_o3[0]);
      }
    }

    /* Clock: '<S4>/Clock' */
    rtb_Clock = cartesian_impedance_control_M->Timing.t[0];
    for (i = 0; i < 7; i++) {
      /* Switch: '<S4>/Switch' */
      if (rtb_Clock > cartesian_impedance_control_P.Switch_Threshold) {
        /* Switch: '<S4>/Switch' */
        cartesian_impedance_control_B.Switch[i] =
          cartesian_impedance_control_B.GetRobotState2_o1[i];
      } else {
        /* Switch: '<S4>/Switch' incorporates:
         *  Constant: '<S4>/Constant'
         */
        cartesian_impedance_control_B.Switch[i] =
          cartesian_impedance_control_P.q_init[i];
      }

      /* End of Switch: '<S4>/Switch' */
    }

    /* S-Function (sdspFromNetwork): '<Root>/UDP Receive from c 1' */
    sErr = GetErrorBuffer
      (&cartesian_impedance_control_DW.UDPReceivefromc1_NetworkLib[0U]);
    samplesRead = 31;
    LibOutputs_Network
      (&cartesian_impedance_control_DW.UDPReceivefromc1_NetworkLib[0U],
       &cartesian_impedance_control_B.u_opt[0U], &samplesRead);
    if (*sErr != 0) {
      rtmSetErrorStatus(cartesian_impedance_control_M, sErr);
      rtmSetStopRequested(cartesian_impedance_control_M, 1);
    }

    /* S-Function (sdspFromNetwork): '<Root>/UDP Receive from c 1' */
    cartesian_impedance_control_B.UDPReceivefromc1_o2 = static_cast<uint16_T>
      (samplesRead);

    /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
     *  Memory: '<Root>/Memory'
     */
    if (cartesian_impedance_control_B.UDPReceivefromc1_o2 > 0) {
      cartesian_impedance_control_B.cnt_o =
        cartesian_impedance_control_DW.Memory_PreviousInput + 1.0;
    } else {
      cartesian_impedance_control_B.cnt_o =
        cartesian_impedance_control_DW.Memory_PreviousInput;
    }

    cartesian_impedance_control_B.enable_out = 1.0;

    /* End of MATLAB Function: '<Root>/MATLAB Function1' */
    for (i = 0; i < 6; i++) {
      /* Constant: '<Root>/Constant3' */
      cartesian_impedance_control_B.Constant3[i] =
        cartesian_impedance_control_P.Constant3_Value[i];
    }

    /* S-Function (s_function_opti_sys_fun_qpp_aba): '<Root>/S-Function1' */

    /* Level2 S-Function Block: '<Root>/S-Function1' (s_function_opti_sys_fun_qpp_aba) */
    {
      SimStruct *rts = cartesian_impedance_control_M->childSfunctions[0];
      sfcnOutputs(rts,0);
    }

    /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
     *  Memory: '<Root>/Memory1'
     *  Memory: '<Root>/Memory2'
     *  S-Function (sdspFromNetwork): '<Root>/UDP Receive from c 1'
     */
    if (cartesian_impedance_control_B.u_opt[0] ==
        cartesian_impedance_control_DW.Memory1_PreviousInput) {
      cartesian_impedance_control_B.data_cnt_o = 1.0;
      cartesian_impedance_control_B.init_cnt_o =
        cartesian_impedance_control_DW.Memory1_PreviousInput + 1.0;
    } else {
      if (cartesian_impedance_control_DW.Memory2_PreviousInput > 4.0) {
        cartesian_impedance_control_B.data_cnt_o = 4.0;
      } else {
        cartesian_impedance_control_B.data_cnt_o =
          cartesian_impedance_control_DW.Memory2_PreviousInput + 1.0;
      }

      cartesian_impedance_control_B.init_cnt_o =
        cartesian_impedance_control_DW.Memory1_PreviousInput;
    }

    cartesian_impedance_control_B.send_cnt =
      cartesian_impedance_control_B.u_opt[0];

    /* End of MATLAB Function: '<Root>/MATLAB Function' */

    /* Outputs for Enabled SubSystem: '<Root>/UDP send to c' incorporates:
     *  EnablePort: '<S5>/Enable'
     */
    cartesian_impedance_control_DW.UDPsendtoc_MODE =
      (cartesian_impedance_control_B.enable_out > 0.0);
    if (cartesian_impedance_control_DW.UDPsendtoc_MODE) {
      /* SignalConversion generated from: '<S5>/UDP Send' */
      cartesian_impedance_control_B.TmpSignalConversionAtUDPSendInp[0] =
        cartesian_impedance_control_B.send_cnt;
      for (i = 0; i < 7; i++) {
        cartesian_impedance_control_B.TmpSignalConversionAtUDPSendInp[i + 1] =
          cartesian_impedance_control_B.Switch[i];
        cartesian_impedance_control_B.TmpSignalConversionAtUDPSendInp[i + 8] =
          cartesian_impedance_control_B.GetRobotState2_o2[i];
      }

      /* End of SignalConversion generated from: '<S5>/UDP Send' */
      srUpdateBC(cartesian_impedance_control_DW.UDPsendtoc_SubsysRanBC);
    }

    /* End of Outputs for SubSystem: '<Root>/UDP send to c' */

    /* S-Function (get_model): '<S1>/Get Model' */
    {
      if (simulinkPandaRobot_1721602.getControlThreadHasBeenSpawned()) {
        // Wait for the control thread signal
        if ((bool)cartesian_impedance_control_DW.GetModel_DWORK1) {
          simulinkPandaRobot_1721602.waitForControlThreadStep();
        }

        // If control loop threw exeption terminate execution
        simulinkPandaRobot_1721602.checkIfAndHandleException();

        // robot pose
        simulinkPandaRobot_1721602.copyRobotPose
          (&cartesian_impedance_control_B.GetModel_o1[0]);

        // bodyJacobian
        simulinkPandaRobot_1721602.copyBodyJacobian
          (&cartesian_impedance_control_B.GetModel_o2[0]);

        // zeroJacobian
        simulinkPandaRobot_1721602.copyZeroJacobian
          (&cartesian_impedance_control_B.GetModel_o3[0]);

        // mass
        simulinkPandaRobot_1721602.copyMass
          (&cartesian_impedance_control_B.GetModel_o4[0]);

        // coriolis
        simulinkPandaRobot_1721602.copyCoriolis
          (&cartesian_impedance_control_B.coriolis[0]);

        // gravity
        simulinkPandaRobot_1721602.copyGravity
          (&cartesian_impedance_control_B.GetModel_o6[0]);
      }
    }

    /* S-Function (get_robot_state): '<Root>/Get Robot State' */
    {
      // Wait for the control thread signal
      if ((bool)cartesian_impedance_control_DW.GetRobotState_DWORK1 &&
          simulinkPandaRobot_1721602.getControlThreadHasBeenSpawned()) {
        simulinkPandaRobot_1721602.waitForControlThreadStep();
      }

      // If control loop threw exeption terminate execution
      simulinkPandaRobot_1721602.checkIfAndHandleException();
      if (!simulinkPandaRobot_1721602.getCurrentlyInFirstControlStep()) {
        std::vector<std::string> output_signals;
        signalsStringToSignalsVector(output_signals,"O_T_EE dq");

        //copy outputs
        simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 0,
          &cartesian_impedance_control_B.GetRobotState_o1[0]);
        simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 1,
          &cartesian_impedance_control_B.GetRobotState_o2[0]);
      }
    }

    /* S-Function (get_initial_robot_state): '<Root>/Get Initial Robot State' */
    {
      // Wait for the control thread signal
      if ((bool)cartesian_impedance_control_DW.GetInitialRobotState_DWORK2 &&
          simulinkPandaRobot_1721602.getControlThreadHasBeenSpawned()) {
        simulinkPandaRobot_1721602.waitForControlThreadStep();
      }

      // If control loop threw exeption terminate execution
      simulinkPandaRobot_1721602.checkIfAndHandleException();
      if (!simulinkPandaRobot_1721602.getCurrentlyInFirstControlStep() && !(bool)
          cartesian_impedance_control_DW.GetInitialRobotState_DWORK1) {
        std::vector<std::string> output_signals;
        signalsStringToSignalsVector(output_signals,"O_T_EE");

        //copy outputs
        simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 0,
          &cartesian_impedance_control_B.GetInitialRobotState[0]);
        cartesian_impedance_control_DW.GetInitialRobotState_DWORK1 = 1;
      }
    }

    for (samplesRead = 0; samplesRead < 3; samplesRead++) {
      int32_T rtb_Subtract_tmp_0;

      /* Selector: '<S1>/Select rotation matrix' incorporates:
       *  Math: '<S1>/Math Function1'
       *  Product: '<S1>/Product4'
       *  S-Function (get_robot_state): '<Root>/Get Robot State'
       *  Sum: '<S1>/Subtract'
       */
      rtb_Subtract_tmp = samplesRead << 2;
      rtb_Subtract[3 * samplesRead] =
        cartesian_impedance_control_B.GetRobotState_o1[rtb_Subtract_tmp];
      i = 3 * samplesRead + 1;
      rtb_Subtract[i] =
        cartesian_impedance_control_B.GetRobotState_o1[rtb_Subtract_tmp + 1];
      rtb_Subtract_tmp_0 = 3 * samplesRead + 2;
      rtb_Subtract[rtb_Subtract_tmp_0] =
        cartesian_impedance_control_B.GetRobotState_o1[rtb_Subtract_tmp + 2];

      /* Product: '<S1>/Product4' incorporates:
       *  Math: '<S1>/Math Function1'
       *  S-Function (get_initial_robot_state): '<Root>/Get Initial Robot State'
       *  Selector: '<S1>/Select rotation matrix1'
       *  Sum: '<S1>/Subtract'
       */
      for (rtb_Subtract_tmp = 0; rtb_Subtract_tmp < 3; rtb_Subtract_tmp++) {
        int32_T rtb_Product4_tmp;
        int32_T rtb_Product4_tmp_0;
        rtb_Product4_tmp = 3 * rtb_Subtract_tmp + samplesRead;
        rtb_Product4[rtb_Product4_tmp] = 0.0;

        /* Selector: '<S1>/Select rotation matrix1' incorporates:
         *  Math: '<S1>/Math Function1'
         *  Product: '<S1>/Product4'
         */
        rtb_Product4_tmp_0 = rtb_Subtract_tmp << 2;
        rtb_Product4[rtb_Product4_tmp] += rtb_Subtract[3 * samplesRead] *
          cartesian_impedance_control_B.GetInitialRobotState[rtb_Product4_tmp_0];
        rtb_Product4[rtb_Product4_tmp] +=
          cartesian_impedance_control_B.GetInitialRobotState[rtb_Product4_tmp_0
          + 1] * rtb_Subtract[i];
        rtb_Product4[rtb_Product4_tmp] +=
          cartesian_impedance_control_B.GetInitialRobotState[rtb_Product4_tmp_0
          + 2] * rtb_Subtract[rtb_Subtract_tmp_0];
      }
    }

    /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
     *  Product: '<S1>/Product4'
     */
    rtb_Clock = (rtb_Product4[0] + rtb_Product4[4]) + rtb_Product4[8];
    if (rtb_Clock > 0.0) {
      S = sqrt(rtb_Clock + 1.0) * 2.0;
      rtb_Clock = (rtb_Product4[5] - rtb_Product4[7]) / S;
      qy = (rtb_Product4[6] - rtb_Product4[2]) / S;
      S = (rtb_Product4[1] - rtb_Product4[3]) / S;
    } else if ((rtb_Product4[0] > rtb_Product4[4]) && (rtb_Product4[0] >
                rtb_Product4[8])) {
      S = sqrt(((rtb_Product4[0] + 1.0) - rtb_Product4[4]) - rtb_Product4[8]) *
        2.0;
      rtb_Clock = 0.25 * S;
      qy = (rtb_Product4[1] + rtb_Product4[3]) / S;
      S = (rtb_Product4[2] + rtb_Product4[6]) / S;
    } else if (rtb_Product4[4] > rtb_Product4[8]) {
      S = sqrt(((rtb_Product4[4] + 1.0) - rtb_Product4[0]) - rtb_Product4[8]) *
        2.0;
      rtb_Clock = (rtb_Product4[1] + rtb_Product4[3]) / S;
      qy = 0.25 * S;
      S = (rtb_Product4[5] + rtb_Product4[7]) / S;
    } else {
      S = sqrt(((rtb_Product4[8] + 1.0) - rtb_Product4[0]) - rtb_Product4[4]) *
        2.0;
      rtb_Clock = (rtb_Product4[2] + rtb_Product4[6]) / S;
      qy = (rtb_Product4[5] + rtb_Product4[7]) / S;
      S *= 0.25;
    }

    for (samplesRead = 0; samplesRead < 3; samplesRead++) {
      /* Product: '<S1>/Product3' incorporates:
       *  MATLAB Function: '<S1>/MATLAB Function'
       *  Sum: '<S1>/Subtract'
       */
      rtb_cartesiandampingdesign[samplesRead + 3] = 0.0;
      rtb_cartesiandampingdesign[samplesRead + 3] += rtb_Subtract[samplesRead] *
        rtb_Clock;
      rtb_cartesiandampingdesign[samplesRead + 3] += rtb_Subtract[samplesRead +
        3] * qy;
      rtb_cartesiandampingdesign[samplesRead + 3] += rtb_Subtract[samplesRead +
        6] * S;

      /* Sum: '<S1>/Subtract2' incorporates:
       *  S-Function (get_initial_robot_state): '<Root>/Get Initial Robot State'
       *  S-Function (get_robot_state): '<Root>/Get Robot State'
       *  Selector: '<S1>/Select rotation matrix3'
       *  Selector: '<S1>/Select rotation matrix4'
       */
      rtb_cartesiandampingdesign[samplesRead] =
        cartesian_impedance_control_B.GetInitialRobotState[samplesRead + 12] -
        cartesian_impedance_control_B.GetRobotState_o1[samplesRead + 12];
    }

    for (samplesRead = 0; samplesRead < 6; samplesRead++) {
      /* Product: '<S1>/Product' incorporates:
       *  Constant: '<S1>/stiffness'
       *  Product: '<S1>/Product2'
       */
      tmp[samplesRead] = 0.0;
      for (rtb_Subtract_tmp = 0; rtb_Subtract_tmp < 6; rtb_Subtract_tmp++) {
        tmp[samplesRead] += cartesian_impedance_control_P.stiffness_Value[6 *
          rtb_Subtract_tmp + samplesRead] *
          rtb_cartesiandampingdesign[rtb_Subtract_tmp];
      }

      rtb_Subtract[samplesRead] = tmp[samplesRead];

      /* End of Product: '<S1>/Product' */

      /* Product: '<S1>/Product2' incorporates:
       *  S-Function (get_model): '<S1>/Get Model'
       */
      tmp_0[samplesRead] = 0.0;
      for (rtb_Subtract_tmp = 0; rtb_Subtract_tmp < 7; rtb_Subtract_tmp++) {
        tmp_0[samplesRead] += cartesian_impedance_control_B.GetModel_o3[6 *
          rtb_Subtract_tmp + samplesRead] *
          cartesian_impedance_control_B.GetRobotState_o2[rtb_Subtract_tmp];
      }
    }

    /* Sum: '<S1>/Subtract' incorporates:
     *  Constant: '<S1>/damping'
     *  Product: '<S1>/Product2'
     */
    for (samplesRead = 0; samplesRead < 6; samplesRead++) {
      rtb_Clock = 0.0;
      for (rtb_Subtract_tmp = 0; rtb_Subtract_tmp < 6; rtb_Subtract_tmp++) {
        rtb_Clock += cartesian_impedance_control_P.damping_Value[6 *
          rtb_Subtract_tmp + samplesRead] * tmp_0[rtb_Subtract_tmp];
      }

      rtb_Subtract[samplesRead] -= rtb_Clock;
    }

    /* End of Sum: '<S1>/Subtract' */

    /* RateLimiter: '<S4>/Rate Limiter' */
    rtb_Clock = cartesian_impedance_control_P.RateLimiter_RisingLim *
      cartesian_impedance_cont_period;
    qy = cartesian_impedance_control_P.RateLimiter_FallingLim *
      cartesian_impedance_cont_period;
    for (i = 0; i < 7; i++) {
      /* Product: '<S1>/Product1' incorporates:
       *  Math: '<S1>/Math Function'
       *  S-Function (get_model): '<S1>/Get Model'
       */
      tmp_1[i] = 0.0;
      for (samplesRead = 0; samplesRead < 6; samplesRead++) {
        tmp_1[i] += cartesian_impedance_control_B.GetModel_o3[6 * i +
          samplesRead] * rtb_Subtract[samplesRead];
      }

      /* Sum: '<S1>/Subtract1' incorporates:
       *  Product: '<S1>/Product1'
       */
      S = cartesian_impedance_control_B.coriolis[i] + tmp_1[i];

      /* RateLimiter: '<S4>/Rate Limiter' */
      cartesian_impedance_control_B.RateLimiter[i] = S -
        cartesian_impedance_control_DW.PrevY[i];
      if (cartesian_impedance_control_B.RateLimiter[i] > rtb_Clock) {
        cartesian_impedance_control_B.RateLimiter[i] =
          cartesian_impedance_control_DW.PrevY[i] + rtb_Clock;
      } else if (cartesian_impedance_control_B.RateLimiter[i] < qy) {
        cartesian_impedance_control_B.RateLimiter[i] =
          cartesian_impedance_control_DW.PrevY[i] + qy;
      } else {
        cartesian_impedance_control_B.RateLimiter[i] = S;
      }

      cartesian_impedance_control_DW.PrevY[i] =
        cartesian_impedance_control_B.RateLimiter[i];
    }

    /* S-Function (apply_control): '<S4>/Apply Control' */
    {
      /* S-Function Block: <S4>/Apply Control */
      if ((bool)cartesian_impedance_control_DW.ApplyControl_DWORK1) {
        // Wait for the control thread signal
        if ((bool)cartesian_impedance_control_DW.ApplyControl_DWORK2) {
          simulinkPandaRobot_1721602.waitForControlThreadStep();
        }

        // If control loop threw exeption terminate execution
        simulinkPandaRobot_1721602.checkIfAndHandleException();

        // copy inputs
        simulinkPandaRobot_1721602.copyInputSignal
          (&cartesian_impedance_control_B.RateLimiter[0], 0);

        // notify control thread that the inputs have been read
        simulinkPandaRobot_1721602.notifyControlThreadToContinue();
      } else if (!(bool)cartesian_impedance_control_DW.ApplyControl_DWORK1) {
        // Its the first time _step() function is called -->
        // Initialize according to settings parsed from the mask
        // and spawn control thread
        simulinkPandaRobot_1721602.applyRobotSettings();
        simulinkPandaRobot_1721602.spawnControlThread();
        cartesian_impedance_control_DW.ApplyControl_DWORK1 = 1;
      }
    }
  }

  /* Matfile logging */
  rt_UpdateTXYLogVars(cartesian_impedance_control_M->rtwLogInfo,
                      (cartesian_impedance_control_M->Timing.t));

  {
    char_T *sErr;

    /* Update for Memory: '<Root>/Memory' */
    cartesian_impedance_control_DW.Memory_PreviousInput =
      cartesian_impedance_control_B.cnt_o;

    /* Update for Memory: '<Root>/Memory1' */
    cartesian_impedance_control_DW.Memory1_PreviousInput =
      cartesian_impedance_control_B.init_cnt_o;

    /* Update for Memory: '<Root>/Memory2' */
    cartesian_impedance_control_DW.Memory2_PreviousInput =
      cartesian_impedance_control_B.data_cnt_o;

    /* Update for Enabled SubSystem: '<Root>/UDP send to c' incorporates:
     *  EnablePort: '<S5>/Enable'
     */
    if (cartesian_impedance_control_DW.UDPsendtoc_MODE) {
      /* Update for S-Function (sdspToNetwork): '<S5>/UDP Send' incorporates:
       *  SignalConversion generated from: '<S5>/UDP Send'
       */
      sErr = GetErrorBuffer(&cartesian_impedance_control_DW.UDPSend_NetworkLib
                            [0U]);
      LibUpdate_Network(&cartesian_impedance_control_DW.UDPSend_NetworkLib[0U],
                        &cartesian_impedance_control_B.TmpSignalConversionAtUDPSendInp
                        [0U], 15);
      if (*sErr != 0) {
        rtmSetErrorStatus(cartesian_impedance_control_M, sErr);
        rtmSetStopRequested(cartesian_impedance_control_M, 1);
      }

      /* End of Update for S-Function (sdspToNetwork): '<S5>/UDP Send' */
    }

    /* End of Update for SubSystem: '<Root>/UDP send to c' */
  }

  /* External mode */
  rtExtModeUploadCheckTrigger(2);

  {                                    /* Sample time: [0.0s, 0.0s] */
    rtExtModeUpload(0, (real_T)cartesian_impedance_control_M->Timing.t[0]);
  }

  {                                    /* Sample time: [0.001s, 0.0s] */
    rtExtModeUpload(1, (real_T)
                    (((cartesian_impedance_control_M->Timing.clockTick1+
                       cartesian_impedance_control_M->Timing.clockTickH1*
                       4294967296.0)) * 0.001));
  }

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.0s, 0.0s] */
    if ((rtmGetTFinal(cartesian_impedance_control_M)!=-1) &&
        !((rtmGetTFinal(cartesian_impedance_control_M)-
           cartesian_impedance_control_M->Timing.t[0]) >
          cartesian_impedance_control_M->Timing.t[0] * (DBL_EPSILON))) {
      rtmSetErrorStatus(cartesian_impedance_control_M, "Simulation finished");
    }

    if (rtmGetStopRequested(cartesian_impedance_control_M)) {
      rtmSetErrorStatus(cartesian_impedance_control_M, "Simulation finished");
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
  if (!(++cartesian_impedance_control_M->Timing.clockTick0)) {
    ++cartesian_impedance_control_M->Timing.clockTickH0;
  }

  cartesian_impedance_control_M->Timing.t[0] =
    cartesian_impedance_control_M->Timing.clockTick0 *
    cartesian_impedance_control_M->Timing.stepSize0 +
    cartesian_impedance_control_M->Timing.clockTickH0 *
    cartesian_impedance_control_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.001s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.001, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    cartesian_impedance_control_M->Timing.clockTick1++;
    if (!cartesian_impedance_control_M->Timing.clockTick1) {
      cartesian_impedance_control_M->Timing.clockTickH1++;
    }
  }
}

/* Model initialize function */
void cartesian_impedance_control_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&cartesian_impedance_control_M->solverInfo,
                          &cartesian_impedance_control_M->Timing.simTimeStep);
    rtsiSetTPtr(&cartesian_impedance_control_M->solverInfo, &rtmGetTPtr
                (cartesian_impedance_control_M));
    rtsiSetStepSizePtr(&cartesian_impedance_control_M->solverInfo,
                       &cartesian_impedance_control_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&cartesian_impedance_control_M->solverInfo,
                          (&rtmGetErrorStatus(cartesian_impedance_control_M)));
    rtsiSetRTModelPtr(&cartesian_impedance_control_M->solverInfo,
                      cartesian_impedance_control_M);
  }

  rtsiSetSimTimeStep(&cartesian_impedance_control_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&cartesian_impedance_control_M->solverInfo,
                    "FixedStepDiscrete");
  cartesian_impedance_control_M->solverInfoPtr =
    (&cartesian_impedance_control_M->solverInfo);

  /* Initialize timing info */
  {
    int_T *mdlTsMap =
      cartesian_impedance_control_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;

    /* polyspace +2 MISRA2012:D4.1 [Justified:Low] "cartesian_impedance_control_M points to
       static memory which is guaranteed to be non-NULL" */
    cartesian_impedance_control_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    cartesian_impedance_control_M->Timing.sampleTimes =
      (&cartesian_impedance_control_M->Timing.sampleTimesArray[0]);
    cartesian_impedance_control_M->Timing.offsetTimes =
      (&cartesian_impedance_control_M->Timing.offsetTimesArray[0]);

    /* task periods */
    cartesian_impedance_control_M->Timing.sampleTimes[0] = (0.0);
    cartesian_impedance_control_M->Timing.sampleTimes[1] = (0.001);

    /* task offsets */
    cartesian_impedance_control_M->Timing.offsetTimes[0] = (0.0);
    cartesian_impedance_control_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(cartesian_impedance_control_M,
             &cartesian_impedance_control_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = cartesian_impedance_control_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    cartesian_impedance_control_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(cartesian_impedance_control_M, -1);
  cartesian_impedance_control_M->Timing.stepSize0 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    cartesian_impedance_control_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(cartesian_impedance_control_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(cartesian_impedance_control_M->rtwLogInfo, (NULL));
    rtliSetLogT(cartesian_impedance_control_M->rtwLogInfo, "tout");
    rtliSetLogX(cartesian_impedance_control_M->rtwLogInfo, "");
    rtliSetLogXFinal(cartesian_impedance_control_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(cartesian_impedance_control_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(cartesian_impedance_control_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(cartesian_impedance_control_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(cartesian_impedance_control_M->rtwLogInfo, 1);
    rtliSetLogY(cartesian_impedance_control_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(cartesian_impedance_control_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(cartesian_impedance_control_M->rtwLogInfo, (NULL));
  }

  /* External mode info */
  cartesian_impedance_control_M->Sizes.checksums[0] = (2347485286U);
  cartesian_impedance_control_M->Sizes.checksums[1] = (3158364773U);
  cartesian_impedance_control_M->Sizes.checksums[2] = (4061902251U);
  cartesian_impedance_control_M->Sizes.checksums[3] = (3987505037U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[5];
    cartesian_impedance_control_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = (sysRanDType *)
      &cartesian_impedance_control_DW.UDPsendtoc_SubsysRanBC;
    rteiSetModelMappingInfoPtr(cartesian_impedance_control_M->extModeInfo,
      &cartesian_impedance_control_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(cartesian_impedance_control_M->extModeInfo,
                        cartesian_impedance_control_M->Sizes.checksums);
    rteiSetTPtr(cartesian_impedance_control_M->extModeInfo, rtmGetTPtr
                (cartesian_impedance_control_M));
  }

  cartesian_impedance_control_M->solverInfoPtr =
    (&cartesian_impedance_control_M->solverInfo);
  cartesian_impedance_control_M->Timing.stepSize = (0.001);
  rtsiSetFixedStepSize(&cartesian_impedance_control_M->solverInfo, 0.001);
  rtsiSetSolverMode(&cartesian_impedance_control_M->solverInfo,
                    SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  (void) memset((static_cast<void *>(&cartesian_impedance_control_B)), 0,
                sizeof(B_cartesian_impedance_control_T));

  /* states (dwork) */
  (void) memset(static_cast<void *>(&cartesian_impedance_control_DW), 0,
                sizeof(DW_cartesian_impedance_control_T));

  /* block instance data */
  {
    {
      simulinkPandaRobot_1721602 = SimulinkPandaRobot( "172.16.0.2",
        0,
        0,
        0,
        cartesian_impedance_control_P.ApplyControl_P1,
        cartesian_impedance_control_P.ApplyControl_P2,
        cartesian_impedance_control_P.ApplyControl_P3,
        cartesian_impedance_control_P.ApplyControl_P4,
        cartesian_impedance_control_P.ApplyControl_P5,
        1,
        cartesian_impedance_control_P.ApplyControl_P6,
        1);
    }
  }

  /* child S-Function registration */
  {
    RTWSfcnInfo *sfcnInfo =
      &cartesian_impedance_control_M->NonInlinedSFcns.sfcnInfo;
    cartesian_impedance_control_M->sfcnInfo = (sfcnInfo);
    rtssSetErrorStatusPtr(sfcnInfo, (&rtmGetErrorStatus
      (cartesian_impedance_control_M)));
    cartesian_impedance_control_M->Sizes.numSampTimes = (2);
    rtssSetNumRootSampTimesPtr(sfcnInfo,
      &cartesian_impedance_control_M->Sizes.numSampTimes);
    cartesian_impedance_control_M->NonInlinedSFcns.taskTimePtrs[0] =
      &(rtmGetTPtr(cartesian_impedance_control_M)[0]);
    cartesian_impedance_control_M->NonInlinedSFcns.taskTimePtrs[1] =
      &(rtmGetTPtr(cartesian_impedance_control_M)[1]);
    rtssSetTPtrPtr(sfcnInfo,
                   cartesian_impedance_control_M->NonInlinedSFcns.taskTimePtrs);
    rtssSetTStartPtr(sfcnInfo, &rtmGetTStart(cartesian_impedance_control_M));
    rtssSetTFinalPtr(sfcnInfo, &rtmGetTFinal(cartesian_impedance_control_M));
    rtssSetTimeOfLastOutputPtr(sfcnInfo, &rtmGetTimeOfLastOutput
      (cartesian_impedance_control_M));
    rtssSetStepSizePtr(sfcnInfo, &cartesian_impedance_control_M->Timing.stepSize);
    rtssSetStopRequestedPtr(sfcnInfo, &rtmGetStopRequested
      (cartesian_impedance_control_M));
    rtssSetDerivCacheNeedsResetPtr(sfcnInfo,
      &cartesian_impedance_control_M->derivCacheNeedsReset);
    rtssSetZCCacheNeedsResetPtr(sfcnInfo,
      &cartesian_impedance_control_M->zCCacheNeedsReset);
    rtssSetContTimeOutputInconsistentWithStateAtMajorStepPtr(sfcnInfo,
      &cartesian_impedance_control_M->CTOutputIncnstWithState);
    rtssSetSampleHitsPtr(sfcnInfo,
                         &cartesian_impedance_control_M->Timing.sampleHits);
    rtssSetPerTaskSampleHitsPtr(sfcnInfo,
      &cartesian_impedance_control_M->Timing.perTaskSampleHits);
    rtssSetSimModePtr(sfcnInfo, &cartesian_impedance_control_M->simMode);
    rtssSetSolverInfoPtr(sfcnInfo, &cartesian_impedance_control_M->solverInfoPtr);
  }

  cartesian_impedance_control_M->Sizes.numSFcns = (1);

  /* register each child */
  {
    (void) memset(static_cast<void *>
                  (&cartesian_impedance_control_M->NonInlinedSFcns.childSFunctions
                   [0]), 0,
                  1*sizeof(SimStruct));
    cartesian_impedance_control_M->childSfunctions =
      (&cartesian_impedance_control_M->NonInlinedSFcns.childSFunctionPtrs[0]);
    cartesian_impedance_control_M->childSfunctions[0] =
      (&cartesian_impedance_control_M->NonInlinedSFcns.childSFunctions[0]);

    /* Level2 S-Function Block: cartesian_impedance_control/<Root>/S-Function1 (s_function_opti_sys_fun_qpp_aba) */
    {
      SimStruct *rts = cartesian_impedance_control_M->childSfunctions[0];

      /* timing info */
      time_T *sfcnPeriod =
        cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.sfcnPeriod;
      time_T *sfcnOffset =
        cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.sfcnOffset;
      int_T *sfcnTsMap =
        cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.sfcnTsMap;
      (void) memset(static_cast<void*>(sfcnPeriod), 0,
                    sizeof(time_T)*1);
      (void) memset(static_cast<void*>(sfcnOffset), 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &cartesian_impedance_control_M->NonInlinedSFcns.blkInfo2
                         [0]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &cartesian_impedance_control_M->NonInlinedSFcns.inputOutputPortInfo2[0]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, cartesian_impedance_control_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &cartesian_impedance_control_M->NonInlinedSFcns.methods2
                           [0]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &cartesian_impedance_control_M->NonInlinedSFcns.methods3
                           [0]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &cartesian_impedance_control_M->NonInlinedSFcns.methods4
                           [0]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &cartesian_impedance_control_M->NonInlinedSFcns.statesInfo2
                         [0]);
        ssSetPeriodicStatesInfo(rts,
          &cartesian_impedance_control_M->NonInlinedSFcns.periodicStatesInfo[0]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 3);
        ssSetPortInfoForInputs(rts,
          &cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.inputPortInfo[0]);
        ssSetPortInfoForInputs(rts,
          &cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.inputPortInfo[0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        ssSetInputPortUnit(rts, 1, 0);
        ssSetInputPortUnit(rts, 2, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);
        ssSetInputPortIsContinuousQuantity(rts, 1, 0);
        ssSetInputPortIsContinuousQuantity(rts, 2, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.iDims0;
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0, &cartesian_impedance_control_B.Switch[0]);
          dimensions[0] = 6;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetInputPortNumDimensions(rts, 0, 2);
          ssSetInputPortWidthAsInt(rts, 0, 6);
        }

        /* port 1 */
        {
          int_T *dimensions = (int_T *)
            &cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.iDims1;
          ssSetInputPortRequiredContiguous(rts, 1, 1);
          ssSetInputPortSignal(rts, 1,
                               &cartesian_impedance_control_B.GetRobotState2_o2
                               [0]);
          dimensions[0] = 6;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 1, dimensions);
          _ssSetInputPortNumDimensions(rts, 1, 2);
          ssSetInputPortWidthAsInt(rts, 1, 6);
        }

        /* port 2 */
        {
          int_T *dimensions = (int_T *)
            &cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.iDims2;
          ssSetInputPortRequiredContiguous(rts, 2, 1);
          ssSetInputPortSignal(rts, 2, cartesian_impedance_control_B.Constant3);
          dimensions[0] = 6;
          dimensions[1] = 1;
          _ssSetInputPortDimensionsPtrAsInt(rts, 2, dimensions);
          _ssSetInputPortNumDimensions(rts, 2, 2);
          ssSetInputPortWidthAsInt(rts, 2, 6);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.outputPortInfo[0]);
        ssSetPortInfoForOutputs(rts,
          &cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);
        _ssSetPortInfo2ForOutputUnits(rts,
          &cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.outputPortUnits
          [0]);
        ssSetOutputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          int_T *dimensions = (int_T *)
            &cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.oDims0;
          dimensions[0] = 6;
          dimensions[1] = 1;
          _ssSetOutputPortDimensionsPtrAsInt(rts, 0, dimensions);
          _ssSetOutputPortNumDimensions(rts, 0, 2);
          ssSetOutputPortWidthAsInt(rts, 0, 6);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            cartesian_impedance_control_B.SFunction1));
        }
      }

      /* path info */
      ssSetModelName(rts, "S-Function1");
      ssSetPath(rts, "cartesian_impedance_control/S-Function1");
      ssSetRTModel(rts,cartesian_impedance_control_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetPWork(rts, (void **)
                 &cartesian_impedance_control_DW.SFunction1_PWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &cartesian_impedance_control_M->NonInlinedSFcns.Sfcn0.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        ssSetNumDWorkAsInt(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 4);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &cartesian_impedance_control_DW.SFunction1_PWORK[0]);
      }

      /* registration */
      s_function_opti_sys_fun_qpp_aba(rts);
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
      _ssSetInputPortConnected(rts, 2, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
      ssSetInputPortBufferDstPort(rts, 2, -1);
    }
  }

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(cartesian_impedance_control_M->rtwLogInfo,
    0.0, rtmGetTFinal(cartesian_impedance_control_M),
    cartesian_impedance_control_M->Timing.stepSize0, (&rtmGetErrorStatus
    (cartesian_impedance_control_M)));

  {
    char_T *sErr;

    /* Start for S-Function (get_robot_state): '<S4>/Get Robot State2' */
    {
      cartesian_impedance_control_DW.GetRobotState2_DWORK1 = (double)
        simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
    }

    /* Start for S-Function (sdspFromNetwork): '<Root>/UDP Receive from c 1' */
    sErr = GetErrorBuffer
      (&cartesian_impedance_control_DW.UDPReceivefromc1_NetworkLib[0U]);
    CreateUDPInterface
      (&cartesian_impedance_control_DW.UDPReceivefromc1_NetworkLib[0U]);
    if (*sErr == 0) {
      LibCreate_Network
        (&cartesian_impedance_control_DW.UDPReceivefromc1_NetworkLib[0U], 0,
         "0.0.0.0", cartesian_impedance_control_P.UDPReceivefromc1_localPort,
         "127.0.0.1", -1, 248, 8, 0);
    }

    if (*sErr == 0) {
      LibStart(&cartesian_impedance_control_DW.UDPReceivefromc1_NetworkLib[0U]);
    }

    if (*sErr != 0) {
      DestroyUDPInterface
        (&cartesian_impedance_control_DW.UDPReceivefromc1_NetworkLib[0U]);
      if (*sErr != 0) {
        rtmSetErrorStatus(cartesian_impedance_control_M, sErr);
        rtmSetStopRequested(cartesian_impedance_control_M, 1);
      }
    }

    /* End of Start for S-Function (sdspFromNetwork): '<Root>/UDP Receive from c 1' */
    for (int32_T i = 0; i < 6; i++) {
      /* Start for Constant: '<Root>/Constant3' */
      cartesian_impedance_control_B.Constant3[i] =
        cartesian_impedance_control_P.Constant3_Value[i];
    }

    /* Start for Enabled SubSystem: '<Root>/UDP send to c' */
    cartesian_impedance_control_DW.UDPsendtoc_MODE = false;

    /* Start for S-Function (sdspToNetwork): '<S5>/UDP Send' */
    sErr = GetErrorBuffer(&cartesian_impedance_control_DW.UDPSend_NetworkLib[0U]);
    CreateUDPInterface(&cartesian_impedance_control_DW.UDPSend_NetworkLib[0U]);
    if (*sErr == 0) {
      LibCreate_Network(&cartesian_impedance_control_DW.UDPSend_NetworkLib[0U],
                        1, "0.0.0.0", -1, "127.0.0.1",
                        cartesian_impedance_control_P.UDPSend_remotePort, 104, 8,
                        0);
    }

    if (*sErr == 0) {
      LibStart(&cartesian_impedance_control_DW.UDPSend_NetworkLib[0U]);
    }

    if (*sErr != 0) {
      DestroyUDPInterface(&cartesian_impedance_control_DW.UDPSend_NetworkLib[0U]);
      if (*sErr != 0) {
        rtmSetErrorStatus(cartesian_impedance_control_M, sErr);
        rtmSetStopRequested(cartesian_impedance_control_M, 1);
      }
    }

    /* End of Start for S-Function (sdspToNetwork): '<S5>/UDP Send' */
    /* End of Start for SubSystem: '<Root>/UDP send to c' */

    /* Start for S-Function (get_model): '<S1>/Get Model' */
    {
      cartesian_impedance_control_DW.GetModel_DWORK1 = (double)
        simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
    }

    /* Start for S-Function (get_robot_state): '<Root>/Get Robot State' */
    {
      cartesian_impedance_control_DW.GetRobotState_DWORK1 = (double)
        simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
    }

    /* Start for S-Function (get_initial_robot_state): '<Root>/Get Initial Robot State' */
    {
      cartesian_impedance_control_DW.GetInitialRobotState_DWORK1 = 0;
      cartesian_impedance_control_DW.GetInitialRobotState_DWORK2 = (double)
        simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
    }

    /* Start for S-Function (apply_control): '<S4>/Apply Control' */
    {
      //Flag for performing initialization in first run of main _step();
      cartesian_impedance_control_DW.ApplyControl_DWORK1 = 0;
      cartesian_impedance_control_DW.ApplyControl_DWORK2 = (double)
        simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
    }
  }

  /* InitializeConditions for Memory: '<Root>/Memory' */
  cartesian_impedance_control_DW.Memory_PreviousInput =
    cartesian_impedance_control_P.Memory_InitialCondition;

  /* InitializeConditions for Memory: '<Root>/Memory1' */
  cartesian_impedance_control_DW.Memory1_PreviousInput =
    cartesian_impedance_control_P.Memory1_InitialCondition;

  /* InitializeConditions for Memory: '<Root>/Memory2' */
  cartesian_impedance_control_DW.Memory2_PreviousInput =
    cartesian_impedance_control_P.Memory2_InitialCondition;

  /* InitializeConditions for RateLimiter: '<S4>/Rate Limiter' */
  for (int32_T i = 0; i < 7; i++) {
    cartesian_impedance_control_DW.PrevY[i] =
      cartesian_impedance_control_P.RateLimiter_IC;
  }

  /* End of InitializeConditions for RateLimiter: '<S4>/Rate Limiter' */
}

/* Model terminate function */
void cartesian_impedance_control_terminate(void)
{
  char_T *sErr;

  /* Terminate for S-Function (sdspFromNetwork): '<Root>/UDP Receive from c 1' */
  sErr = GetErrorBuffer
    (&cartesian_impedance_control_DW.UDPReceivefromc1_NetworkLib[0U]);
  LibTerminate(&cartesian_impedance_control_DW.UDPReceivefromc1_NetworkLib[0U]);
  if (*sErr != 0) {
    rtmSetErrorStatus(cartesian_impedance_control_M, sErr);
    rtmSetStopRequested(cartesian_impedance_control_M, 1);
  }

  LibDestroy(&cartesian_impedance_control_DW.UDPReceivefromc1_NetworkLib[0U], 0);
  DestroyUDPInterface
    (&cartesian_impedance_control_DW.UDPReceivefromc1_NetworkLib[0U]);

  /* End of Terminate for S-Function (sdspFromNetwork): '<Root>/UDP Receive from c 1' */

  /* Terminate for S-Function (s_function_opti_sys_fun_qpp_aba): '<Root>/S-Function1' */
  /* Level2 S-Function Block: '<Root>/S-Function1' (s_function_opti_sys_fun_qpp_aba) */
  {
    SimStruct *rts = cartesian_impedance_control_M->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* Terminate for Enabled SubSystem: '<Root>/UDP send to c' */
  /* Terminate for S-Function (sdspToNetwork): '<S5>/UDP Send' */
  sErr = GetErrorBuffer(&cartesian_impedance_control_DW.UDPSend_NetworkLib[0U]);
  LibTerminate(&cartesian_impedance_control_DW.UDPSend_NetworkLib[0U]);
  if (*sErr != 0) {
    rtmSetErrorStatus(cartesian_impedance_control_M, sErr);
    rtmSetStopRequested(cartesian_impedance_control_M, 1);
  }

  LibDestroy(&cartesian_impedance_control_DW.UDPSend_NetworkLib[0U], 1);
  DestroyUDPInterface(&cartesian_impedance_control_DW.UDPSend_NetworkLib[0U]);

  /* End of Terminate for S-Function (sdspToNetwork): '<S5>/UDP Send' */
  /* End of Terminate for SubSystem: '<Root>/UDP send to c' */
  /* Terminate for S-Function (apply_control): '<S4>/Apply Control' */
  {
    /* S-Function Block: <S4>/Apply Control */
  }
}
