/*
 * cartesian_impedance_control.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "cartesian_impedance_control".
 *
 * Model version              : 8.2
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Thu Sep 26 11:23:31 2024
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
DW_cartesian_impedance_contro_T cartesian_impedance_control_DW;

/* Real-time model */
RT_MODEL_cartesian_impedance__T cartesian_impedance_control_M_ =
  RT_MODEL_cartesian_impedance__T();
RT_MODEL_cartesian_impedance__T *const cartesian_impedance_control_M =
  &cartesian_impedance_control_M_;

/* Model step function */
void cartesian_impedance_control_step(void)
{
  real_T rtb_Product4[9];
  real_T rtb_Subtract[9];
  real_T tmp_1[7];
  real_T rtb_cartesiandampingdesign[6];
  real_T tmp[6];
  real_T tmp_0[6];
  real_T S;
  real_T qy;
  real_T tr;
  int32_T rtb_Subtract_tmp;
  int32_T rtb_Subtract_tmp_0;

  /* S-Function (get_robot_state): '<S2>/Get Robot State2' */
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

  for (int32_T i = 0; i < 3; i++) {
    int32_T rtb_Subtract_tmp_1;

    /* Selector: '<S1>/Select rotation matrix' incorporates:
     *  Math: '<S1>/Math Function1'
     *  Product: '<S1>/Product4'
     *  S-Function (get_robot_state): '<Root>/Get Robot State'
     *  Sum: '<S1>/Subtract'
     */
    rtb_Subtract_tmp = i << 2;
    rtb_Subtract[3 * i] =
      cartesian_impedance_control_B.GetRobotState_o1[rtb_Subtract_tmp];
    rtb_Subtract_tmp_0 = 3 * i + 1;
    rtb_Subtract[rtb_Subtract_tmp_0] =
      cartesian_impedance_control_B.GetRobotState_o1[rtb_Subtract_tmp + 1];
    rtb_Subtract_tmp_1 = 3 * i + 2;
    rtb_Subtract[rtb_Subtract_tmp_1] =
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
      rtb_Product4_tmp = 3 * rtb_Subtract_tmp + i;
      rtb_Product4[rtb_Product4_tmp] = 0.0;

      /* Selector: '<S1>/Select rotation matrix1' incorporates:
       *  Math: '<S1>/Math Function1'
       *  Product: '<S1>/Product4'
       */
      rtb_Product4_tmp_0 = rtb_Subtract_tmp << 2;
      rtb_Product4[rtb_Product4_tmp] += rtb_Subtract[3 * i] *
        cartesian_impedance_control_B.GetInitialRobotState[rtb_Product4_tmp_0];
      rtb_Product4[rtb_Product4_tmp] +=
        cartesian_impedance_control_B.GetInitialRobotState[rtb_Product4_tmp_0 +
        1] * rtb_Subtract[rtb_Subtract_tmp_0];
      rtb_Product4[rtb_Product4_tmp] +=
        cartesian_impedance_control_B.GetInitialRobotState[rtb_Product4_tmp_0 +
        2] * rtb_Subtract[rtb_Subtract_tmp_1];
    }
  }

  /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
   *  Product: '<S1>/Product4'
   */
  tr = (rtb_Product4[0] + rtb_Product4[4]) + rtb_Product4[8];
  if (tr > 0.0) {
    S = sqrt(tr + 1.0) * 2.0;
    tr = (rtb_Product4[5] - rtb_Product4[7]) / S;
    qy = (rtb_Product4[6] - rtb_Product4[2]) / S;
    S = (rtb_Product4[1] - rtb_Product4[3]) / S;
  } else if ((rtb_Product4[0] > rtb_Product4[4]) && (rtb_Product4[0] >
              rtb_Product4[8])) {
    S = sqrt(((rtb_Product4[0] + 1.0) - rtb_Product4[4]) - rtb_Product4[8]) *
      2.0;
    tr = 0.25 * S;
    qy = (rtb_Product4[1] + rtb_Product4[3]) / S;
    S = (rtb_Product4[2] + rtb_Product4[6]) / S;
  } else if (rtb_Product4[4] > rtb_Product4[8]) {
    S = sqrt(((rtb_Product4[4] + 1.0) - rtb_Product4[0]) - rtb_Product4[8]) *
      2.0;
    tr = (rtb_Product4[1] + rtb_Product4[3]) / S;
    qy = 0.25 * S;
    S = (rtb_Product4[5] + rtb_Product4[7]) / S;
  } else {
    S = sqrt(((rtb_Product4[8] + 1.0) - rtb_Product4[0]) - rtb_Product4[4]) *
      2.0;
    tr = (rtb_Product4[2] + rtb_Product4[6]) / S;
    qy = (rtb_Product4[5] + rtb_Product4[7]) / S;
    S *= 0.25;
  }

  for (int32_T i = 0; i < 3; i++) {
    /* Product: '<S1>/Product3' incorporates:
     *  MATLAB Function: '<S1>/MATLAB Function'
     *  Sum: '<S1>/Subtract'
     */
    rtb_cartesiandampingdesign[i + 3] = 0.0;
    rtb_cartesiandampingdesign[i + 3] += rtb_Subtract[i] * tr;
    rtb_cartesiandampingdesign[i + 3] += rtb_Subtract[i + 3] * qy;
    rtb_cartesiandampingdesign[i + 3] += rtb_Subtract[i + 6] * S;

    /* Sum: '<S1>/Subtract2' incorporates:
     *  S-Function (get_initial_robot_state): '<Root>/Get Initial Robot State'
     *  S-Function (get_robot_state): '<Root>/Get Robot State'
     *  Selector: '<S1>/Select rotation matrix3'
     *  Selector: '<S1>/Select rotation matrix4'
     */
    rtb_cartesiandampingdesign[i] =
      cartesian_impedance_control_B.GetInitialRobotState[i + 12] -
      cartesian_impedance_control_B.GetRobotState_o1[i + 12];
  }

  for (int32_T i = 0; i < 6; i++) {
    /* Product: '<S1>/Product' incorporates:
     *  Constant: '<S1>/stiffness'
     *  Product: '<S1>/Product2'
     */
    tmp[i] = 0.0;
    for (rtb_Subtract_tmp = 0; rtb_Subtract_tmp < 6; rtb_Subtract_tmp++) {
      tmp[i] += cartesian_impedance_control_P.stiffness_Value[6 *
        rtb_Subtract_tmp + i] * rtb_cartesiandampingdesign[rtb_Subtract_tmp];
    }

    rtb_Subtract[i] = tmp[i];

    /* End of Product: '<S1>/Product' */

    /* Product: '<S1>/Product2' incorporates:
     *  S-Function (get_model): '<S1>/Get Model'
     */
    tmp_0[i] = 0.0;
    for (rtb_Subtract_tmp = 0; rtb_Subtract_tmp < 7; rtb_Subtract_tmp++) {
      tmp_0[i] += cartesian_impedance_control_B.GetModel_o3[6 * rtb_Subtract_tmp
        + i] * cartesian_impedance_control_B.GetRobotState_o2[rtb_Subtract_tmp];
    }
  }

  /* Sum: '<S1>/Subtract' incorporates:
   *  Constant: '<S1>/damping'
   *  Product: '<S1>/Product2'
   */
  for (int32_T i = 0; i < 6; i++) {
    tr = 0.0;
    for (rtb_Subtract_tmp = 0; rtb_Subtract_tmp < 6; rtb_Subtract_tmp++) {
      tr += cartesian_impedance_control_P.damping_Value[6 * rtb_Subtract_tmp + i]
        * tmp_0[rtb_Subtract_tmp];
    }

    rtb_Subtract[i] -= tr;
  }

  /* End of Sum: '<S1>/Subtract' */

  /* RateLimiter: '<S2>/Rate Limiter' */
  tr = cartesian_impedance_control_P.RateLimiter_RisingLim *
    cartesian_impedance_cont_period;
  qy = cartesian_impedance_control_P.RateLimiter_FallingLim *
    cartesian_impedance_cont_period;
  for (rtb_Subtract_tmp_0 = 0; rtb_Subtract_tmp_0 < 7; rtb_Subtract_tmp_0++) {
    /* Product: '<S1>/Product1' incorporates:
     *  Math: '<S1>/Math Function'
     *  S-Function (get_model): '<S1>/Get Model'
     */
    tmp_1[rtb_Subtract_tmp_0] = 0.0;
    for (int32_T i = 0; i < 6; i++) {
      tmp_1[rtb_Subtract_tmp_0] += cartesian_impedance_control_B.GetModel_o3[6 *
        rtb_Subtract_tmp_0 + i] * rtb_Subtract[i];
    }

    /* Sum: '<S1>/Subtract1' incorporates:
     *  Product: '<S1>/Product1'
     */
    S = cartesian_impedance_control_B.coriolis[rtb_Subtract_tmp_0] +
      tmp_1[rtb_Subtract_tmp_0];

    /* RateLimiter: '<S2>/Rate Limiter' */
    cartesian_impedance_control_B.RateLimiter[rtb_Subtract_tmp_0] = S -
      cartesian_impedance_control_DW.PrevY[rtb_Subtract_tmp_0];
    if (cartesian_impedance_control_B.RateLimiter[rtb_Subtract_tmp_0] > tr) {
      cartesian_impedance_control_B.RateLimiter[rtb_Subtract_tmp_0] =
        cartesian_impedance_control_DW.PrevY[rtb_Subtract_tmp_0] + tr;
    } else if (cartesian_impedance_control_B.RateLimiter[rtb_Subtract_tmp_0] <
               qy) {
      cartesian_impedance_control_B.RateLimiter[rtb_Subtract_tmp_0] =
        cartesian_impedance_control_DW.PrevY[rtb_Subtract_tmp_0] + qy;
    } else {
      cartesian_impedance_control_B.RateLimiter[rtb_Subtract_tmp_0] = S;
    }

    cartesian_impedance_control_DW.PrevY[rtb_Subtract_tmp_0] =
      cartesian_impedance_control_B.RateLimiter[rtb_Subtract_tmp_0];
  }

  /* S-Function (apply_control): '<S2>/Apply Control' */
  {
    /* S-Function Block: <S2>/Apply Control */
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

  /* Matfile logging */
  rt_UpdateTXYLogVars(cartesian_impedance_control_M->rtwLogInfo,
                      (cartesian_impedance_control_M->Timing.t));

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
  rtmSetTPtr(cartesian_impedance_control_M,
             &cartesian_impedance_control_M->Timing.tArray[0]);
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
  cartesian_impedance_control_M->Sizes.checksums[0] = (2512626292U);
  cartesian_impedance_control_M->Sizes.checksums[1] = (1374992255U);
  cartesian_impedance_control_M->Sizes.checksums[2] = (3625393152U);
  cartesian_impedance_control_M->Sizes.checksums[3] = (2317943958U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[2];
    cartesian_impedance_control_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(cartesian_impedance_control_M->extModeInfo,
      &cartesian_impedance_control_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(cartesian_impedance_control_M->extModeInfo,
                        cartesian_impedance_control_M->Sizes.checksums);
    rteiSetTPtr(cartesian_impedance_control_M->extModeInfo, rtmGetTPtr
                (cartesian_impedance_control_M));
  }

  /* block I/O */
  (void) memset((static_cast<void *>(&cartesian_impedance_control_B)), 0,
                sizeof(B_cartesian_impedance_control_T));

  /* states (dwork) */
  (void) memset(static_cast<void *>(&cartesian_impedance_control_DW), 0,
                sizeof(DW_cartesian_impedance_contro_T));

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

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(cartesian_impedance_control_M->rtwLogInfo,
    0.0, rtmGetTFinal(cartesian_impedance_control_M),
    cartesian_impedance_control_M->Timing.stepSize0, (&rtmGetErrorStatus
    (cartesian_impedance_control_M)));

  /* Start for S-Function (get_robot_state): '<S2>/Get Robot State2' */
  {
    cartesian_impedance_control_DW.GetRobotState2_DWORK1 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }

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

  /* Start for S-Function (apply_control): '<S2>/Apply Control' */
  {
    //Flag for performing initialization in first run of main _step();
    cartesian_impedance_control_DW.ApplyControl_DWORK1 = 0;
    cartesian_impedance_control_DW.ApplyControl_DWORK2 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }

  /* InitializeConditions for RateLimiter: '<S2>/Rate Limiter' */
  for (int32_T i = 0; i < 7; i++) {
    cartesian_impedance_control_DW.PrevY[i] =
      cartesian_impedance_control_P.RateLimiter_IC;
  }

  /* End of InitializeConditions for RateLimiter: '<S2>/Rate Limiter' */
}

/* Model terminate function */
void cartesian_impedance_control_terminate(void)
{
  /* Terminate for S-Function (apply_control): '<S2>/Apply Control' */
  {
    /* S-Function Block: <S2>/Apply Control */
  }
}
