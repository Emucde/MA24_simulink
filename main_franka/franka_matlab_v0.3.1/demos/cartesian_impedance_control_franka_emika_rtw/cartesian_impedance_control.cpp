/*
 * cartesian_impedance_control.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "cartesian_impedance_control".
 *
 * Model version              : 1.0
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Wed Oct  2 16:17:47 2024
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "cartesian_impedance_control.h"
#include <math.h>
#include "rtwtypes.h"
#include "cartesian_impedance_control_private.h"
#include <cstring>

extern "C"
{

#include "rt_nonfinite.h"

}

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
  real_T rtb_Selectrotationmatrix[9];
  real_T rtb_cartesiandampingdesign[6];
  real_T tmp[6];
  real_T tmp_0[6];
  real_T S;
  real_T qy;
  real_T tr;
  int32_T rtb_Selectrotationmatrix_tmp;

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

  for (int32_T i = 0; i < 3; i++) {
    int32_T rtb_Selectrotationmatrix_tmp_0;
    int32_T rtb_Selectrotationmatrix_tmp_1;

    /* Sum: '<S1>/Subtract2' incorporates:
     *  S-Function (get_initial_robot_state): '<Root>/Get Initial Robot State'
     *  S-Function (get_robot_state): '<Root>/Get Robot State'
     *  Selector: '<S1>/Select rotation matrix3'
     *  Selector: '<S1>/Select rotation matrix4'
     */
    rtb_cartesiandampingdesign[i] =
      cartesian_impedance_control_B.GetInitialRobotState[i + 12] -
      cartesian_impedance_control_B.GetRobotState_o1[i + 12];

    /* Selector: '<S1>/Select rotation matrix' incorporates:
     *  Math: '<S1>/Math Function1'
     *  Product: '<S1>/Product4'
     *  S-Function (get_robot_state): '<Root>/Get Robot State'
     */
    rtb_Selectrotationmatrix_tmp = i << 2;
    rtb_Selectrotationmatrix[3 * i] =
      cartesian_impedance_control_B.GetRobotState_o1[rtb_Selectrotationmatrix_tmp];
    rtb_Selectrotationmatrix_tmp_0 = 3 * i + 1;
    rtb_Selectrotationmatrix[rtb_Selectrotationmatrix_tmp_0] =
      cartesian_impedance_control_B.GetRobotState_o1[rtb_Selectrotationmatrix_tmp
      + 1];
    rtb_Selectrotationmatrix_tmp_1 = 3 * i + 2;
    rtb_Selectrotationmatrix[rtb_Selectrotationmatrix_tmp_1] =
      cartesian_impedance_control_B.GetRobotState_o1[rtb_Selectrotationmatrix_tmp
      + 2];

    /* Product: '<S1>/Product4' incorporates:
     *  Math: '<S1>/Math Function1'
     *  S-Function (get_initial_robot_state): '<Root>/Get Initial Robot State'
     *  Selector: '<S1>/Select rotation matrix'
     *  Selector: '<S1>/Select rotation matrix1'
     */
    for (rtb_Selectrotationmatrix_tmp = 0; rtb_Selectrotationmatrix_tmp < 3;
         rtb_Selectrotationmatrix_tmp++) {
      int32_T rtb_Product4_tmp;
      int32_T rtb_Product4_tmp_0;
      rtb_Product4_tmp = 3 * rtb_Selectrotationmatrix_tmp + i;
      rtb_Product4[rtb_Product4_tmp] = 0.0;

      /* Selector: '<S1>/Select rotation matrix1' incorporates:
       *  Math: '<S1>/Math Function1'
       *  Product: '<S1>/Product4'
       */
      rtb_Product4_tmp_0 = rtb_Selectrotationmatrix_tmp << 2;
      rtb_Product4[rtb_Product4_tmp] += rtb_Selectrotationmatrix[3 * i] *
        cartesian_impedance_control_B.GetInitialRobotState[rtb_Product4_tmp_0];
      rtb_Product4[rtb_Product4_tmp] +=
        cartesian_impedance_control_B.GetInitialRobotState[rtb_Product4_tmp_0 +
        1] * rtb_Selectrotationmatrix[rtb_Selectrotationmatrix_tmp_0];
      rtb_Product4[rtb_Product4_tmp] +=
        cartesian_impedance_control_B.GetInitialRobotState[rtb_Product4_tmp_0 +
        2] * rtb_Selectrotationmatrix[rtb_Selectrotationmatrix_tmp_1];
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

  /* Product: '<S1>/Product3' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  Selector: '<S1>/Select rotation matrix'
   */
  for (int32_T i = 0; i < 3; i++) {
    rtb_cartesiandampingdesign[i + 3] = 0.0;
    rtb_cartesiandampingdesign[i + 3] += rtb_Selectrotationmatrix[i] * tr;
    rtb_cartesiandampingdesign[i + 3] += rtb_Selectrotationmatrix[i + 3] * qy;
    rtb_cartesiandampingdesign[i + 3] += rtb_Selectrotationmatrix[i + 6] * S;
  }

  /* End of Product: '<S1>/Product3' */
  for (int32_T i = 0; i < 6; i++) {
    /* Product: '<S1>/Product' incorporates:
     *  Constant: '<S1>/stiffness'
     *  Product: '<S1>/Product2'
     */
    tmp[i] = 0.0;
    for (rtb_Selectrotationmatrix_tmp = 0; rtb_Selectrotationmatrix_tmp < 6;
         rtb_Selectrotationmatrix_tmp++) {
      tmp[i] += cartesian_impedance_control_P.stiffness_Value[6 *
        rtb_Selectrotationmatrix_tmp + i] *
        rtb_cartesiandampingdesign[rtb_Selectrotationmatrix_tmp];
    }

    rtb_Selectrotationmatrix[i] = tmp[i];

    /* End of Product: '<S1>/Product' */

    /* Product: '<S1>/Product2' incorporates:
     *  S-Function (get_model): '<S1>/Get Model'
     */
    tmp_0[i] = 0.0;
    for (rtb_Selectrotationmatrix_tmp = 0; rtb_Selectrotationmatrix_tmp < 7;
         rtb_Selectrotationmatrix_tmp++) {
      tmp_0[i] += cartesian_impedance_control_B.GetModel_o3[6 *
        rtb_Selectrotationmatrix_tmp + i] *
        cartesian_impedance_control_B.GetRobotState_o2[rtb_Selectrotationmatrix_tmp];
    }
  }

  /* Sum: '<S1>/Subtract' incorporates:
   *  Constant: '<S1>/damping'
   *  Product: '<S1>/Product2'
   */
  for (int32_T i = 0; i < 6; i++) {
    tr = 0.0;
    for (rtb_Selectrotationmatrix_tmp = 0; rtb_Selectrotationmatrix_tmp < 6;
         rtb_Selectrotationmatrix_tmp++) {
      tr += cartesian_impedance_control_P.damping_Value[6 *
        rtb_Selectrotationmatrix_tmp + i] * tmp_0[rtb_Selectrotationmatrix_tmp];
    }

    rtb_Selectrotationmatrix[i] -= tr;
  }

  /* End of Sum: '<S1>/Subtract' */
  for (int32_T i = 0; i < 7; i++) {
    /* Sum: '<S1>/Subtract1' incorporates:
     *  Math: '<S1>/Math Function'
     *  Product: '<S1>/Product1'
     *  S-Function (get_model): '<S1>/Get Model'
     */
    tr = 0.0;
    for (rtb_Selectrotationmatrix_tmp = 0; rtb_Selectrotationmatrix_tmp < 6;
         rtb_Selectrotationmatrix_tmp++) {
      tr += cartesian_impedance_control_B.GetModel_o3[6 * i +
        rtb_Selectrotationmatrix_tmp] *
        rtb_Selectrotationmatrix[rtb_Selectrotationmatrix_tmp];
    }

    cartesian_impedance_control_B.Subtract1[i] =
      cartesian_impedance_control_B.coriolis[i] + tr;

    /* End of Sum: '<S1>/Subtract1' */
  }

  /* S-Function (apply_control): '<Root>/Apply Control' */
  {
    /* S-Function Block: <Root>/Apply Control */
    if ((bool)cartesian_impedance_control_DW.ApplyControl_DWORK1) {
      // Wait for the control thread signal
      if ((bool)cartesian_impedance_control_DW.ApplyControl_DWORK2) {
        simulinkPandaRobot_1721602.waitForControlThreadStep();
      }

      // If control loop threw exeption terminate execution
      simulinkPandaRobot_1721602.checkIfAndHandleException();

      // copy inputs
      simulinkPandaRobot_1721602.copyInputSignal
        (&cartesian_impedance_control_B.Subtract1[0], 0);

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

  /* S-Function (get_robot_state): '<Root>/Get Robot State2' */
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
      signalsStringToSignalsVector(output_signals,"K_F_ext_hat_K O_T_EE");

      //copy outputs
      simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 0,
        &cartesian_impedance_control_B.GetRobotState2_o1[0]);
      simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 1,
        &cartesian_impedance_control_B.GetRobotState2_o2[0]);
    }
  }

  /* Matfile logging */
  rt_UpdateTXYLogVars(cartesian_impedance_control_M->rtwLogInfo,
                      (&cartesian_impedance_control_M->Timing.taskTime0));

  /* External mode */
  rtExtModeUploadCheckTrigger(1);

  {                                    /* Sample time: [0.001s, 0.0s] */
    rtExtModeUpload(0, (real_T)cartesian_impedance_control_M->Timing.taskTime0);
  }

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.001s, 0.0s] */
    if ((rtmGetTFinal(cartesian_impedance_control_M)!=-1) &&
        !((rtmGetTFinal(cartesian_impedance_control_M)-
           cartesian_impedance_control_M->Timing.taskTime0) >
          cartesian_impedance_control_M->Timing.taskTime0 * (DBL_EPSILON))) {
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

  cartesian_impedance_control_M->Timing.taskTime0 =
    cartesian_impedance_control_M->Timing.clockTick0 *
    cartesian_impedance_control_M->Timing.stepSize0 +
    cartesian_impedance_control_M->Timing.clockTickH0 *
    cartesian_impedance_control_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void cartesian_impedance_control_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));
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
  cartesian_impedance_control_M->Sizes.checksums[0] = (683012358U);
  cartesian_impedance_control_M->Sizes.checksums[1] = (403416137U);
  cartesian_impedance_control_M->Sizes.checksums[2] = (1786134574U);
  cartesian_impedance_control_M->Sizes.checksums[3] = (3252213375U);

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

  /* Start for S-Function (get_model): '<S1>/Get Model' */
  {
    cartesian_impedance_control_DW.GetModel_DWORK1 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }

  /* Start for S-Function (get_initial_robot_state): '<Root>/Get Initial Robot State' */
  {
    cartesian_impedance_control_DW.GetInitialRobotState_DWORK1 = 0;
    cartesian_impedance_control_DW.GetInitialRobotState_DWORK2 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }

  /* Start for S-Function (get_robot_state): '<Root>/Get Robot State' */
  {
    cartesian_impedance_control_DW.GetRobotState_DWORK1 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }

  /* Start for S-Function (apply_control): '<Root>/Apply Control' */
  {
    //Flag for performing initialization in first run of main _step();
    cartesian_impedance_control_DW.ApplyControl_DWORK1 = 0;
    cartesian_impedance_control_DW.ApplyControl_DWORK2 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }

  /* Start for S-Function (get_robot_state): '<Root>/Get Robot State2' */
  {
    cartesian_impedance_control_DW.GetRobotState2_DWORK1 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }
}

/* Model terminate function */
void cartesian_impedance_control_terminate(void)
{
  /* Terminate for S-Function (apply_control): '<Root>/Apply Control' */
  {
    /* S-Function Block: <Root>/Apply Control */
  }
}
