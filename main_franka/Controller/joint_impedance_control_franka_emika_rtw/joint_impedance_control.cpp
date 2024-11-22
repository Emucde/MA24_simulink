/*
 * joint_impedance_control.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "joint_impedance_control".
 *
 * Model version              : 1.0
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Fri Nov 22 16:36:31 2024
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "joint_impedance_control.h"
#include <string.h>
#include <math.h>
#include "rtwtypes.h"
#include "joint_impedance_control_private.h"
#include <cstring>

extern "C"
{

#include "rt_nonfinite.h"

}

SimulinkPandaRobot simulinkPandaRobot_1721602;

/* Block signals (default storage) */
B_joint_impedance_control_T joint_impedance_control_B;

/* Block states (default storage) */
DW_joint_impedance_control_T joint_impedance_control_DW;

/* Real-time model */
RT_MODEL_joint_impedance_cont_T joint_impedance_control_M_ =
  RT_MODEL_joint_impedance_cont_T();
RT_MODEL_joint_impedance_cont_T *const joint_impedance_control_M =
  &joint_impedance_control_M_;

/* Model step function */
void joint_impedance_control_step(void)
{
  real_T rtb_Add5;
  real_T rtb_Product1;
  real_T rtb_Switch2;

  /* S-Function (get_initial_robot_state): '<S2>/Get Initial Robot State1' */
  {
    // Wait for the control thread signal
    if ((bool)joint_impedance_control_DW.GetInitialRobotState1_DWORK2 &&
        simulinkPandaRobot_1721602.getControlThreadHasBeenSpawned()) {
      simulinkPandaRobot_1721602.waitForControlThreadStep();
    }

    // If control loop threw exeption terminate execution
    simulinkPandaRobot_1721602.checkIfAndHandleException();
    if (!simulinkPandaRobot_1721602.getCurrentlyInFirstControlStep() && !(bool)
        joint_impedance_control_DW.GetInitialRobotState1_DWORK1) {
      std::vector<std::string> output_signals;
      signalsStringToSignalsVector(output_signals,"O_T_EE");

      //copy outputs
      simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 0,
        &joint_impedance_control_B.GetInitialRobotState1[0]);
      joint_impedance_control_DW.GetInitialRobotState1_DWORK1 = 1;
    }
  }

  /* S-Function (get_duration_period): '<S2>/Get Duration Period' */
  {
    if (simulinkPandaRobot_1721602.getControlThreadHasBeenSpawned()) {
      // Wait for the control thread signal
      if ((bool)joint_impedance_control_DW.GetDurationPeriod_DWORK1) {
        simulinkPandaRobot_1721602.waitForControlThreadStep();
      }

      // If control loop threw exeption terminate execution
      simulinkPandaRobot_1721602.checkIfAndHandleException();
      joint_impedance_control_B.GetDurationPeriod =
        simulinkPandaRobot_1721602.getSampleTime();
    }
  }

  /* S-Function (get_model): '<S1>/Get Model' */
  {
    if (simulinkPandaRobot_1721602.getControlThreadHasBeenSpawned()) {
      // Wait for the control thread signal
      if ((bool)joint_impedance_control_DW.GetModel_DWORK1) {
        simulinkPandaRobot_1721602.waitForControlThreadStep();
      }

      // If control loop threw exeption terminate execution
      simulinkPandaRobot_1721602.checkIfAndHandleException();

      // robot pose
      simulinkPandaRobot_1721602.copyRobotPose
        (&joint_impedance_control_B.GetModel_o1[0]);

      // bodyJacobian
      simulinkPandaRobot_1721602.copyBodyJacobian
        (&joint_impedance_control_B.GetModel_o2[0]);

      // zeroJacobian
      simulinkPandaRobot_1721602.copyZeroJacobian
        (&joint_impedance_control_B.GetModel_o3[0]);

      // mass
      simulinkPandaRobot_1721602.copyMass
        (&joint_impedance_control_B.GetModel_o4[0]);

      // coriolis
      simulinkPandaRobot_1721602.copyCoriolis
        (&joint_impedance_control_B.GetModel_o5[0]);

      // gravity
      simulinkPandaRobot_1721602.copyGravity
        (&joint_impedance_control_B.GetModel_o6[0]);
    }
  }

  /* Assignment: '<S2>/Assignment3' incorporates:
   *  SignalConversion generated from: '<S2>/Assignment2'
   */
  memcpy(&joint_impedance_control_B.Assignment3[0],
         &joint_impedance_control_B.GetInitialRobotState1[0], sizeof(real_T) <<
         4U);

  /* Product: '<S2>/Product1' incorporates:
   *  Abs: '<S2>/Abs1'
   *  Constant: '<S2>/acceleration_time'
   *  Constant: '<S2>/vel_max'
   *  Product: '<S2>/Divide1'
   */
  rtb_Product1 = fabs(joint_impedance_control_P.vel_max_Value /
                      joint_impedance_control_P.acceleration_time_Value) *
    joint_impedance_control_B.GetDurationPeriod;

  /* Switch: '<S3>/Switch' incorporates:
   *  Constant: '<S2>/velocity_amplitude1'
   */
  if (joint_impedance_control_P.velocity_amplitude1_Value) {
    /* Switch: '<S3>/Switch' incorporates:
     *  Delay: '<S3>/Delay'
     *  Sum: '<S3>/Sum'
     */
    joint_impedance_control_DW.Delay_DSTATE +=
      joint_impedance_control_B.GetDurationPeriod;
  }

  /* End of Switch: '<S3>/Switch' */

  /* Switch: '<S3>/Switch2' incorporates:
   *  Constant: '<S2>/velocity_amplitude7'
   *  Constant: '<S3>/Constant2'
   */
  if (joint_impedance_control_P.velocity_amplitude7_Value) {
    rtb_Switch2 = joint_impedance_control_P.Constant2_Value;
  } else {
    rtb_Switch2 = joint_impedance_control_DW.Delay_DSTATE;
  }

  /* End of Switch: '<S3>/Switch2' */

  /* Switch: '<S2>/Switch1' incorporates:
   *  Constant: '<S2>/run_time'
   *  Constant: '<S2>/vel_max'
   *  Delay: '<S2>/current_velocity'
   *  Logic: '<S2>/Logical Operator'
   *  RelationalOperator: '<S2>/Relational Operator1'
   *  RelationalOperator: '<S2>/Relational Operator3'
   *  Sum: '<S2>/Add3'
   */
  if ((joint_impedance_control_DW.current_velocity_DSTATE <
       joint_impedance_control_P.vel_max_Value) && (rtb_Switch2 <
       joint_impedance_control_P.run_time_Value)) {
    rtb_Add5 = rtb_Product1 + joint_impedance_control_DW.current_velocity_DSTATE;
  } else {
    rtb_Add5 = joint_impedance_control_DW.current_velocity_DSTATE;
  }

  /* End of Switch: '<S2>/Switch1' */

  /* Switch: '<S2>/Switch2' incorporates:
   *  Constant: '<S2>/constant0'
   *  Constant: '<S2>/run_time'
   *  Delay: '<S2>/current_velocity'
   *  Logic: '<S2>/Logical Operator1'
   *  RelationalOperator: '<S2>/Relational Operator4'
   *  RelationalOperator: '<S2>/Relational Operator5'
   *  Sum: '<S2>/Add7'
   */
  if ((rtb_Add5 > joint_impedance_control_P.constant0_Value) && (rtb_Switch2 >
       joint_impedance_control_P.run_time_Value)) {
    rtb_Add5 = joint_impedance_control_DW.current_velocity_DSTATE - rtb_Product1;
  }

  /* End of Switch: '<S2>/Switch2' */

  /* MinMax: '<S2>/Max1' incorporates:
   *  Constant: '<S2>/constant1'
   *  Constant: '<S2>/vel_max'
   *  MinMax: '<S2>/Max'
   */
  joint_impedance_control_DW.current_velocity_DSTATE = fmin
    (joint_impedance_control_P.vel_max_Value, fmax(rtb_Add5,
      joint_impedance_control_P.constant1_Value));

  /* Sum: '<S2>/Add2' incorporates:
   *  Abs: '<S2>/Abs'
   *  Constant: '<S2>/radius'
   *  Delay: '<S2>/angle'
   *  Product: '<S2>/Divide'
   *  Product: '<S2>/Product11'
   */
  joint_impedance_control_DW.angle_DSTATE +=
    joint_impedance_control_DW.current_velocity_DSTATE *
    joint_impedance_control_B.GetDurationPeriod / fabs
    (joint_impedance_control_P.radius_Value);

  /* Switch: '<S2>/Switch3' incorporates:
   *  Constant: '<S2>/constant2'
   *  RelationalOperator: '<S2>/Relational Operator2'
   */
  if (joint_impedance_control_DW.angle_DSTATE >
      joint_impedance_control_P.constant2_Value) {
    /* Sum: '<S2>/Add2' incorporates:
     *  Sum: '<S2>/Add1'
     */
    joint_impedance_control_DW.angle_DSTATE -=
      joint_impedance_control_P.constant2_Value;
  }

  /* End of Switch: '<S2>/Switch3' */

  /* Assignment: '<S2>/Assignment2' incorporates:
   *  Assignment: '<S2>/Assignment3'
   *  Constant: '<S2>/constant'
   *  Constant: '<S2>/radius'
   *  Product: '<S2>/Product5'
   *  Sum: '<S2>/Add4'
   *  Sum: '<S2>/Add6'
   *  Trigonometry: '<S2>/Trigonometric Function3'
   */
  joint_impedance_control_B.Assignment3[13] =
    (joint_impedance_control_P.constant_Value - cos
     (joint_impedance_control_DW.angle_DSTATE)) *
    joint_impedance_control_P.radius_Value +
    joint_impedance_control_B.GetInitialRobotState1[13];

  /* Assignment: '<S2>/Assignment3' incorporates:
   *  Constant: '<S2>/radius'
   *  Product: '<S2>/Product10'
   *  Sum: '<S2>/Add5'
   *  Trigonometry: '<S2>/Trigonometric Function4'
   */
  joint_impedance_control_B.Assignment3[14] =
    joint_impedance_control_P.radius_Value * sin
    (joint_impedance_control_DW.angle_DSTATE) +
    joint_impedance_control_B.GetInitialRobotState1[14];

  /* S-Function (get_robot_state): '<Root>/Get Robot State' */
  {
    // Wait for the control thread signal
    if ((bool)joint_impedance_control_DW.GetRobotState_DWORK1 &&
        simulinkPandaRobot_1721602.getControlThreadHasBeenSpawned()) {
      simulinkPandaRobot_1721602.waitForControlThreadStep();
    }

    // If control loop threw exeption terminate execution
    simulinkPandaRobot_1721602.checkIfAndHandleException();
    if (!simulinkPandaRobot_1721602.getCurrentlyInFirstControlStep()) {
      std::vector<std::string> output_signals;
      signalsStringToSignalsVector(output_signals,"q_d q dq");

      //copy outputs
      simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 0,
        &joint_impedance_control_B.GetRobotState_o1[0]);
      simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 1,
        &joint_impedance_control_B.GetRobotState_o2[0]);
      simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 2,
        &joint_impedance_control_B.GetRobotState_o3[0]);
    }
  }

  for (int32_T i = 0; i < 7; i++) {
    /* Sum: '<S1>/Add2' incorporates:
     *  Constant: '<S1>/d_gains'
     *  Constant: '<S1>/k_gains'
     *  Product: '<S1>/Product1'
     *  Product: '<S1>/Product2'
     *  Sum: '<S1>/Add1'
     *  Sum: '<S1>/Add7'
     */
    joint_impedance_control_B.Add2[i] =
      ((joint_impedance_control_B.GetRobotState_o1[i] -
        joint_impedance_control_B.GetRobotState_o2[i]) *
       joint_impedance_control_P.k_gains_Value[i] -
       joint_impedance_control_P.d_gains_Value[i] *
       joint_impedance_control_B.GetRobotState_o3[i]) +
      joint_impedance_control_B.GetModel_o5[i];
  }

  /* S-Function (apply_control): '<Root>/Apply Control' */
  {
    /* S-Function Block: <Root>/Apply Control */
    if ((bool)joint_impedance_control_DW.ApplyControl_DWORK1) {
      // Wait for the control thread signal
      if ((bool)joint_impedance_control_DW.ApplyControl_DWORK2) {
        simulinkPandaRobot_1721602.waitForControlThreadStep();
      }

      // If control loop threw exeption terminate execution
      simulinkPandaRobot_1721602.checkIfAndHandleException();

      // copy inputs
      simulinkPandaRobot_1721602.copyInputSignal
        (&joint_impedance_control_B.Assignment3[0], 0);
      simulinkPandaRobot_1721602.copyInputSignal
        (&joint_impedance_control_B.Add2[0], 1);

      // notify control thread that the inputs have been read
      simulinkPandaRobot_1721602.notifyControlThreadToContinue();
    } else if (!(bool)joint_impedance_control_DW.ApplyControl_DWORK1) {
      // Its the first time _step() function is called -->
      // Initialize according to settings parsed from the mask
      // and spawn control thread
      simulinkPandaRobot_1721602.applyRobotSettings();
      simulinkPandaRobot_1721602.spawnControlThread();
      joint_impedance_control_DW.ApplyControl_DWORK1 = 1;
    }
  }

  /* S-Function (get_robot_state): '<Root>/Get Robot State2' */
  {
    // Wait for the control thread signal
    if ((bool)joint_impedance_control_DW.GetRobotState2_DWORK1 &&
        simulinkPandaRobot_1721602.getControlThreadHasBeenSpawned()) {
      simulinkPandaRobot_1721602.waitForControlThreadStep();
    }

    // If control loop threw exeption terminate execution
    simulinkPandaRobot_1721602.checkIfAndHandleException();
    if (!simulinkPandaRobot_1721602.getCurrentlyInFirstControlStep()) {
      std::vector<std::string> output_signals;
      signalsStringToSignalsVector(output_signals,"tau_J");

      //copy outputs
      simulinkPandaRobot_1721602.copyOutputSignal(output_signals, 0,
        &joint_impedance_control_B.GetRobotState2[0]);
    }
  }

  for (int32_T i = 0; i < 7; i++) {
    /* Sum: '<Root>/Add3' incorporates:
     *  Sum: '<S1>/Add2'
     */
    joint_impedance_control_B.Add3[i] = joint_impedance_control_B.Add2[i] -
      joint_impedance_control_B.GetRobotState2[i];
  }

  /* Switch: '<S3>/Switch1' incorporates:
   *  Constant: '<S2>/velocity_amplitude7'
   */
  if (joint_impedance_control_P.velocity_amplitude7_Value) {
    /* Switch: '<S3>/Switch' incorporates:
     *  Constant: '<S3>/Constant'
     *  Delay: '<S3>/Delay'
     */
    joint_impedance_control_DW.Delay_DSTATE =
      joint_impedance_control_P.Constant_Value;
  }

  /* End of Switch: '<S3>/Switch1' */

  /* Matfile logging */
  rt_UpdateTXYLogVars(joint_impedance_control_M->rtwLogInfo,
                      (&joint_impedance_control_M->Timing.taskTime0));

  /* External mode */
  rtExtModeUploadCheckTrigger(1);

  {                                    /* Sample time: [0.001s, 0.0s] */
    rtExtModeUpload(0, (real_T)joint_impedance_control_M->Timing.taskTime0);
  }

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.001s, 0.0s] */
    if ((rtmGetTFinal(joint_impedance_control_M)!=-1) &&
        !((rtmGetTFinal(joint_impedance_control_M)-
           joint_impedance_control_M->Timing.taskTime0) >
          joint_impedance_control_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(joint_impedance_control_M, "Simulation finished");
    }

    if (rtmGetStopRequested(joint_impedance_control_M)) {
      rtmSetErrorStatus(joint_impedance_control_M, "Simulation finished");
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
  if (!(++joint_impedance_control_M->Timing.clockTick0)) {
    ++joint_impedance_control_M->Timing.clockTickH0;
  }

  joint_impedance_control_M->Timing.taskTime0 =
    joint_impedance_control_M->Timing.clockTick0 *
    joint_impedance_control_M->Timing.stepSize0 +
    joint_impedance_control_M->Timing.clockTickH0 *
    joint_impedance_control_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void joint_impedance_control_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));
  rtmSetTFinal(joint_impedance_control_M, -1);
  joint_impedance_control_M->Timing.stepSize0 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    joint_impedance_control_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(joint_impedance_control_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(joint_impedance_control_M->rtwLogInfo, (NULL));
    rtliSetLogT(joint_impedance_control_M->rtwLogInfo, "tout");
    rtliSetLogX(joint_impedance_control_M->rtwLogInfo, "");
    rtliSetLogXFinal(joint_impedance_control_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(joint_impedance_control_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(joint_impedance_control_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(joint_impedance_control_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(joint_impedance_control_M->rtwLogInfo, 1);
    rtliSetLogY(joint_impedance_control_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(joint_impedance_control_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(joint_impedance_control_M->rtwLogInfo, (NULL));
  }

  /* External mode info */
  joint_impedance_control_M->Sizes.checksums[0] = (3939466462U);
  joint_impedance_control_M->Sizes.checksums[1] = (1434640865U);
  joint_impedance_control_M->Sizes.checksums[2] = (375093169U);
  joint_impedance_control_M->Sizes.checksums[3] = (3609424086U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[7];
    joint_impedance_control_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(joint_impedance_control_M->extModeInfo,
      &joint_impedance_control_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(joint_impedance_control_M->extModeInfo,
                        joint_impedance_control_M->Sizes.checksums);
    rteiSetTPtr(joint_impedance_control_M->extModeInfo, rtmGetTPtr
                (joint_impedance_control_M));
  }

  /* block I/O */
  (void) memset((static_cast<void *>(&joint_impedance_control_B)), 0,
                sizeof(B_joint_impedance_control_T));

  /* states (dwork) */
  (void) memset(static_cast<void *>(&joint_impedance_control_DW), 0,
                sizeof(DW_joint_impedance_control_T));

  /* block instance data */
  {
    {
      simulinkPandaRobot_1721602 = SimulinkPandaRobot( "172.16.0.2",
        0,
        3,
        0,
        joint_impedance_control_P.ApplyControl_P1,
        joint_impedance_control_P.ApplyControl_P2,
        joint_impedance_control_P.ApplyControl_P3,
        joint_impedance_control_P.ApplyControl_P4,
        joint_impedance_control_P.ApplyControl_P5,
        1,
        joint_impedance_control_P.ApplyControl_P6,
        0);
    }
  }

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(joint_impedance_control_M->rtwLogInfo, 0.0,
    rtmGetTFinal(joint_impedance_control_M),
    joint_impedance_control_M->Timing.stepSize0, (&rtmGetErrorStatus
    (joint_impedance_control_M)));

  /* Start for S-Function (get_initial_robot_state): '<S2>/Get Initial Robot State1' */
  {
    joint_impedance_control_DW.GetInitialRobotState1_DWORK1 = 0;
    joint_impedance_control_DW.GetInitialRobotState1_DWORK2 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }

  /* Start for S-Function (get_duration_period): '<S2>/Get Duration Period' */
  {
    joint_impedance_control_DW.GetDurationPeriod_DWORK1 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }

  /* Start for S-Function (get_model): '<S1>/Get Model' */
  {
    joint_impedance_control_DW.GetModel_DWORK1 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }

  /* Start for S-Function (get_robot_state): '<Root>/Get Robot State' */
  {
    joint_impedance_control_DW.GetRobotState_DWORK1 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }

  /* Start for S-Function (apply_control): '<Root>/Apply Control' */
  {
    //Flag for performing initialization in first run of main _step();
    joint_impedance_control_DW.ApplyControl_DWORK1 = 0;
    joint_impedance_control_DW.ApplyControl_DWORK2 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }

  /* Start for S-Function (get_robot_state): '<Root>/Get Robot State2' */
  {
    joint_impedance_control_DW.GetRobotState2_DWORK1 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }

  /* InitializeConditions for MinMax: '<S2>/Max1' incorporates:
   *  Delay: '<S2>/current_velocity'
   */
  joint_impedance_control_DW.current_velocity_DSTATE =
    joint_impedance_control_P.current_velocity_InitialConditi;

  /* InitializeConditions for Switch: '<S3>/Switch' incorporates:
   *  Delay: '<S3>/Delay'
   */
  joint_impedance_control_DW.Delay_DSTATE =
    joint_impedance_control_P.Delay_InitialCondition;

  /* InitializeConditions for Sum: '<S2>/Add2' incorporates:
   *  Delay: '<S2>/angle'
   */
  joint_impedance_control_DW.angle_DSTATE =
    joint_impedance_control_P.angle_InitialCondition;
}

/* Model terminate function */
void joint_impedance_control_terminate(void)
{
  /* Terminate for S-Function (apply_control): '<Root>/Apply Control' */
  {
    /* S-Function Block: <Root>/Apply Control */
  }
}
