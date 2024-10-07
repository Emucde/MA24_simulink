/*
 * cartesian_impedance_control.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "cartesian_impedance_control".
 *
 * Model version              : 8.158
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Mon Oct  7 11:17:43 2024
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "cartesian_impedance_control.h"
#include "rtwtypes.h"
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <cstring>
#include "cartesian_impedance_control_private.h"

extern "C"
{

#include "rt_nonfinite.h"

}

const real_T cartesian_impedance_cont_period = 0.001;
SimulinkPandaRobot simulinkPandaRobot_17216102;

/* Block signals (default storage) */
B_cartesian_impedance_control_T cartesian_impedance_control_B;

/* Block states (default storage) */
DW_cartesian_impedance_contro_T cartesian_impedance_control_DW;

/* Real-time model */
RT_MODEL_cartesian_impedance__T cartesian_impedance_control_M_ =
  RT_MODEL_cartesian_impedance__T();
RT_MODEL_cartesian_impedance__T *const cartesian_impedance_control_M =
  &cartesian_impedance_control_M_;

/* Forward declaration for local functions */
static int8_T cartesian_impedance_co_filedata(void);
static int8_T cartesian_impedance_cont_cfopen(const char_T *cfilename, const
  char_T *cpermission);
static void cartesian_impedance_getfilestar(real_T fid, FILE* *filestar,
  boolean_T *autoflush);
static int32_T cartesian_impedance_con_cfclose(real_T fid);

/* Function for MATLAB Function: '<Root>/MATLAB Function3' */
static int8_T cartesian_impedance_co_filedata(void)
{
  int32_T k;
  int8_T f;
  boolean_T exitg1;
  f = 0;
  k = 1;
  exitg1 = false;
  while ((!exitg1) && (k - 1 < 20)) {
    if (cartesian_impedance_control_DW.eml_openfiles[static_cast<int8_T>(k) - 1]
        == NULL) {
      f = static_cast<int8_T>(k);
      exitg1 = true;
    } else {
      k++;
    }
  }

  return f;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function3' */
static int8_T cartesian_impedance_cont_cfopen(const char_T *cfilename, const
  char_T *cpermission)
{
  int8_T fileid;
  int8_T j;
  fileid = -1;
  j = cartesian_impedance_co_filedata();
  if (j >= 1) {
    FILE* filestar;
    filestar = fopen(cfilename, cpermission);
    if (filestar != NULL) {
      int32_T tmp;
      cartesian_impedance_control_DW.eml_openfiles[j - 1] = filestar;
      cartesian_impedance_control_DW.eml_autoflush[j - 1] = true;
      tmp = j + 2;
      if (j + 2 > 127) {
        tmp = 127;
      }

      fileid = static_cast<int8_T>(tmp);
    }
  }

  return fileid;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function3' */
static void cartesian_impedance_getfilestar(real_T fid, FILE* *filestar,
  boolean_T *autoflush)
{
  int8_T fileid;
  fileid = static_cast<int8_T>(fid);
  if ((static_cast<int8_T>(fid) < 0) || (fid != static_cast<int8_T>(fid))) {
    fileid = -1;
  }

  if (fileid >= 3) {
    *filestar = cartesian_impedance_control_DW.eml_openfiles[fileid - 3];
    *autoflush = cartesian_impedance_control_DW.eml_autoflush[fileid - 3];
  } else if (fileid == 0) {
    *filestar = stdin;
    *autoflush = true;
  } else if (fileid == 1) {
    *filestar = stdout;
    *autoflush = true;
  } else if (fileid == 2) {
    *filestar = stderr;
    *autoflush = true;
  } else {
    *filestar = NULL;
    *autoflush = true;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function3' */
static int32_T cartesian_impedance_con_cfclose(real_T fid)
{
  FILE* filestar;
  int32_T st;
  int8_T b_fileid;
  int8_T fileid;
  st = -1;
  fileid = static_cast<int8_T>(fid);
  if ((static_cast<int8_T>(fid) < 0) || (fid != static_cast<int8_T>(fid))) {
    fileid = -1;
  }

  b_fileid = fileid;
  if (fileid < 0) {
    b_fileid = -1;
  }

  if (b_fileid >= 3) {
    filestar = cartesian_impedance_control_DW.eml_openfiles[b_fileid - 3];
  } else if (b_fileid == 0) {
    filestar = stdin;
  } else if (b_fileid == 1) {
    filestar = stdout;
  } else if (b_fileid == 2) {
    filestar = stderr;
  } else {
    filestar = NULL;
  }

  if ((filestar != NULL) && (fileid >= 3)) {
    int32_T cst;
    cst = fclose(filestar);
    if (cst == 0) {
      st = 0;
      cartesian_impedance_control_DW.eml_openfiles[fileid - 3] = NULL;
      cartesian_impedance_control_DW.eml_autoflush[fileid - 3] = true;
    }
  }

  return st;
}

/* Model step function */
void cartesian_impedance_control_step(void)
{
  FILE* b_filestar;
  size_t bytesOutSizet;
  real_T a[49];
  real_T d[49];
  real_T d_0[49];
  real_T a_0[7];
  real_T d_1[7];
  real_T tmp[7];
  real_T A_data;
  real_T rtb_Clock;
  real_T tmp_0;
  int32_T j;
  int8_T fileid;
  boolean_T autoflush;

  /* S-Function (get_robot_state): '<S3>/Get Robot State2' */
  {
    // Wait for the control thread signal
    if ((bool)cartesian_impedance_control_DW.GetRobotState2_DWORK1 &&
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
        &cartesian_impedance_control_B.GetRobotState2_o1[0]);
      simulinkPandaRobot_17216102.copyOutputSignal(output_signals, 1,
        &cartesian_impedance_control_B.GetRobotState2_o2[0]);
      simulinkPandaRobot_17216102.copyOutputSignal(output_signals, 2,
        &cartesian_impedance_control_B.GetRobotState2_o3[0]);
    }
  }

  /* Clock: '<S3>/Clock' */
  rtb_Clock = cartesian_impedance_control_M->Timing.t[0];
  for (int32_T i = 0; i < 7; i++) {
    /* Switch: '<S3>/Switch' */
    if (rtb_Clock > cartesian_impedance_control_P.Switch_Threshold) {
      /* Switch: '<S3>/Switch' */
      cartesian_impedance_control_B.Switch[i] =
        cartesian_impedance_control_B.GetRobotState2_o1[i];
    } else {
      /* Switch: '<S3>/Switch' incorporates:
       *  Constant: '<S3>/Constant'
       */
      cartesian_impedance_control_B.Switch[i] =
        cartesian_impedance_control_P.q_init[i];
    }

    /* End of Switch: '<S3>/Switch' */
  }

  /* MATLAB Function: '<Root>/Joinspace controller' incorporates:
   *  Constant: '<Root>/Constant8'
   *  Constant: '<Root>/Constant9'
   *  Constant: '<Root>/q_d_3'
   */
  memset(&d[0], 0, 49U * sizeof(real_T));
  for (j = 0; j < 7; j++) {
    d[j + 7 * j] = cartesian_impedance_control_P.Constant9_Value[j];
  }

  memset(&a[0], 0, 49U * sizeof(real_T));
  for (j = 0; j < 7; j++) {
    a[j + 7 * j] = cartesian_impedance_control_P.Constant8_Value[j];
  }

  for (j = 0; j < 49; j++) {
    d_0[j] = -d[j];
  }

  for (j = 0; j < 7; j++) {
    tmp[j] = cartesian_impedance_control_B.Switch[j] -
      cartesian_impedance_control_P.q_d_3_Value[j];
    d_1[j] = 0.0;
    for (int32_T i = 0; i < 7; i++) {
      d_1[j] += d_0[7 * i + j] *
        cartesian_impedance_control_B.GetRobotState2_o2[i];
    }
  }

  /* RateLimiter: '<S3>/Rate Limiter' */
  rtb_Clock = cartesian_impedance_control_P.RateLimiter_RisingLim *
    cartesian_impedance_cont_period;
  tmp_0 = cartesian_impedance_control_P.RateLimiter_FallingLim *
    cartesian_impedance_cont_period;
  for (int32_T i = 0; i < 7; i++) {
    real_T rtb_tau_m;

    /* MATLAB Function: '<Root>/Joinspace controller' */
    a_0[i] = 0.0;
    for (j = 0; j < 7; j++) {
      a_0[i] += a[7 * j + i] * tmp[j];
    }

    rtb_tau_m = d_1[i] - a_0[i];

    /* RateLimiter: '<S3>/Rate Limiter' */
    cartesian_impedance_control_B.RateLimiter[i] = rtb_tau_m -
      cartesian_impedance_control_DW.PrevY[i];
    if (cartesian_impedance_control_B.RateLimiter[i] > rtb_Clock) {
      /* RateLimiter: '<S3>/Rate Limiter' */
      cartesian_impedance_control_B.RateLimiter[i] =
        cartesian_impedance_control_DW.PrevY[i] + rtb_Clock;
    } else if (cartesian_impedance_control_B.RateLimiter[i] < tmp_0) {
      /* RateLimiter: '<S3>/Rate Limiter' */
      cartesian_impedance_control_B.RateLimiter[i] =
        cartesian_impedance_control_DW.PrevY[i] + tmp_0;
    } else {
      /* RateLimiter: '<S3>/Rate Limiter' */
      cartesian_impedance_control_B.RateLimiter[i] = rtb_tau_m;
    }

    cartesian_impedance_control_DW.PrevY[i] =
      cartesian_impedance_control_B.RateLimiter[i];
  }

  /* S-Function (apply_control): '<S3>/Apply Control' */
  {
    /* S-Function Block: <S3>/Apply Control */
    if ((bool)cartesian_impedance_control_DW.ApplyControl_DWORK1) {
      // Wait for the control thread signal
      if ((bool)cartesian_impedance_control_DW.ApplyControl_DWORK2) {
        simulinkPandaRobot_17216102.waitForControlThreadStep();
      }

      // If control loop threw exeption terminate execution
      simulinkPandaRobot_17216102.checkIfAndHandleException();

      // copy inputs
      simulinkPandaRobot_17216102.copyInputSignal
        (&cartesian_impedance_control_B.RateLimiter[0], 0);

      // notify control thread that the inputs have been read
      simulinkPandaRobot_17216102.notifyControlThreadToContinue();
    } else if (!(bool)cartesian_impedance_control_DW.ApplyControl_DWORK1) {
      // Its the first time _step() function is called -->
      // Initialize according to settings parsed from the mask
      // and spawn control thread
      simulinkPandaRobot_17216102.applyRobotSettings();
      simulinkPandaRobot_17216102.spawnControlThread();
      cartesian_impedance_control_DW.ApplyControl_DWORK1 = 1;
    }
  }

  /* MATLAB Function: '<Root>/MATLAB Function3' */
  fileid = cartesian_impedance_cont_cfopen("data_from_simulink.bin", "wb");
  rtb_Clock = 1.0;
  cartesian_impedance_getfilestar(static_cast<real_T>(fileid), &b_filestar,
    &autoflush);
  if (fileid == 0) {
    b_filestar = NULL;
  }

  if (!(b_filestar == NULL)) {
    bytesOutSizet = fwrite(&rtb_Clock, sizeof(real_T), (size_t)1, b_filestar);
    if (((real_T)bytesOutSizet > 0.0) && autoflush) {
      fflush(b_filestar);
    }
  }

  cartesian_impedance_con_cfclose(static_cast<real_T>(fileid));
  fileid = cartesian_impedance_cont_cfopen("data_from_crocoddyl.bin", "rb");
  cartesian_impedance_getfilestar(static_cast<real_T>(fileid), &b_filestar,
    &autoflush);
  if ((fileid == 0) || (fileid == 1) || (fileid == 2)) {
    b_filestar = NULL;
  }

  if (b_filestar == NULL) {
    j = 0;
  } else {
    bytesOutSizet = fread(&A_data, sizeof(real_T), (size_t)1, b_filestar);
    if ((int32_T)bytesOutSizet + 1 <= 1) {
      A_data = 0.0;
    }

    j = (int32_T)bytesOutSizet;
  }

  cartesian_impedance_control_B.bytes = j;
  cartesian_impedance_con_cfclose(static_cast<real_T>(fileid));
  if (j == 1) {
    cartesian_impedance_control_B.data_out = A_data;
    cartesian_impedance_control_DW.data_prev = A_data;
  } else {
    cartesian_impedance_control_B.data_out =
      cartesian_impedance_control_DW.data_prev;
    cartesian_impedance_control_DW.missed_data_cnt++;
  }

  cartesian_impedance_control_B.missed_data_cnt_o =
    cartesian_impedance_control_DW.missed_data_cnt;

  /* End of MATLAB Function: '<Root>/MATLAB Function3' */

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
  rtsiSetIsMinorTimeStepWithModeChange
    (&cartesian_impedance_control_M->solverInfo, false);
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
  cartesian_impedance_control_M->Sizes.checksums[0] = (3792122270U);
  cartesian_impedance_control_M->Sizes.checksums[1] = (1910117873U);
  cartesian_impedance_control_M->Sizes.checksums[2] = (382551798U);
  cartesian_impedance_control_M->Sizes.checksums[3] = (1584822028U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[3];
    cartesian_impedance_control_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
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
      simulinkPandaRobot_17216102 = SimulinkPandaRobot( "172.16.10.2",
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

  /* Start for S-Function (get_robot_state): '<S3>/Get Robot State2' */
  {
    cartesian_impedance_control_DW.GetRobotState2_DWORK1 = (double)
      simulinkPandaRobot_17216102.establishIfCurrentBlockFirstToBeComputed();
  }

  /* Start for S-Function (apply_control): '<S3>/Apply Control' */
  {
    //Flag for performing initialization in first run of main _step();
    cartesian_impedance_control_DW.ApplyControl_DWORK1 = 0;
    cartesian_impedance_control_DW.ApplyControl_DWORK2 = (double)
      simulinkPandaRobot_17216102.establishIfCurrentBlockFirstToBeComputed();
  }

  {
    FILE* a;

    /* InitializeConditions for RateLimiter: '<S3>/Rate Limiter' */
    for (int32_T i = 0; i < 7; i++) {
      cartesian_impedance_control_DW.PrevY[i] =
        cartesian_impedance_control_P.RateLimiter_IC;
    }

    /* End of InitializeConditions for RateLimiter: '<S3>/Rate Limiter' */

    /* SystemInitialize for MATLAB Function: '<Root>/MATLAB Function3' */
    a = NULL;
    for (int32_T i = 0; i < 20; i++) {
      cartesian_impedance_control_DW.eml_autoflush[i] = false;
      cartesian_impedance_control_DW.eml_openfiles[i] = a;
    }

    cartesian_impedance_control_DW.data_prev = 0.0;
    cartesian_impedance_control_DW.missed_data_cnt = 0.0;

    /* End of SystemInitialize for MATLAB Function: '<Root>/MATLAB Function3' */
  }
}

/* Model terminate function */
void cartesian_impedance_control_terminate(void)
{
  /* Terminate for S-Function (apply_control): '<S3>/Apply Control' */
  {
    /* S-Function Block: <S3>/Apply Control */
  }
}
