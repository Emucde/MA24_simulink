/*
 * cartesian_impedance_control_test.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "cartesian_impedance_control_test".
 *
 * Model version              : 8.63
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Wed Oct  2 16:25:21 2024
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "cartesian_impedance_control_test.h"
#include "rtwtypes.h"
#include "cartesian_impedance_control_test_private.h"
#include <cstring>

extern "C"
{

#include "rt_nonfinite.h"

}

const real_T cartesian_impedance_cont_period = 0.001;
SimulinkPandaRobot simulinkPandaRobot_17216102;

/* Block signals (default storage) */
B_cartesian_impedance_control_T cartesian_impedance_control_t_B;

/* Block states (default storage) */
DW_cartesian_impedance_contro_T cartesian_impedance_control__DW;

/* Real-time model */
RT_MODEL_cartesian_impedance__T cartesian_impedance_control__M_ =
  RT_MODEL_cartesian_impedance__T();
RT_MODEL_cartesian_impedance__T *const cartesian_impedance_control__M =
  &cartesian_impedance_control__M_;

/* Model step function */
void cartesian_impedance_control_test_step(void)
{
  real_T rtb_Clock;
  real_T tmp;

  /* S-Function (get_robot_state): '<S1>/Get Robot State2' */
  {
    // Wait for the control thread signal
    if ((bool)cartesian_impedance_control__DW.GetRobotState2_DWORK1 &&
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
        &cartesian_impedance_control_t_B.GetRobotState2_o1[0]);
      simulinkPandaRobot_17216102.copyOutputSignal(output_signals, 1,
        &cartesian_impedance_control_t_B.GetRobotState2_o2[0]);
      simulinkPandaRobot_17216102.copyOutputSignal(output_signals, 2,
        &cartesian_impedance_control_t_B.GetRobotState2_o3[0]);
    }
  }

  /* Clock: '<S1>/Clock' */
  rtb_Clock = cartesian_impedance_control__M->Timing.t[0];
  for (int32_T i = 0; i < 7; i++) {
    /* Switch: '<S1>/Switch' */
    if (rtb_Clock > cartesian_impedance_control_t_P.Switch_Threshold) {
      /* Switch: '<S1>/Switch' */
      cartesian_impedance_control_t_B.Switch[i] =
        cartesian_impedance_control_t_B.GetRobotState2_o1[i];
    } else {
      /* Switch: '<S1>/Switch' incorporates:
       *  Constant: '<S1>/Constant'
       */
      cartesian_impedance_control_t_B.Switch[i] =
        cartesian_impedance_control_t_P.q_init[i];
    }

    /* End of Switch: '<S1>/Switch' */
  }

  /* RateLimiter: '<S1>/Rate Limiter' incorporates:
   *  Constant: '<Root>/Constant'
   */
  rtb_Clock = cartesian_impedance_control_t_P.RateLimiter_RisingLim *
    cartesian_impedance_cont_period;
  tmp = cartesian_impedance_control_t_P.RateLimiter_FallingLim *
    cartesian_impedance_cont_period;
  for (int32_T i = 0; i < 7; i++) {
    cartesian_impedance_control_t_B.RateLimiter[i] =
      cartesian_impedance_control_t_P.Constant_Value[i] -
      cartesian_impedance_control__DW.PrevY[i];
    if (cartesian_impedance_control_t_B.RateLimiter[i] > rtb_Clock) {
      /* RateLimiter: '<S1>/Rate Limiter' */
      cartesian_impedance_control_t_B.RateLimiter[i] =
        cartesian_impedance_control__DW.PrevY[i] + rtb_Clock;
    } else if (cartesian_impedance_control_t_B.RateLimiter[i] < tmp) {
      /* RateLimiter: '<S1>/Rate Limiter' */
      cartesian_impedance_control_t_B.RateLimiter[i] =
        cartesian_impedance_control__DW.PrevY[i] + tmp;
    } else {
      /* RateLimiter: '<S1>/Rate Limiter' */
      cartesian_impedance_control_t_B.RateLimiter[i] =
        cartesian_impedance_control_t_P.Constant_Value[i];
    }

    cartesian_impedance_control__DW.PrevY[i] =
      cartesian_impedance_control_t_B.RateLimiter[i];
  }

  /* End of RateLimiter: '<S1>/Rate Limiter' */

  /* S-Function (apply_control): '<S1>/Apply Control' */
  {
    /* S-Function Block: <S1>/Apply Control */
    if ((bool)cartesian_impedance_control__DW.ApplyControl_DWORK1) {
      // Wait for the control thread signal
      if ((bool)cartesian_impedance_control__DW.ApplyControl_DWORK2) {
        simulinkPandaRobot_17216102.waitForControlThreadStep();
      }

      // If control loop threw exeption terminate execution
      simulinkPandaRobot_17216102.checkIfAndHandleException();

      // copy inputs
      simulinkPandaRobot_17216102.copyInputSignal
        (&cartesian_impedance_control_t_B.RateLimiter[0], 0);

      // notify control thread that the inputs have been read
      simulinkPandaRobot_17216102.notifyControlThreadToContinue();
    } else if (!(bool)cartesian_impedance_control__DW.ApplyControl_DWORK1) {
      // Its the first time _step() function is called -->
      // Initialize according to settings parsed from the mask
      // and spawn control thread
      simulinkPandaRobot_17216102.applyRobotSettings();
      simulinkPandaRobot_17216102.spawnControlThread();
      cartesian_impedance_control__DW.ApplyControl_DWORK1 = 1;
    }
  }

  /* Matfile logging */
  rt_UpdateTXYLogVars(cartesian_impedance_control__M->rtwLogInfo,
                      (cartesian_impedance_control__M->Timing.t));

  /* External mode */
  rtExtModeUploadCheckTrigger(2);

  {                                    /* Sample time: [0.0s, 0.0s] */
    rtExtModeUpload(0, (real_T)cartesian_impedance_control__M->Timing.t[0]);
  }

  {                                    /* Sample time: [0.001s, 0.0s] */
    rtExtModeUpload(1, (real_T)
                    (((cartesian_impedance_control__M->Timing.clockTick1+
                       cartesian_impedance_control__M->Timing.clockTickH1*
                       4294967296.0)) * 0.001));
  }

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.0s, 0.0s] */
    if ((rtmGetTFinal(cartesian_impedance_control__M)!=-1) &&
        !((rtmGetTFinal(cartesian_impedance_control__M)-
           cartesian_impedance_control__M->Timing.t[0]) >
          cartesian_impedance_control__M->Timing.t[0] * (DBL_EPSILON))) {
      rtmSetErrorStatus(cartesian_impedance_control__M, "Simulation finished");
    }

    if (rtmGetStopRequested(cartesian_impedance_control__M)) {
      rtmSetErrorStatus(cartesian_impedance_control__M, "Simulation finished");
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
  if (!(++cartesian_impedance_control__M->Timing.clockTick0)) {
    ++cartesian_impedance_control__M->Timing.clockTickH0;
  }

  cartesian_impedance_control__M->Timing.t[0] =
    cartesian_impedance_control__M->Timing.clockTick0 *
    cartesian_impedance_control__M->Timing.stepSize0 +
    cartesian_impedance_control__M->Timing.clockTickH0 *
    cartesian_impedance_control__M->Timing.stepSize0 * 4294967296.0;

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
    cartesian_impedance_control__M->Timing.clockTick1++;
    if (!cartesian_impedance_control__M->Timing.clockTick1) {
      cartesian_impedance_control__M->Timing.clockTickH1++;
    }
  }
}

/* Model initialize function */
void cartesian_impedance_control_test_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&cartesian_impedance_control__M->solverInfo,
                          &cartesian_impedance_control__M->Timing.simTimeStep);
    rtsiSetTPtr(&cartesian_impedance_control__M->solverInfo, &rtmGetTPtr
                (cartesian_impedance_control__M));
    rtsiSetStepSizePtr(&cartesian_impedance_control__M->solverInfo,
                       &cartesian_impedance_control__M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&cartesian_impedance_control__M->solverInfo,
                          (&rtmGetErrorStatus(cartesian_impedance_control__M)));
    rtsiSetRTModelPtr(&cartesian_impedance_control__M->solverInfo,
                      cartesian_impedance_control__M);
  }

  rtsiSetSimTimeStep(&cartesian_impedance_control__M->solverInfo,
                     MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange
    (&cartesian_impedance_control__M->solverInfo, false);
  rtsiSetSolverName(&cartesian_impedance_control__M->solverInfo,
                    "FixedStepDiscrete");
  rtmSetTPtr(cartesian_impedance_control__M,
             &cartesian_impedance_control__M->Timing.tArray[0]);
  rtmSetTFinal(cartesian_impedance_control__M, -1);
  cartesian_impedance_control__M->Timing.stepSize0 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    cartesian_impedance_control__M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(cartesian_impedance_control__M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(cartesian_impedance_control__M->rtwLogInfo, (NULL));
    rtliSetLogT(cartesian_impedance_control__M->rtwLogInfo, "tout");
    rtliSetLogX(cartesian_impedance_control__M->rtwLogInfo, "");
    rtliSetLogXFinal(cartesian_impedance_control__M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(cartesian_impedance_control__M->rtwLogInfo, "rt_");
    rtliSetLogFormat(cartesian_impedance_control__M->rtwLogInfo, 0);
    rtliSetLogMaxRows(cartesian_impedance_control__M->rtwLogInfo, 1000);
    rtliSetLogDecimation(cartesian_impedance_control__M->rtwLogInfo, 1);
    rtliSetLogY(cartesian_impedance_control__M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(cartesian_impedance_control__M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(cartesian_impedance_control__M->rtwLogInfo, (NULL));
  }

  /* External mode info */
  cartesian_impedance_control__M->Sizes.checksums[0] = (2248919816U);
  cartesian_impedance_control__M->Sizes.checksums[1] = (2298490692U);
  cartesian_impedance_control__M->Sizes.checksums[2] = (1114522479U);
  cartesian_impedance_control__M->Sizes.checksums[3] = (1152130774U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    cartesian_impedance_control__M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(cartesian_impedance_control__M->extModeInfo,
      &cartesian_impedance_control__M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(cartesian_impedance_control__M->extModeInfo,
                        cartesian_impedance_control__M->Sizes.checksums);
    rteiSetTPtr(cartesian_impedance_control__M->extModeInfo, rtmGetTPtr
                (cartesian_impedance_control__M));
  }

  /* block I/O */
  (void) memset((static_cast<void *>(&cartesian_impedance_control_t_B)), 0,
                sizeof(B_cartesian_impedance_control_T));

  /* states (dwork) */
  (void) memset(static_cast<void *>(&cartesian_impedance_control__DW), 0,
                sizeof(DW_cartesian_impedance_contro_T));

  /* block instance data */
  {
    {
      simulinkPandaRobot_17216102 = SimulinkPandaRobot( "172.16.10.2",
        0,
        0,
        0,
        cartesian_impedance_control_t_P.ApplyControl_P1,
        cartesian_impedance_control_t_P.ApplyControl_P2,
        cartesian_impedance_control_t_P.ApplyControl_P3,
        cartesian_impedance_control_t_P.ApplyControl_P4,
        cartesian_impedance_control_t_P.ApplyControl_P5,
        1,
        cartesian_impedance_control_t_P.ApplyControl_P6,
        1);
    }
  }

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(cartesian_impedance_control__M->rtwLogInfo,
    0.0, rtmGetTFinal(cartesian_impedance_control__M),
    cartesian_impedance_control__M->Timing.stepSize0, (&rtmGetErrorStatus
    (cartesian_impedance_control__M)));

  /* Start for S-Function (get_robot_state): '<S1>/Get Robot State2' */
  {
    cartesian_impedance_control__DW.GetRobotState2_DWORK1 = (double)
      simulinkPandaRobot_17216102.establishIfCurrentBlockFirstToBeComputed();
  }

  /* Start for S-Function (apply_control): '<S1>/Apply Control' */
  {
    //Flag for performing initialization in first run of main _step();
    cartesian_impedance_control__DW.ApplyControl_DWORK1 = 0;
    cartesian_impedance_control__DW.ApplyControl_DWORK2 = (double)
      simulinkPandaRobot_17216102.establishIfCurrentBlockFirstToBeComputed();
  }

  /* InitializeConditions for RateLimiter: '<S1>/Rate Limiter' */
  for (int32_T i = 0; i < 7; i++) {
    cartesian_impedance_control__DW.PrevY[i] =
      cartesian_impedance_control_t_P.RateLimiter_IC;
  }

  /* End of InitializeConditions for RateLimiter: '<S1>/Rate Limiter' */
}

/* Model terminate function */
void cartesian_impedance_control_test_terminate(void)
{
  /* Terminate for S-Function (apply_control): '<S1>/Apply Control' */
  {
    /* S-Function Block: <S1>/Apply Control */
  }
}
