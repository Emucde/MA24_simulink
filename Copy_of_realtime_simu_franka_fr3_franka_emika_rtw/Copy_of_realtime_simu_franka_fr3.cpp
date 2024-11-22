/*
 * Copy_of_realtime_simu_franka_fr3.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Copy_of_realtime_simu_franka_fr3".
 *
 * Model version              : 8.567
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Fri Nov 22 16:40:23 2024
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "Copy_of_realtime_simu_franka_fr3.h"
#include "Copy_of_realtime_simu_franka_fr3_private.h"
#include <cstring>

extern "C"
{

#include "rt_nonfinite.h"

}

SimulinkPandaRobot simulinkPandaRobot_1721602;

/* Block states (default storage) */
DW_Copy_of_realtime_simu_fran_T Copy_of_realtime_simu_franka_DW;

/* Real-time model */
RT_MODEL_Copy_of_realtime_sim_T Copy_of_realtime_simu_franka_M_ =
  RT_MODEL_Copy_of_realtime_sim_T();
RT_MODEL_Copy_of_realtime_sim_T *const Copy_of_realtime_simu_franka_M =
  &Copy_of_realtime_simu_franka_M_;

/* Model step function */
void Copy_of_realtime_simu_franka_fr3_step(void)
{
  /* S-Function (apply_control): '<S1>/Apply Control' incorporates:
   *  Constant: '<Root>/Constant'
   */
  {
    /* S-Function Block: <S1>/Apply Control */
    if ((bool)Copy_of_realtime_simu_franka_DW.ApplyControl_DWORK1) {
      // Wait for the control thread signal
      if ((bool)Copy_of_realtime_simu_franka_DW.ApplyControl_DWORK2) {
        simulinkPandaRobot_1721602.waitForControlThreadStep();
      }

      // If control loop threw exeption terminate execution
      simulinkPandaRobot_1721602.checkIfAndHandleException();

      // copy inputs
      simulinkPandaRobot_1721602.copyInputSignal
        (&Copy_of_realtime_simu_franka__P.Constant_Value[0], 0);

      // notify control thread that the inputs have been read
      simulinkPandaRobot_1721602.notifyControlThreadToContinue();
    } else if (!(bool)Copy_of_realtime_simu_franka_DW.ApplyControl_DWORK1) {
      // Its the first time _step() function is called -->
      // Initialize according to settings parsed from the mask
      // and spawn control thread
      simulinkPandaRobot_1721602.applyRobotSettings();
      simulinkPandaRobot_1721602.spawnControlThread();
      Copy_of_realtime_simu_franka_DW.ApplyControl_DWORK1 = 1;
    }
  }

  /* Matfile logging */
  rt_UpdateTXYLogVars(Copy_of_realtime_simu_franka_M->rtwLogInfo,
                      (&Copy_of_realtime_simu_franka_M->Timing.taskTime0));

  /* External mode */
  rtExtModeUploadCheckTrigger(1);

  {                                    /* Sample time: [0.001s, 0.0s] */
    rtExtModeUpload(0, (real_T)Copy_of_realtime_simu_franka_M->Timing.taskTime0);
  }

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.001s, 0.0s] */
    if ((rtmGetTFinal(Copy_of_realtime_simu_franka_M)!=-1) &&
        !((rtmGetTFinal(Copy_of_realtime_simu_franka_M)-
           Copy_of_realtime_simu_franka_M->Timing.taskTime0) >
          Copy_of_realtime_simu_franka_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(Copy_of_realtime_simu_franka_M, "Simulation finished");
    }

    if (rtmGetStopRequested(Copy_of_realtime_simu_franka_M)) {
      rtmSetErrorStatus(Copy_of_realtime_simu_franka_M, "Simulation finished");
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
  if (!(++Copy_of_realtime_simu_franka_M->Timing.clockTick0)) {
    ++Copy_of_realtime_simu_franka_M->Timing.clockTickH0;
  }

  Copy_of_realtime_simu_franka_M->Timing.taskTime0 =
    Copy_of_realtime_simu_franka_M->Timing.clockTick0 *
    Copy_of_realtime_simu_franka_M->Timing.stepSize0 +
    Copy_of_realtime_simu_franka_M->Timing.clockTickH0 *
    Copy_of_realtime_simu_franka_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void Copy_of_realtime_simu_franka_fr3_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));
  rtmSetTFinal(Copy_of_realtime_simu_franka_M, -1);
  Copy_of_realtime_simu_franka_M->Timing.stepSize0 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    Copy_of_realtime_simu_franka_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(Copy_of_realtime_simu_franka_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(Copy_of_realtime_simu_franka_M->rtwLogInfo, (NULL));
    rtliSetLogT(Copy_of_realtime_simu_franka_M->rtwLogInfo, "tout");
    rtliSetLogX(Copy_of_realtime_simu_franka_M->rtwLogInfo, "");
    rtliSetLogXFinal(Copy_of_realtime_simu_franka_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(Copy_of_realtime_simu_franka_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(Copy_of_realtime_simu_franka_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(Copy_of_realtime_simu_franka_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(Copy_of_realtime_simu_franka_M->rtwLogInfo, 1);
    rtliSetLogY(Copy_of_realtime_simu_franka_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(Copy_of_realtime_simu_franka_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(Copy_of_realtime_simu_franka_M->rtwLogInfo, (NULL));
  }

  /* External mode info */
  Copy_of_realtime_simu_franka_M->Sizes.checksums[0] = (2885269858U);
  Copy_of_realtime_simu_franka_M->Sizes.checksums[1] = (1929557460U);
  Copy_of_realtime_simu_franka_M->Sizes.checksums[2] = (976990518U);
  Copy_of_realtime_simu_franka_M->Sizes.checksums[3] = (3928462425U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    Copy_of_realtime_simu_franka_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(Copy_of_realtime_simu_franka_M->extModeInfo,
      &Copy_of_realtime_simu_franka_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(Copy_of_realtime_simu_franka_M->extModeInfo,
                        Copy_of_realtime_simu_franka_M->Sizes.checksums);
    rteiSetTPtr(Copy_of_realtime_simu_franka_M->extModeInfo, rtmGetTPtr
                (Copy_of_realtime_simu_franka_M));
  }

  /* states (dwork) */
  (void) memset(static_cast<void *>(&Copy_of_realtime_simu_franka_DW), 0,
                sizeof(DW_Copy_of_realtime_simu_fran_T));

  /* block instance data */
  {
    {
      simulinkPandaRobot_1721602 = SimulinkPandaRobot( "172.16.0.2",
        0,
        0,
        0,
        Copy_of_realtime_simu_franka__P.ApplyControl_P1,
        Copy_of_realtime_simu_franka__P.ApplyControl_P2,
        Copy_of_realtime_simu_franka__P.ApplyControl_P3,
        Copy_of_realtime_simu_franka__P.ApplyControl_P4,
        Copy_of_realtime_simu_franka__P.ApplyControl_P5,
        1,
        Copy_of_realtime_simu_franka__P.ApplyControl_P6,
        1);
    }
  }

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(Copy_of_realtime_simu_franka_M->rtwLogInfo,
    0.0, rtmGetTFinal(Copy_of_realtime_simu_franka_M),
    Copy_of_realtime_simu_franka_M->Timing.stepSize0, (&rtmGetErrorStatus
    (Copy_of_realtime_simu_franka_M)));

  /* Start for S-Function (apply_control): '<S1>/Apply Control' incorporates:
   *  Constant: '<Root>/Constant'
   */
  {
    //Flag for performing initialization in first run of main _step();
    Copy_of_realtime_simu_franka_DW.ApplyControl_DWORK1 = 0;
    Copy_of_realtime_simu_franka_DW.ApplyControl_DWORK2 = (double)
      simulinkPandaRobot_1721602.establishIfCurrentBlockFirstToBeComputed();
  }
}

/* Model terminate function */
void Copy_of_realtime_simu_franka_fr3_terminate(void)
{
  /* Terminate for S-Function (apply_control): '<S1>/Apply Control' incorporates:
   *  Constant: '<Root>/Constant'
   */
  {
    /* S-Function Block: <S1>/Apply Control */
  }
}
