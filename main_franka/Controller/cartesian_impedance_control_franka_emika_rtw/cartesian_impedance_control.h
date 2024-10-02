/*
 * cartesian_impedance_control.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "cartesian_impedance_control".
 *
 * Model version              : 8.47
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Wed Oct  2 11:53:02 2024
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_cartesian_impedance_control_h_
#define RTW_HEADER_cartesian_impedance_control_h_
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rt_logging.h"
#include "ext_work.h"
#include "robot_api.h"
#include "franka/model.h"
#include "control_modes.h"
#include "gripper_api.h"
#include "DAHostLib_Network.h"
#include "cartesian_impedance_control_types.h"
#include <string.h>
#include <stddef.h>
#include <float.h>

extern "C"
{

#include "rt_nonfinite.h"

}

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
#define rtmGetRTWExtModeInfo(rtm)      ((rtm)->extModeInfo)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetSampleHitArray
#define rtmGetSampleHitArray(rtm)      ((rtm)->Timing.sampleHitArray)
#endif

#ifndef rtmGetStepSize
#define rtmGetStepSize(rtm)            ((rtm)->Timing.stepSize)
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGet_TimeOfLastOutput
#define rtmGet_TimeOfLastOutput(rtm)   ((rtm)->Timing.timeOfLastOutput)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

#ifndef rtmGetTimeOfLastOutput
#define rtmGetTimeOfLastOutput(rtm)    ((rtm)->Timing.timeOfLastOutput)
#endif

/* Block signals (default storage) */
struct B_cartesian_impedance_control_T {
  real_T GetRobotState2_o1[7];         /* '<S6>/Get Robot State2' */
  real_T GetRobotState2_o2[7];         /* '<S6>/Get Robot State2' */
  real_T GetRobotState2_o3[6];         /* '<S6>/Get Robot State2' */
  real_T Switch[7];                    /* '<S6>/Switch' */
  real_T u_opt[31];                    /* '<Root>/UDP Receive from c 1' */
  real_T Constant3[6];                 /* '<Root>/Constant3' */
  real_T SFunction1[6];                /* '<Root>/S-Function1' */
  real_T H[16];                        /* '<Root>/S-Function2' */
  real_T J[36];                        /* '<Root>/S-Function2' */
  real_T J_p[36];                      /* '<Root>/S-Function2' */
  real_T M[36];                        /* '<Root>/S-Function2' */
  real_T Cg[6];                        /* '<Root>/S-Function2' */
  real_T g[6];                         /* '<Root>/S-Function2' */
  real_T GetModel_o1[16];              /* '<S1>/Get Model' */
  real_T GetModel_o2[42];              /* '<S1>/Get Model' */
  real_T GetModel_o3[42];              /* '<S1>/Get Model' */
  real_T GetModel_o4[49];              /* '<S1>/Get Model' */
  real_T coriolis[7];                  /* '<S1>/Get Model' */
  real_T GetModel_o6[7];               /* '<S1>/Get Model' */
  real_T GetRobotState_o1[16];         /* '<Root>/Get Robot State' */
  real_T GetRobotState_o2[7];          /* '<Root>/Get Robot State' */
  real_T GetInitialRobotState[16];     /* '<Root>/Get Initial Robot State' */
  real_T RateLimiter[7];               /* '<S6>/Rate Limiter' */
  real_T q_o[6];                       /* '<Root>/get x' */
  real_T q_p_0[6];                     /* '<Root>/get x' */
  real_T TmpSignalConversionAtUDPSendInp[15];
  real_T x;                            /* '<Root>/MATLAB Function2' */
  real_T y;                            /* '<Root>/MATLAB Function2' */
  real_T z;                            /* '<Root>/MATLAB Function2' */
  real_T enable_out;                   /* '<Root>/MATLAB Function1' */
  real_T cnt_o;                        /* '<Root>/MATLAB Function1' */
  real_T send_cnt;                     /* '<Root>/MATLAB Function' */
  real_T init_cnt_o;                   /* '<Root>/MATLAB Function' */
  real_T data_cnt_o;                   /* '<Root>/MATLAB Function' */
  uint16_T UDPReceivefromc1_o2;        /* '<Root>/UDP Receive from c 1' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_cartesian_impedance_control_T {
  real_T GetRobotState2_DWORK1;        /* '<S6>/Get Robot State2' */
  real_T UDPReceivefromc1_NetworkLib[137];/* '<Root>/UDP Receive from c 1' */
  real_T Memory_PreviousInput;         /* '<Root>/Memory' */
  real_T Memory1_PreviousInput;        /* '<Root>/Memory1' */
  real_T Memory2_PreviousInput;        /* '<Root>/Memory2' */
  real_T GetModel_DWORK1;              /* '<S1>/Get Model' */
  real_T GetRobotState_DWORK1;         /* '<Root>/Get Robot State' */
  real_T GetInitialRobotState_DWORK1;  /* '<Root>/Get Initial Robot State' */
  real_T GetInitialRobotState_DWORK2;  /* '<Root>/Get Initial Robot State' */
  real_T PrevY[7];                     /* '<S6>/Rate Limiter' */
  real_T ApplyControl_DWORK1;          /* '<S6>/Apply Control' */
  real_T ApplyControl_DWORK2;          /* '<S6>/Apply Control' */
  real_T UDPSend_NetworkLib[137];      /* '<S7>/UDP Send' */
  void *SFunction1_PWORK[4];           /* '<Root>/S-Function1' */
  void *SFunction2_PWORK[8];           /* '<Root>/S-Function2' */
  struct {
    void *LoggedData[3];
  } Scope_PWORK;                       /* '<Root>/Scope' */

  struct {
    void *LoggedData[3];
  } Scope3_PWORK;                      /* '<Root>/Scope3' */

  struct {
    void *LoggedData[3];
  } Scope4_PWORK;                      /* '<Root>/Scope4' */

  struct {
    void *LoggedData[2];
  } Scope1_PWORK;                      /* '<Root>/Scope1' */

  int8_T UDPsendtoc_SubsysRanBC;       /* '<Root>/UDP send to c' */
  boolean_T UDPsendtoc_MODE;           /* '<Root>/UDP send to c' */
};

/* Parameters (default storage) */
struct P_cartesian_impedance_control_T_ {
  real_T q_init[7];                    /* Variable: q_init
                                        * Referenced by: '<S6>/Constant'
                                        */
  int32_T UDPReceivefromc1_localPort;
                                   /* Mask Parameter: UDPReceivefromc1_localPort
                                    * Referenced by: '<Root>/UDP Receive from c 1'
                                    */
  int32_T UDPSend_remotePort;          /* Mask Parameter: UDPSend_remotePort
                                        * Referenced by: '<S7>/UDP Send'
                                        */
  real_T Switch_Threshold;             /* Expression: 0.001
                                        * Referenced by: '<S6>/Switch'
                                        */
  real_T Memory_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<Root>/Memory'
                                        */
  real_T Constant3_Value[6];           /* Expression: zeros(6,1)
                                        * Referenced by: '<Root>/Constant3'
                                        */
  real_T Memory1_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<Root>/Memory1'
                                        */
  real_T Memory2_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<Root>/Memory2'
                                        */
  real_T stiffness_Value[36];
                          /* Expression: diag([600, 600, 600, 50.0, 50.0, 50.0])
                           * Referenced by: '<S1>/stiffness'
                           */
  real_T damping_Value[36];
            /* Expression: diag(2*sqrt([150.0, 150.0, 150.0, 50.0, 50.0, 50.0]))
             * Referenced by: '<S1>/damping'
             */
  real_T RateLimiter_RisingLim;        /* Expression: 1000
                                        * Referenced by: '<S6>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim;       /* Expression: -1000
                                        * Referenced by: '<S6>/Rate Limiter'
                                        */
  real_T RateLimiter_IC;               /* Expression: 0
                                        * Referenced by: '<S6>/Rate Limiter'
                                        */
  real_T ApplyControl_P1[52];          /* Expression: collision_thresholds
                                        * Referenced by: '<S6>/Apply Control'
                                        */
  real_T ApplyControl_P2[7];           /* Expression: joint_impedance
                                        * Referenced by: '<S6>/Apply Control'
                                        */
  real_T ApplyControl_P3[6];           /* Expression: cartesian_impedance
                                        * Referenced by: '<S6>/Apply Control'
                                        */
  real_T ApplyControl_P4[13];          /* Expression: load_inertia
                                        * Referenced by: '<S6>/Apply Control'
                                        */
  real_T ApplyControl_P5[16];          /* Expression: EE_T_K
                                        * Referenced by: '<S6>/Apply Control'
                                        */
  real_T ApplyControl_P6[7];           /* Expression: init_joint_configuration
                                        * Referenced by: '<S6>/Apply Control'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_cartesian_impedance_control_T {
  struct SimStruct_tag * *childSfunctions;
  const char_T *errorStatus;
  SS_SimMode simMode;
  RTWLogInfo *rtwLogInfo;
  RTWExtModeInfo *extModeInfo;
  RTWSolverInfo solverInfo;
  RTWSolverInfo *solverInfoPtr;
  void *sfcnInfo;

  /*
   * NonInlinedSFcns:
   * The following substructure contains information regarding
   * non-inlined s-functions used in the model.
   */
  struct {
    RTWSfcnInfo sfcnInfo;
    time_T *taskTimePtrs[2];
    SimStruct childSFunctions[2];
    SimStruct *childSFunctionPtrs[2];
    struct _ssBlkInfo2 blkInfo2[2];
    struct _ssSFcnModelMethods2 methods2[2];
    struct _ssSFcnModelMethods3 methods3[2];
    struct _ssSFcnModelMethods4 methods4[2];
    struct _ssStatesInfo2 statesInfo2[2];
    ssPeriodicStatesInfo periodicStatesInfo[2];
    struct _ssPortInfo2 inputOutputPortInfo2[2];
    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[3];
      struct _ssInPortUnit inputPortUnits[3];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[3];
      int_T iDims0[2];
      int_T iDims1[2];
      int_T iDims2[2];
      struct _ssPortOutputs outputPortInfo[1];
      struct _ssOutPortUnit outputPortUnits[1];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[1];
      int_T oDims0[2];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn0;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[2];
      struct _ssInPortUnit inputPortUnits[2];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[2];
      int_T iDims0[2];
      int_T iDims1[2];
      struct _ssPortOutputs outputPortInfo[6];
      struct _ssOutPortUnit outputPortUnits[6];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[6];
      int_T oDims0[2];
      int_T oDims1[2];
      int_T oDims2[2];
      int_T oDims3[2];
      int_T oDims4[2];
      int_T oDims5[2];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn1;
  } NonInlinedSFcns;

  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T checksums[4];
    uint32_T options;
    int_T numContStates;
    int_T numU;
    int_T numY;
    int_T numSampTimes;
    int_T numBlocks;
    int_T numBlockIO;
    int_T numBlockPrms;
    int_T numDwork;
    int_T numSFcnPrms;
    int_T numSFcns;
    int_T numIports;
    int_T numOports;
    int_T numNonSampZCs;
    int_T sysDirFeedThru;
    int_T rtwGenSfcn;
  } Sizes;

  /*
   * SpecialInfo:
   * The following substructure contains special information
   * related to other components that are dependent on RTW.
   */
  struct {
    const void *mappingInfo;
  } SpecialInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T stepSize;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tStart;
    time_T tFinal;
    time_T timeOfLastOutput;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *sampleTimes;
    time_T *offsetTimes;
    int_T *sampleTimeTaskIDPtr;
    int_T *sampleHits;
    int_T *perTaskSampleHits;
    time_T *t;
    time_T sampleTimesArray[2];
    time_T offsetTimesArray[2];
    int_T sampleTimeTaskIDArray[2];
    int_T sampleHitArray[2];
    int_T perTaskSampleHitsArray[4];
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
#ifdef __cplusplus

extern "C"
{

#endif

  extern P_cartesian_impedance_control_T cartesian_impedance_control_P;

#ifdef __cplusplus

}

#endif

/* Block signals (default storage) */
#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_cartesian_impedance_control_T cartesian_impedance_control_B;

#ifdef __cplusplus

}

#endif

/* Block states (default storage) */
extern struct DW_cartesian_impedance_control_T cartesian_impedance_control_DW;

#ifdef __cplusplus

extern "C"
{

#endif

  /* Model entry point functions */
  extern void cartesian_impedance_control_initialize(void);
  extern void cartesian_impedance_control_step(void);
  extern void cartesian_impedance_control_terminate(void);

#ifdef __cplusplus

}

#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_cartesian_impedance_control_T *const
    cartesian_impedance_control_M;

#ifdef __cplusplus

}

#endif

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'cartesian_impedance_control'
 * '<S1>'   : 'cartesian_impedance_control/Cartesian Impedance Controller'
 * '<S2>'   : 'cartesian_impedance_control/MATLAB Function'
 * '<S3>'   : 'cartesian_impedance_control/MATLAB Function1'
 * '<S4>'   : 'cartesian_impedance_control/MATLAB Function2'
 * '<S5>'   : 'cartesian_impedance_control/Robot model bus'
 * '<S6>'   : 'cartesian_impedance_control/Subsystem'
 * '<S7>'   : 'cartesian_impedance_control/UDP send to c'
 * '<S8>'   : 'cartesian_impedance_control/get x'
 * '<S9>'   : 'cartesian_impedance_control/Cartesian Impedance Controller/MATLAB Function'
 */
#endif                           /* RTW_HEADER_cartesian_impedance_control_h_ */
