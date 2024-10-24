/*
 * realtime_simu_franka_fr3.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "realtime_simu_franka_fr3".
 *
 * Model version              : 8.320
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Thu Oct 24 16:17:01 2024
 *
 * Target selection: franka_emika_panda.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_realtime_simu_franka_fr3_h_
#define RTW_HEADER_realtime_simu_franka_fr3_h_
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rt_logging.h"
#include "ext_work.h"
#include "robot_api.h"
#include "control_modes.h"
#include "gripper_api.h"
#include "realtime_simu_franka_fr3_types.h"

extern "C"
{

#include "rtGetNaN.h"

}

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetInf.h"

}

#include <stddef.h>
#include <string.h>
#include <float.h>

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
struct B_realtime_simu_franka_fr3_T {
  robot_model robot_model_o;           /* '<Root>/Robot model bus' */
  real_T GetRobotState2_o1[7];         /* '<S8>/Get Robot State2' */
  real_T GetRobotState2_o2[7];         /* '<S8>/Get Robot State2' */
  real_T GetRobotState2_o3[6];         /* '<S8>/Get Robot State2' */
  real_T Switch[7];                    /* '<S8>/Switch' */
  real_T SFunction3_o1[6];             /* '<Root>/S-Function3' */
  real_T SFunction3_o2;                /* '<Root>/S-Function3' */
  real_T ManualSwitch[14];             /* '<Root>/Manual Switch' */
  real_T tau_in[7];                    /* '<Root>/tau_in' */
  real_T SFunction1[7];                /* '<Root>/S-Function1' */
  real_T H[16];                        /* '<Root>/S-Function2' */
  real_T J[42];                        /* '<Root>/S-Function2' */
  real_T J_p[42];                      /* '<Root>/S-Function2' */
  real_T M[49];                        /* '<Root>/S-Function2' */
  real_T Cqg[7];                       /* '<Root>/S-Function2' */
  real_T g[49];                        /* '<Root>/S-Function2' */
  real_T g_f[7];                       /* '<Root>/S-Function2' */
  real_T Gain[7];                      /* '<Root>/Gain' */
  real_T RateLimiter[7];               /* '<S8>/Rate Limiter' */
  real_T TmpSignalConversionAtTAQSigLogg[6];
  /* '<Root>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_Mux2_at_outport_0Inport1' */
  real_T p_d[3];                       /* '<Root>/get data' */
  real_T tau[2];                 /* '<Root>/Reduced Cartesian PD+ Controller' */
  real_T Constant18[395];              /* '<S5>/Constant18' */
  real_T init_guess[92];               /* '<S5>/init_guess' */
  real_T SFunction_o1[2];              /* '<S5>/S-Function' */
  real_T SFunction_o2[92];             /* '<S5>/S-Function' */
  real_T J_yt;                         /* '<S5>/S-Function' */
  real_T J_yr_N;                       /* '<S5>/S-Function' */
  real_T J_yr;                         /* '<S5>/S-Function' */
  real_T SFunction_o6;                 /* '<S5>/S-Function' */
  real_T SFunction_o7;                 /* '<S5>/S-Function' */
  real_T SFunction_o8;                 /* '<S5>/S-Function' */
  real_T mpc_refs[70];                 /* '<S12>/get data' */
  real_T tau_f[2];                     /* '<S5>/CT Regler' */
  real_T simulink_valid_flag;          /* '<Root>/MATLAB Function3' */
  real_T p[3];                         /* '<Root>/MATLAB Function2' */
  real_T tau_c[7];                     /* '<Root>/Joinspace controller' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_realtime_simu_franka_fr3_T {
  real_T init_guess_DSTATE[92];        /* '<S5>/init_guess' */
  real_T GetRobotState2_DWORK1;        /* '<S8>/Get Robot State2' */
  real_T PrevY[7];                     /* '<S8>/Rate Limiter' */
  real_T LastMajorTime;                /* '<S8>/Rate Limiter' */
  real_T ApplyControl_DWORK1;          /* '<S8>/Apply Control' */
  real_T ApplyControl_DWORK2;          /* '<S8>/Apply Control' */
  real_T cnt;                          /* '<Root>/get data' */
  real_T cnt_g;                        /* '<S12>/get data' */
  real_T SFunction_RWORK[3220];        /* '<S5>/S-Function' */
  void *SFunction3_PWORK[4];           /* '<Root>/S-Function3' */
  void *SFunction4_PWORK[4];           /* '<Root>/S-Function4' */
  void *SFunction1_PWORK[4];           /* '<Root>/S-Function1' */
  void *SFunction2_PWORK[9];           /* '<Root>/S-Function2' */
  struct {
    void *LoggedData[4];
  } Scope5_PWORK;                      /* '<Root>/Scope5' */

  struct {
    void *LoggedData[3];
  } Scope_PWORK;                       /* '<Root>/Scope' */

  struct {
    void *LoggedData;
  } errscopeall_PWORK;                 /* '<Root>/err scope all' */

  struct {
    void *LoggedData[2];
  } Scope6_PWORK;                      /* '<Root>/Scope6' */

  struct {
    void *LoggedData;
  } Scope4_PWORK;                      /* '<Root>/Scope4' */

  void *SFunction_PWORK[41];           /* '<S5>/S-Function' */
  struct {
    void *LoggedData;
  } Scope_PWORK_j;                     /* '<S5>/Scope' */

  struct {
    void *LoggedData[7];
  } costfunctionscope_PWORK;           /* '<S5>/cost function scope' */

  int_T SFunction_IWORK[290];          /* '<S5>/S-Function' */
  int8_T MPC8offlinecontrollernlpsolveqr;
             /* '<Root>/MPC8 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  boolean_T icLoad;                    /* '<S5>/init_guess' */
  boolean_T MPC8offlinecontrollernlpsolve_m;
             /* '<Root>/MPC8 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
};

/* Parameters (default storage) */
struct P_realtime_simu_franka_fr3_T_ {
  struct_Yw0vgIQ6yZpeProSkAAsrC param_weight_init;/* Variable: param_weight_init
                                                   * Referenced by: '<S5>/Constant18'
                                                   */
  struct_RCS9FomuFmdbboJvGAYmbB ctrl_param;/* Variable: ctrl_param
                                            * Referenced by: '<Root>/Reduced Cartesian PD+ Controller'
                                            */
  real_T q_init[7];                    /* Variable: q_init
                                        * Referenced by: '<S8>/Constant'
                                        */
  x_d x_d_Y0;                          /* Computed Parameter: x_d_Y0
                                        * Referenced by: '<S5>/x_d'
                                        */
  real_T tau_Y0;                       /* Computed Parameter: tau_Y0
                                        * Referenced by: '<S5>/tau'
                                        */
  real_T Constant_Value[6];            /* Expression: param_MPC8.traj_indices
                                        * Referenced by: '<S5>/Constant'
                                        */
  real_T Switch_Threshold;             /* Expression: 0.001
                                        * Referenced by: '<S8>/Switch'
                                        */
  real_T SFunction3_P1_Size[2];        /* Computed Parameter: SFunction3_P1_Size
                                        * Referenced by: '<Root>/S-Function3'
                                        */
  real_T SFunction3_P1[16];            /* Computed Parameter: SFunction3_P1
                                        * Referenced by: '<Root>/S-Function3'
                                        */
  real_T SFunction3_P2_Size[2];        /* Computed Parameter: SFunction3_P2_Size
                                        * Referenced by: '<Root>/S-Function3'
                                        */
  real_T SFunction3_P2[22];            /* Computed Parameter: SFunction3_P2
                                        * Referenced by: '<Root>/S-Function3'
                                        */
  real_T Constant6_Value[8];           /* Expression: zeros(8,1)
                                        * Referenced by: '<Root>/Constant6'
                                        */
  real_T SFunction4_P1_Size[2];        /* Computed Parameter: SFunction4_P1_Size
                                        * Referenced by: '<Root>/S-Function4'
                                        */
  real_T SFunction4_P1[18];            /* Computed Parameter: SFunction4_P1
                                        * Referenced by: '<Root>/S-Function4'
                                        */
  real_T SFunction4_P2_Size[2];        /* Computed Parameter: SFunction4_P2_Size
                                        * Referenced by: '<Root>/S-Function4'
                                        */
  real_T SFunction4_P2[24];            /* Computed Parameter: SFunction4_P2
                                        * Referenced by: '<Root>/S-Function4'
                                        */
  real_T tau_in_Value[7];              /* Expression: zeros(7,1)
                                        * Referenced by: '<Root>/tau_in'
                                        */
  real_T Constant5_Value;              /* Expression: 5
                                        * Referenced by: '<Root>/Constant5'
                                        */
  real_T Constant4_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/Constant4'
                                        */
  real_T Constant3_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/Constant3'
                                        */
  real_T K_d_Value[6];
               /* Expression: 100*[2.25    2.25    2.25    2.25    2.25    2.25]
                * Referenced by: '<Root>/K_d'
                */
  real_T D_d_Value[6];        /* Expression: 8*[3     3     3     3     3     3]
                               * Referenced by: '<Root>/D_d'
                               */
  real_T q_ref_Value[7];  /* Expression: [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4]
                           * Referenced by: '<Root>/q_ref'
                           */
  real_T K_d_jointspace1_Value[7];     /* Expression: [100 200 500 200 50 50 10]
                                        * Referenced by: '<Root>/K_d_jointspace1'
                                        */
  real_T Gain_Gain;                    /* Expression: sqrt(2)
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T D_d_jointspace2_Value[7];     /* Expression: [0 0 0 0 0 0 0]
                                        * Referenced by: '<Root>/D_d_jointspace2'
                                        */
  real_T K_d_jointspace2_Value[7];     /* Expression: [100 200 500 200 50 50 10]
                                        * Referenced by: '<Root>/K_d_jointspace2'
                                        */
  real_T RateLimiter_RisingLim;        /* Expression: 1000
                                        * Referenced by: '<S8>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim;       /* Expression: -1000
                                        * Referenced by: '<S8>/Rate Limiter'
                                        */
  real_T ApplyControl_P1[52];          /* Expression: collision_thresholds
                                        * Referenced by: '<S8>/Apply Control'
                                        */
  real_T ApplyControl_P2[7];           /* Expression: joint_impedance
                                        * Referenced by: '<S8>/Apply Control'
                                        */
  real_T ApplyControl_P3[6];           /* Expression: cartesian_impedance
                                        * Referenced by: '<S8>/Apply Control'
                                        */
  real_T ApplyControl_P4[13];          /* Expression: load_inertia
                                        * Referenced by: '<S8>/Apply Control'
                                        */
  real_T ApplyControl_P5[16];          /* Expression: EE_T_K
                                        * Referenced by: '<S8>/Apply Control'
                                        */
  real_T ApplyControl_P6[7];           /* Expression: init_joint_configuration
                                        * Referenced by: '<S8>/Apply Control'
                                        */
  uint8_T ManualSwitch2_CurrentSetting;
                             /* Computed Parameter: ManualSwitch2_CurrentSetting
                              * Referenced by: '<Root>/Manual Switch2'
                              */
  uint8_T ManualSwitch_CurrentSetting;
                              /* Computed Parameter: ManualSwitch_CurrentSetting
                               * Referenced by: '<Root>/Manual Switch'
                               */
  uint8_T ManualSwitch3_CurrentSetting;
                             /* Computed Parameter: ManualSwitch3_CurrentSetting
                              * Referenced by: '<Root>/Manual Switch3'
                              */
  uint8_T ManualSwitch1_CurrentSetting;
                             /* Computed Parameter: ManualSwitch1_CurrentSetting
                              * Referenced by: '<Root>/Manual Switch1'
                              */
};

/* Real-time Model Data Structure */
struct tag_RTM_realtime_simu_franka_fr3_T {
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
    SimStruct childSFunctions[5];
    SimStruct *childSFunctionPtrs[5];
    struct _ssBlkInfo2 blkInfo2[5];
    struct _ssSFcnModelMethods2 methods2[5];
    struct _ssSFcnModelMethods3 methods3[5];
    struct _ssSFcnModelMethods4 methods4[5];
    struct _ssStatesInfo2 statesInfo2[5];
    ssPeriodicStatesInfo periodicStatesInfo[5];
    struct _ssPortInfo2 inputOutputPortInfo2[5];
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
      struct _ssPortOutputs outputPortInfo[8];
      struct _ssOutPortUnit outputPortUnits[8];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[8];
      int_T oDims0[2];
      int_T oDims1[2];
      int_T oDims2[2];
      int_T oDims3[2];
      int_T oDims4[2];
      int_T oDims5[2];
      int_T oDims6[2];
      int_T oDims7[2];
      struct _ssDWorkRecord dWork[3];
      struct _ssDWorkAuxRecord dWorkAux[3];
    } Sfcn0;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortOutputs outputPortInfo[2];
      struct _ssOutPortUnit outputPortUnits[2];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[2];
      uint_T attribs[2];
      mxArray *params[2];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn1;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[2];
      struct _ssInPortUnit inputPortUnits[2];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[2];
      uint_T attribs[2];
      mxArray *params[2];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn2;

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
    } Sfcn3;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[2];
      struct _ssInPortUnit inputPortUnits[2];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[2];
      int_T iDims0[2];
      int_T iDims1[2];
      struct _ssPortOutputs outputPortInfo[7];
      struct _ssOutPortUnit outputPortUnits[7];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[7];
      int_T oDims0[2];
      int_T oDims1[2];
      int_T oDims2[2];
      int_T oDims3[2];
      int_T oDims4[2];
      int_T oDims5[2];
      int_T oDims6[2];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn4;
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
    time_T stepSize1;
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

  extern P_realtime_simu_franka_fr3_T realtime_simu_franka_fr3_P;

#ifdef __cplusplus

}

#endif

/* Block signals (default storage) */
#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_realtime_simu_franka_fr3_T realtime_simu_franka_fr3_B;

#ifdef __cplusplus

}

#endif

/* Block states (default storage) */
extern struct DW_realtime_simu_franka_fr3_T realtime_simu_franka_fr3_DW;

#ifdef __cplusplus

extern "C"
{

#endif

  /* Model entry point functions */
  extern void realtime_simu_franka_fr3_initialize(void);
  extern void realtime_simu_franka_fr3_step(void);
  extern void realtime_simu_franka_fr3_terminate(void);

#ifdef __cplusplus

}

#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_realtime_simu_franka_fr3_T *const realtime_simu_franka_fr3_M;

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
 * '<Root>' : 'realtime_simu_franka_fr3'
 * '<S1>'   : 'realtime_simu_franka_fr3/Joinspace controller'
 * '<S2>'   : 'realtime_simu_franka_fr3/MATLAB Function2'
 * '<S3>'   : 'realtime_simu_franka_fr3/MATLAB Function3'
 * '<S4>'   : 'realtime_simu_franka_fr3/MATLAB Function4'
 * '<S5>'   : 'realtime_simu_franka_fr3/MPC8 offline controller nlpsolve (qrqp, nlpsol_sfun)'
 * '<S6>'   : 'realtime_simu_franka_fr3/Reduced Cartesian PD+ Controller'
 * '<S7>'   : 'realtime_simu_franka_fr3/Robot model bus'
 * '<S8>'   : 'realtime_simu_franka_fr3/Subsystem'
 * '<S9>'   : 'realtime_simu_franka_fr3/get data'
 * '<S10>'  : 'realtime_simu_franka_fr3/joint space control fixed q1'
 * '<S11>'  : 'realtime_simu_franka_fr3/MPC8 offline controller nlpsolve (qrqp, nlpsol_sfun)/CT Regler'
 * '<S12>'  : 'realtime_simu_franka_fr3/MPC8 offline controller nlpsolve (qrqp, nlpsol_sfun)/get data subsys'
 * '<S13>'  : 'realtime_simu_franka_fr3/MPC8 offline controller nlpsolve (qrqp, nlpsol_sfun)/get init guess data'
 * '<S14>'  : 'realtime_simu_franka_fr3/MPC8 offline controller nlpsolve (qrqp, nlpsol_sfun)/get data subsys/get data'
 */
#endif                              /* RTW_HEADER_realtime_simu_franka_fr3_h_ */
