/*
 * realtime_simu_franka_fr3.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "realtime_simu_franka_fr3".
 *
 * Model version              : 8.430
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Tue Oct 29 11:44:01 2024
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

#include <stddef.h>
#include <string.h>

extern "C"
{

#include "rtGetInf.h"

}

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

/* Block signals for system '<S12>/CT Regler' */
struct B_CTRegler_realtime_simu_fran_T {
  real_T tau[6];                       /* '<S12>/CT Regler' */
};

/* Block signals for system '<S14>/CT Regler' */
struct B_CTRegler_realtime_simu_fr_p_T {
  real_T tau[6];                       /* '<S14>/CT Regler' */
};

/* Block signals for system '<S16>/CT Regler' */
struct B_CTRegler_realtime_simu_fr_f_T {
  real_T tau[6];                       /* '<S16>/CT Regler' */
};

/* Block signals (default storage) */
struct B_realtime_simu_franka_fr3_T {
  real_T GetRobotState2_o1[7];         /* '<S4>/Get Robot State2' */
  real_T GetRobotState2_o2[7];         /* '<S4>/Get Robot State2' */
  real_T GetRobotState2_o3[6];         /* '<S4>/Get Robot State2' */
  real_T Switch[7];                    /* '<S4>/Switch' */
  real_T Memory[7];                    /* '<Root>/Memory' */
  real_T SFunction1[7];                /* '<Root>/S-Function1' */
  real_T robotmodelsfunction2_o1[16];  /* '<S9>/robot model s-function2' */
  real_T robotmodelsfunction2_o2[42];  /* '<S9>/robot model s-function2' */
  real_T robotmodelsfunction2_o3[42];  /* '<S9>/robot model s-function2' */
  real_T robotmodelsfunction2_o4[49];  /* '<S9>/robot model s-function2' */
  real_T robotmodelsfunction2_o5[7];   /* '<S9>/robot model s-function2' */
  real_T robotmodelsfunction2_o6[49];  /* '<S9>/robot model s-function2' */
  real_T robotmodelsfunction2_o7[7];   /* '<S9>/robot model s-function2' */
  real_T Merge1[6];                    /* '<S2>/Merge1' */
  real_T RateLimiter[7];               /* '<S4>/Rate Limiter' */
  real_T TmpSignalConversionAtTAQSigLogg[6];
  /* '<Root>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_Mux2_at_outport_0Inport1' */
  real_T tau[7];                       /* '<S8>/joint space control fixed q3' */
  real_T p_d[3];                       /* '<Root>/get data' */
  real_T TmpSignalConversionAtSFunction4[14];
  real_T SFunction3_o1[7];             /* '<S6>/S-Function3' */
  real_T SFunction3_o2;                /* '<S6>/S-Function3' */
  real_T tau_croc_red[6];              /* '<S6>/MATLAB Function4' */
  real_T tau_c[7];                     /* '<S5>/Joinspace controller' */
  real_T p[3];                         /* '<Root>/MATLAB Function2' */
  int8_T CastToSingle4;                /* '<S6>/Cast To Single4' */
  int8_T CastToSingle3;                /* '<S6>/Cast To Single3' */
  int8_T CastToSingle2;                /* '<S6>/Cast To Single2' */
  int8_T CastToSingle1;                /* '<S6>/Cast To Single1' */
  int8_T CastToSingle;                 /* '<S6>/Cast To Single' */
  boolean_T Selector8;                 /* '<S2>/Selector8' */
  B_CTRegler_realtime_simu_fran_T sf_CTRegler_o;/* '<S20>/CT Regler' */
  B_CTRegler_realtime_simu_fr_f_T sf_CTRegler_j;/* '<S19>/CT Regler' */
  B_CTRegler_realtime_simu_fr_f_T sf_CTRegler_p;/* '<S16>/CT Regler' */
  B_CTRegler_realtime_simu_fr_p_T sf_CTRegler_k;/* '<S15>/CT Regler' */
  B_CTRegler_realtime_simu_fr_p_T sf_CTRegler_m;/* '<S14>/CT Regler' */
  B_CTRegler_realtime_simu_fran_T sf_CTRegler_i;/* '<S13>/CT Regler' */
  B_CTRegler_realtime_simu_fran_T sf_CTRegler;/* '<S12>/CT Regler' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_realtime_simu_franka_fr3_T {
  real_T GetRobotState2_DWORK1;        /* '<S4>/Get Robot State2' */
  real_T Memory_PreviousInput[7];      /* '<Root>/Memory' */
  real_T PrevY[7];                     /* '<S4>/Rate Limiter' */
  real_T ApplyControl_DWORK1;          /* '<S4>/Apply Control' */
  real_T ApplyControl_DWORK2;          /* '<S4>/Apply Control' */
  real_T cnt;                          /* '<Root>/get data' */
  real_T run_flag;                     /* '<Root>/get data' */
  void *SFunction1_PWORK[4];           /* '<Root>/S-Function1' */
  void *robotmodelsfunction2_PWORK[9]; /* '<S9>/robot model s-function2' */
  struct {
    void *LoggedData[4];
  } Scope5_PWORK;                      /* '<Root>/Scope5' */

  struct {
    void *LoggedData[3];
  } Scope_PWORK;                       /* '<Root>/Scope' */

  struct {
    void *LoggedData;
  } Scope1_PWORK;                      /* '<Root>/Scope1' */

  struct {
    void *LoggedData;
  } errscopeall_PWORK;                 /* '<Root>/err scope all' */

  void *SFunction3_PWORK[4];           /* '<S6>/S-Function3' */
  void *SFunction4_PWORK[12];          /* '<S6>/S-Function4' */
  struct {
    void *LoggedData[2];
  } Scope6_PWORK;                      /* '<S6>/Scope6' */

  struct {
    void *LoggedData[7];
  } costfunctionscope_PWORK;           /* '<S20>/cost function scope' */

  struct {
    void *LoggedData;
  } Scope_PWORK_a;                     /* '<S19>/Scope' */

  struct {
    void *LoggedData[7];
  } costfunctionscope_PWORK_m;         /* '<S19>/cost function scope' */

  struct {
    void *LoggedData[5];
  } costfunctionscope_PWORK_o;         /* '<S18>/cost function scope' */

  struct {
    void *LoggedData[5];
  } costfunctionscope_PWORK_d;         /* '<S17>/cost function scope' */

  struct {
    void *LoggedData[7];
  } costfunctionscope_PWORK_h;         /* '<S16>/cost function scope' */

  struct {
    void *LoggedData[5];
  } costfunctionscope_PWORK_n;         /* '<S15>/cost function scope' */

  struct {
    void *LoggedData[5];
  } costfunctionscope_PWORK_a;         /* '<S14>/cost function scope' */

  struct {
    void *LoggedData[7];
  } costfunctionscope_PWORK_g;         /* '<S13>/cost function scope' */

  struct {
    void *LoggedData[5];
  } costfunctionscope_PWORK_oc;        /* '<S12>/cost function scope' */

  struct {
    void *LoggedData[5];
  } costfunctionscope_PWORK_mr;        /* '<S11>/cost function scope' */

  int8_T Subsystem2_SubsysRanBC;       /* '<Root>/Subsystem2' */
  int8_T Subsystem1_SubsysRanBC;       /* '<Root>/Subsystem1' */
  int8_T PDControllerSubsystem_SubsysRan;/* '<S2>/PD+ Controller Subsystem' */
  int8_T MPC9offlinecontrollernlpsolveqr;
               /* '<S2>/MPC9 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  int8_T MPC8offlinecontrollernlpsolveqr;
               /* '<S2>/MPC8 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  int8_T MPC7offlinecontrollernlpsolveqr;
               /* '<S2>/MPC7 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  int8_T MPC6offlinecontrollernlpsolveqr;
               /* '<S2>/MPC6 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  int8_T MPC14offlinecontrollernlpsolveq;
              /* '<S2>/MPC14 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  int8_T MPC13offlinecontrollernlpsolveq;
              /* '<S2>/MPC13 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  int8_T MPC12offlinecontrollernlpsolveq;
              /* '<S2>/MPC12 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  int8_T MPC11offlinecontrollernlpsolveq;
              /* '<S2>/MPC11 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  int8_T MPC10offlinecontrollernlpsolveq;
              /* '<S2>/MPC10 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  int8_T MPC01offlinecontrollernlpsolveq;
              /* '<S2>/MPC01 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  int8_T CTControllerSubsystem_SubsysRan;/* '<S2>/CT Controller Subsystem' */
  boolean_T Subsystem2_MODE;           /* '<Root>/Subsystem2' */
  boolean_T MPC9offlinecontrollernlpsolve_p;
               /* '<S2>/MPC9 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  boolean_T MPC8offlinecontrollernlpsolve_i;
               /* '<S2>/MPC8 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  boolean_T MPC7offlinecontrollernlpsolve_c;
               /* '<S2>/MPC7 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  boolean_T MPC6offlinecontrollernlpsolve_p;
               /* '<S2>/MPC6 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  boolean_T MPC14offlinecontrollernlpsolv_g;
              /* '<S2>/MPC14 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  boolean_T MPC13offlinecontrollernlpsolv_p;
              /* '<S2>/MPC13 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  boolean_T MPC12offlinecontrollernlpsolv_f;
              /* '<S2>/MPC12 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  boolean_T MPC11offlinecontrollernlpsolv_f;
              /* '<S2>/MPC11 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  boolean_T MPC10offlinecontrollernlpsolv_a;
              /* '<S2>/MPC10 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
  boolean_T MPC01offlinecontrollernlpsolv_e;
              /* '<S2>/MPC01 offline controller nlpsolve (qrqp, nlpsol_sfun)' */
};

/* Parameters (default storage) */
struct P_realtime_simu_franka_fr3_T_ {
  struct_7GwfAYRjAWGpyfrsv1fnuH param_traj;/* Variable: param_traj
                                            * Referenced by: '<S8>/get q_0_ref'
                                            */
  struct_RCS9FomuFmdbboJvGAYmbB ctrl_param;/* Variable: ctrl_param
                                            * Referenced by:
                                            *   '<S10>/Reduced Cartesian CT Controller'
                                            *   '<S21>/Reduced Cartesian PD+ Controller'
                                            */
  struct_MIVHhJW7aTMZId5MkCQfpG param_jointspace_ct;/* Variable: param_jointspace_ct
                                                     * Referenced by:
                                                     *   '<S14>/CT Regler'
                                                     *   '<S15>/CT Regler'
                                                     */
  real_T q_init[7];                    /* Variable: q_init
                                        * Referenced by: '<S4>/Constant'
                                        */
  x_d Merge_InitialOutput;            /* Computed Parameter: Merge_InitialOutput
                                       * Referenced by: '<S2>/Merge'
                                       */
  real_T K_d_Value[6];        /* Expression: 8*[3     3     3     3     3     3]
                               * Referenced by: '<S10>/K_d'
                               */
  real_T K_p_Value[6];
               /* Expression: 100*[2.25    2.25    2.25    2.25    2.25    2.25]
                * Referenced by: '<S10>/K_p'
                */
  real_T D_d_Value[6];        /* Expression: 8*[3     3     3     3     3     3]
                               * Referenced by: '<S21>/D_d'
                               */
  real_T K_d_Value_c[6];
               /* Expression: 100*[2.25    2.25    2.25    2.25    2.25    2.25]
                * Referenced by: '<S21>/K_d'
                */
  real_T tau_Y0;                       /* Computed Parameter: tau_Y0
                                        * Referenced by: '<S5>/tau'
                                        */
  real_T D_d_jointspace2_Value[7];     /* Expression: [0 0 0 0 0 0 0]
                                        * Referenced by: '<S5>/D_d_jointspace2'
                                        */
  real_T K_d_jointspace2_Value[7];     /* Expression: [100 200 500 200 50 50 10]
                                        * Referenced by: '<S5>/K_d_jointspace2'
                                        */
  real_T tau_Y0_d;                     /* Computed Parameter: tau_Y0_d
                                        * Referenced by: '<S6>/tau'
                                        */
  real_T SFunction3_P1_Size[2];        /* Computed Parameter: SFunction3_P1_Size
                                        * Referenced by: '<S6>/S-Function3'
                                        */
  real_T SFunction3_P1[16];            /* Computed Parameter: SFunction3_P1
                                        * Referenced by: '<S6>/S-Function3'
                                        */
  real_T SFunction3_P2_Size[2];        /* Computed Parameter: SFunction3_P2_Size
                                        * Referenced by: '<S6>/S-Function3'
                                        */
  real_T SFunction3_P2[22];            /* Computed Parameter: SFunction3_P2
                                        * Referenced by: '<S6>/S-Function3'
                                        */
  real_T SFunction4_P1_Size[2];        /* Computed Parameter: SFunction4_P1_Size
                                        * Referenced by: '<S6>/S-Function4'
                                        */
  real_T SFunction4_P1[18];            /* Computed Parameter: SFunction4_P1
                                        * Referenced by: '<S6>/S-Function4'
                                        */
  real_T SFunction4_P2_Size[2];        /* Computed Parameter: SFunction4_P2_Size
                                        * Referenced by: '<S6>/S-Function4'
                                        */
  real_T SFunction4_P2[24];            /* Computed Parameter: SFunction4_P2
                                        * Referenced by: '<S6>/S-Function4'
                                        */
  real_T SFunction4_P3_Size[2];        /* Computed Parameter: SFunction4_P3_Size
                                        * Referenced by: '<S6>/S-Function4'
                                        */
  real_T SFunction4_P3[24];            /* Computed Parameter: SFunction4_P3
                                        * Referenced by: '<S6>/S-Function4'
                                        */
  real_T SFunction4_P4_Size[2];        /* Computed Parameter: SFunction4_P4_Size
                                        * Referenced by: '<S6>/S-Function4'
                                        */
  real_T SFunction4_P4[24];            /* Computed Parameter: SFunction4_P4
                                        * Referenced by: '<S6>/S-Function4'
                                        */
  real_T SFunction4_P5_Size[2];        /* Computed Parameter: SFunction4_P5_Size
                                        * Referenced by: '<S6>/S-Function4'
                                        */
  real_T SFunction4_P5[23];            /* Computed Parameter: SFunction4_P5
                                        * Referenced by: '<S6>/S-Function4'
                                        */
  real_T SFunction4_P6_Size[2];        /* Computed Parameter: SFunction4_P6_Size
                                        * Referenced by: '<S6>/S-Function4'
                                        */
  real_T SFunction4_P6[30];            /* Computed Parameter: SFunction4_P6
                                        * Referenced by: '<S6>/S-Function4'
                                        */
  real_T Switch_Threshold;             /* Expression: 0.001
                                        * Referenced by: '<S4>/Switch'
                                        */
  real_T Memory_InitialCondition[7];   /* Expression: zeros(7,1)
                                        * Referenced by: '<Root>/Memory'
                                        */
  real_T Constant4_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/Constant4'
                                        */
  real_T StartTrajectory_Value;        /* Expression: 0
                                        * Referenced by: '<Root>/Start Trajectory'
                                        */
  real_T ResetTrajectory_Value;        /* Expression: 0
                                        * Referenced by: '<Root>/Reset Trajectory'
                                        */
  real_T StopTrajectory_Value;         /* Expression: 0
                                        * Referenced by: '<Root>/Stop Trajectory'
                                        */
  real_T Constant1_Value;              /* Expression: 1
                                        * Referenced by: '<S2>/Constant1'
                                        */
  real_T Constant2_Value;              /* Expression: 2
                                        * Referenced by: '<S2>/Constant2'
                                        */
  real_T idx1_Value;                   /* Expression: 3
                                        * Referenced by: '<S2>/idx1'
                                        */
  real_T idx3_Value;                   /* Expression: 4
                                        * Referenced by: '<S2>/idx3'
                                        */
  real_T idx2_Value;                   /* Expression: 5
                                        * Referenced by: '<S2>/idx2'
                                        */
  real_T idx4_Value;                   /* Expression: 6
                                        * Referenced by: '<S2>/idx4'
                                        */
  real_T idx5_Value;                   /* Expression: 7
                                        * Referenced by: '<S2>/idx5'
                                        */
  real_T idx6_Value;                   /* Expression: 8
                                        * Referenced by: '<S2>/idx6'
                                        */
  real_T idx7_Value;                   /* Expression: 9
                                        * Referenced by: '<S2>/idx7'
                                        */
  real_T idx8_Value;                   /* Expression: 10
                                        * Referenced by: '<S2>/idx8'
                                        */
  real_T idx9_Value;                   /* Expression: 11
                                        * Referenced by: '<S2>/idx9'
                                        */
  real_T idx10_Value;                  /* Expression: 12
                                        * Referenced by: '<S2>/idx10'
                                        */
  real_T Merge1_InitialOutput;       /* Computed Parameter: Merge1_InitialOutput
                                      * Referenced by: '<S2>/Merge1'
                                      */
  real_T Constant2_Value_j;            /* Expression: 1
                                        * Referenced by: '<Root>/Constant2'
                                        */
  real_T K_d_jointspace1_Value[7];     /* Expression: [100 200 500 200 50 50 10]
                                        * Referenced by: '<S8>/K_d_jointspace1'
                                        */
  real_T Gain_Gain;                    /* Expression: sqrt(2)
                                        * Referenced by: '<S8>/Gain'
                                        */
  real_T Constant1_Value_f;            /* Expression: 1
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T q_ref_Value[7];  /* Expression: [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4]
                           * Referenced by: '<Root>/q_ref'
                           */
  real_T RateLimiter_RisingLim;        /* Expression: 1000
                                        * Referenced by: '<S4>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim;       /* Expression: -1000
                                        * Referenced by: '<S4>/Rate Limiter'
                                        */
  real_T RateLimiter_IC;               /* Expression: 0
                                        * Referenced by: '<S4>/Rate Limiter'
                                        */
  real_T ApplyControl_P1[52];          /* Expression: collision_thresholds
                                        * Referenced by: '<S4>/Apply Control'
                                        */
  real_T ApplyControl_P2[7];           /* Expression: joint_impedance
                                        * Referenced by: '<S4>/Apply Control'
                                        */
  real_T ApplyControl_P3[6];           /* Expression: cartesian_impedance
                                        * Referenced by: '<S4>/Apply Control'
                                        */
  real_T ApplyControl_P4[13];          /* Expression: load_inertia
                                        * Referenced by: '<S4>/Apply Control'
                                        */
  real_T ApplyControl_P5[16];          /* Expression: EE_T_K
                                        * Referenced by: '<S4>/Apply Control'
                                        */
  real_T ApplyControl_P6[7];           /* Expression: init_joint_configuration
                                        * Referenced by: '<S4>/Apply Control'
                                        */
  uint32_T trajectoryselector_Value;
                                 /* Computed Parameter: trajectoryselector_Value
                                  * Referenced by: '<Root>/trajectory selector'
                                  */
  uint32_T controllerselector_Value;
                                 /* Computed Parameter: controllerselector_Value
                                  * Referenced by: '<Root>/controller selector'
                                  */
  uint8_T ManualSwitch_CurrentSetting;
                              /* Computed Parameter: ManualSwitch_CurrentSetting
                               * Referenced by: '<Root>/Manual Switch'
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
    SimStruct childSFunctions[4];
    SimStruct *childSFunctionPtrs[4];
    struct _ssBlkInfo2 blkInfo2[4];
    struct _ssSFcnModelMethods2 methods2[4];
    struct _ssSFcnModelMethods3 methods3[4];
    struct _ssSFcnModelMethods4 methods4[4];
    struct _ssStatesInfo2 statesInfo2[4];
    ssPeriodicStatesInfo periodicStatesInfo[4];
    struct _ssPortInfo2 inputOutputPortInfo2[4];
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
    } Sfcn0;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[6];
      struct _ssInPortUnit inputPortUnits[6];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[6];
      uint_T attribs[6];
      mxArray *params[6];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn1;

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
    } Sfcn2;

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
    } Sfcn3;
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
 * '<S1>'   : 'realtime_simu_franka_fr3/Controller Enable'
 * '<S2>'   : 'realtime_simu_franka_fr3/Controller Subsystem'
 * '<S3>'   : 'realtime_simu_franka_fr3/MATLAB Function2'
 * '<S4>'   : 'realtime_simu_franka_fr3/Subsystem'
 * '<S5>'   : 'realtime_simu_franka_fr3/Subsystem1'
 * '<S6>'   : 'realtime_simu_franka_fr3/Subsystem2'
 * '<S7>'   : 'realtime_simu_franka_fr3/get data'
 * '<S8>'   : 'realtime_simu_franka_fr3/get full tau'
 * '<S9>'   : 'realtime_simu_franka_fr3/robot_model_bus_subsys'
 * '<S10>'  : 'realtime_simu_franka_fr3/Controller Subsystem/CT Controller Subsystem'
 * '<S11>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC01 offline controller nlpsolve (qrqp, nlpsol_sfun)'
 * '<S12>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC10 offline controller nlpsolve (qrqp, nlpsol_sfun)'
 * '<S13>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC11 offline controller nlpsolve (qrqp, nlpsol_sfun)'
 * '<S14>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC12 offline controller nlpsolve (qrqp, nlpsol_sfun)'
 * '<S15>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC13 offline controller nlpsolve (qrqp, nlpsol_sfun)'
 * '<S16>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC14 offline controller nlpsolve (qrqp, nlpsol_sfun)'
 * '<S17>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC6 offline controller nlpsolve (qrqp, nlpsol_sfun)'
 * '<S18>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC7 offline controller nlpsolve (qrqp, nlpsol_sfun)'
 * '<S19>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC8 offline controller nlpsolve (qrqp, nlpsol_sfun)'
 * '<S20>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC9 offline controller nlpsolve (qrqp, nlpsol_sfun)'
 * '<S21>'  : 'realtime_simu_franka_fr3/Controller Subsystem/PD+ Controller Subsystem'
 * '<S22>'  : 'realtime_simu_franka_fr3/Controller Subsystem/CT Controller Subsystem/Reduced Cartesian CT Controller'
 * '<S23>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC01 offline controller nlpsolve (qrqp, nlpsol_sfun)/get data subsys'
 * '<S24>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC01 offline controller nlpsolve (qrqp, nlpsol_sfun)/get init guess data'
 * '<S25>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC10 offline controller nlpsolve (qrqp, nlpsol_sfun)/CT Regler'
 * '<S26>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC10 offline controller nlpsolve (qrqp, nlpsol_sfun)/get data subsys'
 * '<S27>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC10 offline controller nlpsolve (qrqp, nlpsol_sfun)/get init guess data'
 * '<S28>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC11 offline controller nlpsolve (qrqp, nlpsol_sfun)/CT Regler'
 * '<S29>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC11 offline controller nlpsolve (qrqp, nlpsol_sfun)/get data subsys'
 * '<S30>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC11 offline controller nlpsolve (qrqp, nlpsol_sfun)/get init guess data'
 * '<S31>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC11 offline controller nlpsolve (qrqp, nlpsol_sfun)/get theta_kp1'
 * '<S32>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC12 offline controller nlpsolve (qrqp, nlpsol_sfun)/CT Regler'
 * '<S33>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC12 offline controller nlpsolve (qrqp, nlpsol_sfun)/get data subsys'
 * '<S34>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC12 offline controller nlpsolve (qrqp, nlpsol_sfun)/get init guess data'
 * '<S35>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC13 offline controller nlpsolve (qrqp, nlpsol_sfun)/CT Regler'
 * '<S36>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC13 offline controller nlpsolve (qrqp, nlpsol_sfun)/get data subsys'
 * '<S37>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC13 offline controller nlpsolve (qrqp, nlpsol_sfun)/get init guess data'
 * '<S38>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC14 offline controller nlpsolve (qrqp, nlpsol_sfun)/CT Regler'
 * '<S39>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC14 offline controller nlpsolve (qrqp, nlpsol_sfun)/get data subsys'
 * '<S40>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC14 offline controller nlpsolve (qrqp, nlpsol_sfun)/get init guess data'
 * '<S41>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC6 offline controller nlpsolve (qrqp, nlpsol_sfun)/get data subsys'
 * '<S42>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC6 offline controller nlpsolve (qrqp, nlpsol_sfun)/get init guess data'
 * '<S43>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC7 offline controller nlpsolve (qrqp, nlpsol_sfun)/get data subsys'
 * '<S44>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC7 offline controller nlpsolve (qrqp, nlpsol_sfun)/get init guess data'
 * '<S45>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC8 offline controller nlpsolve (qrqp, nlpsol_sfun)/CT Regler'
 * '<S46>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC8 offline controller nlpsolve (qrqp, nlpsol_sfun)/get data subsys'
 * '<S47>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC8 offline controller nlpsolve (qrqp, nlpsol_sfun)/get init guess data'
 * '<S48>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC9 offline controller nlpsolve (qrqp, nlpsol_sfun)/CT Regler'
 * '<S49>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC9 offline controller nlpsolve (qrqp, nlpsol_sfun)/get data subsys'
 * '<S50>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC9 offline controller nlpsolve (qrqp, nlpsol_sfun)/get init guess data'
 * '<S51>'  : 'realtime_simu_franka_fr3/Controller Subsystem/MPC9 offline controller nlpsolve (qrqp, nlpsol_sfun)/get ukp1'
 * '<S52>'  : 'realtime_simu_franka_fr3/Controller Subsystem/PD+ Controller Subsystem/Reduced Cartesian PD+ Controller'
 * '<S53>'  : 'realtime_simu_franka_fr3/Subsystem1/Joinspace controller'
 * '<S54>'  : 'realtime_simu_franka_fr3/Subsystem2/MATLAB Function3'
 * '<S55>'  : 'realtime_simu_franka_fr3/Subsystem2/MATLAB Function4'
 * '<S56>'  : 'realtime_simu_franka_fr3/get full tau/get q_0_ref'
 * '<S57>'  : 'realtime_simu_franka_fr3/get full tau/joint space control fixed q3'
 * '<S58>'  : 'realtime_simu_franka_fr3/robot_model_bus_subsys/Robot model bus'
 */
#endif                              /* RTW_HEADER_realtime_simu_franka_fr3_h_ */
