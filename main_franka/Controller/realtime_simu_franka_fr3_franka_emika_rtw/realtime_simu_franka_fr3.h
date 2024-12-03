/*
 * realtime_simu_franka_fr3.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "realtime_simu_franka_fr3".
 *
 * Model version              : 8.566
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Tue Dec  3 19:09:21 2024
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
#include "coder_posix_time.h"
#include "robot_api.h"
#include "control_modes.h"
#include "gripper_api.h"
#include "realtime_simu_franka_fr3_types.h"

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rt_nonfinite.h"

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
  robot_model robot_model_o;           /* '<S7>/Robot model bus' */
  real_T GetRobotState2_o1[7];         /* '<S4>/Get Robot State2' */
  real_T GetRobotState2_o2[7];         /* '<S4>/Get Robot State2' */
  real_T GetRobotState2_o3[6];         /* '<S4>/Get Robot State2' */
  real_T Switch[7];                    /* '<S4>/Switch' */
  real_T robotmodelsfunction2_o1[16];  /* '<S7>/robot model s-function2' */
  real_T robotmodelsfunction2_o2[42];  /* '<S7>/robot model s-function2' */
  real_T robotmodelsfunction2_o3[42];  /* '<S7>/robot model s-function2' */
  real_T robotmodelsfunction2_o4[49];  /* '<S7>/robot model s-function2' */
  real_T robotmodelsfunction2_o5[7];   /* '<S7>/robot model s-function2' */
  real_T robotmodelsfunction2_o6[49];  /* '<S7>/robot model s-function2' */
  real_T robotmodelsfunction2_o7[7];   /* '<S7>/robot model s-function2' */
  real_T Derivative[7];                /* '<Root>/Derivative' */
  real_T Gain[7];                      /* '<S5>/Gain' */
  real_T Switch_n[7];                  /* '<Root>/Switch' */
  real_T Add[7];                       /* '<S3>/Add' */
  real_T q_pp[7];
               /* '<S3>/SigConversion_InsertedFor_Bus Selector1_at_outport_2' */
  real_T q[7]; /* '<S3>/SigConversion_InsertedFor_Bus Selector1_at_outport_0' */
  real_T q_p[7];
               /* '<S3>/SigConversion_InsertedFor_Bus Selector1_at_outport_1' */
  real_T p_emYxByPz[6];
  /* '<S3>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_From25_at_outport_0Inport1' */
  real_T _emsYxByPz[6];
  /* '<S3>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_From26_at_outport_0Inport1' */
  real_T quat_e24[6];
  /* '<S3>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_From38_at_outport_0Inport1' */
  real_T p_emsYxByPz[6];
  /* '<S3>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_From39_at_outport_0Inport1' */
  real_T _erads[6];
  /* '<S3>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_From40_at_outport_0Inport1' */
  real_T _erads_p[6];
  /* '<S3>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_From41_at_outport_0Inport1' */
  real_T freqperTastepHz[2];
  /* '<S3>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_From50_at_outport_0Inport1' */
  real_T RateLimiter[7];               /* '<S4>/Rate Limiter' */
  real_T JJ_Y12_B13_R14[3];
               /* '<S10>/SigConversion_InsertedFor_Bus Selector_at_outport_0' */
  real_T JJ_Y15_B16_R17[3];
               /* '<S10>/SigConversion_InsertedFor_Bus Selector_at_outport_1' */
  real_T JJ_Y23_B24_R25[3];
               /* '<S10>/SigConversion_InsertedFor_Bus Selector_at_outport_2' */
  real_T JJ_Y26_B27_R34[3];
               /* '<S10>/SigConversion_InsertedFor_Bus Selector_at_outport_3' */
  real_T JJ_Y35_B36_R37[3];
               /* '<S10>/SigConversion_InsertedFor_Bus Selector_at_outport_4' */
  real_T JJ_Y45_B46_R47[3];
               /* '<S10>/SigConversion_InsertedFor_Bus Selector_at_outport_5' */
  real_T JJ_Y56_B57_R67[3];
               /* '<S10>/SigConversion_InsertedFor_Bus Selector_at_outport_6' */
  real_T TmpSignalConversionAtSFunction4[14];
  real_T SFunction3_o1[7];             /* '<S8>/S-Function3' */
  real_T SFunction3_o2;                /* '<S8>/S-Function3' */
  real_T K_d[7];                       /* '<S6>/home robot logic' */
  real_T D_d[7];                       /* '<S6>/home robot logic' */
  real_T home_running;                 /* '<S6>/home robot logic' */
  real_T tau[7];                       /* '<S6>/Joinspace controller' */
  real_T q_0_ref_fixed[7];             /* '<S5>/get q_0_ref' */
  real_T freq_per_step;                /* '<S12>/MATLAB Function' */
  real_T freq_per_step_mean;           /* '<S12>/MATLAB Function' */
  real_T f_data_o[301];                /* '<S12>/MATLAB Function' */
  real_T e_x;                          /* '<S3>/calc_errors' */
  real_T e_y;                          /* '<S3>/calc_errors' */
  real_T e_z;                          /* '<S3>/calc_errors' */
  real_T q_err_o[3];                   /* '<S3>/calc_errors' */
  real_T e_x_p;                        /* '<S3>/calc_errors' */
  real_T e_y_p;                        /* '<S3>/calc_errors' */
  real_T e_z_p;                        /* '<S3>/calc_errors' */
  real_T omega_d_err[3];               /* '<S3>/calc_errors' */
  real_T e_x_pp;                       /* '<S3>/calc_errors' */
  real_T e_y_pp;                       /* '<S3>/calc_errors' */
  real_T e_z_pp;                       /* '<S3>/calc_errors' */
  real_T omega_d_p_err[3];             /* '<S3>/calc_errors' */
  real_T p[3];                         /* '<S3>/calc_errors' */
  real_T p_d[3];                       /* '<S3>/calc_errors' */
  real_T p_p[3];                       /* '<S3>/calc_errors' */
  real_T p_d_p[3];                     /* '<S3>/calc_errors' */
  real_T p_pp[3];                      /* '<S3>/calc_errors' */
  real_T p_d_pp[3];                    /* '<S3>/calc_errors' */
  real_T q_e_o[3];                     /* '<S3>/calc_errors' */
  real_T q_d_o[3];                     /* '<S3>/calc_errors' */
  real_T omega_e[3];                   /* '<S3>/calc_errors' */
  real_T omega_d[3];                   /* '<S3>/calc_errors' */
  real_T omega_e_p[3];                 /* '<S3>/calc_errors' */
  real_T omega_d_p[3];                 /* '<S3>/calc_errors' */
  real_T w;                   /* '<S10>/manipulability and collinearity 7DOF' */
  int8_T CastToSingle4;                /* '<S8>/Cast To Single4' */
  int8_T CastToSingle3;                /* '<S8>/Cast To Single3' */
  int8_T CastToSingle2;                /* '<S8>/Cast To Single2' */
  int8_T CastToSingle1;                /* '<S8>/Cast To Single1' */
  int8_T CastToSingle;                 /* '<S8>/Cast To Single' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_realtime_simu_franka_fr3_T {
  sdAmwXbnJnEmimT0NaJRtAD_realt_T savedTime;/* '<S12>/MATLAB Function' */
  real_T filterwindow_PreviousInput[301];/* '<S12>/filter window' */
  real_T GetRobotState2_DWORK1;        /* '<S4>/Get Robot State2' */
  real_T TimeStampA;                   /* '<Root>/Derivative' */
  real_T LastUAtTimeA[7];              /* '<Root>/Derivative' */
  real_T TimeStampB;                   /* '<Root>/Derivative' */
  real_T LastUAtTimeB[7];              /* '<Root>/Derivative' */
  real_T PrevY[7];                     /* '<S4>/Rate Limiter' */
  real_T LastMajorTime;                /* '<S4>/Rate Limiter' */
  real_T ApplyControl_DWORK1;          /* '<S4>/Apply Control' */
  real_T ApplyControl_DWORK2;          /* '<S4>/Apply Control' */
  real_T enabled;                      /* '<S6>/home robot logic' */
  real_T t_start;                      /* '<S6>/home robot logic' */
  real_T cnt;                          /* '<S12>/MATLAB Function' */
  real_T time_start;                   /* '<S12>/MATLAB Function' */
  real_T freq;                         /* '<S12>/MATLAB Function' */
  real_T cnt_c;                        /* '<S2>/MATLAB Function' */
  real_T run_flag;                     /* '<S2>/MATLAB Function' */
  void *robotmodelsfunction2_PWORK[9]; /* '<S7>/robot model s-function2' */
  struct {
    void *LoggedData[3];
  } Scope_PWORK;                       /* '<Root>/Scope' */

  struct {
    void *LoggedData[4];
  } Scope5_PWORK;                      /* '<Root>/Scope5' */

  struct {
    void *LoggedData;
  } Scope1_PWORK;                      /* '<S3>/Scope1' */

  struct {
    void *LoggedData[24];
  } errscopeall_PWORK;                 /* '<S3>/err scope all' */

  struct {
    void *LoggedData[8];
  } Cosinushnlichkeiten7DOF_PWORK;     /* '<S10>/Cosinus Ähnlichkeiten 7DOF' */

  void *SFunction3_PWORK[4];           /* '<S8>/S-Function3' */
  void *SFunction4_PWORK[12];          /* '<S8>/S-Function4' */
  struct {
    void *LoggedData[2];
  } Scope6_PWORK;                      /* '<S8>/Scope6' */

  struct {
    void *LoggedData;
  } Scope_PWORK_k;                     /* '<S6>/Scope' */

  int8_T tau_subsystem_SubsysRanBC;    /* '<Root>/tau_subsystem' */
  int8_T jointspacectlsubsys_SubsysRanBC;/* '<Root>/jointspace ctl subsys' */
  boolean_T enabled_not_empty;         /* '<S6>/home robot logic' */
  boolean_T time_start_not_empty;      /* '<S12>/MATLAB Function' */
  boolean_T freq_not_empty;            /* '<S12>/MATLAB Function' */
  boolean_T savedTime_not_empty;       /* '<S12>/MATLAB Function' */
  boolean_T tau_subsystem_MODE;        /* '<Root>/tau_subsystem' */
  boolean_T jointspacectlsubsys_MODE;  /* '<Root>/jointspace ctl subsys' */
};

/* Parameters (default storage) */
struct P_realtime_simu_franka_fr3_T_ {
  traj_data_bus traj_data_bus_init;    /* Variable: traj_data_bus_init
                                        * Referenced by: '<S2>/Constant3'
                                        */
  struct_YfgHomCZWGPNJ0TPPLZfAE param_traj;/* Variable: param_traj
                                            * Referenced by:
                                            *   '<S5>/get q_0_ref'
                                            *   '<S6>/get reference pose'
                                            */
  struct_BAQoxvk5NEutQSlALFTg0D param_savgol;/* Variable: param_savgol
                                              * Referenced by: '<S12>/MATLAB Function'
                                              */
  real_T q_init[7];                    /* Variable: q_init
                                        * Referenced by: '<S4>/Constant'
                                        */
  real_T tau_Y0;                       /* Computed Parameter: tau_Y0
                                        * Referenced by: '<S6>/tau'
                                        */
  real_T homerunflag_Y0;               /* Computed Parameter: homerunflag_Y0
                                        * Referenced by: '<S6>/home run flag'
                                        */
  real_T t0_Value;                     /* Expression: 3
                                        * Referenced by: '<S6>/t0'
                                        */
  real_T t1_Value;                     /* Expression: 5
                                        * Referenced by: '<S6>/t1'
                                        */
  real_T tend_Value;                   /* Expression: 6
                                        * Referenced by: '<S6>/tend'
                                        */
  real_T tau_Y0_d;                     /* Computed Parameter: tau_Y0_d
                                        * Referenced by: '<S8>/tau'
                                        */
  real_T SFunction3_P1_Size[2];        /* Computed Parameter: SFunction3_P1_Size
                                        * Referenced by: '<S8>/S-Function3'
                                        */
  real_T SFunction3_P1[16];            /* Computed Parameter: SFunction3_P1
                                        * Referenced by: '<S8>/S-Function3'
                                        */
  real_T SFunction3_P2_Size[2];        /* Computed Parameter: SFunction3_P2_Size
                                        * Referenced by: '<S8>/S-Function3'
                                        */
  real_T SFunction3_P2[22];            /* Computed Parameter: SFunction3_P2
                                        * Referenced by: '<S8>/S-Function3'
                                        */
  real_T Constant_Value;               /* Expression: 1
                                        * Referenced by: '<S8>/Constant'
                                        */
  real_T SFunction4_P1_Size[2];        /* Computed Parameter: SFunction4_P1_Size
                                        * Referenced by: '<S8>/S-Function4'
                                        */
  real_T SFunction4_P1[18];            /* Computed Parameter: SFunction4_P1
                                        * Referenced by: '<S8>/S-Function4'
                                        */
  real_T SFunction4_P2_Size[2];        /* Computed Parameter: SFunction4_P2_Size
                                        * Referenced by: '<S8>/S-Function4'
                                        */
  real_T SFunction4_P2[24];            /* Computed Parameter: SFunction4_P2
                                        * Referenced by: '<S8>/S-Function4'
                                        */
  real_T SFunction4_P3_Size[2];        /* Computed Parameter: SFunction4_P3_Size
                                        * Referenced by: '<S8>/S-Function4'
                                        */
  real_T SFunction4_P3[24];            /* Computed Parameter: SFunction4_P3
                                        * Referenced by: '<S8>/S-Function4'
                                        */
  real_T SFunction4_P4_Size[2];        /* Computed Parameter: SFunction4_P4_Size
                                        * Referenced by: '<S8>/S-Function4'
                                        */
  real_T SFunction4_P4[24];            /* Computed Parameter: SFunction4_P4
                                        * Referenced by: '<S8>/S-Function4'
                                        */
  real_T SFunction4_P5_Size[2];        /* Computed Parameter: SFunction4_P5_Size
                                        * Referenced by: '<S8>/S-Function4'
                                        */
  real_T SFunction4_P5[23];            /* Computed Parameter: SFunction4_P5
                                        * Referenced by: '<S8>/S-Function4'
                                        */
  real_T SFunction4_P6_Size[2];        /* Computed Parameter: SFunction4_P6_Size
                                        * Referenced by: '<S8>/S-Function4'
                                        */
  real_T SFunction4_P6[30];            /* Computed Parameter: SFunction4_P6
                                        * Referenced by: '<S8>/S-Function4'
                                        */
  real_T filterwindow_InitialCondition[301];/* Expression: zeros(param_savgol.N, 1)
                                             * Referenced by: '<S12>/filter window'
                                             */
  real_T Switch_Threshold;             /* Expression: 0.001
                                        * Referenced by: '<S4>/Switch'
                                        */
  real_T K_d_jointspace1_Value[7];     /* Expression: [100 200 500 200 50 50 10]
                                        * Referenced by: '<S5>/K_d_jointspace1'
                                        */
  real_T StartTrajectory_Value;        /* Expression: 0
                                        * Referenced by: '<Root>/Start Trajectory'
                                        */
  real_T home_Value;                   /* Expression: 0
                                        * Referenced by: '<Root>/home'
                                        */
  real_T ResetTrajectory_Value;        /* Expression: 0
                                        * Referenced by: '<Root>/Reset Trajectory'
                                        */
  real_T StopTrajectory_Value;         /* Expression: 0
                                        * Referenced by: '<Root>/Stop Trajectory'
                                        */
  real_T use_crocoddyl_flag_Value;     /* Expression: 1
                                        * Referenced by: '<Root>/use_crocoddyl_flag'
                                        */
  real_T Gain_Gain;                    /* Expression: 2
                                        * Referenced by: '<S5>/Gain'
                                        */
  real_T Constant_Value_b[7];          /* Expression: zeros(7,1)
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T homemode_Value;               /* Expression: 0
                                        * Referenced by: '<Root>/home mode'
                                        */
  real_T Constant1_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T Switch_Threshold_c;           /* Expression: 0
                                        * Referenced by: '<Root>/Switch'
                                        */
  real_T RateLimiter_RisingLim;        /* Expression: 1000
                                        * Referenced by: '<S4>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim;       /* Expression: -1000
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
  real_T use_casadi_flag_Value;        /* Expression: 0
                                        * Referenced by: '<Root>/use_casadi_flag'
                                        */
  uint32_T trajectoryselector_Value;
                                 /* Computed Parameter: trajectoryselector_Value
                                  * Referenced by: '<Root>/trajectory selector'
                                  */
  uint32_T controllerselector_Value;
                                 /* Computed Parameter: controllerselector_Value
                                  * Referenced by: '<Root>/controller selector'
                                  */
  uint8_T ManualSwitch1_CurrentSetting;
                             /* Computed Parameter: ManualSwitch1_CurrentSetting
                              * Referenced by: '<Root>/Manual Switch1'
                              */
  uint8_T ManualSwitch_CurrentSetting;
                              /* Computed Parameter: ManualSwitch_CurrentSetting
                               * Referenced by: '<Root>/Manual Switch'
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
    SimStruct childSFunctions[3];
    SimStruct *childSFunctionPtrs[3];
    struct _ssBlkInfo2 blkInfo2[3];
    struct _ssSFcnModelMethods2 methods2[3];
    struct _ssSFcnModelMethods3 methods3[3];
    struct _ssSFcnModelMethods4 methods4[3];
    struct _ssStatesInfo2 statesInfo2[3];
    ssPeriodicStatesInfo periodicStatesInfo[3];
    struct _ssPortInfo2 inputOutputPortInfo2[3];
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
    } Sfcn2;
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
 * '<S2>'   : 'realtime_simu_franka_fr3/Create Trajectory'
 * '<S3>'   : 'realtime_simu_franka_fr3/Debug Subsystem'
 * '<S4>'   : 'realtime_simu_franka_fr3/Subsystem'
 * '<S5>'   : 'realtime_simu_franka_fr3/get full tau'
 * '<S6>'   : 'realtime_simu_franka_fr3/jointspace ctl subsys'
 * '<S7>'   : 'realtime_simu_franka_fr3/robot_model_bus_subsys'
 * '<S8>'   : 'realtime_simu_franka_fr3/tau_subsystem'
 * '<S9>'   : 'realtime_simu_franka_fr3/Create Trajectory/MATLAB Function'
 * '<S10>'  : 'realtime_simu_franka_fr3/Debug Subsystem/Subsystem Reference'
 * '<S11>'  : 'realtime_simu_franka_fr3/Debug Subsystem/calc_errors'
 * '<S12>'  : 'realtime_simu_franka_fr3/Debug Subsystem/tic-toc'
 * '<S13>'  : 'realtime_simu_franka_fr3/Debug Subsystem/Subsystem Reference/manipulability and collinearity 7DOF'
 * '<S14>'  : 'realtime_simu_franka_fr3/Debug Subsystem/tic-toc/MATLAB Function'
 * '<S15>'  : 'realtime_simu_franka_fr3/get full tau/get q_0_ref'
 * '<S16>'  : 'realtime_simu_franka_fr3/get full tau/joint space control fixed q3'
 * '<S17>'  : 'realtime_simu_franka_fr3/jointspace ctl subsys/Joinspace controller'
 * '<S18>'  : 'realtime_simu_franka_fr3/jointspace ctl subsys/get reference pose'
 * '<S19>'  : 'realtime_simu_franka_fr3/jointspace ctl subsys/home robot logic'
 * '<S20>'  : 'realtime_simu_franka_fr3/robot_model_bus_subsys/Robot model bus'
 */
#endif                              /* RTW_HEADER_realtime_simu_franka_fr3_h_ */
