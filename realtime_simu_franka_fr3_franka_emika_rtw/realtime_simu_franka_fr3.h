/*
 * realtime_simu_franka_fr3.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "realtime_simu_franka_fr3".
 *
 * Model version              : 8.19
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Sat Mar 15 13:36:29 2025
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

/* Block signals for system '<S16>/get EKF joint values' */
struct B_getEKFjointvalues_realtime__T {
  real_T q[7];                         /* '<S16>/get EKF joint values' */
  real_T q_p[7];                       /* '<S16>/get EKF joint values' */
  real_T q_pp[7];                      /* '<S16>/get EKF joint values' */
};

/* Block signals for system '<S19>/Robot model bus' */
struct B_Robotmodelbus_realtime_simu_T {
  robot_model robot_model_b;           /* '<S19>/Robot model bus' */
};

/* Block signals for system '<S19>/zero fixed states' */
struct B_zerofixedstates_realtime_si_T {
  real_T q_red[7];                     /* '<S19>/zero fixed states' */
  real_T q_p_red[7];                   /* '<S19>/zero fixed states' */
  real_T q_pp_red[7];                  /* '<S19>/zero fixed states' */
};

/* Block states (default storage) for system '<S13>/no EKF' */
struct DW_noEKF_realtime_simu_franka_T {
  int8_T noEKF_SubsysRanBC;            /* '<S13>/no EKF' */
};

/* Block signals (default storage) */
struct B_realtime_simu_franka_fr3_T {
  robot_model Merge;                   /* '<S6>/Merge' */
  robot_model Merge_i;                 /* '<S13>/Merge' */
  robot_model robot_model_a;           /* '<S9>/Robot model bus' */
  real_T GetRobotState2_o1[7];         /* '<S5>/Get Robot State2' */
  real_T GetRobotState2_o2[7];         /* '<S5>/Get Robot State2' */
  real_T GetRobotState2_o3[6];         /* '<S5>/Get Robot State2' */
  real_T Switch[7];                    /* '<S5>/Switch' */
  real_T robotmodelsfunction2_o1[16];  /* '<S9>/robot model s-function2' */
  real_T robotmodelsfunction2_o2[42];  /* '<S9>/robot model s-function2' */
  real_T robotmodelsfunction2_o3[42];  /* '<S9>/robot model s-function2' */
  real_T robotmodelsfunction2_o4[49];  /* '<S9>/robot model s-function2' */
  real_T robotmodelsfunction2_o5[7];   /* '<S9>/robot model s-function2' */
  real_T robotmodelsfunction2_o6[49];  /* '<S9>/robot model s-function2' */
  real_T robotmodelsfunction2_o7[7];   /* '<S9>/robot model s-function2' */
  real_T Derivative[7];                /* '<Root>/Derivative' */
  real_T uk_prev[7];                   /* '<S6>/uk_prev' */
  real_T uk_prev_e[7];                 /* '<S13>/uk_prev' */
  real_T robotmodelsfunction1_o1[16];  /* '<S9>/robot model s-function1' */
  real_T robotmodelsfunction1_o2[42];  /* '<S9>/robot model s-function1' */
  real_T robotmodelsfunction1_o3[42];  /* '<S9>/robot model s-function1' */
  real_T robotmodelsfunction1_o4[49];  /* '<S9>/robot model s-function1' */
  real_T robotmodelsfunction1_o5[7];   /* '<S9>/robot model s-function1' */
  real_T robotmodelsfunction1_o6[49];  /* '<S9>/robot model s-function1' */
  real_T robotmodelsfunction1_o7[7];   /* '<S9>/robot model s-function1' */
  real_T q_pp[7];
               /* '<S4>/SigConversion_InsertedFor_Bus Selector1_at_outport_2' */
  real_T Merge1[7];                    /* '<S2>/Merge1' */
  real_T Switch_b[7];                  /* '<Root>/Switch' */
  real_T Add[7];                       /* '<S4>/Add' */
  real_T q[7]; /* '<S4>/SigConversion_InsertedFor_Bus Selector1_at_outport_0' */
  real_T q_p[7];
               /* '<S4>/SigConversion_InsertedFor_Bus Selector1_at_outport_1' */
  real_T p_emYxByPz[6];
  /* '<S4>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_From25_at_outport_0Inport1' */
  real_T _emsYxByPz[6];
  /* '<S4>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_From26_at_outport_0Inport1' */
  real_T quat_e24[6];
  /* '<S4>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_From38_at_outport_0Inport1' */
  real_T p_emsYxByPz[6];
  /* '<S4>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_From39_at_outport_0Inport1' */
  real_T _erads[6];
  /* '<S4>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_From40_at_outport_0Inport1' */
  real_T _erads_p[6];
  /* '<S4>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_From41_at_outport_0Inport1' */
  real_T freqperTastepHz[2];
  /* '<S4>/TmpSignal ConversionAtTAQSigLogging_InsertedFor_From50_at_outport_0Inport1' */
  real_T RateLimiter[7];               /* '<S5>/Rate Limiter' */
  real_T JJ_Y12_B13_R14[3];
               /* '<S24>/SigConversion_InsertedFor_Bus Selector_at_outport_0' */
  real_T JJ_Y15_B16_R17[3];
               /* '<S24>/SigConversion_InsertedFor_Bus Selector_at_outport_1' */
  real_T JJ_Y23_B24_R25[3];
               /* '<S24>/SigConversion_InsertedFor_Bus Selector_at_outport_2' */
  real_T JJ_Y26_B27_R34[3];
               /* '<S24>/SigConversion_InsertedFor_Bus Selector_at_outport_3' */
  real_T JJ_Y35_B36_R37[3];
               /* '<S24>/SigConversion_InsertedFor_Bus Selector_at_outport_4' */
  real_T JJ_Y45_B46_R47[3];
               /* '<S24>/SigConversion_InsertedFor_Bus Selector_at_outport_5' */
  real_T JJ_Y56_B57_R67[3];
               /* '<S24>/SigConversion_InsertedFor_Bus Selector_at_outport_6' */
  real_T TmpSignalConversionAtSFunction4[14];
  real_T SFunction3_o1[7];             /* '<S10>/S-Function3' */
  real_T tau[7];                       /* '<S10>/torque safety' */
  real_T q_red[7];                     /* '<S9>/zero fixed states' */
  real_T q_p_red[7];                   /* '<S9>/zero fixed states' */
  real_T q_pp_red[7];                  /* '<S9>/zero fixed states' */
  real_T K_d[7];                       /* '<S8>/home robot logic' */
  real_T D_d[7];                       /* '<S8>/home robot logic' */
  real_T home_running;                 /* '<S8>/home robot logic' */
  real_T tau_k[7];                     /* '<S8>/Joinspace controller' */
  real_T tau_p[7];                     /* '<S7>/joint space control fixed q3' */
  real_T Constant[196];                /* '<S29>/Constant' */
  real_T y_kxk_measured[14];
  real_T xk_minus[14];                 /* '<S29>/xk_minus' */
  real_T Constant1[196];               /* '<S29>/Constant1' */
  real_T Pk_minus[196];                /* '<S29>/Pk_minus' */
  real_T xk_plus[14];             /* '<S29>/Reduced System sfun casadi solve' */
  real_T xkp1_minus[14];          /* '<S29>/Reduced System sfun casadi solve' */
  real_T Pkp1_minus[196];         /* '<S29>/Reduced System sfun casadi solve' */
  real_T robotmodelsfunction2_o1_o[16];/* '<S32>/robot model s-function2' */
  real_T robotmodelsfunction2_o2_c[42];/* '<S32>/robot model s-function2' */
  real_T robotmodelsfunction2_o3_p[42];/* '<S32>/robot model s-function2' */
  real_T robotmodelsfunction2_o4_m[49];/* '<S32>/robot model s-function2' */
  real_T robotmodelsfunction2_o5_n[7]; /* '<S32>/robot model s-function2' */
  real_T robotmodelsfunction2_o6_g[49];/* '<S32>/robot model s-function2' */
  real_T robotmodelsfunction2_o7_e[7]; /* '<S32>/robot model s-function2' */
  real_T robotmodelsfunction1_o1_c[16];/* '<S32>/robot model s-function1' */
  real_T robotmodelsfunction1_o2_e[42];/* '<S32>/robot model s-function1' */
  real_T robotmodelsfunction1_o3_c[42];/* '<S32>/robot model s-function1' */
  real_T robotmodelsfunction1_o4_p[49];/* '<S32>/robot model s-function1' */
  real_T robotmodelsfunction1_o5_b[7]; /* '<S32>/robot model s-function1' */
  real_T robotmodelsfunction1_o6_d[49];/* '<S32>/robot model s-function1' */
  real_T robotmodelsfunction1_o7_m[7]; /* '<S32>/robot model s-function1' */
  real_T freq_per_step;                /* '<S26>/MATLAB Function' */
  real_T freq_per_step_mean;           /* '<S26>/MATLAB Function' */
  real_T f_data_o[301];                /* '<S26>/MATLAB Function' */
  real_T e_x;                          /* '<S4>/calc_errors' */
  real_T e_y;                          /* '<S4>/calc_errors' */
  real_T e_z;                          /* '<S4>/calc_errors' */
  real_T q_err_o[3];                   /* '<S4>/calc_errors' */
  real_T e_x_p;                        /* '<S4>/calc_errors' */
  real_T e_y_p;                        /* '<S4>/calc_errors' */
  real_T e_z_p;                        /* '<S4>/calc_errors' */
  real_T omega_d_err[3];               /* '<S4>/calc_errors' */
  real_T e_x_pp;                       /* '<S4>/calc_errors' */
  real_T e_y_pp;                       /* '<S4>/calc_errors' */
  real_T e_z_pp;                       /* '<S4>/calc_errors' */
  real_T omega_d_p_err[3];             /* '<S4>/calc_errors' */
  real_T p[3];                         /* '<S4>/calc_errors' */
  real_T p_d[3];                       /* '<S4>/calc_errors' */
  real_T p_p[3];                       /* '<S4>/calc_errors' */
  real_T p_d_p[3];                     /* '<S4>/calc_errors' */
  real_T p_pp[3];                      /* '<S4>/calc_errors' */
  real_T p_d_pp[3];                    /* '<S4>/calc_errors' */
  real_T q_e_o[3];                     /* '<S4>/calc_errors' */
  real_T q_d_o[3];                     /* '<S4>/calc_errors' */
  real_T omega_e[3];                   /* '<S4>/calc_errors' */
  real_T omega_d[3];                   /* '<S4>/calc_errors' */
  real_T omega_e_p[3];                 /* '<S4>/calc_errors' */
  real_T omega_d_p[3];                 /* '<S4>/calc_errors' */
  real_T w;                   /* '<S24>/manipulability and collinearity 7DOF' */
  real_T Constant_p[196];              /* '<S16>/Constant' */
  real_T y_kxk_measured_n[14];
  real_T xk_minus_d[14];               /* '<S16>/xk_minus' */
  real_T Constant1_a[196];             /* '<S16>/Constant1' */
  real_T Pk_minus_d[196];              /* '<S16>/Pk_minus' */
  real_T xk_plus_j[14];           /* '<S16>/Reduced System sfun casadi solve' */
  real_T xkp1_minus_a[14];        /* '<S16>/Reduced System sfun casadi solve' */
  real_T Pkp1_minus_k[196];       /* '<S16>/Reduced System sfun casadi solve' */
  real_T robotmodelsfunction2_o1_b[16];/* '<S19>/robot model s-function2' */
  real_T robotmodelsfunction2_o2_h[42];/* '<S19>/robot model s-function2' */
  real_T robotmodelsfunction2_o3_o[42];/* '<S19>/robot model s-function2' */
  real_T robotmodelsfunction2_o4_d[49];/* '<S19>/robot model s-function2' */
  real_T robotmodelsfunction2_o5_k[7]; /* '<S19>/robot model s-function2' */
  real_T robotmodelsfunction2_o6_b[49];/* '<S19>/robot model s-function2' */
  real_T robotmodelsfunction2_o7_l[7]; /* '<S19>/robot model s-function2' */
  real_T robotmodelsfunction1_o1_i[16];/* '<S19>/robot model s-function1' */
  real_T robotmodelsfunction1_o2_a[42];/* '<S19>/robot model s-function1' */
  real_T robotmodelsfunction1_o3_p[42];/* '<S19>/robot model s-function1' */
  real_T robotmodelsfunction1_o4_a[49];/* '<S19>/robot model s-function1' */
  real_T robotmodelsfunction1_o5_o[7]; /* '<S19>/robot model s-function1' */
  real_T robotmodelsfunction1_o6_h[49];/* '<S19>/robot model s-function1' */
  real_T robotmodelsfunction1_o7_md[7];/* '<S19>/robot model s-function1' */
  int8_T SFunction3_o2;                /* '<S10>/S-Function3' */
  int8_T CastToSingle4;                /* '<S10>/Cast To Single4' */
  int8_T CastToSingle3;                /* '<S10>/Cast To Single3' */
  int8_T CastToSingle2;                /* '<S10>/Cast To Single2' */
  int8_T CastToSingle1;                /* '<S10>/Cast To Single1' */
  int8_T CastToSingle;                 /* '<S10>/Cast To Single' */
  int8_T CastToSingle5;                /* '<S10>/Cast To Single5' */
  B_Robotmodelbus_realtime_simu_T sf_Robotmodelbus1;/* '<S9>/Robot model bus1' */
  B_zerofixedstates_realtime_si_T sf_zerofixedstates_i;/* '<S32>/zero fixed states' */
  B_Robotmodelbus_realtime_simu_T sf_Robotmodelbus1_h;/* '<S32>/Robot model bus1' */
  B_Robotmodelbus_realtime_simu_T sf_Robotmodelbus_e;/* '<S32>/Robot model bus' */
  B_getEKFjointvalues_realtime__T sf_getEKFjointvalues_g;/* '<S29>/get EKF joint values' */
  B_zerofixedstates_realtime_si_T sf_zerofixedstates_a;/* '<S19>/zero fixed states' */
  B_Robotmodelbus_realtime_simu_T sf_Robotmodelbus1_e;/* '<S19>/Robot model bus1' */
  B_Robotmodelbus_realtime_simu_T sf_Robotmodelbus_b;/* '<S19>/Robot model bus' */
  B_getEKFjointvalues_realtime__T sf_getEKFjointvalues;/* '<S16>/get EKF joint values' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_realtime_simu_franka_fr3_T {
  sdAmwXbnJnEmimT0NaJRtAD_realt_T savedTime;/* '<S26>/MATLAB Function' */
  real_T uk_prev_DSTATE[7];            /* '<S6>/uk_prev' */
  real_T uk_prev_DSTATE_b[7];          /* '<S13>/uk_prev' */
  real_T xk_minus_DSTATE[14];          /* '<S29>/xk_minus' */
  real_T Pk_minus_DSTATE[196];         /* '<S29>/Pk_minus' */
  real_T xk_minus_DSTATE_m[14];        /* '<S16>/xk_minus' */
  real_T Pk_minus_DSTATE_b[196];       /* '<S16>/Pk_minus' */
  real_T filterwindow_PreviousInput[301];/* '<S26>/filter window' */
  real_T GetRobotState2_DWORK1;        /* '<S5>/Get Robot State2' */
  real_T TimeStampA;                   /* '<Root>/Derivative' */
  real_T LastUAtTimeA[7];              /* '<Root>/Derivative' */
  real_T TimeStampB;                   /* '<Root>/Derivative' */
  real_T LastUAtTimeB[7];              /* '<Root>/Derivative' */
  real_T PrevY[7];                     /* '<S5>/Rate Limiter' */
  real_T LastMajorTime;                /* '<S5>/Rate Limiter' */
  real_T ApplyControl_DWORK1;          /* '<S5>/Apply Control' */
  real_T ApplyControl_DWORK2;          /* '<S5>/Apply Control' */
  real_T cnt;                          /* '<S10>/torque safety' */
  real_T tau_prev[7];                  /* '<S10>/torque safety' */
  real_T enabled;                      /* '<S8>/home robot logic' */
  real_T t_start;                      /* '<S8>/home robot logic' */
  real_T cnt_g;                        /* '<S26>/MATLAB Function' */
  real_T time_start;                   /* '<S26>/MATLAB Function' */
  real_T freq;                         /* '<S26>/MATLAB Function' */
  real_T cnt_p;                        /* '<S3>/MATLAB Function' */
  real_T run_flag;                     /* '<S3>/MATLAB Function' */
  real_T cnt_o;                        /* '<S12>/PD+ MATLAB Function' */
  real_T run_flag_c;                   /* '<S12>/PD+ MATLAB Function' */
  real_T cnt_d;                        /* '<S11>/CT MATLAB Function' */
  real_T run_flag_d;                   /* '<S11>/CT MATLAB Function' */
  void *robotmodelsfunction2_PWORK[9]; /* '<S9>/robot model s-function2' */
  void *robotmodelsfunction1_PWORK[9]; /* '<S9>/robot model s-function1' */
  struct {
    void *LoggedData[24];
  } errscopeall_PWORK;                 /* '<S4>/err scope all' */

  struct {
    void *LoggedData;
  } Scope1_PWORK;                      /* '<S4>/Scope1' */

  struct {
    void *LoggedData[4];
  } Scope5_PWORK;                      /* '<Root>/Scope5' */

  struct {
    void *LoggedData[3];
  } Scope_PWORK;                       /* '<Root>/Scope' */

  struct {
    void *LoggedData[8];
  } Cosinushnlichkeiten7DOF_PWORK;     /* '<S24>/Cosinus Ähnlichkeiten 7DOF' */

  void *SFunction3_PWORK[4];           /* '<S10>/S-Function3' */
  void *SFunction4_PWORK[14];          /* '<S10>/S-Function4' */
  struct {
    void *LoggedData[2];
  } Scope6_PWORK;                      /* '<S10>/Scope6' */

  struct {
    void *LoggedData;
  } Scope_PWORK_l;                     /* '<S8>/Scope' */

  void *ReducedSystemsfuncasadisolve_PW[9];
                                  /* '<S29>/Reduced System sfun casadi solve' */
  void *robotmodelsfunction2_PWORK_g[9];/* '<S32>/robot model s-function2' */
  void *robotmodelsfunction1_PWORK_i[9];/* '<S32>/robot model s-function1' */
  void *ReducedSystemsfuncasadisolve__f[9];
                                  /* '<S16>/Reduced System sfun casadi solve' */
  void *robotmodelsfunction2_PWORK_a[9];/* '<S19>/robot model s-function2' */
  void *robotmodelsfunction1_PWORK_l[9];/* '<S19>/robot model s-function1' */
  int8_T tau_subsystem_SubsysRanBC;    /* '<Root>/tau_subsystem' */
  int8_T jointspacectlsubsys_SubsysRanBC;/* '<Root>/jointspace ctl subsys' */
  int8_T EKF_SubsysRanBC;              /* '<S6>/EKF' */
  int8_T EKF_SubsysRanBC_p;            /* '<S13>/EKF' */
  int8_T PDControllerSubsystem_SubsysRan;/* '<S2>/PD+ Controller Subsystem' */
  int8_T CTControllerSubsystem_SubsysRan;/* '<S2>/CT Controller Subsystem' */
  boolean_T icLoad;                    /* '<S6>/uk_prev' */
  boolean_T icLoad_f;                  /* '<S13>/uk_prev' */
  boolean_T enabled_not_empty;         /* '<S8>/home robot logic' */
  boolean_T icLoad_d;                  /* '<S29>/xk_minus' */
  boolean_T icLoad_b;                  /* '<S29>/Pk_minus' */
  boolean_T time_start_not_empty;      /* '<S26>/MATLAB Function' */
  boolean_T freq_not_empty;            /* '<S26>/MATLAB Function' */
  boolean_T savedTime_not_empty;       /* '<S26>/MATLAB Function' */
  boolean_T icLoad_c;                  /* '<S16>/xk_minus' */
  boolean_T icLoad_br;                 /* '<S16>/Pk_minus' */
  boolean_T tau_subsystem_MODE;        /* '<Root>/tau_subsystem' */
  boolean_T jointspacectlsubsys_MODE;  /* '<Root>/jointspace ctl subsys' */
  DW_noEKF_realtime_simu_franka_T noEKF_b;/* '<S6>/no EKF' */
  DW_noEKF_realtime_simu_franka_T noEKF;/* '<S13>/no EKF' */
};

/* Parameters (default storage) */
struct P_realtime_simu_franka_fr3_T_ {
  traj_data_bus traj_data_bus_init;    /* Variable: traj_data_bus_init
                                        * Referenced by: '<S3>/Constant3'
                                        */
  struct_8XmNMPeT2ODBNqIYbYAfuC param_savgol;/* Variable: param_savgol
                                              * Referenced by: '<S26>/MATLAB Function'
                                              */
  struct_JvkHcS4XIB8CTs3kuHQtSG param_EKF;/* Variable: param_EKF
                                           * Referenced by:
                                           *   '<S29>/Constant2'
                                           *   '<S16>/Constant'
                                           *   '<S16>/Constant1'
                                           *   '<S16>/Constant2'
                                           */
  struct_PHouuFe0XK8XUK8ogF1YfB ctrl_param;/* Variable: ctrl_param
                                            * Referenced by:
                                            *   '<S11>/CT MATLAB Function'
                                            *   '<S12>/PD+ MATLAB Function'
                                            */
  struct_1o1LUnBmhtekBXGDAaMnRD param_robot;/* Variable: param_robot
                                             * Referenced by:
                                             *   '<S9>/zero fixed states'
                                             *   '<S32>/zero fixed states'
                                             *   '<S19>/zero fixed states'
                                             */
  real_T q_init[7];                    /* Variable: q_init
                                        * Referenced by: '<S5>/Constant'
                                        */
  robot_model Merge_InitialOutput;    /* Computed Parameter: Merge_InitialOutput
                                       * Referenced by: '<S6>/Merge'
                                       */
  robot_model Merge_InitialOutput_p;/* Computed Parameter: Merge_InitialOutput_p
                                     * Referenced by: '<S13>/Merge'
                                     */
  real_T Gain_Gain;                    /* Expression: sqrt(2)
                                        * Referenced by: '<S11>/Gain'
                                        */
  real_T D_d_Value[6];                 /* Expression: [20 20 20 20 20 20]
                                        * Referenced by: '<S11>/D_d'
                                        */
  real_T K_p2_Value[6];            /* Expression: [500, 500, 500, 200, 200, 200]
                                    * Referenced by: '<S11>/K_p2'
                                    */
  real_T Gain_Gain_d;                  /* Expression: sqrt(2)
                                        * Referenced by: '<S12>/Gain'
                                        */
  real_T D_d_Value_k[6];               /* Expression: [20 20 20 20 20 20]
                                        * Referenced by: '<S12>/D_d'
                                        */
  real_T K_d2_Value[6];               /* Expression: [200, 200, 200, 20, 20, 20]
                                       * Referenced by: '<S12>/K_d2'
                                       */
  real_T on_Value;                     /* Expression: 1
                                        * Referenced by: '<S2>/on'
                                        */
  real_T off_Value;                    /* Expression: 0
                                        * Referenced by: '<S2>/off'
                                        */
  real_T Constant_Value[196];
                       /* Expression: diag([ones(1,7)*5e-12, ones(1,7)*6.43e-7])
                        * Referenced by: '<S29>/Constant'
                        */
  real_T Constant1_Value[196];
                             /* Expression: diag([1e5*ones(1,7), 1e5*ones(1,7)])
                              * Referenced by: '<S29>/Constant1'
                              */
  real_T on_Value_a;                   /* Expression: 1
                                        * Referenced by: '<Root>/on'
                                        */
  real_T off_Value_l;                  /* Expression: 0
                                        * Referenced by: '<Root>/off'
                                        */
  real_T tau_Y0;                       /* Computed Parameter: tau_Y0
                                        * Referenced by: '<S8>/tau'
                                        */
  real_T homerunflag_Y0;               /* Computed Parameter: homerunflag_Y0
                                        * Referenced by: '<S8>/home run flag'
                                        */
  real_T K_d_init_Value[7];            /* Expression: [2 2 2 2 2 2 2]
                                        * Referenced by: '<S8>/K_d_init'
                                        */
  real_T t0_Value;                     /* Expression: 3
                                        * Referenced by: '<S8>/t0'
                                        */
  real_T t1_Value;                     /* Expression: 5
                                        * Referenced by: '<S8>/t1'
                                        */
  real_T tend_Value;                   /* Expression: 6
                                        * Referenced by: '<S8>/tend'
                                        */
  real_T K_d_t0_Value[7];              /* Expression: [10 10 10 10 10 10 10]
                                        * Referenced by: '<S8>/K_d_t0'
                                        */
  real_T K_d_t1_Value[7];          /* Expression: [1000 1000 1000 1000 50 50 50]
                                    * Referenced by: '<S8>/K_d_t1'
                                    */
  real_T tau_Y0_j;                     /* Computed Parameter: tau_Y0_j
                                        * Referenced by: '<S10>/tau'
                                        */
  real_T SFunction3_P1_Size[2];        /* Computed Parameter: SFunction3_P1_Size
                                        * Referenced by: '<S10>/S-Function3'
                                        */
  real_T SFunction3_P1[16];            /* Computed Parameter: SFunction3_P1
                                        * Referenced by: '<S10>/S-Function3'
                                        */
  real_T SFunction3_P2_Size[2];        /* Computed Parameter: SFunction3_P2_Size
                                        * Referenced by: '<S10>/S-Function3'
                                        */
  real_T SFunction3_P2[22];            /* Computed Parameter: SFunction3_P2
                                        * Referenced by: '<S10>/S-Function3'
                                        */
  real_T state_valid_Value;            /* Expression: 1
                                        * Referenced by: '<S10>/state_valid'
                                        */
  real_T torque_valid_Value;           /* Expression: 0
                                        * Referenced by: '<S10>/torque_valid'
                                        */
  real_T SFunction4_P1_Size[2];        /* Computed Parameter: SFunction4_P1_Size
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P1[18];            /* Computed Parameter: SFunction4_P1
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P2_Size[2];        /* Computed Parameter: SFunction4_P2_Size
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P2[24];            /* Computed Parameter: SFunction4_P2
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P3_Size[2];        /* Computed Parameter: SFunction4_P3_Size
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P3[24];            /* Computed Parameter: SFunction4_P3
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P4_Size[2];        /* Computed Parameter: SFunction4_P4_Size
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P4[24];            /* Computed Parameter: SFunction4_P4
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P5_Size[2];        /* Computed Parameter: SFunction4_P5_Size
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P5[23];            /* Computed Parameter: SFunction4_P5
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P6_Size[2];        /* Computed Parameter: SFunction4_P6_Size
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P6[30];            /* Computed Parameter: SFunction4_P6
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P7_Size[2];        /* Computed Parameter: SFunction4_P7_Size
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P7[22];            /* Computed Parameter: SFunction4_P7
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P8_Size[2];        /* Computed Parameter: SFunction4_P8_Size
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T SFunction4_P8[21];            /* Computed Parameter: SFunction4_P8
                                        * Referenced by: '<S10>/S-Function4'
                                        */
  real_T filterwindow_InitialCondition[301];/* Expression: zeros(param_savgol.N, 1)
                                             * Referenced by: '<S26>/filter window'
                                             */
  real_T Switch_Threshold;             /* Expression: 0.001
                                        * Referenced by: '<S5>/Switch'
                                        */
  real_T use_casadi_flag_Value;        /* Expression: 1
                                        * Referenced by: '<Root>/use_casadi_flag'
                                        */
  real_T homemode_Value;               /* Expression: 0
                                        * Referenced by: '<Root>/home mode'
                                        */
  real_T home_Value;                   /* Expression: 0
                                        * Referenced by: '<Root>/home'
                                        */
  real_T Constant1_Value_j;            /* Expression: 1
                                        * Referenced by: '<Root>/Constant1'
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
  real_T use_crocoddyl_flag_Value;     /* Expression: 1
                                        * Referenced by: '<Root>/use_crocoddyl_flag'
                                        */
  real_T Constant1_Value_n;            /* Expression: 1
                                        * Referenced by: '<S2>/Constant1'
                                        */
  real_T Constant2_Value;              /* Expression: 2
                                        * Referenced by: '<S2>/Constant2'
                                        */
  real_T Merge1_InitialOutput;       /* Computed Parameter: Merge1_InitialOutput
                                      * Referenced by: '<S2>/Merge1'
                                      */
  real_T K_d_jointspace1_Value[7];     /* Expression: [100 200 500 200 50 50 10]
                                        * Referenced by: '<S7>/K_d_jointspace1'
                                        */
  real_T Gain_Gain_l;                  /* Expression: 2
                                        * Referenced by: '<S7>/Gain'
                                        */
  real_T Constant_Value_i[7];          /* Expression: zeros(7,1)
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T Switch_Threshold_n;           /* Expression: 0
                                        * Referenced by: '<Root>/Switch'
                                        */
  real_T RateLimiter_RisingLim;        /* Expression: 1000
                                        * Referenced by: '<S5>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim;       /* Expression: -1000
                                        * Referenced by: '<S5>/Rate Limiter'
                                        */
  real_T ApplyControl_P1[52];          /* Expression: collision_thresholds
                                        * Referenced by: '<S5>/Apply Control'
                                        */
  real_T ApplyControl_P2[7];           /* Expression: joint_impedance
                                        * Referenced by: '<S5>/Apply Control'
                                        */
  real_T ApplyControl_P3[6];           /* Expression: cartesian_impedance
                                        * Referenced by: '<S5>/Apply Control'
                                        */
  real_T ApplyControl_P4[13];          /* Expression: load_inertia
                                        * Referenced by: '<S5>/Apply Control'
                                        */
  real_T ApplyControl_P5[16];          /* Expression: EE_T_K
                                        * Referenced by: '<S5>/Apply Control'
                                        */
  real_T ApplyControl_P6[7];           /* Expression: init_joint_configuration
                                        * Referenced by: '<S5>/Apply Control'
                                        */
  uint32_T controllerselector_Value;
                                 /* Computed Parameter: controllerselector_Value
                                  * Referenced by: '<Root>/controller selector'
                                  */
  uint32_T trajectoryselector_Value;
                                 /* Computed Parameter: trajectoryselector_Value
                                  * Referenced by: '<Root>/trajectory selector'
                                  */
  uint8_T ManualSwitch_CurrentSetting;
                              /* Computed Parameter: ManualSwitch_CurrentSetting
                               * Referenced by: '<S11>/Manual Switch'
                               */
  uint8_T ManualSwitch_CurrentSetting_h;
                            /* Computed Parameter: ManualSwitch_CurrentSetting_h
                             * Referenced by: '<S12>/Manual Switch'
                             */
  uint8_T ManualSwitch_CurrentSetting_a;
                            /* Computed Parameter: ManualSwitch_CurrentSetting_a
                             * Referenced by: '<Root>/Manual Switch'
                             */
  uint8_T ManualSwitch1_CurrentSetting;
                             /* Computed Parameter: ManualSwitch1_CurrentSetting
                              * Referenced by: '<Root>/Manual Switch1'
                              */
  uint8_T ManualSwitch2_CurrentSetting;
                             /* Computed Parameter: ManualSwitch2_CurrentSetting
                              * Referenced by: '<Root>/Manual Switch2'
                              */
  uint8_T ManualSwitch1_CurrentSetting_m;
                           /* Computed Parameter: ManualSwitch1_CurrentSetting_m
                            * Referenced by: '<S2>/Manual Switch1'
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
    SimStruct childSFunctions[10];
    SimStruct *childSFunctionPtrs[10];
    struct _ssBlkInfo2 blkInfo2[10];
    struct _ssSFcnModelMethods2 methods2[10];
    struct _ssSFcnModelMethods3 methods3[10];
    struct _ssSFcnModelMethods4 methods4[10];
    struct _ssStatesInfo2 statesInfo2[10];
    ssPeriodicStatesInfo periodicStatesInfo[10];
    struct _ssPortInfo2 inputOutputPortInfo2[10];
    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[6];
      struct _ssInPortUnit inputPortUnits[6];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[6];
      int_T iDims0[2];
      int_T iDims1[2];
      int_T iDims2[2];
      int_T iDims3[2];
      int_T iDims4[2];
      int_T iDims5[2];
      struct _ssPortOutputs outputPortInfo[3];
      struct _ssOutPortUnit outputPortUnits[3];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[3];
      int_T oDims0[2];
      int_T oDims1[2];
      int_T oDims2[2];
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

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[6];
      struct _ssInPortUnit inputPortUnits[6];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[6];
      int_T iDims0[2];
      int_T iDims1[2];
      int_T iDims2[2];
      int_T iDims3[2];
      int_T iDims4[2];
      int_T iDims5[2];
      struct _ssPortOutputs outputPortInfo[3];
      struct _ssOutPortUnit outputPortUnits[3];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[3];
      int_T oDims0[2];
      int_T oDims1[2];
      int_T oDims2[2];
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
    } Sfcn5;

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
    } Sfcn6;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[7];
      struct _ssInPortUnit inputPortUnits[7];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[7];
      uint_T attribs[8];
      mxArray *params[8];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn7;

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
    } Sfcn8;

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
    } Sfcn9;
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
 * '<S3>'   : 'realtime_simu_franka_fr3/Create Trajectory'
 * '<S4>'   : 'realtime_simu_franka_fr3/Debug Subsystem'
 * '<S5>'   : 'realtime_simu_franka_fr3/Subsystem'
 * '<S6>'   : 'realtime_simu_franka_fr3/Subsystem1'
 * '<S7>'   : 'realtime_simu_franka_fr3/get full tau'
 * '<S8>'   : 'realtime_simu_franka_fr3/jointspace ctl subsys'
 * '<S9>'   : 'realtime_simu_franka_fr3/robot_model_bus_subsys'
 * '<S10>'  : 'realtime_simu_franka_fr3/tau_subsystem'
 * '<S11>'  : 'realtime_simu_franka_fr3/Controller Subsystem/CT Controller Subsystem'
 * '<S12>'  : 'realtime_simu_franka_fr3/Controller Subsystem/PD+ Controller Subsystem'
 * '<S13>'  : 'realtime_simu_franka_fr3/Controller Subsystem/Subsystem'
 * '<S14>'  : 'realtime_simu_franka_fr3/Controller Subsystem/CT Controller Subsystem/CT MATLAB Function'
 * '<S15>'  : 'realtime_simu_franka_fr3/Controller Subsystem/PD+ Controller Subsystem/PD+ MATLAB Function'
 * '<S16>'  : 'realtime_simu_franka_fr3/Controller Subsystem/Subsystem/EKF'
 * '<S17>'  : 'realtime_simu_franka_fr3/Controller Subsystem/Subsystem/no EKF'
 * '<S18>'  : 'realtime_simu_franka_fr3/Controller Subsystem/Subsystem/EKF/get EKF joint values'
 * '<S19>'  : 'realtime_simu_franka_fr3/Controller Subsystem/Subsystem/EKF/robot_model_bus_subsys'
 * '<S20>'  : 'realtime_simu_franka_fr3/Controller Subsystem/Subsystem/EKF/robot_model_bus_subsys/Robot model bus'
 * '<S21>'  : 'realtime_simu_franka_fr3/Controller Subsystem/Subsystem/EKF/robot_model_bus_subsys/Robot model bus1'
 * '<S22>'  : 'realtime_simu_franka_fr3/Controller Subsystem/Subsystem/EKF/robot_model_bus_subsys/zero fixed states'
 * '<S23>'  : 'realtime_simu_franka_fr3/Create Trajectory/MATLAB Function'
 * '<S24>'  : 'realtime_simu_franka_fr3/Debug Subsystem/Subsystem Reference'
 * '<S25>'  : 'realtime_simu_franka_fr3/Debug Subsystem/calc_errors'
 * '<S26>'  : 'realtime_simu_franka_fr3/Debug Subsystem/tic-toc'
 * '<S27>'  : 'realtime_simu_franka_fr3/Debug Subsystem/Subsystem Reference/manipulability and collinearity 7DOF'
 * '<S28>'  : 'realtime_simu_franka_fr3/Debug Subsystem/tic-toc/MATLAB Function'
 * '<S29>'  : 'realtime_simu_franka_fr3/Subsystem1/EKF'
 * '<S30>'  : 'realtime_simu_franka_fr3/Subsystem1/no EKF'
 * '<S31>'  : 'realtime_simu_franka_fr3/Subsystem1/EKF/get EKF joint values'
 * '<S32>'  : 'realtime_simu_franka_fr3/Subsystem1/EKF/robot_model_bus_subsys'
 * '<S33>'  : 'realtime_simu_franka_fr3/Subsystem1/EKF/robot_model_bus_subsys/Robot model bus'
 * '<S34>'  : 'realtime_simu_franka_fr3/Subsystem1/EKF/robot_model_bus_subsys/Robot model bus1'
 * '<S35>'  : 'realtime_simu_franka_fr3/Subsystem1/EKF/robot_model_bus_subsys/zero fixed states'
 * '<S36>'  : 'realtime_simu_franka_fr3/get full tau/get q_0_ref'
 * '<S37>'  : 'realtime_simu_franka_fr3/get full tau/joint space control fixed q3'
 * '<S38>'  : 'realtime_simu_franka_fr3/jointspace ctl subsys/Joinspace controller'
 * '<S39>'  : 'realtime_simu_franka_fr3/jointspace ctl subsys/get reference pose'
 * '<S40>'  : 'realtime_simu_franka_fr3/jointspace ctl subsys/home robot logic'
 * '<S41>'  : 'realtime_simu_franka_fr3/robot_model_bus_subsys/Robot model bus'
 * '<S42>'  : 'realtime_simu_franka_fr3/robot_model_bus_subsys/Robot model bus1'
 * '<S43>'  : 'realtime_simu_franka_fr3/robot_model_bus_subsys/zero fixed states'
 * '<S44>'  : 'realtime_simu_franka_fr3/tau_subsystem/torque safety'
 */
#endif                              /* RTW_HEADER_realtime_simu_franka_fr3_h_ */
