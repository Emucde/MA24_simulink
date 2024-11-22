/*
 * joint_impedance_control.h
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

#ifndef RTW_HEADER_joint_impedance_control_h_
#define RTW_HEADER_joint_impedance_control_h_
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "ext_work.h"
#include "robot_api.h"
#include "franka/model.h"
#include "control_modes.h"
#include "gripper_api.h"
#include "joint_impedance_control_types.h"
#include <float.h>
#include <stddef.h>
#include <string.h>

extern "C"
{

#include "rt_nonfinite.h"

}

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
#define rtmGetRTWExtModeInfo(rtm)      ((rtm)->extModeInfo)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
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
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

/* Block signals (default storage) */
struct B_joint_impedance_control_T {
  real_T GetInitialRobotState1[16];    /* '<S2>/Get Initial Robot State1' */
  real_T GetDurationPeriod;            /* '<S2>/Get Duration Period' */
  real_T GetModel_o1[16];              /* '<S1>/Get Model' */
  real_T GetModel_o2[42];              /* '<S1>/Get Model' */
  real_T GetModel_o3[42];              /* '<S1>/Get Model' */
  real_T GetModel_o4[49];              /* '<S1>/Get Model' */
  real_T GetModel_o5[7];               /* '<S1>/Get Model' */
  real_T GetModel_o6[7];               /* '<S1>/Get Model' */
  real_T Assignment3[16];              /* '<S2>/Assignment3' */
  real_T GetRobotState_o1[7];          /* '<Root>/Get Robot State' */
  real_T GetRobotState_o2[7];          /* '<Root>/Get Robot State' */
  real_T GetRobotState_o3[7];          /* '<Root>/Get Robot State' */
  real_T Add2[7];                      /* '<S1>/Add2' */
  real_T GetRobotState2[7];            /* '<Root>/Get Robot State2' */
  real_T Add3[7];                      /* '<Root>/Add3' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_joint_impedance_control_T {
  real_T current_velocity_DSTATE;      /* '<S2>/current_velocity' */
  real_T Delay_DSTATE;                 /* '<S3>/Delay' */
  real_T angle_DSTATE;                 /* '<S2>/angle' */
  real_T GetInitialRobotState1_DWORK1; /* '<S2>/Get Initial Robot State1' */
  real_T GetInitialRobotState1_DWORK2; /* '<S2>/Get Initial Robot State1' */
  real_T GetDurationPeriod_DWORK1;     /* '<S2>/Get Duration Period' */
  real_T GetModel_DWORK1;              /* '<S1>/Get Model' */
  real_T GetRobotState_DWORK1;         /* '<Root>/Get Robot State' */
  real_T ApplyControl_DWORK1;          /* '<Root>/Apply Control' */
  real_T ApplyControl_DWORK2;          /* '<Root>/Apply Control' */
  real_T GetRobotState2_DWORK1;        /* '<Root>/Get Robot State2' */
  struct {
    void *LoggedData[3];
  } ControlObjectiveScope_PWORK;       /* '<Root>/Control Objective Scope' */
};

/* Parameters (default storage) */
struct P_joint_impedance_control_T_ {
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S3>/Constant'
                                        */
  real_T Constant2_Value;              /* Expression: 0
                                        * Referenced by: '<S3>/Constant2'
                                        */
  real_T radius_Value;                 /* Expression: 0.05
                                        * Referenced by: '<S2>/radius'
                                        */
  real_T constant_Value;               /* Expression: 1
                                        * Referenced by: '<S2>/constant'
                                        */
  real_T vel_max_Value;                /* Expression: 0.25
                                        * Referenced by: '<S2>/vel_max'
                                        */
  real_T current_velocity_InitialConditi;/* Expression: 0
                                          * Referenced by: '<S2>/current_velocity'
                                          */
  real_T acceleration_time_Value;      /* Expression: 2.0
                                        * Referenced by: '<S2>/acceleration_time'
                                        */
  real_T Delay_InitialCondition;       /* Expression: 0
                                        * Referenced by: '<S3>/Delay'
                                        */
  real_T run_time_Value;               /* Expression: 20.0
                                        * Referenced by: '<S2>/run_time'
                                        */
  real_T constant0_Value;              /* Expression: 0
                                        * Referenced by: '<S2>/constant0'
                                        */
  real_T constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S2>/constant1'
                                        */
  real_T angle_InitialCondition;       /* Expression: 0
                                        * Referenced by: '<S2>/angle'
                                        */
  real_T constant2_Value;              /* Expression: 2*pi
                                        * Referenced by: '<S2>/constant2'
                                        */
  real_T k_gains_Value[7];
                 /* Expression: [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]
                  * Referenced by: '<S1>/k_gains'
                  */
  real_T d_gains_Value[7];
                       /* Expression: [50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0]
                        * Referenced by: '<S1>/d_gains'
                        */
  real_T ApplyControl_P1[52];          /* Expression: collision_thresholds
                                        * Referenced by: '<Root>/Apply Control'
                                        */
  real_T ApplyControl_P2[7];           /* Expression: joint_impedance
                                        * Referenced by: '<Root>/Apply Control'
                                        */
  real_T ApplyControl_P3[6];           /* Expression: cartesian_impedance
                                        * Referenced by: '<Root>/Apply Control'
                                        */
  real_T ApplyControl_P4[13];          /* Expression: load_inertia
                                        * Referenced by: '<Root>/Apply Control'
                                        */
  real_T ApplyControl_P5[16];          /* Expression: EE_T_K
                                        * Referenced by: '<Root>/Apply Control'
                                        */
  real_T ApplyControl_P6[7];           /* Expression: init_joint_configuration
                                        * Referenced by: '<Root>/Apply Control'
                                        */
  boolean_T velocity_amplitude7_Value; /* Expression: false
                                        * Referenced by: '<S2>/velocity_amplitude7'
                                        */
  boolean_T velocity_amplitude1_Value; /* Expression: true
                                        * Referenced by: '<S2>/velocity_amplitude1'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_joint_impedance_contr_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWExtModeInfo *extModeInfo;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T checksums[4];
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
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (default storage) */
#ifdef __cplusplus

extern "C"
{

#endif

  extern P_joint_impedance_control_T joint_impedance_control_P;

#ifdef __cplusplus

}

#endif

/* Block signals (default storage) */
#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_joint_impedance_control_T joint_impedance_control_B;

#ifdef __cplusplus

}

#endif

/* Block states (default storage) */
extern struct DW_joint_impedance_control_T joint_impedance_control_DW;

#ifdef __cplusplus

extern "C"
{

#endif

  /* Model entry point functions */
  extern void joint_impedance_control_initialize(void);
  extern void joint_impedance_control_step(void);
  extern void joint_impedance_control_terminate(void);

#ifdef __cplusplus

}

#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_joint_impedance_cont_T *const joint_impedance_control_M;

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
 * '<Root>' : 'joint_impedance_control'
 * '<S1>'   : 'joint_impedance_control/Joint Impedance Controller'
 * '<S2>'   : 'joint_impedance_control/Motion Gen Cartesian Space Draw Circle'
 * '<S3>'   : 'joint_impedance_control/Motion Gen Cartesian Space Draw Circle/counter'
 * '<S4>'   : 'joint_impedance_control/Motion Gen Cartesian Space Draw Circle/counter/DocBlock'
 */
#endif                               /* RTW_HEADER_joint_impedance_control_h_ */
