/*
 * cartesian_impedance_control.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "cartesian_impedance_control".
 *
 * Model version              : 8.2
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Thu Sep 26 11:23:31 2024
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
#include "cartesian_impedance_control_types.h"
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
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
struct B_cartesian_impedance_control_T {
  real_T GetRobotState2_o1[7];         /* '<S2>/Get Robot State2' */
  real_T GetRobotState2_o2[7];         /* '<S2>/Get Robot State2' */
  real_T GetRobotState2_o3[6];         /* '<S2>/Get Robot State2' */
  real_T GetModel_o1[16];              /* '<S1>/Get Model' */
  real_T GetModel_o2[42];              /* '<S1>/Get Model' */
  real_T GetModel_o3[42];              /* '<S1>/Get Model' */
  real_T GetModel_o4[49];              /* '<S1>/Get Model' */
  real_T coriolis[7];                  /* '<S1>/Get Model' */
  real_T GetModel_o6[7];               /* '<S1>/Get Model' */
  real_T GetRobotState_o1[16];         /* '<Root>/Get Robot State' */
  real_T GetRobotState_o2[7];          /* '<Root>/Get Robot State' */
  real_T GetInitialRobotState[16];     /* '<Root>/Get Initial Robot State' */
  real_T RateLimiter[7];               /* '<S2>/Rate Limiter' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_cartesian_impedance_contro_T {
  real_T GetRobotState2_DWORK1;        /* '<S2>/Get Robot State2' */
  real_T GetModel_DWORK1;              /* '<S1>/Get Model' */
  real_T GetRobotState_DWORK1;         /* '<Root>/Get Robot State' */
  real_T GetInitialRobotState_DWORK1;  /* '<Root>/Get Initial Robot State' */
  real_T GetInitialRobotState_DWORK2;  /* '<Root>/Get Initial Robot State' */
  real_T PrevY[7];                     /* '<S2>/Rate Limiter' */
  real_T ApplyControl_DWORK1;          /* '<S2>/Apply Control' */
  real_T ApplyControl_DWORK2;          /* '<S2>/Apply Control' */
  struct {
    void *LoggedData;
  } Scope_PWORK;                       /* '<Root>/Scope' */
};

/* Parameters (default storage) */
struct P_cartesian_impedance_control_T_ {
  real_T stiffness_Value[36];
                          /* Expression: diag([600, 600, 600, 50.0, 50.0, 50.0])
                           * Referenced by: '<S1>/stiffness'
                           */
  real_T damping_Value[36];
            /* Expression: diag(2*sqrt([150.0, 150.0, 150.0, 50.0, 50.0, 50.0]))
             * Referenced by: '<S1>/damping'
             */
  real_T RateLimiter_RisingLim;        /* Expression: 1000
                                        * Referenced by: '<S2>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim;       /* Expression: -1000
                                        * Referenced by: '<S2>/Rate Limiter'
                                        */
  real_T RateLimiter_IC;               /* Expression: 0
                                        * Referenced by: '<S2>/Rate Limiter'
                                        */
  real_T ApplyControl_P1[52];          /* Expression: collision_thresholds
                                        * Referenced by: '<S2>/Apply Control'
                                        */
  real_T ApplyControl_P2[7];           /* Expression: joint_impedance
                                        * Referenced by: '<S2>/Apply Control'
                                        */
  real_T ApplyControl_P3[6];           /* Expression: cartesian_impedance
                                        * Referenced by: '<S2>/Apply Control'
                                        */
  real_T ApplyControl_P4[13];          /* Expression: load_inertia
                                        * Referenced by: '<S2>/Apply Control'
                                        */
  real_T ApplyControl_P5[16];          /* Expression: EE_T_K
                                        * Referenced by: '<S2>/Apply Control'
                                        */
  real_T ApplyControl_P6[7];           /* Expression: init_joint_configuration
                                        * Referenced by: '<S2>/Apply Control'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_cartesian_impedance_c_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWExtModeInfo *extModeInfo;
  RTWSolverInfo solverInfo;

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
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
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
extern struct DW_cartesian_impedance_contro_T cartesian_impedance_control_DW;

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

  extern RT_MODEL_cartesian_impedance__T *const cartesian_impedance_control_M;

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
 * '<S2>'   : 'cartesian_impedance_control/Subsystem'
 * '<S3>'   : 'cartesian_impedance_control/Cartesian Impedance Controller/MATLAB Function'
 */
#endif                           /* RTW_HEADER_cartesian_impedance_control_h_ */
