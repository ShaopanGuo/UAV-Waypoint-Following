/*
 * PosControl_Sim.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "PosControl_Sim".
 *
 * Model version              : 1.0
 * Simulink Coder version : 9.3 (R2020a) 18-Nov-2019
 * C source code generated on : Tue Mar 30 22:24:46 2021
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_PosControl_Sim_h_
#define RTW_HEADER_PosControl_Sim_h_
#include <math.h>
#include <string.h>
#include <float.h>
#include <stddef.h>
#ifndef PosControl_Sim_COMMON_INCLUDES_
# define PosControl_Sim_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* PosControl_Sim_COMMON_INCLUDES_ */

#include "PosControl_Sim_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetRTWLogInfo
# define rtmGetRTWLogInfo(rtm)         ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

/* Block signals for system '<S5>/deadzone1' */
typedef struct {
  real_T y;                            /* '<S5>/deadzone1' */
} B_deadzone1_PosControl_Sim_T;

/* Block signals (default storage) */
typedef struct {
  real_T Constant1[3];                 /* '<S50>/Constant1' */
  real_T xe[3];                        /* '<S50>/xe' */
  real_T Sum8;                         /* '<S63>/Sum8' */
  real_T Sum9;                         /* '<S63>/Sum9' */
  real_T Sum10;                        /* '<S63>/Sum10' */
  real_T Clock;                        /* '<S1>/Clock' */
  real_T Saturation10;                 /* '<S6>/Saturation10' */
  real_T deg2rad1;                     /* '<S6>/deg2rad1' */
  real_T deg2rad2;                     /* '<S6>/deg2rad2' */
  real_T deg2rad3;                     /* '<S6>/deg2rad3' */
  real_T Constant2[3];                 /* '<S49>/Constant2' */
  real_T Product1[3];                  /* '<S49>/Product1' */
  real_T Constant1_d[3];               /* '<S49>/Constant1' */
  real_T DiscreteTimeIntegrator;       /* '<S35>/Discrete-Time Integrator' */
  real_T DerivativeGain;               /* '<S35>/Derivative Gain' */
  real_T Diff;                         /* '<S38>/Diff' */
  real_T DiscreteTimeIntegrator_o;     /* '<S36>/Discrete-Time Integrator' */
  real_T DerivativeGain_g;             /* '<S36>/Derivative Gain' */
  real_T Diff_h;                       /* '<S39>/Diff' */
  real_T DiscreteTimeIntegrator_e;     /* '<S26>/Discrete-Time Integrator' */
  real_T DerivativeGain_m;             /* '<S26>/Derivative Gain' */
  real_T Diff_o;                       /* '<S29>/Diff' */
  real_T DiscreteTimeIntegrator_er;    /* '<S24>/Discrete-Time Integrator' */
  real_T DerivativeGain_j;             /* '<S24>/Derivative Gain' */
  real_T Diff_i;                       /* '<S28>/Diff' */
  real_T DiscreteTimeIntegrator_m;     /* '<S27>/Discrete-Time Integrator' */
  real_T DerivativeGain_n;             /* '<S27>/Derivative Gain' */
  real_T Diff_d;                       /* '<S30>/Diff' */
  real_T IntegralGain;                 /* '<S24>/Integral Gain' */
  real_T IntegralGain_m;               /* '<S26>/Integral Gain' */
  real_T IntegralGain_p;               /* '<S27>/Integral Gain' */
  real_T DiscreteTimeIntegrator_b;     /* '<S37>/Discrete-Time Integrator' */
  real_T DerivativeGain_d;             /* '<S37>/Derivative Gain' */
  real_T Diff_e;                       /* '<S40>/Diff' */
  real_T IntegralGain_e;               /* '<S35>/Integral Gain' */
  real_T IntegralGain_i;               /* '<S36>/Integral Gain' */
  real_T IntegralGain_l;               /* '<S37>/Integral Gain' */
  real_T Product1_o[3];                /* '<S2>/Product1' */
  real_T Sum1;                         /* '<S45>/Sum1' */
  real_T Sum1_k;                       /* '<S46>/Sum1' */
  real_T Sum1_k3;                      /* '<S47>/Sum1' */
  real_T Sum1_b;                       /* '<S48>/Sum1' */
  real_T Divide[9];                    /* '<S49>/Divide' */
  real_T Product[3];                   /* '<S49>/Product' */
  real_T Sum1_o[3];                    /* '<S49>/Sum1' */
  real_T qdot[4];                      /* '<S50>/Quaternions model' */
  real_T y[3];                         /* '<S50>/Quaternion to Euler' */
  real_T q[4];                         /* '<S50>/Euler angle to quaternion' */
  real_T vx_d;                         /* '<S1>/MATLAB Function2' */
  real_T vy_d;                         /* '<S1>/MATLAB Function2' */
  real_T vz_d;                         /* '<S1>/MATLAB Function2' */
  real_T x_d;                          /* '<S1>/MATLAB Function2' */
  real_T y_d;                          /* '<S1>/MATLAB Function2' */
  real_T z_d;                          /* '<S1>/MATLAB Function2' */
  real_T reset_i[4];                   /* '<S1>/MATLAB Function1' */
  uint32_T DataTypeConversion4;        /* '<S3>/Data Type Conversion4' */
  uint32_T TmpSignalConversionAtPacknet_fd[4];
  real32_T DataTypeConversion5[4];     /* '<S3>/Data Type Conversion5' */
  real32_T DataTypeConversion6[3];     /* '<S3>/Data Type Conversion6' */
  real32_T DataTypeConversion7;        /* '<S3>/Data Type Conversion7' */
  real32_T TmpSignalConversionAtPacknet__i[4];
  real32_T TmpSignalConversionAtPacknet__h[4];
  real32_T TmpSignalConversionAtPacknet__g[4];
  real32_T TmpSignalConversionAtPacknet_gm[4];
  real32_T TmpSignalConversionAtPacknet__f[4];
  real32_T TmpSignalConversionAtPacknet__o[4];
  real32_T TmpSignalConversionAtPacknet__n[4];
  real32_T TmpSignalConversionAtPacknet__a[4];
  real32_T TmpSignalConversionAtPacknet_nd[4];
  uint8_T RateTransition[408];         /* '<S3>/Rate Transition' */
  uint8_T Packnet_fdmPacketforFlightGear[408];
                                 /* '<S3>/Pack net_fdm Packet for FlightGear' */
  B_deadzone1_PosControl_Sim_T sf_deadzone4;/* '<S6>/deadzone4' */
  B_deadzone1_PosControl_Sim_T sf_deadzone3_g;/* '<S6>/deadzone3' */
  B_deadzone1_PosControl_Sim_T sf_deadzone2_b;/* '<S6>/deadzone2' */
  B_deadzone1_PosControl_Sim_T sf_deadzone1_d;/* '<S6>/deadzone1' */
  B_deadzone1_PosControl_Sim_T sf_deadzone3;/* '<S5>/deadzone3' */
  B_deadzone1_PosControl_Sim_T sf_deadzone2;/* '<S5>/deadzone2' */
  B_deadzone1_PosControl_Sim_T sf_deadzone1;/* '<S5>/deadzone1' */
} B_PosControl_Sim_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S35>/Discrete-Time Integrator' */
  real_T UD_DSTATE;                    /* '<S38>/UD' */
  real_T DiscreteTimeIntegrator_DSTATE_f;/* '<S36>/Discrete-Time Integrator' */
  real_T UD_DSTATE_g;                  /* '<S39>/UD' */
  real_T DiscreteTimeIntegrator_DSTATE_k;/* '<S26>/Discrete-Time Integrator' */
  real_T UD_DSTATE_p;                  /* '<S29>/UD' */
  real_T DiscreteTimeIntegrator_DSTATE_o;/* '<S24>/Discrete-Time Integrator' */
  real_T UD_DSTATE_m;                  /* '<S28>/UD' */
  real_T DiscreteTimeIntegrator_DSTATE_l;/* '<S27>/Discrete-Time Integrator' */
  real_T UD_DSTATE_h;                  /* '<S30>/UD' */
  real_T DiscreteTimeIntegrator_DSTATE_j;/* '<S37>/Discrete-Time Integrator' */
  real_T UD_DSTATE_b;                  /* '<S40>/UD' */
  real_T Divide_DWORK4[9];             /* '<S49>/Divide' */
  int_T q_IWORK;                       /* '<S50>/q' */
  int_T xe_IWORK;                      /* '<S50>/xe' */
  int_T vb_IWORK;                      /* '<S49>/vb' */
  int_T pqr_IWORK;                     /* '<S49>/p,q,r' */
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S35>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_n;/* '<S36>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_m;/* '<S26>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_p;/* '<S24>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_i;/* '<S27>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_b;/* '<S37>/Discrete-Time Integrator' */
  uint8_T RateTransition_Buffer0[408]; /* '<S3>/Rate Transition' */
  uint8_T UDPSend_DWORK1[20];          /* '<S62>/UDP Send' */
} DW_PosControl_Sim_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Motor_Dynamics_1_CSTATE;      /* '<S42>/Motor_Dynamics_1' */
  real_T Motor_Dynamics_2_CSTATE;      /* '<S42>/Motor_Dynamics_2' */
  real_T Motor_Dynamics_3_CSTATE;      /* '<S42>/Motor_Dynamics_3' */
  real_T Motor_Dynamics_4_CSTATE;      /* '<S42>/Motor_Dynamics_4' */
  real_T q_CSTATE[4];                  /* '<S50>/q' */
  real_T xe_CSTATE[3];                 /* '<S50>/xe' */
  real_T vb_CSTATE[3];                 /* '<S49>/vb' */
  real_T pqr_CSTATE[3];                /* '<S49>/p,q,r' */
} X_PosControl_Sim_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Motor_Dynamics_1_CSTATE;      /* '<S42>/Motor_Dynamics_1' */
  real_T Motor_Dynamics_2_CSTATE;      /* '<S42>/Motor_Dynamics_2' */
  real_T Motor_Dynamics_3_CSTATE;      /* '<S42>/Motor_Dynamics_3' */
  real_T Motor_Dynamics_4_CSTATE;      /* '<S42>/Motor_Dynamics_4' */
  real_T q_CSTATE[4];                  /* '<S50>/q' */
  real_T xe_CSTATE[3];                 /* '<S50>/xe' */
  real_T vb_CSTATE[3];                 /* '<S49>/vb' */
  real_T pqr_CSTATE[3];                /* '<S49>/p,q,r' */
} XDot_PosControl_Sim_T;

/* State disabled  */
typedef struct {
  boolean_T Motor_Dynamics_1_CSTATE;   /* '<S42>/Motor_Dynamics_1' */
  boolean_T Motor_Dynamics_2_CSTATE;   /* '<S42>/Motor_Dynamics_2' */
  boolean_T Motor_Dynamics_3_CSTATE;   /* '<S42>/Motor_Dynamics_3' */
  boolean_T Motor_Dynamics_4_CSTATE;   /* '<S42>/Motor_Dynamics_4' */
  boolean_T q_CSTATE[4];               /* '<S50>/q' */
  boolean_T xe_CSTATE[3];              /* '<S50>/xe' */
  boolean_T vb_CSTATE[3];              /* '<S49>/vb' */
  boolean_T pqr_CSTATE[3];             /* '<S49>/p,q,r' */
} XDis_PosControl_Sim_T;

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[4];                        /* derivatives */
} ODE4_IntgData;

#endif

/* Parameters (default storage) */
struct P_PosControl_Sim_T_ {
  real_T DEG2RAD;                      /* Variable: DEG2RAD
                                        * Referenced by:
                                        *   '<S6>/deg2rad1'
                                        *   '<S6>/deg2rad2'
                                        *   '<S6>/deg2rad3'
                                        *   '<S21>/Saturation1'
                                        *   '<S21>/Saturation2'
                                        */
  real_T Kd_PITCH_AngleRate;           /* Variable: Kd_PITCH_AngleRate
                                        * Referenced by: '<S24>/Derivative Gain'
                                        */
  real_T Kd_ROLL_AngleRate;            /* Variable: Kd_ROLL_AngleRate
                                        * Referenced by: '<S26>/Derivative Gain'
                                        */
  real_T Kd_YAW_AngleRate;             /* Variable: Kd_YAW_AngleRate
                                        * Referenced by: '<S27>/Derivative Gain'
                                        */
  real_T Ki_PITCH_AngleRate;           /* Variable: Ki_PITCH_AngleRate
                                        * Referenced by: '<S24>/Integral Gain'
                                        */
  real_T Ki_ROLL_AngleRate;            /* Variable: Ki_ROLL_AngleRate
                                        * Referenced by: '<S26>/Integral Gain'
                                        */
  real_T Ki_YAW_AngleRate;             /* Variable: Ki_YAW_AngleRate
                                        * Referenced by: '<S27>/Integral Gain'
                                        */
  real_T Kp_PITCH_ANGLE;               /* Variable: Kp_PITCH_ANGLE
                                        * Referenced by: '<S23>/Gain'
                                        */
  real_T Kp_PITCH_AngleRate;           /* Variable: Kp_PITCH_AngleRate
                                        * Referenced by: '<S24>/Gain'
                                        */
  real_T Kp_ROLL_ANGLE;                /* Variable: Kp_ROLL_ANGLE
                                        * Referenced by: '<S25>/Gain'
                                        */
  real_T Kp_ROLL_AngleRate;            /* Variable: Kp_ROLL_AngleRate
                                        * Referenced by: '<S26>/Gain'
                                        */
  real_T Kp_YAW_AngleRate;             /* Variable: Kp_YAW_AngleRate
                                        * Referenced by: '<S27>/Gain'
                                        */
  real_T Kpxp;                         /* Variable: Kpxp
                                        * Referenced by: '<S32>/Gain1'
                                        */
  real_T Kpyp;                         /* Variable: Kpyp
                                        * Referenced by: '<S33>/Gain1'
                                        */
  real_T Kpzp;                         /* Variable: Kpzp
                                        * Referenced by: '<S34>/Gain1'
                                        */
  real_T Kvxd;                         /* Variable: Kvxd
                                        * Referenced by: '<S35>/Derivative Gain'
                                        */
  real_T Kvxi;                         /* Variable: Kvxi
                                        * Referenced by: '<S35>/Integral Gain'
                                        */
  real_T Kvxp;                         /* Variable: Kvxp
                                        * Referenced by: '<S35>/Gain'
                                        */
  real_T Kvyd;                         /* Variable: Kvyd
                                        * Referenced by: '<S36>/Derivative Gain'
                                        */
  real_T Kvyi;                         /* Variable: Kvyi
                                        * Referenced by: '<S36>/Integral Gain'
                                        */
  real_T Kvyp;                         /* Variable: Kvyp
                                        * Referenced by: '<S36>/Gain'
                                        */
  real_T Kvzd;                         /* Variable: Kvzd
                                        * Referenced by: '<S37>/Derivative Gain'
                                        */
  real_T Kvzi;                         /* Variable: Kvzi
                                        * Referenced by: '<S37>/Integral Gain'
                                        */
  real_T Kvzp;                         /* Variable: Kvzp
                                        * Referenced by: '<S37>/Gain'
                                        */
  real_T MAX_CONTROL_ANGLE_PITCH;      /* Variable: MAX_CONTROL_ANGLE_PITCH
                                        * Referenced by:
                                        *   '<S6>/Gain3'
                                        *   '<S21>/Saturation2'
                                        */
  real_T MAX_CONTROL_ANGLE_RATE_PITCH; /* Variable: MAX_CONTROL_ANGLE_RATE_PITCH
                                        * Referenced by: '<S20>/Saturation3'
                                        */
  real_T MAX_CONTROL_ANGLE_RATE_ROLL;  /* Variable: MAX_CONTROL_ANGLE_RATE_ROLL
                                        * Referenced by: '<S20>/Saturation5'
                                        */
  real_T MAX_CONTROL_ANGLE_RATE_Y;     /* Variable: MAX_CONTROL_ANGLE_RATE_Y
                                        * Referenced by: '<S6>/Gain2'
                                        */
  real_T MAX_CONTROL_ANGLE_ROLL;       /* Variable: MAX_CONTROL_ANGLE_ROLL
                                        * Referenced by:
                                        *   '<S6>/Gain1'
                                        *   '<S21>/Saturation1'
                                        */
  real_T MAX_CONTROL_VELOCITY_XY;      /* Variable: MAX_CONTROL_VELOCITY_XY
                                        * Referenced by:
                                        *   '<S5>/max_vx'
                                        *   '<S5>/max_vy'
                                        *   '<S32>/Saturation3'
                                        *   '<S33>/Saturation3'
                                        */
  real_T MAX_CONTROL_VELOCITY_Z;       /* Variable: MAX_CONTROL_VELOCITY_Z
                                        * Referenced by:
                                        *   '<S5>/max_vz'
                                        *   '<S34>/Saturation3'
                                        */
  real_T ModelInit_AngEuler[3];        /* Variable: ModelInit_AngEuler
                                        * Referenced by: '<S50>/Constant'
                                        */
  real_T ModelInit_PosE[3];            /* Variable: ModelInit_PosE
                                        * Referenced by: '<S50>/Constant1'
                                        */
  real_T ModelInit_Rads;               /* Variable: ModelInit_Rads
                                        * Referenced by:
                                        *   '<S42>/Motor_Dynamics_1'
                                        *   '<S42>/Motor_Dynamics_2'
                                        *   '<S42>/Motor_Dynamics_3'
                                        *   '<S42>/Motor_Dynamics_4'
                                        */
  real_T ModelInit_RateB[3];           /* Variable: ModelInit_RateB
                                        * Referenced by: '<S49>/Constant1'
                                        */
  real_T ModelInit_VelB[3];            /* Variable: ModelInit_VelB
                                        * Referenced by: '<S49>/Constant2'
                                        */
  real_T ModelParam_motorCr;           /* Variable: ModelParam_motorCr
                                        * Referenced by:
                                        *   '<S45>/Gain'
                                        *   '<S46>/Gain'
                                        *   '<S47>/Gain'
                                        *   '<S48>/Gain'
                                        */
  real_T ModelParam_motorJm;           /* Variable: ModelParam_motorJm
                                        * Referenced by: '<S41>/Jrp'
                                        */
  real_T ModelParam_motorT;            /* Variable: ModelParam_motorT
                                        * Referenced by:
                                        *   '<S42>/Motor_Dynamics_1'
                                        *   '<S42>/Motor_Dynamics_2'
                                        *   '<S42>/Motor_Dynamics_3'
                                        *   '<S42>/Motor_Dynamics_4'
                                        */
  real_T ModelParam_motorWb;           /* Variable: ModelParam_motorWb
                                        * Referenced by:
                                        *   '<S45>/Constant'
                                        *   '<S46>/Constant'
                                        *   '<S47>/Constant'
                                        *   '<S48>/Constant'
                                        */
  real_T ModelParam_rotorCm;           /* Variable: ModelParam_rotorCm
                                        * Referenced by: '<S41>/Constant4'
                                        */
  real_T ModelParam_rotorCt;           /* Variable: ModelParam_rotorCt
                                        * Referenced by: '<S41>/Constant3'
                                        */
  real_T ModelParam_uavCCm[3];         /* Variable: ModelParam_uavCCm
                                        * Referenced by: '<S41>/Constant1'
                                        */
  real_T ModelParam_uavCd;             /* Variable: ModelParam_uavCd
                                        * Referenced by: '<S41>/Constant7'
                                        */
  real_T ModelParam_uavJ[9];           /* Variable: ModelParam_uavJ
                                        * Referenced by: '<S49>/J '
                                        */
  real_T ModelParam_uavMass;           /* Variable: ModelParam_uavMass
                                        * Referenced by:
                                        *   '<S2>/mass'
                                        *   '<S49>/ Mass '
                                        */
  real_T ModelParam_uavR;              /* Variable: ModelParam_uavR
                                        * Referenced by: '<S41>/Constant5'
                                        */
  real_T Saturation_I_RP_Max;          /* Variable: Saturation_I_RP_Max
                                        * Referenced by:
                                        *   '<S24>/Discrete-Time Integrator'
                                        *   '<S26>/Discrete-Time Integrator'
                                        */
  real_T Saturation_I_RP_Min;          /* Variable: Saturation_I_RP_Min
                                        * Referenced by:
                                        *   '<S24>/Discrete-Time Integrator'
                                        *   '<S26>/Discrete-Time Integrator'
                                        */
  real_T Saturation_I_Y_Max;           /* Variable: Saturation_I_Y_Max
                                        * Referenced by: '<S27>/Discrete-Time Integrator'
                                        */
  real_T Saturation_I_Y_Min;           /* Variable: Saturation_I_Y_Min
                                        * Referenced by: '<S27>/Discrete-Time Integrator'
                                        */
  real_T Saturation_I_ah;              /* Variable: Saturation_I_ah
                                        * Referenced by:
                                        *   '<S35>/Discrete-Time Integrator'
                                        *   '<S36>/Discrete-Time Integrator'
                                        */
  real_T Saturation_I_az;              /* Variable: Saturation_I_az
                                        * Referenced by: '<S37>/Discrete-Time Integrator'
                                        */
  real_T THR_HOVER;                    /* Variable: THR_HOVER
                                        * Referenced by:
                                        *   '<S6>/thr_hover '
                                        *   '<S21>/Constant'
                                        */
  real_T DiscreteDerivative_ICPrevScaled;
                              /* Mask Parameter: DiscreteDerivative_ICPrevScaled
                               * Referenced by: '<S38>/UD'
                               */
  real_T DiscreteDerivative_ICPrevScal_f;
                              /* Mask Parameter: DiscreteDerivative_ICPrevScal_f
                               * Referenced by: '<S39>/UD'
                               */
  real_T DiscreteDerivative_ICPrevScal_i;
                              /* Mask Parameter: DiscreteDerivative_ICPrevScal_i
                               * Referenced by: '<S29>/UD'
                               */
  real_T DiscreteDerivative_ICPrevScal_c;
                              /* Mask Parameter: DiscreteDerivative_ICPrevScal_c
                               * Referenced by: '<S28>/UD'
                               */
  real_T DiscreteDerivative_ICPrevSca_iz;
                              /* Mask Parameter: DiscreteDerivative_ICPrevSca_iz
                               * Referenced by: '<S30>/UD'
                               */
  real_T DiscreteDerivative_ICPrevSca_f3;
                              /* Mask Parameter: DiscreteDerivative_ICPrevSca_f3
                               * Referenced by: '<S40>/UD'
                               */
  int32_T Sendnet_fdmPackettoFlightGear_D;
                              /* Mask Parameter: Sendnet_fdmPackettoFlightGear_D
                               * Referenced by: '<S62>/UDP Send'
                               */
  real_T az_UpperSat;                  /* Expression: 0.4
                                        * Referenced by: '<S21>/az'
                                        */
  real_T az_LowerSat;                  /* Expression: -0.4
                                        * Referenced by: '<S21>/az'
                                        */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S21>/Gain'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 0.9
                                        * Referenced by: '<S21>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: 0
                                        * Referenced by: '<S21>/Saturation'
                                        */
  real_T PacketSize_Value;             /* Expression: 408
                                        * Referenced by: '<S62>/PacketSize'
                                        */
  real_T Constant_Value;               /* Expression: 4
                                        * Referenced by: '<S3>/Constant'
                                        */
  real_T Motor_Dynamics_1_B;           /* Expression: 1
                                        * Referenced by: '<S42>/Motor_Dynamics_1'
                                        */
  real_T Motor_Dynamics_2_B;           /* Expression: 1
                                        * Referenced by: '<S42>/Motor_Dynamics_2'
                                        */
  real_T Motor_Dynamics_3_B;           /* Expression: 1
                                        * Referenced by: '<S42>/Motor_Dynamics_3'
                                        */
  real_T Motor_Dynamics_4_B;           /* Expression: 1
                                        * Referenced by: '<S42>/Motor_Dynamics_4'
                                        */
  real_T Gain_Gain_j;                  /* Expression: 30/pi
                                        * Referenced by: '<S2>/Gain'
                                        */
  real_T Gain_Gain_b;                  /* Expression: -1
                                        * Referenced by: '<S3>/Gain'
                                        */
  real_T Gain5_Gain;                   /* Expression: 1
                                        * Referenced by: '<S3>/Gain5'
                                        */
  real_T Gain3_Gain;                   /* Expression: -1
                                        * Referenced by: '<S3>/Gain3'
                                        */
  real_T Gain6_Gain;                   /* Expression: 1
                                        * Referenced by: '<S3>/Gain6'
                                        */
  real_T Gain1_Gain;                   /* Expression: 10
                                        * Referenced by: '<S3>/Gain1'
                                        */
  real_T Step_Time;                    /* Expression: 0
                                        * Referenced by: '<S3>/Step'
                                        */
  real_T Step_Y0;                      /* Expression: 0
                                        * Referenced by: '<S3>/Step'
                                        */
  real_T Step_YFinal;                  /* Expression: 1
                                        * Referenced by: '<S3>/Step'
                                        */
  real_T Gain1_Gain_l;                 /* Expression: 10/earthRadius
                                        * Referenced by: '<S63>/Gain1'
                                        */
  real_T Constant2_Value;              /* Expression: -2.1361
                                        * Referenced by: '<S63>/Constant2'
                                        */
  real_T Gain4_Gain;                   /* Expression: 1
                                        * Referenced by: '<S63>/Gain4'
                                        */
  real_T Gain2_Gain;                   /* Expression: 10/earthRadius
                                        * Referenced by: '<S63>/Gain2'
                                        */
  real_T Constant3_Value;              /* Expression: 0.65673
                                        * Referenced by: '<S63>/Constant3'
                                        */
  real_T Gain8_Gain;                   /* Expression: -10
                                        * Referenced by: '<S63>/Gain8'
                                        */
  real_T Constant1_Value;              /* Expression: 5
                                        * Referenced by: '<S63>/Constant1'
                                        */
  real_T Packnet_fdmPacketforFlightGear_;/* Expression: SampleTime
                                          * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
                                          */
  real_T SimulationPace_P1;            /* Expression: SimulationPace
                                        * Referenced by: '<S3>/Simulation Pace'
                                        */
  real_T SimulationPace_P2;            /* Expression: SleepMode
                                        * Referenced by: '<S3>/Simulation Pace'
                                        */
  real_T SimulationPace_P3;            /* Expression: OutputPaceError
                                        * Referenced by: '<S3>/Simulation Pace'
                                        */
  real_T SimulationPace_P4;            /* Expression: SampleTime
                                        * Referenced by: '<S3>/Simulation Pace'
                                        */
  real_T Constant_Value_h;             /* Expression: 2
                                        * Referenced by: '<S1>/Constant'
                                        */
  real_T Saturation8_UpperSat;         /* Expression: 1
                                        * Referenced by: '<S5>/Saturation8'
                                        */
  real_T Saturation8_LowerSat;         /* Expression: -1
                                        * Referenced by: '<S5>/Saturation8'
                                        */
  real_T Saturation9_UpperSat;         /* Expression: 1
                                        * Referenced by: '<S5>/Saturation9'
                                        */
  real_T Saturation9_LowerSat;         /* Expression: -1
                                        * Referenced by: '<S5>/Saturation9'
                                        */
  real_T Saturation10_UpperSat;        /* Expression: 1
                                        * Referenced by: '<S5>/Saturation10'
                                        */
  real_T Saturation10_LowerSat;        /* Expression: -1
                                        * Referenced by: '<S5>/Saturation10'
                                        */
  real_T Constant_Value_hk;            /* Expression: 1
                                        * Referenced by: '<S6>/Constant'
                                        */
  real_T Gain_Gain_e;                  /* Expression: 0.5
                                        * Referenced by: '<S6>/Gain'
                                        */
  real_T Saturation10_UpperSat_e;      /* Expression: 1
                                        * Referenced by: '<S6>/Saturation10'
                                        */
  real_T Saturation10_LowerSat_g;      /* Expression: 0
                                        * Referenced by: '<S6>/Saturation10'
                                        */
  real_T Saturation9_UpperSat_o;       /* Expression: 1
                                        * Referenced by: '<S6>/Saturation9'
                                        */
  real_T Saturation9_LowerSat_l;       /* Expression: -1
                                        * Referenced by: '<S6>/Saturation9'
                                        */
  real_T Saturation8_UpperSat_d;       /* Expression: 1
                                        * Referenced by: '<S6>/Saturation8'
                                        */
  real_T Saturation8_LowerSat_c;       /* Expression: -1
                                        * Referenced by: '<S6>/Saturation8'
                                        */
  real_T Saturation7_UpperSat;         /* Expression: 1
                                        * Referenced by: '<S6>/Saturation7'
                                        */
  real_T Saturation7_LowerSat;         /* Expression: -1
                                        * Referenced by: '<S6>/Saturation7'
                                        */
  real_T DiscreteTimeIntegrator_gainval;
                           /* Computed Parameter: DiscreteTimeIntegrator_gainval
                            * Referenced by: '<S35>/Discrete-Time Integrator'
                            */
  real_T DiscreteTimeIntegrator_IC;    /* Expression: 0
                                        * Referenced by: '<S35>/Discrete-Time Integrator'
                                        */
  real_T TSamp_WtEt;                   /* Computed Parameter: TSamp_WtEt
                                        * Referenced by: '<S38>/TSamp'
                                        */
  real_T DiscreteTimeIntegrator_gainva_m;
                          /* Computed Parameter: DiscreteTimeIntegrator_gainva_m
                           * Referenced by: '<S36>/Discrete-Time Integrator'
                           */
  real_T DiscreteTimeIntegrator_IC_e;  /* Expression: 0
                                        * Referenced by: '<S36>/Discrete-Time Integrator'
                                        */
  real_T TSamp_WtEt_n;                 /* Computed Parameter: TSamp_WtEt_n
                                        * Referenced by: '<S39>/TSamp'
                                        */
  real_T Switch1_1_Threshold;          /* Expression: 2
                                        * Referenced by: '<S10>/Switch1'
                                        */
  real_T DiscreteTimeIntegrator_gainva_h;
                          /* Computed Parameter: DiscreteTimeIntegrator_gainva_h
                           * Referenced by: '<S26>/Discrete-Time Integrator'
                           */
  real_T DiscreteTimeIntegrator_IC_m;  /* Expression: 0
                                        * Referenced by: '<S26>/Discrete-Time Integrator'
                                        */
  real_T TSamp_WtEt_k;                 /* Computed Parameter: TSamp_WtEt_k
                                        * Referenced by: '<S29>/TSamp'
                                        */
  real_T Saturation_UpperSat_f;        /* Expression: 1
                                        * Referenced by: '<S20>/Saturation'
                                        */
  real_T Saturation_LowerSat_h;        /* Expression: -1
                                        * Referenced by: '<S20>/Saturation'
                                        */
  real_T Switch1_2_Threshold;          /* Expression: 2
                                        * Referenced by: '<S10>/Switch1'
                                        */
  real_T DiscreteTimeIntegrator_gainva_g;
                          /* Computed Parameter: DiscreteTimeIntegrator_gainva_g
                           * Referenced by: '<S24>/Discrete-Time Integrator'
                           */
  real_T DiscreteTimeIntegrator_IC_ek; /* Expression: 0
                                        * Referenced by: '<S24>/Discrete-Time Integrator'
                                        */
  real_T TSamp_WtEt_d;                 /* Computed Parameter: TSamp_WtEt_d
                                        * Referenced by: '<S28>/TSamp'
                                        */
  real_T Saturation1_UpperSat;         /* Expression: 1
                                        * Referenced by: '<S20>/Saturation1'
                                        */
  real_T Saturation1_LowerSat;         /* Expression: -1
                                        * Referenced by: '<S20>/Saturation1'
                                        */
  real_T DiscreteTimeIntegrator_gainva_c;
                          /* Computed Parameter: DiscreteTimeIntegrator_gainva_c
                           * Referenced by: '<S27>/Discrete-Time Integrator'
                           */
  real_T DiscreteTimeIntegrator_IC_f;  /* Expression: 0
                                        * Referenced by: '<S27>/Discrete-Time Integrator'
                                        */
  real_T TSamp_WtEt_kk;                /* Computed Parameter: TSamp_WtEt_kk
                                        * Referenced by: '<S30>/TSamp'
                                        */
  real_T Saturation2_UpperSat;         /* Expression: 0.5
                                        * Referenced by: '<S20>/Saturation2'
                                        */
  real_T Saturation2_LowerSat;         /* Expression: -0.5
                                        * Referenced by: '<S20>/Saturation2'
                                        */
  real_T DiscreteTimeIntegrator_gainva_j;
                          /* Computed Parameter: DiscreteTimeIntegrator_gainva_j
                           * Referenced by: '<S37>/Discrete-Time Integrator'
                           */
  real_T DiscreteTimeIntegrator_IC_g;  /* Expression: 0
                                        * Referenced by: '<S37>/Discrete-Time Integrator'
                                        */
  real_T TSamp_WtEt_b;                 /* Computed Parameter: TSamp_WtEt_b
                                        * Referenced by: '<S40>/TSamp'
                                        */
  real_T Switch2_Threshold;            /* Expression: 1
                                        * Referenced by: '<S10>/Switch2'
                                        */
  real_T Output_Limits1_UpperSat;      /* Expression: 2000
                                        * Referenced by: '<S10>/Output_Limits1'
                                        */
  real_T Output_Limits1_LowerSat;      /* Expression: 1000
                                        * Referenced by: '<S10>/Output_Limits1'
                                        */
  real_T Constant2_Value_b[4];         /* Expression: [1000,1000,1000,1000]
                                        * Referenced by: '<S2>/Constant2'
                                        */
  real_T g_Value[3];               /* Expression: [0,0,ModelParam_envGravityAcc]
                                    * Referenced by: '<S2>/g'
                                    */
  real_T Gain1_Gain_p;                 /* Expression: 1/1000
                                        * Referenced by: '<S2>/Gain1'
                                        */
  real_T Saturation_UpperSat_i;        /* Expression: 1
                                        * Referenced by: '<S2>/Saturation'
                                        */
  real_T Saturation_LowerSat_k;        /* Expression: 0
                                        * Referenced by: '<S2>/Saturation'
                                        */
  real_T Signal_Saturation_1_UpperSat; /* Expression: 1
                                        * Referenced by: '<S42>/Signal_Saturation_1'
                                        */
  real_T Signal_Saturation_1_LowerSat; /* Expression: 0
                                        * Referenced by: '<S42>/Signal_Saturation_1'
                                        */
  real_T Signal_Saturation_2_UpperSat; /* Expression: 1
                                        * Referenced by: '<S42>/Signal_Saturation_2'
                                        */
  real_T Signal_Saturation_2_LowerSat; /* Expression: 0
                                        * Referenced by: '<S42>/Signal_Saturation_2'
                                        */
  real_T Signal_Saturation_3_UpperSat; /* Expression: 1
                                        * Referenced by: '<S42>/Signal_Saturation_3'
                                        */
  real_T Signal_Saturation_3_LowerSat; /* Expression: 0
                                        * Referenced by: '<S42>/Signal_Saturation_3'
                                        */
  real_T Signal_Saturation_4_UpperSat; /* Expression: 1
                                        * Referenced by: '<S42>/Signal_Saturation_4'
                                        */
  real_T Signal_Saturation_4_LowerSat; /* Expression: 0
                                        * Referenced by: '<S42>/Signal_Saturation_4'
                                        */
  real_T Constant_Value_p[9];          /* Expression: eye(3)
                                        * Referenced by: '<S49>/Constant'
                                        */
  uint16_T Packnet_fdmPacketforFlightGea_l[2];
                          /* Computed Parameter: Packnet_fdmPacketforFlightGea_l
                           * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
                           */
  uint16_T Packnet_fdmPacketforFlightGea_j[2];
                          /* Computed Parameter: Packnet_fdmPacketforFlightGea_j
                           * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
                           */
  uint16_T Packnet_fdmPacketforFlightGe_ls[3];
                          /* Computed Parameter: Packnet_fdmPacketforFlightGe_ls
                           * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
                           */
  uint16_T Packnet_fdmPacketforFlightGea_n[2];
                          /* Computed Parameter: Packnet_fdmPacketforFlightGea_n
                           * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
                           */
  uint16_T Packnet_fdmPacketforFlightGea_c[2];
                          /* Computed Parameter: Packnet_fdmPacketforFlightGea_c
                           * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
                           */
  uint16_T Packnet_fdmPacketforFlightGea_i[3];
                          /* Computed Parameter: Packnet_fdmPacketforFlightGea_i
                           * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
                           */
  uint16_T Packnet_fdmPacketforFlightGea_f[3];
                          /* Computed Parameter: Packnet_fdmPacketforFlightGea_f
                           * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
                           */
  uint16_T _Value;                     /* Computed Parameter: _Value
                                        * Referenced by: '<Root>/          '
                                        */
  uint16_T u_Value;                    /* Computed Parameter: u_Value
                                        * Referenced by: '<Root>/   1'
                                        */
  uint16_T u_Value_o;                  /* Computed Parameter: u_Value_o
                                        * Referenced by: '<Root>/   2'
                                        */
  uint16_T u_Value_b;                  /* Computed Parameter: u_Value_b
                                        * Referenced by: '<Root>/   3'
                                        */
  uint8_T RateTransition_InitialCondition;
                          /* Computed Parameter: RateTransition_InitialCondition
                           * Referenced by: '<S3>/Rate Transition'
                           */
};

/* Real-time Model Data Structure */
struct tag_RTM_PosControl_Sim_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;
  X_PosControl_Sim_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[17];
  real_T odeF[4][17];
  ODE4_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

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
    boolean_T firstInitCondFlag;
    struct {
      uint8_T TID[4];
    } TaskCounters;

    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[4];
  } Timing;
};

/* Block parameters (default storage) */
extern P_PosControl_Sim_T PosControl_Sim_P;

/* Block signals (default storage) */
extern B_PosControl_Sim_T PosControl_Sim_B;

/* Continuous states (default storage) */
extern X_PosControl_Sim_T PosControl_Sim_X;

/* Block states (default storage) */
extern DW_PosControl_Sim_T PosControl_Sim_DW;

/* Model entry point functions */
extern void PosControl_Sim_initialize(void);
extern void PosControl_Sim_step(void);
extern void PosControl_Sim_terminate(void);

/* Real-time Model object */
extern RT_MODEL_PosControl_Sim_T *const PosControl_Sim_M;

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
 * '<Root>' : 'PosControl_Sim'
 * '<S1>'   : 'PosControl_Sim/Control System'
 * '<S2>'   : 'PosControl_Sim/Quadcopter Dynamics'
 * '<S3>'   : 'PosControl_Sim/To FlightGear'
 * '<S4>'   : 'PosControl_Sim/scope'
 * '<S5>'   : 'PosControl_Sim/Control System/InputConditioning'
 * '<S6>'   : 'PosControl_Sim/Control System/InputConditioning1'
 * '<S7>'   : 'PosControl_Sim/Control System/LED '
 * '<S8>'   : 'PosControl_Sim/Control System/MATLAB Function1'
 * '<S9>'   : 'PosControl_Sim/Control System/MATLAB Function2'
 * '<S10>'  : 'PosControl_Sim/Control System/Subsystem'
 * '<S11>'  : 'PosControl_Sim/Control System/InputConditioning/deadzone1'
 * '<S12>'  : 'PosControl_Sim/Control System/InputConditioning/deadzone2'
 * '<S13>'  : 'PosControl_Sim/Control System/InputConditioning/deadzone3'
 * '<S14>'  : 'PosControl_Sim/Control System/InputConditioning1/deadzone1'
 * '<S15>'  : 'PosControl_Sim/Control System/InputConditioning1/deadzone2'
 * '<S16>'  : 'PosControl_Sim/Control System/InputConditioning1/deadzone3'
 * '<S17>'  : 'PosControl_Sim/Control System/InputConditioning1/deadzone4'
 * '<S18>'  : 'PosControl_Sim/Control System/InputConditioning1/f1'
 * '<S19>'  : 'PosControl_Sim/Control System/LED /Compare To Constant3'
 * '<S20>'  : 'PosControl_Sim/Control System/Subsystem/AttitudeControl'
 * '<S21>'  : 'PosControl_Sim/Control System/Subsystem/position_control'
 * '<S22>'  : 'PosControl_Sim/Control System/Subsystem/pwm_out2'
 * '<S23>'  : 'PosControl_Sim/Control System/Subsystem/AttitudeControl/pitch_angle '
 * '<S24>'  : 'PosControl_Sim/Control System/Subsystem/AttitudeControl/pitch_rate'
 * '<S25>'  : 'PosControl_Sim/Control System/Subsystem/AttitudeControl/roll_angle '
 * '<S26>'  : 'PosControl_Sim/Control System/Subsystem/AttitudeControl/roll_rate'
 * '<S27>'  : 'PosControl_Sim/Control System/Subsystem/AttitudeControl/yaw_rate'
 * '<S28>'  : 'PosControl_Sim/Control System/Subsystem/AttitudeControl/pitch_rate/Discrete Derivative'
 * '<S29>'  : 'PosControl_Sim/Control System/Subsystem/AttitudeControl/roll_rate/Discrete Derivative'
 * '<S30>'  : 'PosControl_Sim/Control System/Subsystem/AttitudeControl/yaw_rate/Discrete Derivative'
 * '<S31>'  : 'PosControl_Sim/Control System/Subsystem/position_control/MATLAB Function'
 * '<S32>'  : 'PosControl_Sim/Control System/Subsystem/position_control/px'
 * '<S33>'  : 'PosControl_Sim/Control System/Subsystem/position_control/py'
 * '<S34>'  : 'PosControl_Sim/Control System/Subsystem/position_control/pz'
 * '<S35>'  : 'PosControl_Sim/Control System/Subsystem/position_control/vx '
 * '<S36>'  : 'PosControl_Sim/Control System/Subsystem/position_control/vy '
 * '<S37>'  : 'PosControl_Sim/Control System/Subsystem/position_control/vz '
 * '<S38>'  : 'PosControl_Sim/Control System/Subsystem/position_control/vx /Discrete Derivative'
 * '<S39>'  : 'PosControl_Sim/Control System/Subsystem/position_control/vy /Discrete Derivative'
 * '<S40>'  : 'PosControl_Sim/Control System/Subsystem/position_control/vz /Discrete Derivative'
 * '<S41>'  : 'PosControl_Sim/Quadcopter Dynamics/Control Effectiveness Model'
 * '<S42>'  : 'PosControl_Sim/Quadcopter Dynamics/Propulsor Model'
 * '<S43>'  : 'PosControl_Sim/Quadcopter Dynamics/Rigid Model'
 * '<S44>'  : 'PosControl_Sim/Quadcopter Dynamics/Control Effectiveness Model/Propeller Model'
 * '<S45>'  : 'PosControl_Sim/Quadcopter Dynamics/Propulsor Model/Throttle To rads Gain _1'
 * '<S46>'  : 'PosControl_Sim/Quadcopter Dynamics/Propulsor Model/Throttle To rads Gain _2'
 * '<S47>'  : 'PosControl_Sim/Quadcopter Dynamics/Propulsor Model/Throttle To rads Gain _3'
 * '<S48>'  : 'PosControl_Sim/Quadcopter Dynamics/Propulsor Model/Throttle To rads Gain _4'
 * '<S49>'  : 'PosControl_Sim/Quadcopter Dynamics/Rigid Model/Rigid-Body Dynamic Model'
 * '<S50>'  : 'PosControl_Sim/Quadcopter Dynamics/Rigid Model/Rigid-Body Kinematic Model '
 * '<S51>'  : 'PosControl_Sim/Quadcopter Dynamics/Rigid Model/Rigid-Body Dynamic Model/J*w'
 * '<S52>'  : 'PosControl_Sim/Quadcopter Dynamics/Rigid Model/Rigid-Body Dynamic Model/wx(Jw)'
 * '<S53>'  : 'PosControl_Sim/Quadcopter Dynamics/Rigid Model/Rigid-Body Dynamic Model/wxVb'
 * '<S54>'  : 'PosControl_Sim/Quadcopter Dynamics/Rigid Model/Rigid-Body Dynamic Model/wx(Jw)/Subsystem'
 * '<S55>'  : 'PosControl_Sim/Quadcopter Dynamics/Rigid Model/Rigid-Body Dynamic Model/wx(Jw)/Subsystem1'
 * '<S56>'  : 'PosControl_Sim/Quadcopter Dynamics/Rigid Model/Rigid-Body Dynamic Model/wxVb/Subsystem'
 * '<S57>'  : 'PosControl_Sim/Quadcopter Dynamics/Rigid Model/Rigid-Body Dynamic Model/wxVb/Subsystem1'
 * '<S58>'  : 'PosControl_Sim/Quadcopter Dynamics/Rigid Model/Rigid-Body Kinematic Model /Euler angle to quaternion'
 * '<S59>'  : 'PosControl_Sim/Quadcopter Dynamics/Rigid Model/Rigid-Body Kinematic Model /Quaternion to Euler'
 * '<S60>'  : 'PosControl_Sim/Quadcopter Dynamics/Rigid Model/Rigid-Body Kinematic Model /Quaternion to rotation matrix'
 * '<S61>'  : 'PosControl_Sim/Quadcopter Dynamics/Rigid Model/Rigid-Body Kinematic Model /Quaternions model'
 * '<S62>'  : 'PosControl_Sim/To FlightGear/Send net_fdm Packet to FlightGear'
 * '<S63>'  : 'PosControl_Sim/To FlightGear/Subsystem'
 * '<S64>'  : 'PosControl_Sim/scope/XY'
 */
#endif                                 /* RTW_HEADER_PosControl_Sim_h_ */
