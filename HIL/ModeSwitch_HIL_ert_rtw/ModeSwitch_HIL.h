/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ModeSwitch_HIL.h
 *
 * Code generated for Simulink model 'ModeSwitch_HIL'.
 *
 * Model version                  : 1.0
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Tue Apr 20 22:13:00 2021
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_ModeSwitch_HIL_h_
#define RTW_HEADER_ModeSwitch_HIL_h_
#include <stddef.h>
#include <math.h>
#include <string.h>
#ifndef ModeSwitch_HIL_COMMON_INCLUDES_
#define ModeSwitch_HIL_COMMON_INCLUDES_
#include <drivers/drv_rc_input.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <px4_defines.h>
#include <uORB/uORB.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>
#include <drivers/drv_board_led.h>
#include <uORB/topics/led_control.h>
#include <stdlib.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include "rtwtypes.h"
#endif                                 /* ModeSwitch_HIL_COMMON_INCLUDES_ */

#include "ModeSwitch_HIL_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

typedef struct pollfd pollfd_t;
typedef struct led_control_s led_control_s_t;
typedef struct actuator_outputs_s actuator_outputs_s_t;
typedef struct pollfd pollfd_t;
typedef struct vehicle_local_position_s vehicle_local_position_SL;

/* Block signals for system '<S5>/deadzone1' */
typedef struct {
  real_T y;                            /* '<S5>/deadzone1' */
} B_deadzone1_ModeSwitch_HIL_T;

/* Block signals (default storage) */
typedef struct {
  vehicle_local_position_SL uORBReadFunctionCallTrigger;
                                    /* '<S3>/uORB Read Function-Call Trigger' */
  real_T Output_Limits1[4];            /* '<S11>/Output_Limits1' */
  real_T Sum1;                         /* '<S21>/Sum1' */
  real_T max_vx;                       /* '<S5>/max_vx' */
  real_T max_vy;                       /* '<S5>/max_vy' */
  real_T max_vz;                       /* '<S5>/max_vz' */
  real_T Gain;                         /* '<S6>/Gain' */
  real_T TSamp;                        /* '<S39>/TSamp' */
  real_T Sum_i;                        /* '<S36>/Sum' */
  real_T DiscreteTimeIntegrator_n;     /* '<S27>/Discrete-Time Integrator' */
  real_T TSamp_b;                      /* '<S40>/TSamp' */
  real_T Sum_d;                        /* '<S37>/Sum' */
  real_T TSamp_k;                      /* '<S30>/TSamp' */
  real_T Saturation;                   /* '<S21>/Saturation' */
  real_T Saturation1;                  /* '<S21>/Saturation1' */
  real_T TSamp_i;                      /* '<S31>/TSamp' */
  real_T Saturation2;                  /* '<S21>/Saturation2' */
  real_T TSamp_d;                      /* '<S41>/TSamp' */
  real_T u0;
  real_T u0_tmp;
  real32_T sensor_combined_o1;         /* '<Root>/sensor_combined' */
  real32_T sensor_combined_o2;         /* '<Root>/sensor_combined' */
  real32_T sensor_combined_o3;         /* '<Root>/sensor_combined' */
  real32_T vehicle_attitude[4];        /* '<Root>/vehicle_attitude' */
  real32_T aSq;
  real32_T bSq;
  real32_T cSq;
  RGBLED_MODE_ENUM Switch;             /* '<S2>/Switch' */
  RGBLED_COLOR_ENUM Switch1;           /* '<S2>/Switch1' */
  uint16_T input_rc_o1;                /* '<Root>/input_rc' */
  uint16_T input_rc_o2;                /* '<Root>/input_rc' */
  uint16_T input_rc_o3;                /* '<Root>/input_rc' */
  uint16_T input_rc_o4;                /* '<Root>/input_rc' */
  uint16_T input_rc_o5;                /* '<Root>/input_rc' */
  uint16_T input_rc_o6;                /* '<Root>/input_rc' */
  B_deadzone1_ModeSwitch_HIL_T sf_deadzone4;/* '<S6>/deadzone4' */
  B_deadzone1_ModeSwitch_HIL_T sf_deadzone3_o;/* '<S6>/deadzone3' */
  B_deadzone1_ModeSwitch_HIL_T sf_deadzone2_g;/* '<S6>/deadzone2' */
  B_deadzone1_ModeSwitch_HIL_T sf_deadzone1_l;/* '<S6>/deadzone1' */
  B_deadzone1_ModeSwitch_HIL_T sf_deadzone3;/* '<S5>/deadzone3' */
  B_deadzone1_ModeSwitch_HIL_T sf_deadzone2;/* '<S5>/deadzone2' */
  B_deadzone1_ModeSwitch_HIL_T sf_deadzone1;/* '<S5>/deadzone1' */
} B_ModeSwitch_HIL_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  actuator_outputs_s_t uORBWrite_uorb_msg;/* '<Root>/uORB Write' */
  led_control_s_t RGB_LED_sl_led_control_s;/* '<Root>/RGB_LED' */
  pollfd_t input_rc_input_rc_fd;       /* '<Root>/input_rc' */
  pollfd_t uORBReadFunctionCallTrigger_uOR;
                                    /* '<S3>/uORB Read Function-Call Trigger' */
  pollfd_t sensor_combined_sensor_fd;  /* '<Root>/sensor_combined' */
  pollfd_t vehicle_attitude_vehicle_attitu;/* '<Root>/vehicle_attitude' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S36>/Discrete-Time Integrator' */
  real_T UD_DSTATE;                    /* '<S39>/UD' */
  real_T DiscreteTimeIntegrator_DSTATE_m;/* '<S37>/Discrete-Time Integrator' */
  real_T UD_DSTATE_g;                  /* '<S40>/UD' */
  real_T DiscreteTimeIntegrator_DSTATE_b;/* '<S27>/Discrete-Time Integrator' */
  real_T UD_DSTATE_i;                  /* '<S30>/UD' */
  real_T DiscreteTimeIntegrator_DSTATE_g;/* '<S25>/Discrete-Time Integrator' */
  real_T UD_DSTATE_d;                  /* '<S29>/UD' */
  real_T DiscreteTimeIntegrator_DSTATE_c;/* '<S28>/Discrete-Time Integrator' */
  real_T UD_DSTATE_e;                  /* '<S31>/UD' */
  real_T DiscreteTimeIntegrator_DSTAT_c5;/* '<S38>/Discrete-Time Integrator' */
  real_T UD_DSTATE_j;                  /* '<S41>/UD' */
  real_T x1;                           /* '<S1>/MATLAB Function2' */
  real_T y1;                           /* '<S1>/MATLAB Function2' */
  real_T z1;                           /* '<S1>/MATLAB Function2' */
  real_T hold_x_flag;                  /* '<S1>/MATLAB Function2' */
  real_T hold_y_flag;                  /* '<S1>/MATLAB Function2' */
  real_T hold_z_flag;                  /* '<S1>/MATLAB Function2' */
  real_T waypoint_1_flag;              /* '<S1>/MATLAB Function2' */
  real_T waypoint_2_flag;              /* '<S1>/MATLAB Function2' */
  real_T waypoint_3_flag;              /* '<S1>/MATLAB Function2' */
  real_T waypoint_4_flag;              /* '<S1>/MATLAB Function2' */
  orb_advert_t RGB_LED_orb_advert_t;   /* '<Root>/RGB_LED' */
  orb_advert_t uORBWrite_uorb_advert;  /* '<Root>/uORB Write' */
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S36>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_h;/* '<S37>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_g;/* '<S27>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_hk;/* '<S25>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_gt;/* '<S28>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_p;/* '<S38>/Discrete-Time Integrator' */
  boolean_T z1_not_empty;              /* '<S1>/MATLAB Function2' */
} DW_ModeSwitch_HIL_T;

/* Real-time Model Data Structure */
struct tag_RTM_ModeSwitch_HIL_T {
  const char_T *errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint16_T TID[2];
    } TaskCounters;
  } Timing;
};

/* Block signals (default storage) */
extern B_ModeSwitch_HIL_T ModeSwitch_HIL_B;

/* Block states (default storage) */
extern DW_ModeSwitch_HIL_T ModeSwitch_HIL_DW;

/* Model entry point functions */
extern void ModeSwitch_HIL_initialize(void);
extern void ModeSwitch_HIL_step(void);
extern void ModeSwitch_HIL_terminate(void);

/* Real-time Model object */
extern RT_MODEL_ModeSwitch_HIL_T *const ModeSwitch_HIL_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S29>/Data Type Duplicate' : Unused code path elimination
 * Block '<S30>/Data Type Duplicate' : Unused code path elimination
 * Block '<S31>/Data Type Duplicate' : Unused code path elimination
 * Block '<S39>/Data Type Duplicate' : Unused code path elimination
 * Block '<S40>/Data Type Duplicate' : Unused code path elimination
 * Block '<S41>/Data Type Duplicate' : Unused code path elimination
 * Block '<S33>/Gain1' : Eliminated nontunable gain of 1
 * Block '<S34>/Gain1' : Eliminated nontunable gain of 1
 * Block '<Root>/Data Type Conversion2' : Eliminate redundant data type conversion
 */

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
 * '<Root>' : 'ModeSwitch_HIL'
 * '<S1>'   : 'ModeSwitch_HIL/Control System'
 * '<S2>'   : 'ModeSwitch_HIL/RGB_Mode_Subsystem'
 * '<S3>'   : 'ModeSwitch_HIL/localPosition '
 * '<S4>'   : 'ModeSwitch_HIL/quat2eul'
 * '<S5>'   : 'ModeSwitch_HIL/Control System/InputConditioning'
 * '<S6>'   : 'ModeSwitch_HIL/Control System/InputConditioning1'
 * '<S7>'   : 'ModeSwitch_HIL/Control System/LED '
 * '<S8>'   : 'ModeSwitch_HIL/Control System/MATLAB Function1'
 * '<S9>'   : 'ModeSwitch_HIL/Control System/MATLAB Function2'
 * '<S10>'  : 'ModeSwitch_HIL/Control System/MATLAB Function3'
 * '<S11>'  : 'ModeSwitch_HIL/Control System/Subsystem'
 * '<S12>'  : 'ModeSwitch_HIL/Control System/InputConditioning/deadzone1'
 * '<S13>'  : 'ModeSwitch_HIL/Control System/InputConditioning/deadzone2'
 * '<S14>'  : 'ModeSwitch_HIL/Control System/InputConditioning/deadzone3'
 * '<S15>'  : 'ModeSwitch_HIL/Control System/InputConditioning1/deadzone1'
 * '<S16>'  : 'ModeSwitch_HIL/Control System/InputConditioning1/deadzone2'
 * '<S17>'  : 'ModeSwitch_HIL/Control System/InputConditioning1/deadzone3'
 * '<S18>'  : 'ModeSwitch_HIL/Control System/InputConditioning1/deadzone4'
 * '<S19>'  : 'ModeSwitch_HIL/Control System/InputConditioning1/f1'
 * '<S20>'  : 'ModeSwitch_HIL/Control System/LED /Compare To Constant3'
 * '<S21>'  : 'ModeSwitch_HIL/Control System/Subsystem/AttitudeControl'
 * '<S22>'  : 'ModeSwitch_HIL/Control System/Subsystem/position_control'
 * '<S23>'  : 'ModeSwitch_HIL/Control System/Subsystem/pwm_out2'
 * '<S24>'  : 'ModeSwitch_HIL/Control System/Subsystem/AttitudeControl/pitch_angle '
 * '<S25>'  : 'ModeSwitch_HIL/Control System/Subsystem/AttitudeControl/pitch_rate'
 * '<S26>'  : 'ModeSwitch_HIL/Control System/Subsystem/AttitudeControl/roll_angle '
 * '<S27>'  : 'ModeSwitch_HIL/Control System/Subsystem/AttitudeControl/roll_rate'
 * '<S28>'  : 'ModeSwitch_HIL/Control System/Subsystem/AttitudeControl/yaw_rate'
 * '<S29>'  : 'ModeSwitch_HIL/Control System/Subsystem/AttitudeControl/pitch_rate/Discrete Derivative'
 * '<S30>'  : 'ModeSwitch_HIL/Control System/Subsystem/AttitudeControl/roll_rate/Discrete Derivative'
 * '<S31>'  : 'ModeSwitch_HIL/Control System/Subsystem/AttitudeControl/yaw_rate/Discrete Derivative'
 * '<S32>'  : 'ModeSwitch_HIL/Control System/Subsystem/position_control/MATLAB Function'
 * '<S33>'  : 'ModeSwitch_HIL/Control System/Subsystem/position_control/px'
 * '<S34>'  : 'ModeSwitch_HIL/Control System/Subsystem/position_control/py'
 * '<S35>'  : 'ModeSwitch_HIL/Control System/Subsystem/position_control/pz'
 * '<S36>'  : 'ModeSwitch_HIL/Control System/Subsystem/position_control/vx '
 * '<S37>'  : 'ModeSwitch_HIL/Control System/Subsystem/position_control/vy '
 * '<S38>'  : 'ModeSwitch_HIL/Control System/Subsystem/position_control/vz '
 * '<S39>'  : 'ModeSwitch_HIL/Control System/Subsystem/position_control/vx /Discrete Derivative'
 * '<S40>'  : 'ModeSwitch_HIL/Control System/Subsystem/position_control/vy /Discrete Derivative'
 * '<S41>'  : 'ModeSwitch_HIL/Control System/Subsystem/position_control/vz /Discrete Derivative'
 */
#endif                                 /* RTW_HEADER_ModeSwitch_HIL_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
