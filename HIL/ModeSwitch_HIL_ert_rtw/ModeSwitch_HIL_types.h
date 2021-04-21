/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ModeSwitch_HIL_types.h
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

#ifndef RTW_HEADER_ModeSwitch_HIL_types_h_
#define RTW_HEADER_ModeSwitch_HIL_types_h_
#include "rtwtypes.h"
#include <uORB/topics/vehicle_local_position.h>

/* Model Code Variants */
#ifndef DEFINED_TYPEDEF_FOR_RGBLED_MODE_ENUM_
#define DEFINED_TYPEDEF_FOR_RGBLED_MODE_ENUM_

typedef enum {
  SL_MODE_OFF = 0,                     /* Default value */
  SL_MODE_ON,
  SL_MODE_DISABLED,
  SL_MODE_BLINK_SLOW,
  SL_MODE_BLINK_NORMAL,
  SL_MODE_BLINK_FAST,
  SL_MODE_BREATHE
} RGBLED_MODE_ENUM;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RGBLED_COLOR_ENUM_
#define DEFINED_TYPEDEF_FOR_RGBLED_COLOR_ENUM_

typedef enum {
  SL_COLOR_OFF = 0,                    /* Default value */
  SL_COLOR_RED,
  SL_COLOR_GREEN,
  SL_COLOR_BLUE,
  SL_COLOR_YELLOW,
  SL_COLOR_PURPLE,
  SL_COLOR_AMBER,
  SL_COLOR_CYAN,
  SL_COLOR_WHITE
} RGBLED_COLOR_ENUM;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_ModeSwitch_HIL_T RT_MODEL_ModeSwitch_HIL_T;

#endif                                 /* RTW_HEADER_ModeSwitch_HIL_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
