/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ModeSwitch_HIL.c
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

#include "ModeSwitch_HIL.h"
#include "ModeSwitch_HIL_private.h"

/* Block signals (default storage) */
B_ModeSwitch_HIL_T ModeSwitch_HIL_B;

/* Block states (default storage) */
DW_ModeSwitch_HIL_T ModeSwitch_HIL_DW;

/* Real-time model */
static RT_MODEL_ModeSwitch_HIL_T ModeSwitch_HIL_M_;
RT_MODEL_ModeSwitch_HIL_T *const ModeSwitch_HIL_M = &ModeSwitch_HIL_M_;
static void rate_scheduler(void);

/*
 *   This function updates active task flag for each subrate.
 * The function is called at model base rate, hence the
 * generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (ModeSwitch_HIL_M->Timing.TaskCounters.TID[1])++;
  if ((ModeSwitch_HIL_M->Timing.TaskCounters.TID[1]) > 249) {/* Sample time: [1.0s, 0.0s] */
    ModeSwitch_HIL_M->Timing.TaskCounters.TID[1] = 0;
  }
}

/*
 * Output and update for atomic system:
 *    '<S5>/deadzone1'
 *    '<S5>/deadzone2'
 *    '<S5>/deadzone3'
 *    '<S6>/deadzone1'
 *    '<S6>/deadzone2'
 *    '<S6>/deadzone3'
 *    '<S6>/deadzone4'
 */
void ModeSwitch_HIL_deadzone1(real_T rtu_u, B_deadzone1_ModeSwitch_HIL_T *localB)
{
  real_T u;
  u = rtu_u;

  /* Function description: */
  /*   Control the range of the remote control input from RCMin to RCMax, and */
  /*   set a dead zone with a dead zone size of ±deadZone. The output y is */
  /*   normalized to 0~1.You can get the RC calibration parameters in QGC. */
  /* MATLAB Function 'Control System/InputConditioning/deadzone1': '<S12>:1' */
  /* '<S12>:1:6' RCMin = 1100; */
  /* '<S12>:1:7' RCMax = 1900; */
  /* '<S12>:1:8' RCMid = (RCMin + RCMax)/2; */
  /* '<S12>:1:9' deadZoneRate = 0.05; */
  /* '<S12>:1:10' deadZone = deadZoneRate*(RCMax - RCMin); */
  /* '<S12>:1:11' k = 1/(RCMax - RCMid - deadZone); */
  /*  saturation */
  /* '<S12>:1:13' if(u < RCMin) */
  if (rtu_u < 1100.0) {
    /* '<S12>:1:14' u = RCMin; */
    u = 1100.0;
  } else if (rtu_u > 1900.0) {
    /* '<S12>:1:15' elseif(u > RCMax) */
    /* '<S12>:1:16' u = RCMax; */
    u = 1900.0;
  }

  /*  dead zone and normalize */
  /* '<S12>:1:19' if(u > RCMid + deadZone) */
  if (u > 1540.0) {
    /* '<S12>:1:20' y = (u-RCMid - deadZone)*k; */
    localB->y = ((u - 1500.0) - 40.0) * 0.0027777777777777779;
  } else if (u < 1460.0) {
    /* '<S12>:1:21' elseif(u < RCMid - deadZone) */
    /* '<S12>:1:22' y = (u - RCMid + deadZone)*k; */
    localB->y = ((u - 1500.0) + 40.0) * 0.0027777777777777779;
  } else {
    /* '<S12>:1:23' else */
    /* '<S12>:1:24' y = 0; */
    localB->y = 0.0;
  }
}

real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
  int32_T u0_0;
  int32_T u1_0;
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0F) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = (real32_T)atan2((real32_T)u0_0, (real32_T)u1_0);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (real32_T)atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void ModeSwitch_HIL_step(void)
{
  int32_T hold_x;
  int32_T hold_y;
  int32_T hold_z;
  int32_T rtb_reset_i_idx_3;
  real32_T dSq;
  real32_T dcm02;
  real32_T dcm02_tmp;
  real32_T dcm02_tmp_0;
  real32_T dcm12;
  real32_T dcm12_tmp;
  real32_T rtb_phi_d;
  int8_T rtb_control_mode;

  /* S-Function (sfun_px4_input_rc): '<Root>/input_rc'
   *
   * Block description for '<Root>/input_rc':
   *  RC Input Block
   *
   *  This block provides user input control to the model.
   *  It uses the input_rc uORB topic.
   *
   *  The user has the ability to choose which channels are available as outputs from this block and also some optional outputs. These include
   *  Channels 1 through 18
   *  double data type indicating the PWM value from the controller
   *  measured pulse widths for each of the supported channels
   *  Channel Count
   *  uint32 data type of the number of channels which are detector by the PX4
   *  RC Failsafe
   *  boolean data types indicating that the RC Tx is sending the FailSafe signal (if equipped and properly setup)
   *  explicit failsafe flag: true on TX failure or TX out of range , false otherwise.
   *  Only the true state is reliable, as there are some (PPM) receivers on the market going into failsafe without telling us explicitly.
   *  RC Input Source
   *  Enumeration data type indicating which source the RC input is from.
   *  Valid values are found in the ENUM file: RC_INPUT_SOURCE_ENUM.m
   *            Enumeration members for class 'RC_INPUT_SOURCE_ENUM':
   *            RCINPUT_SOURCE_UNKNOWN         (0)
   *            RCINPUT_SOURCE_PX4FMU_PPM      (1)
   *            RCINPUT_SOURCE_PX4IO_PPM       (2)
   *            RCINPUT_SOURCE_PX4IO_SPEKTRUM  (3)
   *            RCINPUT_SOURCE_PX4IO_SBUS      (4)
   *            RCINPUT_SOURCE_PX4IO_ST24      (5)
   *            RCINPUT_SOURCE_MAVLINK         (6)
   *            RCINPUT_SOURCE_QURT            (7)
   *            RCINPUT_SOURCE_PX4FMU_SPEKTRUM (8)
   *            RCINPUT_SOURCE_PX4FMU_SBUS     (9)
   *            RCINPUT_SOURCE_PX4FMU_ST24     (10)
   *            RCINPUT_SOURCE_PX4FMU_SUMD     (11)
   *            RCINPUT_SOURCE_PX4FMU_DSM      (12)
   *            RCINPUT_SOURCE_PX4IO_SUMD      (13)
   *
   *  RSSI - Receive signal strength index
   *  receive signal strength indicator (RSSI): < 0: Undefined, 0: no signal, 255: full reception
   *  RC Lost Connection
   *  boolean data type indicating RC receiver connection status
   *  True, if no frame has arrived in the expected time, false otherwise.
   *  True usally means that the receiver has been disconnected, but can also indicate a radio link loss on "stupid" systems.
   *  Will remain false, if a RX with failsafe option continues to transmit frames after a link loss.
   *
   *  Sample Model: px4demo_input_rc.slx
   */
  {
    bool updated;
    orb_check(ModeSwitch_HIL_DW.input_rc_input_rc_fd.fd, &updated);
    if (updated) {
      struct rc_input_values pwm_inputs;

      /* copy input_rc raw data into local buffer (uint16)*/
      orb_copy(ORB_ID(input_rc), ModeSwitch_HIL_DW.input_rc_input_rc_fd.fd,
               &pwm_inputs);
      ModeSwitch_HIL_B.input_rc_o1 = pwm_inputs.values[0];
      ModeSwitch_HIL_B.input_rc_o2 = pwm_inputs.values[1];
      ModeSwitch_HIL_B.input_rc_o3 = pwm_inputs.values[2];
      ModeSwitch_HIL_B.input_rc_o4 = pwm_inputs.values[3];
      ModeSwitch_HIL_B.input_rc_o5 = pwm_inputs.values[4];
      ModeSwitch_HIL_B.input_rc_o6 = pwm_inputs.values[5];
    }
  }

  /* RateTransition: '<Root>/Rate Transition' */
  if (ModeSwitch_HIL_M->Timing.TaskCounters.TID[1] == 0) {
    /* Switch: '<S2>/Switch' incorporates:
     *  Constant: '<S20>/Constant'
     *  DataTypeConversion: '<S1>/Conversion6'
     *  RelationalOperator: '<S20>/Compare'
     *  Switch: '<S2>/Switch1'
     */
    if (ModeSwitch_HIL_B.input_rc_o6 >= 1500) {
      /* Switch: '<S2>/Switch' incorporates:
       *  Constant: '<S2>/Constant'
       */
      ModeSwitch_HIL_B.Switch = SL_MODE_BLINK_FAST;

      /* Switch: '<S2>/Switch1' incorporates:
       *  Constant: '<S2>/Constant3'
       */
      ModeSwitch_HIL_B.Switch1 = SL_COLOR_RED;
    } else {
      /* Switch: '<S2>/Switch' incorporates:
       *  Constant: '<S2>/Constant1'
       */
      ModeSwitch_HIL_B.Switch = SL_MODE_BREATHE;

      /* Switch: '<S2>/Switch1' incorporates:
       *  Constant: '<S2>/Constant2'
       */
      ModeSwitch_HIL_B.Switch1 = SL_COLOR_GREEN;
    }

    /* End of Switch: '<S2>/Switch' */

    /* S-Function (sfun_px4_rgbled): '<Root>/RGB_LED' */
    ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s.mode = ModeSwitch_HIL_B.Switch;
    ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s.color = ModeSwitch_HIL_B.Switch1;
    ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s.num_blinks = 0;
    ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s.priority = 0;
    ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s.timestamp = hrt_absolute_time();
    orb_publish(ORB_ID(led_control), ModeSwitch_HIL_DW.RGB_LED_orb_advert_t ,
                &ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s);
  }

  /* End of RateTransition: '<Root>/Rate Transition' */

  /* S-Function (sfun_px4_uorb_write): '<Root>/uORB Write' incorporates:
   *  Constant: '<Root>/Constant'
   */
  {
    //struct actuator_outputs_s InputStruct;
    //memset( &InputStruct, 0, sizeof(InputStruct));

    /* assign input parameters to struct */
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[0] =
      ModeSwitch_HIL_B.Output_Limits1[0];
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[1] =
      ModeSwitch_HIL_B.Output_Limits1[1];
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[2] =
      ModeSwitch_HIL_B.Output_Limits1[2];
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[3] =
      ModeSwitch_HIL_B.Output_Limits1[3];
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[4] = 0.0;
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[5] = 0.0;
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[6] = 0.0;
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[7] = 0.0;
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[8] = 0.0;
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[9] = 0.0;
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[10] = 0.0;
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[11] = 0.0;
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[12] = 0.0;
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[13] = 0.0;
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[14] = 0.0;
    ModeSwitch_HIL_DW.uORBWrite_uorb_msg.output[15] = 0.0;

    /* Publish data for subscribers */
    orb_publish(ORB_ID(actuator_outputs),
                ModeSwitch_HIL_DW.uORBWrite_uorb_advert,
                &ModeSwitch_HIL_DW.uORBWrite_uorb_msg);
  }

  /* S-Function (sfun_px4_uorb_read_topic): '<S3>/uORB Read Function-Call Trigger' */
  {
    //interval val: 1
    bool updated;
    orb_check(ModeSwitch_HIL_DW.uORBReadFunctionCallTrigger_uOR.fd, &updated);
    if (updated) {
      /* obtained uorb data */
      /* copy sensors raw data into local buffer */
      orb_copy(ORB_ID(vehicle_local_position),
               ModeSwitch_HIL_DW.uORBReadFunctionCallTrigger_uOR.fd,
               &ModeSwitch_HIL_B.uORBReadFunctionCallTrigger);
    }
  }

  /* S-Function (sfun_px4_sensor_combined): '<Root>/sensor_combined'
   *
   * Block description for '<Root>/sensor_combined':
   *  Sensor Combined Block
   *
   *  This block enables access to the various sensors available on the px4fmu-v2 hardware.
   *  The user can use these signals in the Simulink control model.
   *  The sample time needs to be provided in the mask dialog.
   *  Optional output ports can also be selected.
   *  Refer to the sample model: px4demo_attitude_control.slx
   *
   *  Signal definitions:
   *  Magnetometer (x,y,z) - single values – Magnetic field in NED body frame, in Gauss
   *  Accelerometer (x,y,z) - single values – Acceleration in NED body frame, in m/s^2
   *  Gyroscope (p,q,r) - single values – Angular velocity in radians per second
   *  Barometer (Altitude) - single value – Barometric pressure, already temperature compensated (millibars)
   *  RunTime (timestamp) - double value – Timestamp in microseconds since boot, from gyro
   *
   *  The sensor_combined block needs to have the px4io service running on the PX4 hardware in order to get valid signal values.

   */
  {
    bool updated;
    orb_check(ModeSwitch_HIL_DW.sensor_combined_sensor_fd.fd, &updated);
    if (updated) {
      /* obtained data sensor combined */
      struct sensor_combined_s raw;

      /* copy sensors raw data into local buffer */
      orb_copy(ORB_ID(sensor_combined),
               ModeSwitch_HIL_DW.sensor_combined_sensor_fd.fd, &raw);

      /* read out the gyro X,Y,Z */
      ModeSwitch_HIL_B.sensor_combined_o1 = (float)raw.gyro_rad[0];
      ModeSwitch_HIL_B.sensor_combined_o2 = (float)raw.gyro_rad[1];
      ModeSwitch_HIL_B.sensor_combined_o3 = (float)raw.gyro_rad[2];
    }
  }

  /* S-Function (sfun_px4_vehicle_attitude): '<Root>/vehicle_attitude'
   *
   * Block description for '<Root>/vehicle_attitude':
   *  This block gives access to the running service that calculates the vehicle’s attitude.  A uORB topic (vehicle_attitude (attitude measurements)) publisher MUST be running in order for this block to provide valid signal values.  The available ones as of v1.3 are:
   *  attitude_estimator_ekf – EKF-Extended Kalman Filter for attitude estimation
   *  attitude_estimator_so3 – SO(3)-attitude estimation by using accelerometer, gyroscopes and magnetometer
   *  One of these MUST be running on the px4fmu in order for this block to return valid values.  Refer to the sample model: px4demo_attitude_control.slx. Attitude in NED (North-East-Down) body frame in SI units.
   *  Signal definitions:
   *  Roll – single value, Roll angle (rad, Tait-Bryan, NED)
   *  Pitch – single value, Pitch angle (rad, Tait-Bryan, NED)
   *  Yaw – single value, Yaw angle (rad, Tait-Bryan, NED)
   *  Quaternion (NED) – single(4) values (optional based on the uORB publisher)
   *
   */
  {
    bool updated;
    orb_check(ModeSwitch_HIL_DW.vehicle_attitude_vehicle_attitu.fd, &updated);
    if (updated) {
      struct vehicle_attitude_s raw;
      orb_copy(ORB_ID(vehicle_attitude),
               ModeSwitch_HIL_DW.vehicle_attitude_vehicle_attitu.fd, &raw);

      /* read out the Quaternion values */
      ModeSwitch_HIL_B.vehicle_attitude[0] = raw.q[0];
      ModeSwitch_HIL_B.vehicle_attitude[1] = raw.q[1];
      ModeSwitch_HIL_B.vehicle_attitude[2] = raw.q[2];
      ModeSwitch_HIL_B.vehicle_attitude[3] = raw.q[3];
    }
  }

  /* MATLAB Function: '<Root>/quat2eul' */
  /*  Conversion from Quaternion to Euler angles based on the PX4 Firmware */
  /*  v1.6.5 release. */
  /*   */
  /*  Quaternion -> DCM -> Euler Angles */
  /*  INPUT: */
  /*    q: Quaternion vector of the format: (a + bi + cj + dk) */
  /*  OUTPUT: */
  /*    [phi,theta,psi]: Euler angles in radians. */
  /* MATLAB Function 'quat2eul': '<S4>:1' */
  /* '<S4>:1:12' a = q(1); */
  /* '<S4>:1:13' b = q(2); */
  /* '<S4>:1:14' c = q(3); */
  /* '<S4>:1:15' d = q(4); */
  /* '<S4>:1:17' aSq = a*a; */
  ModeSwitch_HIL_B.aSq = ModeSwitch_HIL_B.vehicle_attitude[0] *
    ModeSwitch_HIL_B.vehicle_attitude[0];

  /* '<S4>:1:18' bSq = b*b; */
  ModeSwitch_HIL_B.bSq = ModeSwitch_HIL_B.vehicle_attitude[1] *
    ModeSwitch_HIL_B.vehicle_attitude[1];

  /* '<S4>:1:19' cSq = c*c; */
  ModeSwitch_HIL_B.cSq = ModeSwitch_HIL_B.vehicle_attitude[2] *
    ModeSwitch_HIL_B.vehicle_attitude[2];

  /* '<S4>:1:20' dSq = d*d; */
  dSq = ModeSwitch_HIL_B.vehicle_attitude[3] *
    ModeSwitch_HIL_B.vehicle_attitude[3];

  /* '<S4>:1:22' d_pi2 = pi/2; */
  /*  quaternion_to_dcm */
  /* '<S4>:1:25' dcm00 = aSq + bSq - cSq - dSq; */
  /*  dcm01 = 2*(b*c - a*d); */
  /* '<S4>:1:27' dcm02 = 2*(a*c + b*d); */
  dcm02_tmp = ModeSwitch_HIL_B.vehicle_attitude[1] *
    ModeSwitch_HIL_B.vehicle_attitude[3];
  dcm02_tmp_0 = ModeSwitch_HIL_B.vehicle_attitude[0] *
    ModeSwitch_HIL_B.vehicle_attitude[2];
  dcm02 = (dcm02_tmp_0 + dcm02_tmp) * 2.0F;

  /* '<S4>:1:28' dcm10 = 2*(b*c + a*d); */
  /*  dcm11 = aSq - bSq + cSq - dSq; */
  /* '<S4>:1:30' dcm12 = 2*(c*d - a*b); */
  rtb_phi_d = ModeSwitch_HIL_B.vehicle_attitude[0] *
    ModeSwitch_HIL_B.vehicle_attitude[1];
  dcm12_tmp = ModeSwitch_HIL_B.vehicle_attitude[2] *
    ModeSwitch_HIL_B.vehicle_attitude[3];
  dcm12 = (dcm12_tmp - rtb_phi_d) * 2.0F;

  /* '<S4>:1:31' dcm20 = 2*(b*d - a*c); */
  /* '<S4>:1:32' dcm21 = 2*(a*b + c*d); */
  /* '<S4>:1:33' dcm22 = aSq - bSq - cSq + dSq; */
  /*  dcm_to_euler */
  /* '<S4>:1:36' theta = asin(-dcm20); */
  dcm02_tmp = (real32_T)asin(-((dcm02_tmp - dcm02_tmp_0) * 2.0F));

  /* '<S4>:1:37' if abs(theta - d_pi2) < 1.0e-3 */
  if ((real32_T)fabs(dcm02_tmp - 1.57079637F) < 0.001) {
    /* '<S4>:1:38' phi = single(0.0); */
    rtb_phi_d = 0.0F;

    /* '<S4>:1:39' psi = atan2(dcm12, dcm02); */
    ModeSwitch_HIL_B.aSq = rt_atan2f_snf(dcm12, dcm02);
  } else if ((real32_T)fabs(dcm02_tmp + 1.57079637F) < 0.001) {
    /* '<S4>:1:40' elseif abs(theta + d_pi2) < 1.0e-3 */
    /* '<S4>:1:41' phi = single(0.0); */
    rtb_phi_d = 0.0F;

    /* '<S4>:1:42' psi = atan2(-dcm12, -dcm02); */
    ModeSwitch_HIL_B.aSq = rt_atan2f_snf(-dcm12, -dcm02);
  } else {
    /* '<S4>:1:43' else */
    /* '<S4>:1:44' phi = atan2(dcm21, dcm22); */
    rtb_phi_d = rt_atan2f_snf((rtb_phi_d + dcm12_tmp) * 2.0F,
      ((ModeSwitch_HIL_B.aSq - ModeSwitch_HIL_B.bSq) - ModeSwitch_HIL_B.cSq) +
      dSq);

    /* '<S4>:1:45' psi = atan2(dcm10, dcm00); */
    ModeSwitch_HIL_B.aSq = rt_atan2f_snf((ModeSwitch_HIL_B.vehicle_attitude[1] *
      ModeSwitch_HIL_B.vehicle_attitude[2] + ModeSwitch_HIL_B.vehicle_attitude[0]
      * ModeSwitch_HIL_B.vehicle_attitude[3]) * 2.0F, ((ModeSwitch_HIL_B.aSq +
      ModeSwitch_HIL_B.bSq) - ModeSwitch_HIL_B.cSq) - dSq);
  }

  /* MATLAB Function: '<S5>/deadzone1' incorporates:
   *  DataTypeConversion: '<S1>/Conversion1'
   */
  ModeSwitch_HIL_deadzone1((real_T)ModeSwitch_HIL_B.input_rc_o2,
    &ModeSwitch_HIL_B.sf_deadzone1);

  /* MATLAB Function: '<S5>/deadzone2' incorporates:
   *  DataTypeConversion: '<S1>/Conversion2'
   */
  ModeSwitch_HIL_deadzone1((real_T)ModeSwitch_HIL_B.input_rc_o1,
    &ModeSwitch_HIL_B.sf_deadzone2);

  /* Saturate: '<S5>/Saturation8' */
  if (ModeSwitch_HIL_B.sf_deadzone1.y > 1.0) {
    ModeSwitch_HIL_B.u0 = 1.0;
  } else if (ModeSwitch_HIL_B.sf_deadzone1.y < -1.0) {
    ModeSwitch_HIL_B.u0 = -1.0;
  } else {
    ModeSwitch_HIL_B.u0 = ModeSwitch_HIL_B.sf_deadzone1.y;
  }

  /* End of Saturate: '<S5>/Saturation8' */

  /* Gain: '<S5>/max_vx' */
  ModeSwitch_HIL_B.max_vx = -5.0 * ModeSwitch_HIL_B.u0;

  /* Saturate: '<S5>/Saturation9' */
  if (ModeSwitch_HIL_B.sf_deadzone2.y > 1.0) {
    ModeSwitch_HIL_B.u0 = 1.0;
  } else if (ModeSwitch_HIL_B.sf_deadzone2.y < -1.0) {
    ModeSwitch_HIL_B.u0 = -1.0;
  } else {
    ModeSwitch_HIL_B.u0 = ModeSwitch_HIL_B.sf_deadzone2.y;
  }

  /* End of Saturate: '<S5>/Saturation9' */

  /* Gain: '<S5>/max_vy' */
  ModeSwitch_HIL_B.max_vy = 5.0 * ModeSwitch_HIL_B.u0;

  /* MATLAB Function: '<S5>/deadzone3' incorporates:
   *  DataTypeConversion: '<S1>/Conversion3'
   */
  ModeSwitch_HIL_deadzone1((real_T)ModeSwitch_HIL_B.input_rc_o3,
    &ModeSwitch_HIL_B.sf_deadzone3);

  /* Saturate: '<S5>/Saturation10' */
  if (ModeSwitch_HIL_B.sf_deadzone3.y > 1.0) {
    ModeSwitch_HIL_B.u0 = 1.0;
  } else if (ModeSwitch_HIL_B.sf_deadzone3.y < -1.0) {
    ModeSwitch_HIL_B.u0 = -1.0;
  } else {
    ModeSwitch_HIL_B.u0 = ModeSwitch_HIL_B.sf_deadzone3.y;
  }

  /* End of Saturate: '<S5>/Saturation10' */

  /* Gain: '<S5>/max_vz' */
  ModeSwitch_HIL_B.max_vz = -3.0 * ModeSwitch_HIL_B.u0;

  /* MATLAB Function: '<S6>/deadzone3' incorporates:
   *  DataTypeConversion: '<S1>/Conversion3'
   */
  ModeSwitch_HIL_deadzone1((real_T)ModeSwitch_HIL_B.input_rc_o3,
    &ModeSwitch_HIL_B.sf_deadzone3_o);

  /* MATLAB Function: '<S6>/deadzone1' incorporates:
   *  DataTypeConversion: '<S1>/Conversion2'
   */
  /* Function description: */
  /*   throttle curve. When the throttle is in the middle position, the output */
  /*   is hovering throttle value. */
  /* Input: */
  /*   in: normalized throttle, range from 0 to 1 */
  /*   hover: hovering throttle value */
  /* Output: */
  /*   y: processed throttle value */
  /* MATLAB Function 'Control System/InputConditioning1/f1': '<S19>:1' */
  /* '<S19>:1:10' if in < 0.5 */
  ModeSwitch_HIL_deadzone1((real_T)ModeSwitch_HIL_B.input_rc_o1,
    &ModeSwitch_HIL_B.sf_deadzone1_l);

  /* MATLAB Function: '<S6>/deadzone2' incorporates:
   *  DataTypeConversion: '<S1>/Conversion1'
   */
  ModeSwitch_HIL_deadzone1((real_T)ModeSwitch_HIL_B.input_rc_o2,
    &ModeSwitch_HIL_B.sf_deadzone2_g);

  /* MATLAB Function: '<S6>/deadzone4' incorporates:
   *  DataTypeConversion: '<S1>/Conversion4'
   */
  ModeSwitch_HIL_deadzone1((real_T)ModeSwitch_HIL_B.input_rc_o4,
    &ModeSwitch_HIL_B.sf_deadzone4);

  /* MATLAB Function: '<S1>/MATLAB Function1' */
  /* Function description: */
  /*   Map the remote control CH5 channel to control mode */
  /* input: */
  /*   ch5: Remote control CH5 channel input */
  /*   control_mode: Custom control mode */
  /*   0: stablelize mode&#xFF1B;1: height control mode; 2: position control mode */
  /* MATLAB Function 'Control System/MATLAB Function1': '<S8>:1' */
  /* '<S8>:1:8' if ch5 < 1400 */
  if (ModeSwitch_HIL_B.input_rc_o5 < 1400) {
    /*      control_mode = int8(0); */
    /* '<S8>:1:10' control_mode = int8(3); */
    rtb_control_mode = 3;
  } else if (ModeSwitch_HIL_B.input_rc_o5 < 1600) {
    /* '<S8>:1:11' elseif ch5 < 1600 */
    /* '<S8>:1:12' control_mode = int8(1); */
    rtb_control_mode = 1;
  } else {
    /* '<S8>:1:13' else */
    /* '<S8>:1:14' control_mode = int8(2); */
    rtb_control_mode = 2;
  }

  /* End of MATLAB Function: '<S1>/MATLAB Function1' */

  /* MATLAB Function: '<S1>/MATLAB Function2' incorporates:
   *  DataTypeConversion: '<S1>/Conversion10'
   *  DataTypeConversion: '<S1>/Conversion5'
   *  DataTypeConversion: '<S1>/Conversion7'
   *  DataTypeConversion: '<S1>/Conversion8'
   *  DataTypeConversion: '<S1>/Conversion9'
   */
  /* Function description: */
  /*   Mode switch implementation. */
  /* Input: */
  /*   mode */
  /*   vxd, vyd, vzd: The desired velocity specified by the remote control */
  /*   x, y, z: current position */
  /*   vx, vy,vz: current velocity */
  /* Output: */
  /*   vx_d, vy_d, vz_d, x_d, y_d, z_d: desired velocity and position */
  /* MATLAB Function 'Control System/MATLAB Function2': '<S9>:1' */
  /* '<S9>:1:12' if isempty(x1) */
  /* '<S9>:1:16' if isempty(y1) */
  /* '<S9>:1:20' if isempty(z1) */
  if (!ModeSwitch_HIL_DW.z1_not_empty) {
    /* '<S9>:1:21' z1 = z; */
    ModeSwitch_HIL_DW.z1 = ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.z;
    ModeSwitch_HIL_DW.z1_not_empty = true;
  }

  /* '<S9>:1:24' if isempty(hold_x_flag) */
  /* '<S9>:1:28' if isempty(hold_y_flag) */
  /* '<S9>:1:32' if isempty(hold_z_flag) */
  /*  Guo 2021/4/20 */
  /* '<S9>:1:39' if isempty(waypoint_1_flag) */
  /* '<S9>:1:44' if isempty(waypoint_2_flag) */
  /* '<S9>:1:49' if isempty(waypoint_3_flag) */
  /* '<S9>:1:54' if isempty(waypoint_4_flag) */
  /*  */
  /* '<S9>:1:59' if mode == int8(0) */
  switch (rtb_control_mode) {
   case 1:
    /* '<S9>:1:62' elseif mode == int8(1) */
    /*  height control mode */
    /* '<S9>:1:63' x1 = x; */
    ModeSwitch_HIL_DW.x1 = ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.x;

    /* '<S9>:1:63' y1 = y; */
    ModeSwitch_HIL_DW.y1 = ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.y;

    /* '<S9>:1:64' vx_d = vx; */
    ModeSwitch_HIL_B.max_vx = ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.vx;

    /* '<S9>:1:64' vy_d = vy; */
    ModeSwitch_HIL_B.max_vy = ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.vy;

    /* '<S9>:1:66' if abs(vzd) < 0.001 && abs(vz) < 6 */
    if ((fabs(ModeSwitch_HIL_B.max_vz) < 0.001) && (fabs
         (ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.vz) < 6.0)) {
      /* '<S9>:1:67' hold_z = 1; */
      hold_z = 1;
    } else {
      /* '<S9>:1:68' else */
      /* '<S9>:1:69' hold_z = 0; */
      hold_z = 0;

      /* '<S9>:1:70' hold_z_flag = 0; */
      ModeSwitch_HIL_DW.hold_z_flag = 0.0;
    }

    /* '<S9>:1:72' if (hold_z > 0.5) && (hold_z_flag < 0.5) */
    if ((hold_z > 0.5) && (ModeSwitch_HIL_DW.hold_z_flag < 0.5)) {
      /* '<S9>:1:73' z1 = z; */
      ModeSwitch_HIL_DW.z1 = ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.z;

      /* '<S9>:1:74' hold_z_flag = 1; */
      ModeSwitch_HIL_DW.hold_z_flag = 1.0;
    }

    /* '<S9>:1:76' if hold_z < 0.5 */
    if (hold_z < 0.5) {
      /* '<S9>:1:77' z1 = z; */
      ModeSwitch_HIL_DW.z1 = ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.z;

      /* '<S9>:1:78' hold_z_flag = 0; */
      ModeSwitch_HIL_DW.hold_z_flag = 0.0;
    }

    /* '<S9>:1:80' vz_d = vzd; */
    /*  Guo 2021/4/20 waypoint mode */
    break;

   case 3:
    /* '<S9>:1:82' elseif mode == int8(3) */
    /* '<S9>:1:83' e = 1; */
    /* '<S9>:1:84' if waypoint_1_flag == 0 */
    if (ModeSwitch_HIL_DW.waypoint_1_flag == 0.0) {
      /* '<S9>:1:85' x1 = waypoints(1,1); */
      ModeSwitch_HIL_DW.x1 = 0.0;

      /* '<S9>:1:86' y1 = waypoints(1,2); */
      ModeSwitch_HIL_DW.y1 = 0.0;

      /* '<S9>:1:87' if abs(x-x1)<e && abs(y-y1)<e */
      if ((fabs(ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.x -
                ModeSwitch_HIL_DW.x1) < 1.0) && (fabs
           (ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.y -
            ModeSwitch_HIL_DW.y1) < 1.0)) {
        /* '<S9>:1:88' waypoint_1_flag = 1; */
        ModeSwitch_HIL_DW.waypoint_1_flag = 1.0;
      }
    } else if (ModeSwitch_HIL_DW.waypoint_2_flag == 0.0) {
      /* '<S9>:1:90' elseif waypoint_2_flag == 0 */
      /* '<S9>:1:91' x1 = waypoints(2,1); */
      ModeSwitch_HIL_DW.x1 = 0.0;

      /* '<S9>:1:92' y1 = waypoints(2,2); */
      ModeSwitch_HIL_DW.y1 = -20.0;

      /* '<S9>:1:93' if abs(x-x1)<e && abs(y-y1)<e */
      if ((fabs(ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.x -
                ModeSwitch_HIL_DW.x1) < 1.0) && (fabs
           (ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.y -
            ModeSwitch_HIL_DW.y1) < 1.0)) {
        /* '<S9>:1:94' waypoint_2_flag = 1; */
        ModeSwitch_HIL_DW.waypoint_2_flag = 1.0;
      }
    } else if (ModeSwitch_HIL_DW.waypoint_3_flag == 0.0) {
      /* '<S9>:1:96' elseif waypoint_3_flag == 0 */
      /* '<S9>:1:97' x1 = waypoints(3,1); */
      ModeSwitch_HIL_DW.x1 = 20.0;

      /* '<S9>:1:98' y1 = waypoints(3,2); */
      ModeSwitch_HIL_DW.y1 = -20.0;

      /* '<S9>:1:99' if abs(x-x1)<e && abs(y-y1)<e */
      if ((fabs(ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.x -
                ModeSwitch_HIL_DW.x1) < 1.0) && (fabs
           (ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.y -
            ModeSwitch_HIL_DW.y1) < 1.0)) {
        /* '<S9>:1:100' waypoint_3_flag = 1; */
        ModeSwitch_HIL_DW.waypoint_3_flag = 1.0;
      }
    } else if (ModeSwitch_HIL_DW.waypoint_4_flag == 0.0) {
      /* '<S9>:1:102' elseif waypoint_4_flag == 0 */
      /* '<S9>:1:103' x1 = waypoints(4,1); */
      ModeSwitch_HIL_DW.x1 = 20.0;

      /* '<S9>:1:104' y1 = waypoints(4,2); */
      ModeSwitch_HIL_DW.y1 = 0.0;

      /* '<S9>:1:105' if abs(x-x1)<e && abs(y-y1)<e */
      if ((fabs(ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.x -
                ModeSwitch_HIL_DW.x1) < 1.0) && (fabs
           (ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.y -
            ModeSwitch_HIL_DW.y1) < 1.0)) {
        /* '<S9>:1:106' waypoint_4_flag = 1; */
        ModeSwitch_HIL_DW.waypoint_4_flag = 1.0;
      }
    } else {
      /* '<S9>:1:108' else */
      /* '<S9>:1:109' x1 = waypoints(1,1); */
      ModeSwitch_HIL_DW.x1 = 0.0;

      /* '<S9>:1:110' y1 = waypoints(1,2); */
      ModeSwitch_HIL_DW.y1 = 0.0;

      /* '<S9>:1:111' if abs(x-x1)<e && abs(y-y1)<e */
      if ((fabs(ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.x -
                ModeSwitch_HIL_DW.x1) < 1.0) && (fabs
           (ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.y -
            ModeSwitch_HIL_DW.y1) < 1.0)) {
        /* '<S9>:1:112' x1 = waypoints(1,1); */
        ModeSwitch_HIL_DW.x1 = 0.0;

        /* '<S9>:1:113' y1 = waypoints(1,2); */
        ModeSwitch_HIL_DW.y1 = 0.0;

        /*              waypoint_1_flag = 0; */
        /*              waypoint_2_flag = 0; */
        /*              waypoint_3_flag = 0; */
        /*              waypoint_4_flag = 0; */
      }
    }

    /*      vx_d = vxd; vy_d = vyd; vz_d = vzd; */
    /* '<S9>:1:121' vx_d = (x1-x)/10; */
    ModeSwitch_HIL_B.max_vx = (ModeSwitch_HIL_DW.x1 -
      ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.x) / 10.0;

    /* '<S9>:1:122' vy_d = (y1-y)/10; */
    ModeSwitch_HIL_B.max_vy = (ModeSwitch_HIL_DW.y1 -
      ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.y) / 10.0;

    /* '<S9>:1:123' vz_d = 0; */
    ModeSwitch_HIL_B.max_vz = 0.0;

    /*  position control mode    */
    break;

   default:
    /* '<S9>:1:125' else */
    /* '<S9>:1:126' waypoint_1_flag = 0; */
    ModeSwitch_HIL_DW.waypoint_1_flag = 0.0;

    /* '<S9>:1:127' waypoint_2_flag = 0; */
    ModeSwitch_HIL_DW.waypoint_2_flag = 0.0;

    /* '<S9>:1:128' waypoint_3_flag = 0; */
    ModeSwitch_HIL_DW.waypoint_3_flag = 0.0;

    /* '<S9>:1:129' waypoint_4_flag = 0; */
    ModeSwitch_HIL_DW.waypoint_4_flag = 0.0;

    /* '<S9>:1:130' if abs(vxd) < 0.001 && abs(vx) < 8 */
    if ((fabs(ModeSwitch_HIL_B.max_vx) < 0.001) && (fabs
         (ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.vx) < 8.0)) {
      /* '<S9>:1:131' hold_x = 1; */
      hold_x = 1;
    } else {
      /* '<S9>:1:132' else */
      /* '<S9>:1:133' hold_x = 0; */
      hold_x = 0;

      /* '<S9>:1:134' hold_x_flag = 0; */
      ModeSwitch_HIL_DW.hold_x_flag = 0.0;
    }

    /* '<S9>:1:136' if abs(vyd) < 0.001 && abs(vy) < 8 */
    if ((fabs(ModeSwitch_HIL_B.max_vy) < 0.001) && (fabs
         (ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.vy) < 8.0)) {
      /* '<S9>:1:137' hold_y = 1; */
      hold_y = 1;
    } else {
      /* '<S9>:1:138' else */
      /* '<S9>:1:139' hold_y = 0; */
      hold_y = 0;

      /* '<S9>:1:140' hold_y_flag = 0; */
      ModeSwitch_HIL_DW.hold_y_flag = 0.0;
    }

    /* '<S9>:1:142' if abs(vzd) < 0.001 && abs(vz) < 6 */
    if ((fabs(ModeSwitch_HIL_B.max_vz) < 0.001) && (fabs
         (ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.vz) < 6.0)) {
      /* '<S9>:1:143' hold_z = 1; */
      hold_z = 1;
    } else {
      /* '<S9>:1:144' else */
      /* '<S9>:1:145' hold_z = 0; */
      hold_z = 0;

      /* '<S9>:1:146' hold_z_flag = 0; */
      ModeSwitch_HIL_DW.hold_z_flag = 0.0;
    }

    /* '<S9>:1:149' if (hold_x > 0.5) && (hold_x_flag < 0.5) */
    if ((hold_x > 0.5) && (ModeSwitch_HIL_DW.hold_x_flag < 0.5)) {
      /* '<S9>:1:150' x1 = x; */
      ModeSwitch_HIL_DW.x1 = ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.x;

      /* '<S9>:1:151' hold_x_flag = 1; */
      ModeSwitch_HIL_DW.hold_x_flag = 1.0;
    }

    /* '<S9>:1:153' if (hold_y > 0.5) && (hold_y_flag < 0.5) */
    if ((hold_y > 0.5) && (ModeSwitch_HIL_DW.hold_y_flag < 0.5)) {
      /* '<S9>:1:154' y1 = y; */
      ModeSwitch_HIL_DW.y1 = ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.y;

      /* '<S9>:1:155' hold_y_flag = 1; */
      ModeSwitch_HIL_DW.hold_y_flag = 1.0;
    }

    /*  throttle in the middle position, hold the height */
    /* '<S9>:1:158' if (hold_z > 0.5) && (hold_z_flag < 0.5) */
    if ((hold_z > 0.5) && (ModeSwitch_HIL_DW.hold_z_flag < 0.5)) {
      /* '<S9>:1:159' z1 = z; */
      ModeSwitch_HIL_DW.z1 = ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.z;

      /* '<S9>:1:160' hold_z_flag = 1; */
      ModeSwitch_HIL_DW.hold_z_flag = 1.0;
    }

    /* When the RC sticks(CH1, CH2, CH3, CH4) deviate from the middle dead zone range, the position is the current position.  */
    /* The position controller only works with the velocity loop */
    /* '<S9>:1:164' if hold_x < 0.5 */
    if (hold_x < 0.5) {
      /* '<S9>:1:165' x1 = x; */
      ModeSwitch_HIL_DW.x1 = ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.x;

      /* '<S9>:1:166' hold_x_flag = 0; */
      ModeSwitch_HIL_DW.hold_x_flag = 0.0;
    }

    /* '<S9>:1:168' if hold_y < 0.5 */
    if (hold_y < 0.5) {
      /* '<S9>:1:169' y1 = y; */
      ModeSwitch_HIL_DW.y1 = ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.y;

      /* '<S9>:1:170' hold_y_flag = 0; */
      ModeSwitch_HIL_DW.hold_y_flag = 0.0;
    }

    /* '<S9>:1:172' if hold_z < 0.5 */
    if (hold_z < 0.5) {
      /* '<S9>:1:173' z1 = z; */
      ModeSwitch_HIL_DW.z1 = ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.z;

      /* '<S9>:1:174' hold_z_flag = 0; */
      ModeSwitch_HIL_DW.hold_z_flag = 0.0;
    }

    /* '<S9>:1:176' vx_d = vxd; */
    /* '<S9>:1:176' vy_d = vyd; */
    /* '<S9>:1:176' vz_d = vzd; */
    break;
  }

  /* MATLAB Function: '<S1>/MATLAB Function3' incorporates:
   *  DataTypeConversion: '<S1>/Conversion3'
   */
  /* '<S9>:1:179' x_d = x1; */
  /* '<S9>:1:180' y_d = y1; */
  /* '<S9>:1:181' z_d = z1; */
  /* Function description: */
  /*   reset Integral when throttle < 1250 */
  /* MATLAB Function 'Control System/MATLAB Function3': '<S10>:1' */
  /* '<S10>:1:4' if ch3 < 1250 */
  if (ModeSwitch_HIL_B.input_rc_o3 < 1250) {
    /* '<S10>:1:5' reset_i = [1 1 1 1]; */
    hold_x = 1;
    hold_y = 1;
    hold_z = 1;
    rtb_reset_i_idx_3 = 1;
  } else if (rtb_control_mode == 1) {
    /* '<S10>:1:8' elseif mode == 1 */
    /* '<S10>:1:9' reset_i = [0 0 1 0]; */
    hold_x = 0;
    hold_y = 0;
    hold_z = 1;
    rtb_reset_i_idx_3 = 0;
  } else {
    /* '<S10>:1:10' else */
    /* '<S10>:1:11' reset_i = [0 0 0 0]; */
    hold_x = 0;
    hold_y = 0;
    hold_z = 0;
    rtb_reset_i_idx_3 = 0;
  }

  /* End of MATLAB Function: '<S1>/MATLAB Function3' */

  /* Sum: '<S22>/Sum1' incorporates:
   *  DataTypeConversion: '<S1>/Conversion5'
   *  MATLAB Function: '<S1>/MATLAB Function2'
   */
  ModeSwitch_HIL_B.u0 = ModeSwitch_HIL_DW.x1 -
    ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.x;

  /* Saturate: '<S33>/Saturation3' */
  if (ModeSwitch_HIL_B.u0 > 5.0) {
    ModeSwitch_HIL_B.u0 = 5.0;
  } else if (ModeSwitch_HIL_B.u0 < -5.0) {
    ModeSwitch_HIL_B.u0 = -5.0;
  }

  /* End of Saturate: '<S33>/Saturation3' */

  /* Sum: '<S22>/Sum7' incorporates:
   *  DataTypeConversion: '<S1>/Conversion9'
   */
  ModeSwitch_HIL_B.max_vx = (ModeSwitch_HIL_B.max_vx + ModeSwitch_HIL_B.u0) -
    ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.vx;

  /* DiscreteIntegrator: '<S36>/Discrete-Time Integrator' */
  if ((hold_z != 0) || (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRese != 0))
  {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE = 0.0;
  }

  if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE >= 3.43) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE = 3.43;
  } else if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE <= -3.43) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE = -3.43;
  }

  /* SampleTimeMath: '<S39>/TSamp' incorporates:
   *  Gain: '<S36>/Derivative Gain'
   *
   * About '<S39>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  ModeSwitch_HIL_B.TSamp = 0.01 * ModeSwitch_HIL_B.max_vx * 250.0;

  /* Sum: '<S36>/Sum' incorporates:
   *  DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
   *  Gain: '<S36>/Gain'
   *  Sum: '<S39>/Diff'
   *  UnitDelay: '<S39>/UD'
   *
   * Block description for '<S39>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S39>/UD':
   *
   *  Store in Global RAM
   */
  ModeSwitch_HIL_B.Sum_i = (2.5 * ModeSwitch_HIL_B.max_vx +
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE) + (ModeSwitch_HIL_B.TSamp -
    ModeSwitch_HIL_DW.UD_DSTATE);

  /* Sum: '<S22>/Sum3' incorporates:
   *  DataTypeConversion: '<S1>/Conversion7'
   *  MATLAB Function: '<S1>/MATLAB Function2'
   */
  ModeSwitch_HIL_B.u0 = ModeSwitch_HIL_DW.y1 -
    ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.y;

  /* Saturate: '<S34>/Saturation3' */
  if (ModeSwitch_HIL_B.u0 > 5.0) {
    ModeSwitch_HIL_B.u0 = 5.0;
  } else if (ModeSwitch_HIL_B.u0 < -5.0) {
    ModeSwitch_HIL_B.u0 = -5.0;
  }

  /* End of Saturate: '<S34>/Saturation3' */

  /* Sum: '<S22>/Sum2' incorporates:
   *  DataTypeConversion: '<S1>/Conversion10'
   */
  ModeSwitch_HIL_B.max_vy = (ModeSwitch_HIL_B.max_vy + ModeSwitch_HIL_B.u0) -
    ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.vy;

  /* DiscreteIntegrator: '<S37>/Discrete-Time Integrator' */
  if ((hold_z != 0) || (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRe_h != 0))
  {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_m = 0.0;
  }

  if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_m >= 3.43) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_m = 3.43;
  } else if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_m <= -3.43) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_m = -3.43;
  }

  /* SampleTimeMath: '<S40>/TSamp' incorporates:
   *  Gain: '<S37>/Derivative Gain'
   *
   * About '<S40>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  ModeSwitch_HIL_B.TSamp_b = 0.01 * ModeSwitch_HIL_B.max_vy * 250.0;

  /* Sum: '<S37>/Sum' incorporates:
   *  DiscreteIntegrator: '<S37>/Discrete-Time Integrator'
   *  Gain: '<S37>/Gain'
   *  Sum: '<S40>/Diff'
   *  UnitDelay: '<S40>/UD'
   *
   * Block description for '<S40>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S40>/UD':
   *
   *  Store in Global RAM
   */
  ModeSwitch_HIL_B.Sum_d = (2.5 * ModeSwitch_HIL_B.max_vy +
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_m) +
    (ModeSwitch_HIL_B.TSamp_b - ModeSwitch_HIL_DW.UD_DSTATE_g);

  /* Switch generated from: '<S11>/Switch1' incorporates:
   *  Gain: '<S6>/Gain1'
   *  Gain: '<S6>/deg2rad1'
   *  Saturate: '<S6>/Saturation9'
   */
  /* Function description: */
  /*   Convert the desired horizontal acceleration to the desired attitude angle */
  /* Input: */
  /*   eax, eaxy: desired horizontal acceleration */
  /*   psi: yaw */
  /* Output: */
  /*   phi, theta: desired roll angle and desired pitch angle */
  /* MATLAB Function 'Control System/Subsystem/position_control/MATLAB Function': '<S32>:1' */
  /* '<S32>:1:9' g = 9.8; */
  /* '<S32>:1:10' phi = (-sin(psi)*eax + cos(psi)*eay)/g; */
  /* '<S32>:1:11' theta = (-cos(psi)*eax - sin(psi)*eay)/g; */
  if (rtb_control_mode >= 2) {
    /* MATLAB Function: '<S22>/MATLAB Function' incorporates:
     *  DataTypeConversion: '<S1>/Conversion17'
     */
    ModeSwitch_HIL_B.DiscreteTimeIntegrator_n = (-sin(ModeSwitch_HIL_B.aSq) *
      ModeSwitch_HIL_B.Sum_i + cos(ModeSwitch_HIL_B.aSq) *
      ModeSwitch_HIL_B.Sum_d) / 9.8;

    /* Saturate: '<S22>/Saturation1' */
    if (ModeSwitch_HIL_B.DiscreteTimeIntegrator_n > 0.61086550000000006) {
      ModeSwitch_HIL_B.DiscreteTimeIntegrator_n = 0.61086550000000006;
    } else if (ModeSwitch_HIL_B.DiscreteTimeIntegrator_n < -0.61086550000000006)
    {
      ModeSwitch_HIL_B.DiscreteTimeIntegrator_n = -0.61086550000000006;
    }

    /* End of Saturate: '<S22>/Saturation1' */
  } else {
    if (ModeSwitch_HIL_B.sf_deadzone1_l.y > 1.0) {
      /* Saturate: '<S6>/Saturation9' */
      ModeSwitch_HIL_B.u0 = 1.0;
    } else if (ModeSwitch_HIL_B.sf_deadzone1_l.y < -1.0) {
      /* Saturate: '<S6>/Saturation9' */
      ModeSwitch_HIL_B.u0 = -1.0;
    } else {
      /* Saturate: '<S6>/Saturation9' */
      ModeSwitch_HIL_B.u0 = ModeSwitch_HIL_B.sf_deadzone1_l.y;
    }

    ModeSwitch_HIL_B.DiscreteTimeIntegrator_n = 35.0 * ModeSwitch_HIL_B.u0 *
      0.0174533;
  }

  /* Gain: '<S26>/Gain' incorporates:
   *  DataTypeConversion: '<S1>/Conversion15'
   *  Sum: '<S21>/Sum18'
   */
  ModeSwitch_HIL_B.u0 = (ModeSwitch_HIL_B.DiscreteTimeIntegrator_n - rtb_phi_d) *
    6.5;

  /* Saturate: '<S21>/Saturation5' */
  if (ModeSwitch_HIL_B.u0 > 220.0) {
    ModeSwitch_HIL_B.u0 = 220.0;
  } else if (ModeSwitch_HIL_B.u0 < -220.0) {
    ModeSwitch_HIL_B.u0 = -220.0;
  }

  /* End of Saturate: '<S21>/Saturation5' */

  /* Sum: '<S21>/Sum21' incorporates:
   *  DataTypeConversion: '<S1>/Conversion12'
   */
  ModeSwitch_HIL_B.DiscreteTimeIntegrator_n = ModeSwitch_HIL_B.u0 -
    ModeSwitch_HIL_B.sensor_combined_o1;

  /* DiscreteIntegrator: '<S27>/Discrete-Time Integrator' */
  if ((hold_x != 0) || (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRe_g != 0))
  {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_b = 0.0;
  }

  if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_b >= 0.3) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_b = 0.3;
  } else if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_b <= -0.3) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_b = -0.3;
  }

  /* SampleTimeMath: '<S30>/TSamp' incorporates:
   *  Gain: '<S27>/Derivative Gain'
   *
   * About '<S30>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  ModeSwitch_HIL_B.TSamp_k = 0.001 * ModeSwitch_HIL_B.DiscreteTimeIntegrator_n *
    250.0;

  /* Sum: '<S27>/Sum' incorporates:
   *  DiscreteIntegrator: '<S27>/Discrete-Time Integrator'
   *  Gain: '<S27>/Gain'
   *  Sum: '<S30>/Diff'
   *  UnitDelay: '<S30>/UD'
   *
   * Block description for '<S30>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S30>/UD':
   *
   *  Store in Global RAM
   */
  ModeSwitch_HIL_B.Saturation = (0.1 * ModeSwitch_HIL_B.DiscreteTimeIntegrator_n
    + ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_b) +
    (ModeSwitch_HIL_B.TSamp_k - ModeSwitch_HIL_DW.UD_DSTATE_i);

  /* Saturate: '<S21>/Saturation' */
  if (ModeSwitch_HIL_B.Saturation > 1.0) {
    ModeSwitch_HIL_B.Saturation = 1.0;
  } else if (ModeSwitch_HIL_B.Saturation < -1.0) {
    ModeSwitch_HIL_B.Saturation = -1.0;
  }

  /* End of Saturate: '<S21>/Saturation' */

  /* Switch generated from: '<S11>/Switch1' incorporates:
   *  Gain: '<S6>/Gain3'
   *  Gain: '<S6>/deg2rad2'
   *  Saturate: '<S6>/Saturation8'
   */
  if (rtb_control_mode >= 2) {
    /* MATLAB Function: '<S22>/MATLAB Function' incorporates:
     *  DataTypeConversion: '<S1>/Conversion17'
     */
    ModeSwitch_HIL_B.Sum_i = (-cos(ModeSwitch_HIL_B.aSq) *
      ModeSwitch_HIL_B.Sum_i - sin(ModeSwitch_HIL_B.aSq) *
      ModeSwitch_HIL_B.Sum_d) / 9.8;

    /* Saturate: '<S22>/Saturation2' */
    if (ModeSwitch_HIL_B.Sum_i > 0.30543275000000003) {
      ModeSwitch_HIL_B.Sum_i = 0.30543275000000003;
    } else if (ModeSwitch_HIL_B.Sum_i < -0.30543275000000003) {
      ModeSwitch_HIL_B.Sum_i = -0.30543275000000003;
    }

    /* End of Saturate: '<S22>/Saturation2' */
  } else {
    if (ModeSwitch_HIL_B.sf_deadzone2_g.y > 1.0) {
      /* Saturate: '<S6>/Saturation8' */
      ModeSwitch_HIL_B.u0 = 1.0;
    } else if (ModeSwitch_HIL_B.sf_deadzone2_g.y < -1.0) {
      /* Saturate: '<S6>/Saturation8' */
      ModeSwitch_HIL_B.u0 = -1.0;
    } else {
      /* Saturate: '<S6>/Saturation8' */
      ModeSwitch_HIL_B.u0 = ModeSwitch_HIL_B.sf_deadzone2_g.y;
    }

    ModeSwitch_HIL_B.Sum_i = 35.0 * ModeSwitch_HIL_B.u0 * 0.0174533;
  }

  /* Gain: '<S24>/Gain' incorporates:
   *  DataTypeConversion: '<S1>/Conversion16'
   *  MATLAB Function: '<Root>/quat2eul'
   *  Sum: '<S21>/Sum19'
   */
  ModeSwitch_HIL_B.u0 = (ModeSwitch_HIL_B.Sum_i - dcm02_tmp) * 6.5;

  /* Saturate: '<S21>/Saturation3' */
  if (ModeSwitch_HIL_B.u0 > 220.0) {
    ModeSwitch_HIL_B.u0 = 220.0;
  } else if (ModeSwitch_HIL_B.u0 < -220.0) {
    ModeSwitch_HIL_B.u0 = -220.0;
  }

  /* End of Saturate: '<S21>/Saturation3' */

  /* Sum: '<S21>/Sum22' incorporates:
   *  DataTypeConversion: '<S1>/Conversion13'
   */
  ModeSwitch_HIL_B.Sum_i = ModeSwitch_HIL_B.u0 -
    ModeSwitch_HIL_B.sensor_combined_o2;

  /* DiscreteIntegrator: '<S25>/Discrete-Time Integrator' */
  if ((hold_x != 0) || (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevR_hk != 0))
  {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_g = 0.0;
  }

  if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_g >= 0.3) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_g = 0.3;
  } else if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_g <= -0.3) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_g = -0.3;
  }

  /* SampleTimeMath: '<S29>/TSamp' incorporates:
   *  Gain: '<S25>/Derivative Gain'
   *
   * About '<S29>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  ModeSwitch_HIL_B.Sum_d = 0.001 * ModeSwitch_HIL_B.Sum_i * 250.0;

  /* Sum: '<S25>/Sum' incorporates:
   *  DiscreteIntegrator: '<S25>/Discrete-Time Integrator'
   *  Gain: '<S25>/Gain'
   *  Sum: '<S29>/Diff'
   *  UnitDelay: '<S29>/UD'
   *
   * Block description for '<S29>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S29>/UD':
   *
   *  Store in Global RAM
   */
  ModeSwitch_HIL_B.Saturation1 = (0.1 * ModeSwitch_HIL_B.Sum_i +
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_g) + (ModeSwitch_HIL_B.Sum_d
    - ModeSwitch_HIL_DW.UD_DSTATE_d);

  /* Saturate: '<S21>/Saturation1' */
  if (ModeSwitch_HIL_B.Saturation1 > 1.0) {
    ModeSwitch_HIL_B.Saturation1 = 1.0;
  } else if (ModeSwitch_HIL_B.Saturation1 < -1.0) {
    ModeSwitch_HIL_B.Saturation1 = -1.0;
  }

  /* End of Saturate: '<S21>/Saturation1' */

  /* Saturate: '<S6>/Saturation7' */
  if (ModeSwitch_HIL_B.sf_deadzone4.y > 1.0) {
    ModeSwitch_HIL_B.u0 = 1.0;
  } else if (ModeSwitch_HIL_B.sf_deadzone4.y < -1.0) {
    ModeSwitch_HIL_B.u0 = -1.0;
  } else {
    ModeSwitch_HIL_B.u0 = ModeSwitch_HIL_B.sf_deadzone4.y;
  }

  /* End of Saturate: '<S6>/Saturation7' */

  /* Sum: '<S21>/Sum1' incorporates:
   *  DataTypeConversion: '<S1>/Conversion14'
   *  Gain: '<S6>/Gain2'
   *  Gain: '<S6>/deg2rad3'
   */
  ModeSwitch_HIL_B.Sum1 = 200.0 * ModeSwitch_HIL_B.u0 * 0.0174533 -
    ModeSwitch_HIL_B.sensor_combined_o3;

  /* DiscreteIntegrator: '<S28>/Discrete-Time Integrator' */
  if ((hold_y != 0) || (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevR_gt != 0))
  {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_c = 0.0;
  }

  if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_c >= 0.2) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_c = 0.2;
  } else if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_c <= -0.2) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_c = -0.2;
  }

  /* SampleTimeMath: '<S31>/TSamp' incorporates:
   *  Gain: '<S28>/Derivative Gain'
   *
   * About '<S31>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  ModeSwitch_HIL_B.TSamp_i = 0.0 * ModeSwitch_HIL_B.Sum1 * 250.0;

  /* Sum: '<S28>/Sum' incorporates:
   *  DiscreteIntegrator: '<S28>/Discrete-Time Integrator'
   *  Gain: '<S28>/Gain'
   *  Sum: '<S31>/Diff'
   *  UnitDelay: '<S31>/UD'
   *
   * Block description for '<S31>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S31>/UD':
   *
   *  Store in Global RAM
   */
  ModeSwitch_HIL_B.Saturation2 = (0.3 * ModeSwitch_HIL_B.Sum1 +
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_c) +
    (ModeSwitch_HIL_B.TSamp_i - ModeSwitch_HIL_DW.UD_DSTATE_e);

  /* Saturate: '<S21>/Saturation2' */
  if (ModeSwitch_HIL_B.Saturation2 > 0.5) {
    ModeSwitch_HIL_B.Saturation2 = 0.5;
  } else if (ModeSwitch_HIL_B.Saturation2 < -0.5) {
    ModeSwitch_HIL_B.Saturation2 = -0.5;
  }

  /* End of Saturate: '<S21>/Saturation2' */

  /* Gain: '<S35>/Gain1' incorporates:
   *  DataTypeConversion: '<S1>/Conversion8'
   *  MATLAB Function: '<S1>/MATLAB Function2'
   *  Sum: '<S22>/Sum4'
   */
  ModeSwitch_HIL_B.u0 = (ModeSwitch_HIL_DW.z1 -
    ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.z) * 4.0;

  /* Saturate: '<S35>/Saturation3' */
  if (ModeSwitch_HIL_B.u0 > 3.0) {
    ModeSwitch_HIL_B.u0 = 3.0;
  } else if (ModeSwitch_HIL_B.u0 < -3.0) {
    ModeSwitch_HIL_B.u0 = -3.0;
  }

  /* End of Saturate: '<S35>/Saturation3' */

  /* Sum: '<S22>/Sum' incorporates:
   *  DataTypeConversion: '<S1>/Conversion11'
   */
  ModeSwitch_HIL_B.max_vz = (ModeSwitch_HIL_B.max_vz + ModeSwitch_HIL_B.u0) -
    ModeSwitch_HIL_B.uORBReadFunctionCallTrigger.vz;

  /* DiscreteIntegrator: '<S38>/Discrete-Time Integrator' */
  if ((rtb_reset_i_idx_3 != 0) ||
      (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRe_p != 0)) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTAT_c5 = 0.0;
  }

  if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTAT_c5 >= 5.0) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTAT_c5 = 5.0;
  } else if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTAT_c5 <= -5.0) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTAT_c5 = -5.0;
  }

  /* SampleTimeMath: '<S41>/TSamp' incorporates:
   *  Gain: '<S38>/Derivative Gain'
   *
   * About '<S41>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  ModeSwitch_HIL_B.TSamp_d = 0.005 * ModeSwitch_HIL_B.max_vz * 250.0;

  /* Sum: '<S38>/Sum' incorporates:
   *  DiscreteIntegrator: '<S38>/Discrete-Time Integrator'
   *  Gain: '<S38>/Gain'
   *  Sum: '<S41>/Diff'
   *  Switch: '<S11>/Switch2'
   *  UnitDelay: '<S41>/UD'
   *
   * Block description for '<S41>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S41>/UD':
   *
   *  Store in Global RAM
   */
  ModeSwitch_HIL_B.u0 = (0.45 * ModeSwitch_HIL_B.max_vz +
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTAT_c5) +
    (ModeSwitch_HIL_B.TSamp_d - ModeSwitch_HIL_DW.UD_DSTATE_j);

  /* Saturate: '<S22>/az' incorporates:
   *  Switch: '<S11>/Switch2'
   */
  if (ModeSwitch_HIL_B.u0 > 0.4) {
    ModeSwitch_HIL_B.u0 = 0.4;
  } else if (ModeSwitch_HIL_B.u0 < -0.4) {
    ModeSwitch_HIL_B.u0 = -0.4;
  }

  /* End of Saturate: '<S22>/az' */

  /* Saturate: '<S22>/Saturation' incorporates:
   *  Constant: '<S22>/Constant'
   *  Gain: '<S22>/Gain'
   *  Sum: '<S22>/Sum6'
   *  Switch: '<S11>/Switch2'
   */
  if (-(ModeSwitch_HIL_B.u0 - 0.609) > 0.9) {
    ModeSwitch_HIL_B.Gain = 0.9;
  } else {
    ModeSwitch_HIL_B.Gain = -(ModeSwitch_HIL_B.u0 - 0.609);
  }

  /* End of Saturate: '<S22>/Saturation' */

  /* MATLAB Function: '<S11>/pwm_out2' */
  /* Function description: */
  /*   Control allocation. The quadrotor type is X-configuration, */
  /*   and the airframe is as follows: */
  /* 3&#x2193;   1&#x2191; */
  /*   \ / */
  /*   / \ */
  /* 2&#x2191;   4&#x2193; */
  /* Input&#xFF1A; */
  /*   Roll, Pitch, Yaw: attitude controller output. */
  /*   Thrust */
  /* MATLAB Function 'Control System/Subsystem/pwm_out2': '<S23>:1' */
  /* '<S23>:1:13' idle_PWM = 1000; */
  /* '<S23>:1:14' scale = 1000; */
  /* '<S23>:1:16' M1 = (Thrust - Roll + Pitch + Yaw) * scale + idle_PWM; */
  /* '<S23>:1:17' M2 = (Thrust + Roll - Pitch + Yaw) * scale + idle_PWM; */
  /* '<S23>:1:18' M3 = (Thrust + Roll + Pitch - Yaw) * scale + idle_PWM; */
  /* '<S23>:1:19' M4 = (Thrust - Roll - Pitch - Yaw) * scale + idle_PWM; */
  ModeSwitch_HIL_B.u0_tmp = ModeSwitch_HIL_B.Gain - ModeSwitch_HIL_B.Saturation;
  ModeSwitch_HIL_B.u0 = ((ModeSwitch_HIL_B.u0_tmp + ModeSwitch_HIL_B.Saturation1)
    + ModeSwitch_HIL_B.Saturation2) * 1000.0 + 1000.0;

  /* Saturate: '<S11>/Output_Limits1' */
  if (ModeSwitch_HIL_B.u0 > 2000.0) {
    /* Saturate: '<S11>/Output_Limits1' */
    ModeSwitch_HIL_B.Output_Limits1[0] = 2000.0;
  } else if (ModeSwitch_HIL_B.u0 < 1000.0) {
    /* Saturate: '<S11>/Output_Limits1' */
    ModeSwitch_HIL_B.Output_Limits1[0] = 1000.0;
  } else {
    /* Saturate: '<S11>/Output_Limits1' */
    ModeSwitch_HIL_B.Output_Limits1[0] = ModeSwitch_HIL_B.u0;
  }

  /* MATLAB Function: '<S11>/pwm_out2' */
  ModeSwitch_HIL_B.Saturation += ModeSwitch_HIL_B.Gain;
  ModeSwitch_HIL_B.u0 = ((ModeSwitch_HIL_B.Saturation -
    ModeSwitch_HIL_B.Saturation1) + ModeSwitch_HIL_B.Saturation2) * 1000.0 +
    1000.0;

  /* Saturate: '<S11>/Output_Limits1' */
  if (ModeSwitch_HIL_B.u0 > 2000.0) {
    /* Saturate: '<S11>/Output_Limits1' */
    ModeSwitch_HIL_B.Output_Limits1[1] = 2000.0;
  } else if (ModeSwitch_HIL_B.u0 < 1000.0) {
    /* Saturate: '<S11>/Output_Limits1' */
    ModeSwitch_HIL_B.Output_Limits1[1] = 1000.0;
  } else {
    /* Saturate: '<S11>/Output_Limits1' */
    ModeSwitch_HIL_B.Output_Limits1[1] = ModeSwitch_HIL_B.u0;
  }

  /* MATLAB Function: '<S11>/pwm_out2' */
  ModeSwitch_HIL_B.u0 = ((ModeSwitch_HIL_B.Saturation +
    ModeSwitch_HIL_B.Saturation1) - ModeSwitch_HIL_B.Saturation2) * 1000.0 +
    1000.0;

  /* Saturate: '<S11>/Output_Limits1' */
  if (ModeSwitch_HIL_B.u0 > 2000.0) {
    /* Saturate: '<S11>/Output_Limits1' */
    ModeSwitch_HIL_B.Output_Limits1[2] = 2000.0;
  } else if (ModeSwitch_HIL_B.u0 < 1000.0) {
    /* Saturate: '<S11>/Output_Limits1' */
    ModeSwitch_HIL_B.Output_Limits1[2] = 1000.0;
  } else {
    /* Saturate: '<S11>/Output_Limits1' */
    ModeSwitch_HIL_B.Output_Limits1[2] = ModeSwitch_HIL_B.u0;
  }

  /* MATLAB Function: '<S11>/pwm_out2' */
  ModeSwitch_HIL_B.u0 = ((ModeSwitch_HIL_B.u0_tmp - ModeSwitch_HIL_B.Saturation1)
    - ModeSwitch_HIL_B.Saturation2) * 1000.0 + 1000.0;

  /* Saturate: '<S11>/Output_Limits1' */
  if (ModeSwitch_HIL_B.u0 > 2000.0) {
    /* Saturate: '<S11>/Output_Limits1' */
    ModeSwitch_HIL_B.Output_Limits1[3] = 2000.0;
  } else if (ModeSwitch_HIL_B.u0 < 1000.0) {
    /* Saturate: '<S11>/Output_Limits1' */
    ModeSwitch_HIL_B.Output_Limits1[3] = 1000.0;
  } else {
    /* Saturate: '<S11>/Output_Limits1' */
    ModeSwitch_HIL_B.Output_Limits1[3] = ModeSwitch_HIL_B.u0;
  }

  /* Update for DiscreteIntegrator: '<S36>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S36>/Integral Gain'
   */
  ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE += 0.4 *
    ModeSwitch_HIL_B.max_vx * 0.004;
  if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE >= 3.43) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE = 3.43;
  } else if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE <= -3.43) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE = -3.43;
  }

  if (hold_z > 0) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRese = 1;
  } else {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRese = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S36>/Discrete-Time Integrator' */

  /* Update for UnitDelay: '<S39>/UD'
   *
   * Block description for '<S39>/UD':
   *
   *  Store in Global RAM
   */
  ModeSwitch_HIL_DW.UD_DSTATE = ModeSwitch_HIL_B.TSamp;

  /* Update for DiscreteIntegrator: '<S37>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S37>/Integral Gain'
   */
  ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_m += 0.4 *
    ModeSwitch_HIL_B.max_vy * 0.004;
  if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_m >= 3.43) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_m = 3.43;
  } else if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_m <= -3.43) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_m = -3.43;
  }

  if (hold_z > 0) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRe_h = 1;
  } else {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRe_h = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S37>/Discrete-Time Integrator' */

  /* Update for UnitDelay: '<S40>/UD'
   *
   * Block description for '<S40>/UD':
   *
   *  Store in Global RAM
   */
  ModeSwitch_HIL_DW.UD_DSTATE_g = ModeSwitch_HIL_B.TSamp_b;

  /* Update for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S27>/Integral Gain'
   */
  ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_b += 0.02 *
    ModeSwitch_HIL_B.DiscreteTimeIntegrator_n * 0.004;
  if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_b >= 0.3) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_b = 0.3;
  } else if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_b <= -0.3) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_b = -0.3;
  }

  if (hold_x > 0) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRe_g = 1;
  } else {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRe_g = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' */

  /* Update for UnitDelay: '<S30>/UD'
   *
   * Block description for '<S30>/UD':
   *
   *  Store in Global RAM
   */
  ModeSwitch_HIL_DW.UD_DSTATE_i = ModeSwitch_HIL_B.TSamp_k;

  /* Update for DiscreteIntegrator: '<S25>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S25>/Integral Gain'
   */
  ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_g += 0.02 *
    ModeSwitch_HIL_B.Sum_i * 0.004;
  if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_g >= 0.3) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_g = 0.3;
  } else if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_g <= -0.3) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_g = -0.3;
  }

  if (hold_x > 0) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevR_hk = 1;
  } else {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevR_hk = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S25>/Discrete-Time Integrator' */

  /* Update for UnitDelay: '<S29>/UD'
   *
   * Block description for '<S29>/UD':
   *
   *  Store in Global RAM
   */
  ModeSwitch_HIL_DW.UD_DSTATE_d = ModeSwitch_HIL_B.Sum_d;

  /* Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S28>/Integral Gain'
   */
  ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_c += 0.001 *
    ModeSwitch_HIL_B.Sum1 * 0.004;
  if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_c >= 0.2) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_c = 0.2;
  } else if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_c <= -0.2) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTATE_c = -0.2;
  }

  if (hold_y > 0) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevR_gt = 1;
  } else {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevR_gt = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator' */

  /* Update for UnitDelay: '<S31>/UD'
   *
   * Block description for '<S31>/UD':
   *
   *  Store in Global RAM
   */
  ModeSwitch_HIL_DW.UD_DSTATE_e = ModeSwitch_HIL_B.TSamp_i;

  /* Update for DiscreteIntegrator: '<S38>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S38>/Integral Gain'
   */
  ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTAT_c5 += 0.01 *
    ModeSwitch_HIL_B.max_vz * 0.004;
  if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTAT_c5 >= 5.0) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTAT_c5 = 5.0;
  } else if (ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTAT_c5 <= -5.0) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_DSTAT_c5 = -5.0;
  }

  if (rtb_reset_i_idx_3 > 0) {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRe_p = 1;
  } else {
    ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRe_p = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S38>/Discrete-Time Integrator' */

  /* Update for UnitDelay: '<S41>/UD'
   *
   * Block description for '<S41>/UD':
   *
   *  Store in Global RAM
   */
  ModeSwitch_HIL_DW.UD_DSTATE_j = ModeSwitch_HIL_B.TSamp_d;
  rate_scheduler();
}

/* Model initialize function */
void ModeSwitch_HIL_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)ModeSwitch_HIL_M, 0,
                sizeof(RT_MODEL_ModeSwitch_HIL_T));

  /* block I/O */
  (void) memset(((void *) &ModeSwitch_HIL_B), 0,
                sizeof(B_ModeSwitch_HIL_T));

  {
    ModeSwitch_HIL_B.Switch = SL_MODE_OFF;
    ModeSwitch_HIL_B.Switch1 = SL_COLOR_OFF;
  }

  /* states (dwork) */
  (void) memset((void *)&ModeSwitch_HIL_DW, 0,
                sizeof(DW_ModeSwitch_HIL_T));

  /* Start for S-Function (sfun_px4_input_rc): '<Root>/input_rc'
   *
   * Block description for '<Root>/input_rc':
   *  RC Input Block
   *
   *  This block provides user input control to the model.
   *  It uses the input_rc uORB topic.
   *
   *  The user has the ability to choose which channels are available as outputs from this block and also some optional outputs. These include
   *  Channels 1 through 18
   *  double data type indicating the PWM value from the controller
   *  measured pulse widths for each of the supported channels
   *  Channel Count
   *  uint32 data type of the number of channels which are detector by the PX4
   *  RC Failsafe
   *  boolean data types indicating that the RC Tx is sending the FailSafe signal (if equipped and properly setup)
   *  explicit failsafe flag: true on TX failure or TX out of range , false otherwise.
   *  Only the true state is reliable, as there are some (PPM) receivers on the market going into failsafe without telling us explicitly.
   *  RC Input Source
   *  Enumeration data type indicating which source the RC input is from.
   *  Valid values are found in the ENUM file: RC_INPUT_SOURCE_ENUM.m
   *            Enumeration members for class 'RC_INPUT_SOURCE_ENUM':
   *            RCINPUT_SOURCE_UNKNOWN         (0)
   *            RCINPUT_SOURCE_PX4FMU_PPM      (1)
   *            RCINPUT_SOURCE_PX4IO_PPM       (2)
   *            RCINPUT_SOURCE_PX4IO_SPEKTRUM  (3)
   *            RCINPUT_SOURCE_PX4IO_SBUS      (4)
   *            RCINPUT_SOURCE_PX4IO_ST24      (5)
   *            RCINPUT_SOURCE_MAVLINK         (6)
   *            RCINPUT_SOURCE_QURT            (7)
   *            RCINPUT_SOURCE_PX4FMU_SPEKTRUM (8)
   *            RCINPUT_SOURCE_PX4FMU_SBUS     (9)
   *            RCINPUT_SOURCE_PX4FMU_ST24     (10)
   *            RCINPUT_SOURCE_PX4FMU_SUMD     (11)
   *            RCINPUT_SOURCE_PX4FMU_DSM      (12)
   *            RCINPUT_SOURCE_PX4IO_SUMD      (13)
   *
   *  RSSI - Receive signal strength index
   *  receive signal strength indicator (RSSI): < 0: Undefined, 0: no signal, 255: full reception
   *  RC Lost Connection
   *  boolean data type indicating RC receiver connection status
   *  True, if no frame has arrived in the expected time, false otherwise.
   *  True usally means that the receiver has been disconnected, but can also indicate a radio link loss on "stupid" systems.
   *  Will remain false, if a RX with failsafe option continues to transmit frames after a link loss.
   *
   *  Sample Model: px4demo_input_rc.slx
   */
  {
    /* S-Function Block: <Root>/input_rc */
    /* subscribe to PWM RC input topic */
    int fd = orb_subscribe(ORB_ID(input_rc));
    ModeSwitch_HIL_DW.input_rc_input_rc_fd.fd = fd;
    ModeSwitch_HIL_DW.input_rc_input_rc_fd.events = POLLIN;
    orb_set_interval(fd, 1);
    PX4_INFO("* Subscribed to input_rc topic (fd = %d)*\n", fd);
  }

  /* Start for S-Function (sfun_px4_rgbled): '<Root>/RGB_LED' */
  {
    // enable RGBLED, set intitial mode and color
    // more devices will be 1, 2, etc
    ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s.led_mask = 0xff;
    ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s.mode = MODE_OFF;
    ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s.priority = 0;
    ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s.timestamp = hrt_absolute_time();
    ModeSwitch_HIL_DW.RGB_LED_orb_advert_t = orb_advertise_queue(ORB_ID
      (led_control), &ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s,
      LED_UORB_QUEUE_LENGTH);
  }

  /* Start for S-Function (sfun_px4_uorb_write): '<Root>/uORB Write' incorporates:
   *  Constant: '<Root>/Constant'
   */
  {
    /* S-Function Block: <Root>/uORB Write */
    /* Initializing topic: actuator_outputs */
    struct actuator_outputs_s initialize_topic;
    memset( &initialize_topic, 0, sizeof(initialize_topic));
    ModeSwitch_HIL_DW.uORBWrite_uorb_advert = orb_advertise(ORB_ID
      (actuator_outputs), &initialize_topic);
    if (ModeSwitch_HIL_DW.uORBWrite_uorb_advert != 0) {
      PX4_INFO("Started advertising actuator_outputs");
    }
  }

  /* Start for S-Function (sfun_px4_uorb_read_topic): '<S3>/uORB Read Function-Call Trigger' */
  {
    /* S-Function Block: <S3>/uORB Read Function-Call Trigger */
    /* subscribe to vehicle_local_position topic */
    int fd = orb_subscribe(ORB_ID(vehicle_local_position));
    ModeSwitch_HIL_DW.uORBReadFunctionCallTrigger_uOR.fd = fd;
    ModeSwitch_HIL_DW.uORBReadFunctionCallTrigger_uOR.events = POLLIN;
    orb_set_interval(fd, 1);
    PX4_INFO("* Subscribed to topic: vehicle_local_position (fd = %d)*\n", fd);
  }

  /* Start for S-Function (sfun_px4_sensor_combined): '<Root>/sensor_combined'
   *
   * Block description for '<Root>/sensor_combined':
   *  Sensor Combined Block
   *
   *  This block enables access to the various sensors available on the px4fmu-v2 hardware.
   *  The user can use these signals in the Simulink control model.
   *  The sample time needs to be provided in the mask dialog.
   *  Optional output ports can also be selected.
   *  Refer to the sample model: px4demo_attitude_control.slx
   *
   *  Signal definitions:
   *  Magnetometer (x,y,z) - single values – Magnetic field in NED body frame, in Gauss
   *  Accelerometer (x,y,z) - single values – Acceleration in NED body frame, in m/s^2
   *  Gyroscope (p,q,r) - single values – Angular velocity in radians per second
   *  Barometer (Altitude) - single value – Barometric pressure, already temperature compensated (millibars)
   *  RunTime (timestamp) - double value – Timestamp in microseconds since boot, from gyro
   *
   *  The sensor_combined block needs to have the px4io service running on the PX4 hardware in order to get valid signal values.

   */
  {
    /* S-Function Block: <Root>/sensor_combined */
    /* subscribe to sensor_combined topic */
    int fd = orb_subscribe(ORB_ID(sensor_combined));
    ModeSwitch_HIL_DW.sensor_combined_sensor_fd.fd = fd;
    ModeSwitch_HIL_DW.sensor_combined_sensor_fd.events = POLLIN;
    orb_set_interval(fd, 1);
    PX4_INFO("* Subscribed to sensor_combined topic (fd = %d)*\n", fd);
  }

  /* Start for S-Function (sfun_px4_vehicle_attitude): '<Root>/vehicle_attitude'
   *
   * Block description for '<Root>/vehicle_attitude':
   *  This block gives access to the running service that calculates the vehicle’s attitude.  A uORB topic (vehicle_attitude (attitude measurements)) publisher MUST be running in order for this block to provide valid signal values.  The available ones as of v1.3 are:
   *  attitude_estimator_ekf – EKF-Extended Kalman Filter for attitude estimation
   *  attitude_estimator_so3 – SO(3)-attitude estimation by using accelerometer, gyroscopes and magnetometer
   *  One of these MUST be running on the px4fmu in order for this block to return valid values.  Refer to the sample model: px4demo_attitude_control.slx. Attitude in NED (North-East-Down) body frame in SI units.
   *  Signal definitions:
   *  Roll – single value, Roll angle (rad, Tait-Bryan, NED)
   *  Pitch – single value, Pitch angle (rad, Tait-Bryan, NED)
   *  Yaw – single value, Yaw angle (rad, Tait-Bryan, NED)
   *  Quaternion (NED) – single(4) values (optional based on the uORB publisher)
   *
   */
  {
    /* S-Function Block: <Root>/vehicle_attitude */
    /* subscribe to PWM RC input topic */
    int fd = orb_subscribe(ORB_ID(vehicle_attitude));
    ModeSwitch_HIL_DW.vehicle_attitude_vehicle_attitu.fd = fd;
    ModeSwitch_HIL_DW.vehicle_attitude_vehicle_attitu.events = POLLIN;
    orb_set_interval(fd, 1);
    PX4_INFO("* Subscribed to vehicle_attitude topic (fd = %d)*\n", fd);
  }

  /* InitializeConditions for DiscreteIntegrator: '<S36>/Discrete-Time Integrator' */
  ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRese = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S37>/Discrete-Time Integrator' */
  ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRe_h = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' */
  ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRe_g = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S25>/Discrete-Time Integrator' */
  ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevR_hk = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S28>/Discrete-Time Integrator' */
  ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevR_gt = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S38>/Discrete-Time Integrator' */
  ModeSwitch_HIL_DW.DiscreteTimeIntegrator_PrevRe_p = 0;

  /* SystemInitialize for MATLAB Function: '<S1>/MATLAB Function2' */
  ModeSwitch_HIL_DW.z1_not_empty = false;

  /* '<S9>:1:13' x1 = 0; */
  ModeSwitch_HIL_DW.x1 = 0.0;

  /* '<S9>:1:17' y1 = 0; */
  ModeSwitch_HIL_DW.y1 = 0.0;

  /* '<S9>:1:25' hold_x_flag = 0; */
  ModeSwitch_HIL_DW.hold_x_flag = 0.0;

  /* '<S9>:1:29' hold_y_flag = 0; */
  ModeSwitch_HIL_DW.hold_y_flag = 0.0;

  /* '<S9>:1:33' hold_z_flag = 0; */
  ModeSwitch_HIL_DW.hold_z_flag = 0.0;

  /* '<S9>:1:40' waypoint_1_flag = 0; */
  ModeSwitch_HIL_DW.waypoint_1_flag = 0.0;

  /* '<S9>:1:45' waypoint_2_flag = 0; */
  ModeSwitch_HIL_DW.waypoint_2_flag = 0.0;

  /* '<S9>:1:50' waypoint_3_flag = 0; */
  ModeSwitch_HIL_DW.waypoint_3_flag = 0.0;

  /* '<S9>:1:55' waypoint_4_flag = 0; */
  ModeSwitch_HIL_DW.waypoint_4_flag = 0.0;
}

/* Model terminate function */
void ModeSwitch_HIL_terminate(void)
{
  /* Terminate for S-Function (sfun_px4_input_rc): '<Root>/input_rc'
   *
   * Block description for '<Root>/input_rc':
   *  RC Input Block
   *
   *  This block provides user input control to the model.
   *  It uses the input_rc uORB topic.
   *
   *  The user has the ability to choose which channels are available as outputs from this block and also some optional outputs. These include
   *  Channels 1 through 18
   *  double data type indicating the PWM value from the controller
   *  measured pulse widths for each of the supported channels
   *  Channel Count
   *  uint32 data type of the number of channels which are detector by the PX4
   *  RC Failsafe
   *  boolean data types indicating that the RC Tx is sending the FailSafe signal (if equipped and properly setup)
   *  explicit failsafe flag: true on TX failure or TX out of range , false otherwise.
   *  Only the true state is reliable, as there are some (PPM) receivers on the market going into failsafe without telling us explicitly.
   *  RC Input Source
   *  Enumeration data type indicating which source the RC input is from.
   *  Valid values are found in the ENUM file: RC_INPUT_SOURCE_ENUM.m
   *            Enumeration members for class 'RC_INPUT_SOURCE_ENUM':
   *            RCINPUT_SOURCE_UNKNOWN         (0)
   *            RCINPUT_SOURCE_PX4FMU_PPM      (1)
   *            RCINPUT_SOURCE_PX4IO_PPM       (2)
   *            RCINPUT_SOURCE_PX4IO_SPEKTRUM  (3)
   *            RCINPUT_SOURCE_PX4IO_SBUS      (4)
   *            RCINPUT_SOURCE_PX4IO_ST24      (5)
   *            RCINPUT_SOURCE_MAVLINK         (6)
   *            RCINPUT_SOURCE_QURT            (7)
   *            RCINPUT_SOURCE_PX4FMU_SPEKTRUM (8)
   *            RCINPUT_SOURCE_PX4FMU_SBUS     (9)
   *            RCINPUT_SOURCE_PX4FMU_ST24     (10)
   *            RCINPUT_SOURCE_PX4FMU_SUMD     (11)
   *            RCINPUT_SOURCE_PX4FMU_DSM      (12)
   *            RCINPUT_SOURCE_PX4IO_SUMD      (13)
   *
   *  RSSI - Receive signal strength index
   *  receive signal strength indicator (RSSI): < 0: Undefined, 0: no signal, 255: full reception
   *  RC Lost Connection
   *  boolean data type indicating RC receiver connection status
   *  True, if no frame has arrived in the expected time, false otherwise.
   *  True usally means that the receiver has been disconnected, but can also indicate a radio link loss on "stupid" systems.
   *  Will remain false, if a RX with failsafe option continues to transmit frames after a link loss.
   *
   *  Sample Model: px4demo_input_rc.slx
   */

  /* Close uORB service used in the S-Function Block: <Root>/input_rc */
  close(ModeSwitch_HIL_DW.input_rc_input_rc_fd.fd);

  /* Terminate for S-Function (sfun_px4_rgbled): '<Root>/RGB_LED' */

  /* Turn off LED */
  ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s.led_mask = 0xff;
  ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s.mode = MODE_OFF;
  ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s.priority = 0;
  ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s.timestamp = hrt_absolute_time();
  ModeSwitch_HIL_DW.RGB_LED_orb_advert_t = orb_advertise_queue(ORB_ID
    (led_control), &ModeSwitch_HIL_DW.RGB_LED_sl_led_control_s,
    LED_UORB_QUEUE_LENGTH);

  /* Close uORB service used in the S-Function Block: <Root>/RGB_LED */
  orb_unadvertise(ModeSwitch_HIL_DW.RGB_LED_orb_advert_t);

  /* Terminate for S-Function (sfun_px4_uorb_write): '<Root>/uORB Write' incorporates:
   *  Constant: '<Root>/Constant'
   */

  /* Close uORB service used in the S-Function Block: <Root>/uORB Write */
  orb_unadvertise(ModeSwitch_HIL_DW.uORBWrite_uorb_advert);

  /* Terminate for S-Function (sfun_px4_uorb_read_topic): '<S3>/uORB Read Function-Call Trigger' */

  /* Close uORB service used in the S-Function Block: <S3>/uORB Read Function-Call Trigger */
  close(ModeSwitch_HIL_DW.uORBReadFunctionCallTrigger_uOR.fd);

  /* Terminate for S-Function (sfun_px4_sensor_combined): '<Root>/sensor_combined'
   *
   * Block description for '<Root>/sensor_combined':
   *  Sensor Combined Block
   *
   *  This block enables access to the various sensors available on the px4fmu-v2 hardware.
   *  The user can use these signals in the Simulink control model.
   *  The sample time needs to be provided in the mask dialog.
   *  Optional output ports can also be selected.
   *  Refer to the sample model: px4demo_attitude_control.slx
   *
   *  Signal definitions:
   *  Magnetometer (x,y,z) - single values – Magnetic field in NED body frame, in Gauss
   *  Accelerometer (x,y,z) - single values – Acceleration in NED body frame, in m/s^2
   *  Gyroscope (p,q,r) - single values – Angular velocity in radians per second
   *  Barometer (Altitude) - single value – Barometric pressure, already temperature compensated (millibars)
   *  RunTime (timestamp) - double value – Timestamp in microseconds since boot, from gyro
   *
   *  The sensor_combined block needs to have the px4io service running on the PX4 hardware in order to get valid signal values.

   */

  /* Close uORB service used in the S-Function Block: <Root>/sensor_combined */
  close(ModeSwitch_HIL_DW.sensor_combined_sensor_fd.fd);

  /* Terminate for S-Function (sfun_px4_vehicle_attitude): '<Root>/vehicle_attitude'
   *
   * Block description for '<Root>/vehicle_attitude':
   *  This block gives access to the running service that calculates the vehicle’s attitude.  A uORB topic (vehicle_attitude (attitude measurements)) publisher MUST be running in order for this block to provide valid signal values.  The available ones as of v1.3 are:
   *  attitude_estimator_ekf – EKF-Extended Kalman Filter for attitude estimation
   *  attitude_estimator_so3 – SO(3)-attitude estimation by using accelerometer, gyroscopes and magnetometer
   *  One of these MUST be running on the px4fmu in order for this block to return valid values.  Refer to the sample model: px4demo_attitude_control.slx. Attitude in NED (North-East-Down) body frame in SI units.
   *  Signal definitions:
   *  Roll – single value, Roll angle (rad, Tait-Bryan, NED)
   *  Pitch – single value, Pitch angle (rad, Tait-Bryan, NED)
   *  Yaw – single value, Yaw angle (rad, Tait-Bryan, NED)
   *  Quaternion (NED) – single(4) values (optional based on the uORB publisher)
   *
   */

  /* Close uORB service used in the S-Function Block: <Root>/vehicle_attitude */
  close(ModeSwitch_HIL_DW.vehicle_attitude_vehicle_attitu.fd);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
