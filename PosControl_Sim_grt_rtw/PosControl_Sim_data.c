/*
 * PosControl_Sim_data.c
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

#include "PosControl_Sim.h"
#include "PosControl_Sim_private.h"

/* Block parameters (default storage) */
P_PosControl_Sim_T PosControl_Sim_P = {
  /* Variable: DEG2RAD
   * Referenced by:
   *   '<S6>/deg2rad1'
   *   '<S6>/deg2rad2'
   *   '<S6>/deg2rad3'
   *   '<S21>/Saturation1'
   *   '<S21>/Saturation2'
   */
  0.0174533,

  /* Variable: Kd_PITCH_AngleRate
   * Referenced by: '<S24>/Derivative Gain'
   */
  0.001,

  /* Variable: Kd_ROLL_AngleRate
   * Referenced by: '<S26>/Derivative Gain'
   */
  0.001,

  /* Variable: Kd_YAW_AngleRate
   * Referenced by: '<S27>/Derivative Gain'
   */
  0.0,

  /* Variable: Ki_PITCH_AngleRate
   * Referenced by: '<S24>/Integral Gain'
   */
  0.02,

  /* Variable: Ki_ROLL_AngleRate
   * Referenced by: '<S26>/Integral Gain'
   */
  0.02,

  /* Variable: Ki_YAW_AngleRate
   * Referenced by: '<S27>/Integral Gain'
   */
  0.01,

  /* Variable: Kp_PITCH_ANGLE
   * Referenced by: '<S23>/Gain'
   */
  6.5,

  /* Variable: Kp_PITCH_AngleRate
   * Referenced by: '<S24>/Gain'
   */
  0.1,

  /* Variable: Kp_ROLL_ANGLE
   * Referenced by: '<S25>/Gain'
   */
  6.5,

  /* Variable: Kp_ROLL_AngleRate
   * Referenced by: '<S26>/Gain'
   */
  0.1,

  /* Variable: Kp_YAW_AngleRate
   * Referenced by: '<S27>/Gain'
   */
  0.5,

  /* Variable: Kpxp
   * Referenced by: '<S32>/Gain1'
   */
  1.0,

  /* Variable: Kpyp
   * Referenced by: '<S33>/Gain1'
   */
  1.0,

  /* Variable: Kpzp
   * Referenced by: '<S34>/Gain1'
   */
  4.0,

  /* Variable: Kvxd
   * Referenced by: '<S35>/Derivative Gain'
   */
  0.01,

  /* Variable: Kvxi
   * Referenced by: '<S35>/Integral Gain'
   */
  0.4,

  /* Variable: Kvxp
   * Referenced by: '<S35>/Gain'
   */
  2.5,

  /* Variable: Kvyd
   * Referenced by: '<S36>/Derivative Gain'
   */
  0.01,

  /* Variable: Kvyi
   * Referenced by: '<S36>/Integral Gain'
   */
  0.4,

  /* Variable: Kvyp
   * Referenced by: '<S36>/Gain'
   */
  2.5,

  /* Variable: Kvzd
   * Referenced by: '<S37>/Derivative Gain'
   */
  0.005,

  /* Variable: Kvzi
   * Referenced by: '<S37>/Integral Gain'
   */
  0.01,

  /* Variable: Kvzp
   * Referenced by: '<S37>/Gain'
   */
  0.45,

  /* Variable: MAX_CONTROL_ANGLE_PITCH
   * Referenced by:
   *   '<S6>/Gain3'
   *   '<S21>/Saturation2'
   */
  35.0,

  /* Variable: MAX_CONTROL_ANGLE_RATE_PITCH
   * Referenced by: '<S20>/Saturation3'
   */
  220.0,

  /* Variable: MAX_CONTROL_ANGLE_RATE_ROLL
   * Referenced by: '<S20>/Saturation5'
   */
  220.0,

  /* Variable: MAX_CONTROL_ANGLE_RATE_Y
   * Referenced by: '<S6>/Gain2'
   */
  200.0,

  /* Variable: MAX_CONTROL_ANGLE_ROLL
   * Referenced by:
   *   '<S6>/Gain1'
   *   '<S21>/Saturation1'
   */
  35.0,

  /* Variable: MAX_CONTROL_VELOCITY_XY
   * Referenced by:
   *   '<S5>/max_vx'
   *   '<S5>/max_vy'
   *   '<S32>/Saturation3'
   *   '<S33>/Saturation3'
   */
  5.0,

  /* Variable: MAX_CONTROL_VELOCITY_Z
   * Referenced by:
   *   '<S5>/max_vz'
   *   '<S34>/Saturation3'
   */
  3.0,

  /* Variable: ModelInit_AngEuler
   * Referenced by: '<S50>/Constant'
   */
  { 0.0, 0.0, 0.0 },

  /* Variable: ModelInit_PosE
   * Referenced by: '<S50>/Constant1'
   */
  { 0.0, 0.0, -100.0 },

  /* Variable: ModelInit_Rads
   * Referenced by:
   *   '<S42>/Motor_Dynamics_1'
   *   '<S42>/Motor_Dynamics_2'
   *   '<S42>/Motor_Dynamics_3'
   *   '<S42>/Motor_Dynamics_4'
   */
  0.0,

  /* Variable: ModelInit_RateB
   * Referenced by: '<S49>/Constant1'
   */
  { 0.0, 0.0, 0.0 },

  /* Variable: ModelInit_VelB
   * Referenced by: '<S49>/Constant2'
   */
  { 0.0, 0.0, 0.0 },

  /* Variable: ModelParam_motorCr
   * Referenced by:
   *   '<S45>/Gain'
   *   '<S46>/Gain'
   *   '<S47>/Gain'
   *   '<S48>/Gain'
   */
  1148.0,

  /* Variable: ModelParam_motorJm
   * Referenced by: '<S41>/Jrp'
   */
  0.0001287,

  /* Variable: ModelParam_motorT
   * Referenced by:
   *   '<S42>/Motor_Dynamics_1'
   *   '<S42>/Motor_Dynamics_2'
   *   '<S42>/Motor_Dynamics_3'
   *   '<S42>/Motor_Dynamics_4'
   */
  0.02,

  /* Variable: ModelParam_motorWb
   * Referenced by:
   *   '<S45>/Constant'
   *   '<S46>/Constant'
   *   '<S47>/Constant'
   *   '<S48>/Constant'
   */
  -141.4,

  /* Variable: ModelParam_rotorCm
   * Referenced by: '<S41>/Constant4'
   */
  1.779E-7,

  /* Variable: ModelParam_rotorCt
   * Referenced by: '<S41>/Constant3'
   */
  1.105E-5,

  /* Variable: ModelParam_uavCCm
   * Referenced by: '<S41>/Constant1'
   */
  { 0.0035, 0.0039, 0.0034 },

  /* Variable: ModelParam_uavCd
   * Referenced by: '<S41>/Constant7'
   */
  0.055,

  /* Variable: ModelParam_uavJ
   * Referenced by: '<S49>/J '
   */
  { 0.0211, 0.0, 0.0, 0.0, 0.0219, 0.0, 0.0, 0.0, 0.0366 },

  /* Variable: ModelParam_uavMass
   * Referenced by:
   *   '<S2>/mass'
   *   '<S49>/ Mass '
   */
  1.4,

  /* Variable: ModelParam_uavR
   * Referenced by: '<S41>/Constant5'
   */
  0.225,

  /* Variable: Saturation_I_RP_Max
   * Referenced by:
   *   '<S24>/Discrete-Time Integrator'
   *   '<S26>/Discrete-Time Integrator'
   */
  0.3,

  /* Variable: Saturation_I_RP_Min
   * Referenced by:
   *   '<S24>/Discrete-Time Integrator'
   *   '<S26>/Discrete-Time Integrator'
   */
  -0.3,

  /* Variable: Saturation_I_Y_Max
   * Referenced by: '<S27>/Discrete-Time Integrator'
   */
  0.2,

  /* Variable: Saturation_I_Y_Min
   * Referenced by: '<S27>/Discrete-Time Integrator'
   */
  -0.2,

  /* Variable: Saturation_I_ah
   * Referenced by:
   *   '<S35>/Discrete-Time Integrator'
   *   '<S36>/Discrete-Time Integrator'
   */
  3.43,

  /* Variable: Saturation_I_az
   * Referenced by: '<S37>/Discrete-Time Integrator'
   */
  5.0,

  /* Variable: THR_HOVER
   * Referenced by:
   *   '<S6>/thr_hover '
   *   '<S21>/Constant'
   */
  0.609,

  /* Mask Parameter: DiscreteDerivative_ICPrevScaled
   * Referenced by: '<S38>/UD'
   */
  0.0,

  /* Mask Parameter: DiscreteDerivative_ICPrevScal_f
   * Referenced by: '<S39>/UD'
   */
  0.0,

  /* Mask Parameter: DiscreteDerivative_ICPrevScal_i
   * Referenced by: '<S29>/UD'
   */
  0.0,

  /* Mask Parameter: DiscreteDerivative_ICPrevScal_c
   * Referenced by: '<S28>/UD'
   */
  0.0,

  /* Mask Parameter: DiscreteDerivative_ICPrevSca_iz
   * Referenced by: '<S30>/UD'
   */
  0.0,

  /* Mask Parameter: DiscreteDerivative_ICPrevSca_f3
   * Referenced by: '<S40>/UD'
   */
  0.0,

  /* Mask Parameter: Sendnet_fdmPackettoFlightGear_D
   * Referenced by: '<S62>/UDP Send'
   */
  5502,

  /* Expression: 0.4
   * Referenced by: '<S21>/az'
   */
  0.4,

  /* Expression: -0.4
   * Referenced by: '<S21>/az'
   */
  -0.4,

  /* Expression: -1
   * Referenced by: '<S21>/Gain'
   */
  -1.0,

  /* Expression: 0.9
   * Referenced by: '<S21>/Saturation'
   */
  0.9,

  /* Expression: 0
   * Referenced by: '<S21>/Saturation'
   */
  0.0,

  /* Expression: 408
   * Referenced by: '<S62>/PacketSize'
   */
  408.0,

  /* Expression: 4
   * Referenced by: '<S3>/Constant'
   */
  4.0,

  /* Expression: 1
   * Referenced by: '<S42>/Motor_Dynamics_1'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S42>/Motor_Dynamics_2'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S42>/Motor_Dynamics_3'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S42>/Motor_Dynamics_4'
   */
  1.0,

  /* Expression: 30/pi
   * Referenced by: '<S2>/Gain'
   */
  9.5492965855137211,

  /* Expression: -1
   * Referenced by: '<S3>/Gain'
   */
  -1.0,

  /* Expression: 1
   * Referenced by: '<S3>/Gain5'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<S3>/Gain3'
   */
  -1.0,

  /* Expression: 1
   * Referenced by: '<S3>/Gain6'
   */
  1.0,

  /* Expression: 10
   * Referenced by: '<S3>/Gain1'
   */
  10.0,

  /* Expression: 0
   * Referenced by: '<S3>/Step'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S3>/Step'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S3>/Step'
   */
  1.0,

  /* Expression: 10/earthRadius
   * Referenced by: '<S63>/Gain1'
   */
  1.5696123057604772E-6,

  /* Expression: -2.1361
   * Referenced by: '<S63>/Constant2'
   */
  -2.1361,

  /* Expression: 1
   * Referenced by: '<S63>/Gain4'
   */
  1.0,

  /* Expression: 10/earthRadius
   * Referenced by: '<S63>/Gain2'
   */
  1.5696123057604772E-6,

  /* Expression: 0.65673
   * Referenced by: '<S63>/Constant3'
   */
  0.65673,

  /* Expression: -10
   * Referenced by: '<S63>/Gain8'
   */
  -10.0,

  /* Expression: 5
   * Referenced by: '<S63>/Constant1'
   */
  5.0,

  /* Expression: SampleTime
   * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
   */
  0.01,

  /* Expression: SimulationPace
   * Referenced by: '<S3>/Simulation Pace'
   */
  1.0,

  /* Expression: SleepMode
   * Referenced by: '<S3>/Simulation Pace'
   */
  2.0,

  /* Expression: OutputPaceError
   * Referenced by: '<S3>/Simulation Pace'
   */
  0.0,

  /* Expression: SampleTime
   * Referenced by: '<S3>/Simulation Pace'
   */
  -1.0,

  /* Expression: 2
   * Referenced by: '<S1>/Constant'
   */
  2.0,

  /* Expression: 1
   * Referenced by: '<S5>/Saturation8'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<S5>/Saturation8'
   */
  -1.0,

  /* Expression: 1
   * Referenced by: '<S5>/Saturation9'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<S5>/Saturation9'
   */
  -1.0,

  /* Expression: 1
   * Referenced by: '<S5>/Saturation10'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<S5>/Saturation10'
   */
  -1.0,

  /* Expression: 1
   * Referenced by: '<S6>/Constant'
   */
  1.0,

  /* Expression: 0.5
   * Referenced by: '<S6>/Gain'
   */
  0.5,

  /* Expression: 1
   * Referenced by: '<S6>/Saturation10'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S6>/Saturation10'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S6>/Saturation9'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<S6>/Saturation9'
   */
  -1.0,

  /* Expression: 1
   * Referenced by: '<S6>/Saturation8'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<S6>/Saturation8'
   */
  -1.0,

  /* Expression: 1
   * Referenced by: '<S6>/Saturation7'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<S6>/Saturation7'
   */
  -1.0,

  /* Computed Parameter: DiscreteTimeIntegrator_gainval
   * Referenced by: '<S35>/Discrete-Time Integrator'
   */
  0.001,

  /* Expression: 0
   * Referenced by: '<S35>/Discrete-Time Integrator'
   */
  0.0,

  /* Computed Parameter: TSamp_WtEt
   * Referenced by: '<S38>/TSamp'
   */
  1000.0,

  /* Computed Parameter: DiscreteTimeIntegrator_gainva_m
   * Referenced by: '<S36>/Discrete-Time Integrator'
   */
  0.001,

  /* Expression: 0
   * Referenced by: '<S36>/Discrete-Time Integrator'
   */
  0.0,

  /* Computed Parameter: TSamp_WtEt_n
   * Referenced by: '<S39>/TSamp'
   */
  1000.0,

  /* Expression: 2
   * Referenced by: '<S10>/Switch1'
   */
  2.0,

  /* Computed Parameter: DiscreteTimeIntegrator_gainva_h
   * Referenced by: '<S26>/Discrete-Time Integrator'
   */
  0.001,

  /* Expression: 0
   * Referenced by: '<S26>/Discrete-Time Integrator'
   */
  0.0,

  /* Computed Parameter: TSamp_WtEt_k
   * Referenced by: '<S29>/TSamp'
   */
  1000.0,

  /* Expression: 1
   * Referenced by: '<S20>/Saturation'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<S20>/Saturation'
   */
  -1.0,

  /* Expression: 2
   * Referenced by: '<S10>/Switch1'
   */
  2.0,

  /* Computed Parameter: DiscreteTimeIntegrator_gainva_g
   * Referenced by: '<S24>/Discrete-Time Integrator'
   */
  0.001,

  /* Expression: 0
   * Referenced by: '<S24>/Discrete-Time Integrator'
   */
  0.0,

  /* Computed Parameter: TSamp_WtEt_d
   * Referenced by: '<S28>/TSamp'
   */
  1000.0,

  /* Expression: 1
   * Referenced by: '<S20>/Saturation1'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<S20>/Saturation1'
   */
  -1.0,

  /* Computed Parameter: DiscreteTimeIntegrator_gainva_c
   * Referenced by: '<S27>/Discrete-Time Integrator'
   */
  0.001,

  /* Expression: 0
   * Referenced by: '<S27>/Discrete-Time Integrator'
   */
  0.0,

  /* Computed Parameter: TSamp_WtEt_kk
   * Referenced by: '<S30>/TSamp'
   */
  1000.0,

  /* Expression: 0.5
   * Referenced by: '<S20>/Saturation2'
   */
  0.5,

  /* Expression: -0.5
   * Referenced by: '<S20>/Saturation2'
   */
  -0.5,

  /* Computed Parameter: DiscreteTimeIntegrator_gainva_j
   * Referenced by: '<S37>/Discrete-Time Integrator'
   */
  0.001,

  /* Expression: 0
   * Referenced by: '<S37>/Discrete-Time Integrator'
   */
  0.0,

  /* Computed Parameter: TSamp_WtEt_b
   * Referenced by: '<S40>/TSamp'
   */
  1000.0,

  /* Expression: 1
   * Referenced by: '<S10>/Switch2'
   */
  1.0,

  /* Expression: 2000
   * Referenced by: '<S10>/Output_Limits1'
   */
  2000.0,

  /* Expression: 1000
   * Referenced by: '<S10>/Output_Limits1'
   */
  1000.0,

  /* Expression: [1000,1000,1000,1000]
   * Referenced by: '<S2>/Constant2'
   */
  { 1000.0, 1000.0, 1000.0, 1000.0 },

  /* Expression: [0,0,ModelParam_envGravityAcc]
   * Referenced by: '<S2>/g'
   */
  { 0.0, 0.0, 9.8 },

  /* Expression: 1/1000
   * Referenced by: '<S2>/Gain1'
   */
  0.001,

  /* Expression: 1
   * Referenced by: '<S2>/Saturation'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S2>/Saturation'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S42>/Signal_Saturation_1'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S42>/Signal_Saturation_1'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S42>/Signal_Saturation_2'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S42>/Signal_Saturation_2'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S42>/Signal_Saturation_3'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S42>/Signal_Saturation_3'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S42>/Signal_Saturation_4'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S42>/Signal_Saturation_4'
   */
  0.0,

  /* Expression: eye(3)
   * Referenced by: '<S49>/Constant'
   */
  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  /* Computed Parameter: Packnet_fdmPacketforFlightGea_l
   * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
   */
  { 50U, 52U },

  /* Computed Parameter: Packnet_fdmPacketforFlightGea_j
   * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
   */
  { 111U, 110U },

  /* Computed Parameter: Packnet_fdmPacketforFlightGe_ls
   * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
   */
  { 111U, 102U, 102U },

  /* Computed Parameter: Packnet_fdmPacketforFlightGea_n
   * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
   */
  { 111U, 110U },

  /* Computed Parameter: Packnet_fdmPacketforFlightGea_c
   * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
   */
  { 111U, 110U },

  /* Computed Parameter: Packnet_fdmPacketforFlightGea_i
   * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
   */
  { 111U, 102U, 102U },

  /* Computed Parameter: Packnet_fdmPacketforFlightGea_f
   * Referenced by: '<S3>/Pack net_fdm Packet for FlightGear'
   */
  { 111U, 102U, 102U },

  /* Computed Parameter: _Value
   * Referenced by: '<Root>/          '
   */
  1500U,

  /* Computed Parameter: u_Value
   * Referenced by: '<Root>/   1'
   */
  1500U,

  /* Computed Parameter: u_Value_o
   * Referenced by: '<Root>/   2'
   */
  1500U,

  /* Computed Parameter: u_Value_b
   * Referenced by: '<Root>/   3'
   */
  1500U,

  /* Computed Parameter: RateTransition_InitialCondition
   * Referenced by: '<S3>/Rate Transition'
   */
  0U
};
