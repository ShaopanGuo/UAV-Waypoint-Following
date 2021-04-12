/*
 * PosControl_Sim.c
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

/* Block signals (default storage) */
B_PosControl_Sim_T PosControl_Sim_B;

/* Continuous states */
X_PosControl_Sim_T PosControl_Sim_X;

/* Block states (default storage) */
DW_PosControl_Sim_T PosControl_Sim_DW;

/* Real-time model */
RT_MODEL_PosControl_Sim_T PosControl_Sim_M_;
RT_MODEL_PosControl_Sim_T *const PosControl_Sim_M = &PosControl_Sim_M_;
static void rate_scheduler(void);

/* Byte swap algorithm for 32-bit values */
uint32_T rt_htonl(uint32_T val)
{
  uint32_T valnet;
  valnet = ((val ) >> 24 ) |
    ((val & 0x00FF0000) >> 8 ) |
    ((val & 0x0000FF00) << 8 ) |
    ((val ) << 24 );
  return(valnet);
}

/* Endian determination utility */
boolean_T rt_IsLittleEndian( void )
{
  uint32_T endian = 1;
  int8_T *bytes = (int8_T *) &endian;
  return ((boolean_T)(bytes[0] == 1));
}

/* FlightGear v24 data packet function */
void asbFg24net_fdm( uint8_T *out )
{
  uint32_T *dest = (uint32_T *) out;
  if (rt_IsLittleEndian()) {
    *dest = rt_htonl(24U);
  } else {
    *dest = 24U;
  }
}

/* FlightGear 24 data group 1 set function */
void asbFg24SetGrp1( uint8_T *out, const real_T longitudeVal, const real_T
                    latitudeVal, const real_T altitudeVal, const real32_T phiVal,
                    const real32_T thetaVal, const real32_T psiVal )
{
  if (rt_IsLittleEndian()) {
    rt_htond( &out[ 8], longitudeVal );
    rt_htond( &out[ 16], latitudeVal );
    rt_htond( &out[ 24], altitudeVal );
    rt_htonf( &out[ 36], phiVal );
    rt_htonf( &out[ 40], thetaVal );
    rt_htonf( &out[ 44], psiVal );
  } else {
    rt_htondbe( &out[ 8], longitudeVal );
    rt_htondbe( &out[ 16], latitudeVal );
    rt_htondbe( &out[ 24], altitudeVal );
    rt_htonfbe( &out[ 36], phiVal );
    rt_htonfbe( &out[ 40], thetaVal );
    rt_htonfbe( &out[ 44], psiVal );
  }
}

/* FlightGear 24 data group 3 set function */
void asbFg24SetGrp3( uint8_T *out, const real32_T elevatorVal, const real32_T
                    elevator_trim_tabVal, const real32_T left_flapVal, const
                    real32_T right_flapVal, const real32_T left_aileronVal,
                    const real32_T right_aileronVal, const real32_T rudderVal,
                    const real32_T nose_wheelVal, const real32_T speedbrakeVal,
                    const real32_T spoilersVal )
{
  if (rt_IsLittleEndian()) {
    rt_htonf( &out[368], elevatorVal );
    rt_htonf( &out[372], elevator_trim_tabVal );
    rt_htonf( &out[376], left_flapVal );
    rt_htonf( &out[380], right_flapVal );
    rt_htonf( &out[384], left_aileronVal );
    rt_htonf( &out[388], right_aileronVal );
    rt_htonf( &out[392], rudderVal );
    rt_htonf( &out[396], nose_wheelVal );
    rt_htonf( &out[400], speedbrakeVal );
    rt_htonf( &out[404], spoilersVal );
  } else {
    rt_htonfbe( &out[368], elevatorVal );
    rt_htonfbe( &out[372], elevator_trim_tabVal );
    rt_htonfbe( &out[376], left_flapVal );
    rt_htonfbe( &out[380], right_flapVal );
    rt_htonfbe( &out[384], left_aileronVal );
    rt_htonfbe( &out[388], right_aileronVal );
    rt_htonfbe( &out[392], rudderVal );
    rt_htonfbe( &out[396], nose_wheelVal );
    rt_htonfbe( &out[400], speedbrakeVal );
    rt_htonfbe( &out[404], spoilersVal );
  }
}

/* FlightGear 24 data group 4 set function */
void asbFg24SetGrp4( uint8_T *out, const uint32_T num_enginesVal, const uint32_T
                    *eng_stateVal, const real32_T *rpmVal, const real32_T
                    *fuel_flowVal, const real32_T *fuel_pxVal, const real32_T
                    *egtVal, const real32_T *chtVal, const real32_T *mp_osiVal,
                    const real32_T *titVal, const real32_T *oil_tempVal, const
                    real32_T *oil_pxVal, const uint32_T num_tanksVal, const
                    real32_T *fuel_quantityVal )
{
  if (rt_IsLittleEndian()) {
    rt_htonu32( &out[120], num_enginesVal );
    rt_htonu32( &out[124], eng_stateVal[0] );
    rt_htonu32( &out[128], eng_stateVal[1] );
    rt_htonu32( &out[132], eng_stateVal[2] );
    rt_htonu32( &out[136], eng_stateVal[3] );
    rt_htonf( &out[140], rpmVal[0] );
    rt_htonf( &out[144], rpmVal[1] );
    rt_htonf( &out[148], rpmVal[2] );
    rt_htonf( &out[152], rpmVal[3] );
    rt_htonf( &out[156], fuel_flowVal[0] );
    rt_htonf( &out[160], fuel_flowVal[1] );
    rt_htonf( &out[164], fuel_flowVal[2] );
    rt_htonf( &out[168], fuel_flowVal[3] );
    rt_htonf( &out[172], fuel_pxVal[0] );
    rt_htonf( &out[176], fuel_pxVal[1] );
    rt_htonf( &out[180], fuel_pxVal[2] );
    rt_htonf( &out[184], fuel_pxVal[3] );
    rt_htonf( &out[188], egtVal[0] );
    rt_htonf( &out[192], egtVal[1] );
    rt_htonf( &out[196], egtVal[2] );
    rt_htonf( &out[200], egtVal[3] );
    rt_htonf( &out[204], chtVal[0] );
    rt_htonf( &out[208], chtVal[1] );
    rt_htonf( &out[212], chtVal[2] );
    rt_htonf( &out[216], chtVal[3] );
    rt_htonf( &out[220], mp_osiVal[0] );
    rt_htonf( &out[224], mp_osiVal[1] );
    rt_htonf( &out[228], mp_osiVal[2] );
    rt_htonf( &out[232], mp_osiVal[3] );
    rt_htonf( &out[236], titVal[0] );
    rt_htonf( &out[240], titVal[1] );
    rt_htonf( &out[244], titVal[2] );
    rt_htonf( &out[248], titVal[3] );
    rt_htonf( &out[252], oil_tempVal[0] );
    rt_htonf( &out[256], oil_tempVal[1] );
    rt_htonf( &out[260], oil_tempVal[2] );
    rt_htonf( &out[264], oil_tempVal[3] );
    rt_htonf( &out[268], oil_pxVal[0] );
    rt_htonf( &out[272], oil_pxVal[1] );
    rt_htonf( &out[276], oil_pxVal[2] );
    rt_htonf( &out[280], oil_pxVal[3] );
    rt_htonu32( &out[284], num_tanksVal );
    rt_htonf( &out[288], fuel_quantityVal[0] );
    rt_htonf( &out[292], fuel_quantityVal[1] );
    rt_htonf( &out[296], fuel_quantityVal[2] );
    rt_htonf( &out[300], fuel_quantityVal[3] );
  } else {
    rt_htonu32be( &out[120], num_enginesVal );
    rt_htonu32be( &out[124], eng_stateVal[0] );
    rt_htonu32be( &out[128], eng_stateVal[1] );
    rt_htonu32be( &out[132], eng_stateVal[2] );
    rt_htonu32be( &out[136], eng_stateVal[3] );
    rt_htonfbe( &out[140], rpmVal[0] );
    rt_htonfbe( &out[144], rpmVal[1] );
    rt_htonfbe( &out[148], rpmVal[2] );
    rt_htonfbe( &out[152], rpmVal[3] );
    rt_htonfbe( &out[156], fuel_flowVal[0] );
    rt_htonfbe( &out[160], fuel_flowVal[1] );
    rt_htonfbe( &out[164], fuel_flowVal[2] );
    rt_htonfbe( &out[168], fuel_flowVal[3] );
    rt_htonfbe( &out[172], fuel_pxVal[0] );
    rt_htonfbe( &out[176], fuel_pxVal[1] );
    rt_htonfbe( &out[180], fuel_pxVal[2] );
    rt_htonfbe( &out[184], fuel_pxVal[3] );
    rt_htonfbe( &out[188], egtVal[0] );
    rt_htonfbe( &out[192], egtVal[1] );
    rt_htonfbe( &out[196], egtVal[2] );
    rt_htonfbe( &out[200], egtVal[3] );
    rt_htonfbe( &out[204], chtVal[0] );
    rt_htonfbe( &out[208], chtVal[1] );
    rt_htonfbe( &out[212], chtVal[2] );
    rt_htonfbe( &out[216], chtVal[3] );
    rt_htonfbe( &out[220], mp_osiVal[0] );
    rt_htonfbe( &out[224], mp_osiVal[1] );
    rt_htonfbe( &out[228], mp_osiVal[2] );
    rt_htonfbe( &out[232], mp_osiVal[3] );
    rt_htonfbe( &out[236], titVal[0] );
    rt_htonfbe( &out[240], titVal[1] );
    rt_htonfbe( &out[244], titVal[2] );
    rt_htonfbe( &out[248], titVal[3] );
    rt_htonfbe( &out[252], oil_tempVal[0] );
    rt_htonfbe( &out[256], oil_tempVal[1] );
    rt_htonfbe( &out[260], oil_tempVal[2] );
    rt_htonfbe( &out[264], oil_tempVal[3] );
    rt_htonfbe( &out[268], oil_pxVal[0] );
    rt_htonfbe( &out[272], oil_pxVal[1] );
    rt_htonfbe( &out[276], oil_pxVal[2] );
    rt_htonfbe( &out[280], oil_pxVal[3] );
    rt_htonu32be( &out[284], num_tanksVal );
    rt_htonfbe( &out[288], fuel_quantityVal[0] );
    rt_htonfbe( &out[292], fuel_quantityVal[1] );
    rt_htonfbe( &out[296], fuel_quantityVal[2] );
    rt_htonfbe( &out[300], fuel_quantityVal[3] );
  }
}

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
  (PosControl_Sim_M->Timing.TaskCounters.TID[2])++;
  if ((PosControl_Sim_M->Timing.TaskCounters.TID[2]) > 4) {/* Sample time: [0.005s, 0.0s] */
    PosControl_Sim_M->Timing.TaskCounters.TID[2] = 0;
  }

  (PosControl_Sim_M->Timing.TaskCounters.TID[3])++;
  if ((PosControl_Sim_M->Timing.TaskCounters.TID[3]) > 9) {/* Sample time: [0.01s, 0.0s] */
    PosControl_Sim_M->Timing.TaskCounters.TID[3] = 0;
  }
}

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 17;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  PosControl_Sim_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  PosControl_Sim_step();
  PosControl_Sim_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  PosControl_Sim_step();
  PosControl_Sim_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  PosControl_Sim_step();
  PosControl_Sim_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
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
void PosControl_Sim_deadzone1(real_T rtu_u, B_deadzone1_PosControl_Sim_T *localB)
{
  real_T u;
  u = rtu_u;

  /* MATLAB Function 'Control System/InputConditioning/deadzone1': '<S11>:1' */
  /* '<S11>:1:6' */
  /* '<S11>:1:7' */
  /* '<S11>:1:8' */
  /* '<S11>:1:10' */
  /* '<S11>:1:11' */
  if (rtu_u < 1100.0) {
    /* '<S11>:1:13' */
    /* '<S11>:1:14' */
    u = 1100.0;
  } else {
    if (rtu_u > 1900.0) {
      /* '<S11>:1:15' */
      /* '<S11>:1:16' */
      u = 1900.0;
    }
  }

  if (u > 1540.0) {
    /* '<S11>:1:19' */
    /* '<S11>:1:20' */
    localB->y = ((u - 1500.0) - 40.0) * 0.0027777777777777779;
  } else if (u < 1460.0) {
    /* '<S11>:1:21' */
    /* '<S11>:1:22' */
    localB->y = ((u - 1500.0) + 40.0) * 0.0027777777777777779;
  } else {
    /* '<S11>:1:24' */
    localB->y = 0.0;
  }
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(u0_0, u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

void rt_mrdivided3x3_snf(const real_T u0[9], const real_T u1[9], real_T y[9])
{
  real_T A[9];
  int32_T r1;
  int32_T r2;
  int32_T r3;
  real_T maxval;
  real_T a21;
  int32_T rtemp;
  int32_T y_tmp;
  real_T y_tmp_0;
  int32_T y_tmp_1;
  real_T y_tmp_2;
  real_T y_tmp_3;
  real_T y_tmp_4;
  memcpy(&A[0], &u1[0], 9U * sizeof(real_T));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(u1[0]);
  a21 = fabs(u1[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabs(u1[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  A[r2] = u1[r2] / u1[r1];
  A[r3] /= A[r1];
  A[r2 + 3] -= A[r1 + 3] * A[r2];
  A[r3 + 3] -= A[r1 + 3] * A[r3];
  A[r2 + 6] -= A[r1 + 6] * A[r2];
  A[r3 + 6] -= A[r1 + 6] * A[r3];
  if (fabs(A[r3 + 3]) > fabs(A[r2 + 3])) {
    rtemp = r2 + 1;
    r2 = r3;
    r3 = rtemp - 1;
  }

  A[r3 + 3] /= A[r2 + 3];
  A[r3 + 6] -= A[r3 + 3] * A[r2 + 6];
  y[3 * r1] = u0[0] / A[r1];
  maxval = A[r1 + 3];
  y[3 * r2] = u0[3] - y[3 * r1] * maxval;
  a21 = A[r1 + 6];
  y[3 * r3] = u0[6] - y[3 * r1] * a21;
  y_tmp_0 = A[r2 + 3];
  y[3 * r2] /= y_tmp_0;
  y_tmp_2 = A[r2 + 6];
  y[3 * r3] -= y[3 * r2] * y_tmp_2;
  y_tmp_3 = A[r3 + 6];
  y[3 * r3] /= y_tmp_3;
  y_tmp_4 = A[r3 + 3];
  y[3 * r2] -= y[3 * r3] * y_tmp_4;
  y[3 * r1] -= y[3 * r3] * A[r3];
  y[3 * r1] -= y[3 * r2] * A[r2];
  rtemp = 3 * r1 + 1;
  y[rtemp] = u0[1] / A[r1];
  y_tmp = 3 * r2 + 1;
  y[y_tmp] = u0[4] - y[rtemp] * maxval;
  y_tmp_1 = 3 * r3 + 1;
  y[y_tmp_1] = u0[7] - y[rtemp] * a21;
  y[y_tmp] /= y_tmp_0;
  y[y_tmp_1] -= y[y_tmp] * y_tmp_2;
  y[y_tmp_1] /= y_tmp_3;
  y[y_tmp] -= y[y_tmp_1] * y_tmp_4;
  y[rtemp] -= y[y_tmp_1] * A[r3];
  y[rtemp] -= y[y_tmp] * A[r2];
  rtemp = 3 * r1 + 2;
  y[rtemp] = u0[2] / A[r1];
  y_tmp = 3 * r2 + 2;
  y[y_tmp] = u0[5] - y[rtemp] * maxval;
  y_tmp_1 = 3 * r3 + 2;
  y[y_tmp_1] = u0[8] - y[rtemp] * a21;
  y[y_tmp] /= y_tmp_0;
  y[y_tmp_1] -= y[y_tmp] * y_tmp_2;
  y[y_tmp_1] /= y_tmp_3;
  y[y_tmp] -= y[y_tmp_1] * y_tmp_4;
  y[rtemp] -= y[y_tmp_1] * A[r3];
  y[rtemp] -= y[y_tmp] * A[r2];
}

/* Model step function */
void PosControl_Sim_step(void)
{
  /* local block i/o variables */
  real_T rtb_Conversion1;
  real_T rtb_Conversion2;
  real_T rtb_Conversion3;
  real_T rtb_Conversion4;
  real_T rtb_TSamp;
  real_T rtb_TSamp_j;
  real_T rtb_TSamp_k;
  real_T rtb_TSamp_a;
  real_T rtb_TSamp_e;
  real_T rtb_TSamp_b;
  int32_T x1;
  int32_T b_y1;
  real_T rtb_ixj;
  real_T rtb_Sum_aw[3];
  real_T rtb_kxi;
  real_T rtb_ixk;
  real_T rtb_jxi;
  real_T rtb_max_vx;
  real_T rtb_max_vy;
  real_T rtb_max_vz;
  real_T rtb_Mp[3];
  real_T rtb_Motor_Dynamics_1;
  real_T rtb_Motor_Dynamics_2;
  real_T rtb_Motor_Dynamics_3;
  real_T rtb_Motor_Dynamics_4;
  real_T rtb_Sum_p;
  real_T rtb_Reb[9];
  real_T tmp[12];
  real_T tmp_0[3];
  real_T tmp_1[16];
  real_T rtb_Divide_idx_0;
  real_T rtb_Divide_idx_1;
  real_T rtb_Divide_idx_2;
  real_T rtb_Divide_idx_3;
  real_T y_tmp;
  real_T y_tmp_0;
  real_T y_tmp_1;
  real_T y_tmp_2;
  real_T y_tmp_3;
  real_T y_tmp_4;
  real_T y_tmp_5;
  if (rtmIsMajorTimeStep(PosControl_Sim_M)) {
    /* set solver stop time */
    if (!(PosControl_Sim_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&PosControl_Sim_M->solverInfo,
                            ((PosControl_Sim_M->Timing.clockTickH0 + 1) *
        PosControl_Sim_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&PosControl_Sim_M->solverInfo,
                            ((PosControl_Sim_M->Timing.clockTick0 + 1) *
        PosControl_Sim_M->Timing.stepSize0 +
        PosControl_Sim_M->Timing.clockTickH0 *
        PosControl_Sim_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(PosControl_Sim_M)) {
    PosControl_Sim_M->Timing.t[0] = rtsiGetT(&PosControl_Sim_M->solverInfo);
  }

  /* RateTransition: '<S3>/Rate Transition' */
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[2] == 0) {
    if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
        PosControl_Sim_M->Timing.TaskCounters.TID[3] == 0) {
      memcpy(&PosControl_Sim_B.RateTransition[0],
             &PosControl_Sim_DW.RateTransition_Buffer0[0], 408U * sizeof(uint8_T));
    }
  }

  /* End of RateTransition: '<S3>/Rate Transition' */
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[3] == 0) {
    /* DataTypeConversion: '<S3>/Data Type Conversion4' incorporates:
     *  Constant: '<S3>/Constant'
     */
    rtb_ixj = floor(PosControl_Sim_P.Constant_Value);
    if (rtIsNaN(rtb_ixj) || rtIsInf(rtb_ixj)) {
      rtb_ixj = 0.0;
    } else {
      rtb_ixj = fmod(rtb_ixj, 4.294967296E+9);
    }

    PosControl_Sim_B.DataTypeConversion4 = rtb_ixj < 0.0 ? (uint32_T)-(int32_T)
      (uint32_T)-rtb_ixj : (uint32_T)rtb_ixj;

    /* End of DataTypeConversion: '<S3>/Data Type Conversion4' */
  }

  /* StateSpace: '<S42>/Motor_Dynamics_1' */
  rtb_Motor_Dynamics_1 = 1.0 / PosControl_Sim_P.ModelParam_motorT *
    PosControl_Sim_X.Motor_Dynamics_1_CSTATE;

  /* StateSpace: '<S42>/Motor_Dynamics_2' */
  rtb_Motor_Dynamics_2 = 1.0 / PosControl_Sim_P.ModelParam_motorT *
    PosControl_Sim_X.Motor_Dynamics_2_CSTATE;

  /* StateSpace: '<S42>/Motor_Dynamics_3' */
  rtb_Motor_Dynamics_3 = 1.0 / PosControl_Sim_P.ModelParam_motorT *
    PosControl_Sim_X.Motor_Dynamics_3_CSTATE;

  /* StateSpace: '<S42>/Motor_Dynamics_4' */
  rtb_Motor_Dynamics_4 = 1.0 / PosControl_Sim_P.ModelParam_motorT *
    PosControl_Sim_X.Motor_Dynamics_4_CSTATE;

  /* DataTypeConversion: '<S3>/Data Type Conversion5' incorporates:
   *  Gain: '<S2>/Gain'
   *  Gain: '<S3>/Gain'
   *  Gain: '<S3>/Gain1'
   *  Gain: '<S3>/Gain3'
   *  Gain: '<S3>/Gain5'
   *  Gain: '<S3>/Gain6'
   */
  PosControl_Sim_B.DataTypeConversion5[0] = (real32_T)
    (PosControl_Sim_P.Gain_Gain_j * rtb_Motor_Dynamics_1 *
     PosControl_Sim_P.Gain_Gain_b * PosControl_Sim_P.Gain1_Gain);
  PosControl_Sim_B.DataTypeConversion5[1] = (real32_T)
    (PosControl_Sim_P.Gain_Gain_j * rtb_Motor_Dynamics_2 *
     PosControl_Sim_P.Gain5_Gain * PosControl_Sim_P.Gain1_Gain);
  PosControl_Sim_B.DataTypeConversion5[2] = (real32_T)
    (PosControl_Sim_P.Gain_Gain_j * rtb_Motor_Dynamics_3 *
     PosControl_Sim_P.Gain3_Gain * PosControl_Sim_P.Gain1_Gain);
  PosControl_Sim_B.DataTypeConversion5[3] = (real32_T)
    (PosControl_Sim_P.Gain_Gain_j * rtb_Motor_Dynamics_4 *
     PosControl_Sim_P.Gain6_Gain * PosControl_Sim_P.Gain1_Gain);
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S50>/Euler angle to quaternion' incorporates:
     *  Constant: '<S50>/Constant'
     */
    /* MATLAB Function 'Quadcopter Dynamics/Rigid Model/Rigid-Body Kinematic Model /Euler angle to quaternion': '<S58>:1' */
    /* '<S58>:1:4' */
    /* '<S58>:1:5' */
    /* '<S58>:1:6' */
    /* '<S58>:1:7' */
    /* '<S58>:1:8' */
    rtb_ixj = sin(PosControl_Sim_P.ModelInit_AngEuler[0] / 2.0);
    rtb_Sum_p = cos(PosControl_Sim_P.ModelInit_AngEuler[1] / 2.0);
    rtb_ixk = cos(PosControl_Sim_P.ModelInit_AngEuler[2] / 2.0);
    rtb_jxi = cos(PosControl_Sim_P.ModelInit_AngEuler[0] / 2.0);
    rtb_kxi = sin(PosControl_Sim_P.ModelInit_AngEuler[1] / 2.0);
    rtb_Divide_idx_0 = sin(PosControl_Sim_P.ModelInit_AngEuler[2] / 2.0);
    PosControl_Sim_B.q[0] = rtb_jxi * rtb_Sum_p * rtb_ixk + rtb_ixj * rtb_kxi *
      rtb_Divide_idx_0;
    PosControl_Sim_B.q[1] = rtb_ixj * rtb_Sum_p * rtb_ixk - rtb_jxi * rtb_kxi *
      rtb_Divide_idx_0;
    PosControl_Sim_B.q[2] = cos(PosControl_Sim_P.ModelInit_AngEuler[0] / 2.0) *
      sin(PosControl_Sim_P.ModelInit_AngEuler[1] / 2.0) * rtb_ixk + sin
      (PosControl_Sim_P.ModelInit_AngEuler[0] / 2.0) * cos
      (PosControl_Sim_P.ModelInit_AngEuler[1] / 2.0) * rtb_Divide_idx_0;
    PosControl_Sim_B.q[3] = cos(PosControl_Sim_P.ModelInit_AngEuler[0] / 2.0) *
      cos(PosControl_Sim_P.ModelInit_AngEuler[1] / 2.0) * rtb_Divide_idx_0 - sin
      (PosControl_Sim_P.ModelInit_AngEuler[0] / 2.0) * sin
      (PosControl_Sim_P.ModelInit_AngEuler[1] / 2.0) * rtb_ixk;
  }

  /* Integrator: '<S50>/q' */
  if (PosControl_Sim_DW.q_IWORK != 0) {
    PosControl_Sim_X.q_CSTATE[0] = PosControl_Sim_B.q[0];
    PosControl_Sim_X.q_CSTATE[1] = PosControl_Sim_B.q[1];
    PosControl_Sim_X.q_CSTATE[2] = PosControl_Sim_B.q[2];
    PosControl_Sim_X.q_CSTATE[3] = PosControl_Sim_B.q[3];
  }

  /* Sqrt: '<S50>/Sqrt' incorporates:
   *  DotProduct: '<S50>/Dot Product'
   *  Integrator: '<S50>/q'
   */
  rtb_ixj = sqrt(((PosControl_Sim_X.q_CSTATE[0] * PosControl_Sim_X.q_CSTATE[0] +
                   PosControl_Sim_X.q_CSTATE[1] * PosControl_Sim_X.q_CSTATE[1])
                  + PosControl_Sim_X.q_CSTATE[2] * PosControl_Sim_X.q_CSTATE[2])
                 + PosControl_Sim_X.q_CSTATE[3] * PosControl_Sim_X.q_CSTATE[3]);

  /* Product: '<S50>/Divide' incorporates:
   *  Integrator: '<S50>/q'
   */
  rtb_Divide_idx_0 = PosControl_Sim_X.q_CSTATE[0] / rtb_ixj;
  rtb_Divide_idx_1 = PosControl_Sim_X.q_CSTATE[1] / rtb_ixj;
  rtb_Divide_idx_2 = PosControl_Sim_X.q_CSTATE[2] / rtb_ixj;
  rtb_Divide_idx_3 = PosControl_Sim_X.q_CSTATE[3] / rtb_ixj;

  /* MATLAB Function: '<S50>/Quaternion to Euler' incorporates:
   *  MATLAB Function: '<S50>/Quaternion to rotation matrix'
   */
  /* MATLAB Function 'Quadcopter Dynamics/Rigid Model/Rigid-Body Kinematic Model /Quaternion to Euler': '<S59>:1' */
  /* '<S59>:1:4' */
  /* '<S59>:1:5' */
  /* '<S59>:1:6' */
  /* '<S59>:1:7' */
  rtb_Sum_p = rtb_Divide_idx_2 * rtb_Divide_idx_2;
  rtb_ixk = rtb_Divide_idx_1 * rtb_Divide_idx_1;
  rtb_jxi = rtb_Divide_idx_2 * rtb_Divide_idx_3;
  rtb_kxi = rtb_Divide_idx_0 * rtb_Divide_idx_1;
  y_tmp_5 = (rtb_kxi + rtb_jxi) * 2.0;
  PosControl_Sim_B.y[0] = rt_atan2d_snf(y_tmp_5, 1.0 - (rtb_ixk + rtb_Sum_p) *
    2.0);
  y_tmp_2 = rtb_Divide_idx_1 * rtb_Divide_idx_3;
  y_tmp_3 = rtb_Divide_idx_0 * rtb_Divide_idx_2;
  PosControl_Sim_B.y[1] = asin((y_tmp_3 - y_tmp_2) * 2.0);
  y_tmp = rtb_Divide_idx_3 * rtb_Divide_idx_3;
  y_tmp_0 = rtb_Divide_idx_1 * rtb_Divide_idx_2;
  y_tmp_1 = rtb_Divide_idx_0 * rtb_Divide_idx_3;
  y_tmp_4 = (y_tmp_1 + y_tmp_0) * 2.0;
  PosControl_Sim_B.y[2] = rt_atan2d_snf(y_tmp_4, 1.0 - (rtb_Sum_p + y_tmp) * 2.0);

  /* DataTypeConversion: '<S3>/Data Type Conversion6' */
  PosControl_Sim_B.DataTypeConversion6[0] = (real32_T)PosControl_Sim_B.y[0];
  PosControl_Sim_B.DataTypeConversion6[1] = (real32_T)PosControl_Sim_B.y[1];
  PosControl_Sim_B.DataTypeConversion6[2] = (real32_T)PosControl_Sim_B.y[2];

  /* Step: '<S3>/Step' */
  if (PosControl_Sim_M->Timing.t[0] < PosControl_Sim_P.Step_Time) {
    /* DataTypeConversion: '<S3>/Data Type Conversion7' */
    PosControl_Sim_B.DataTypeConversion7 = (real32_T)PosControl_Sim_P.Step_Y0;
  } else {
    /* DataTypeConversion: '<S3>/Data Type Conversion7' */
    PosControl_Sim_B.DataTypeConversion7 = (real32_T)
      PosControl_Sim_P.Step_YFinal;
  }

  /* End of Step: '<S3>/Step' */
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* Constant: '<S50>/Constant1' */
    PosControl_Sim_B.Constant1[0] = PosControl_Sim_P.ModelInit_PosE[0];
    PosControl_Sim_B.Constant1[1] = PosControl_Sim_P.ModelInit_PosE[1];
    PosControl_Sim_B.Constant1[2] = PosControl_Sim_P.ModelInit_PosE[2];
  }

  /* Integrator: '<S50>/xe' */
  if (PosControl_Sim_DW.xe_IWORK != 0) {
    PosControl_Sim_X.xe_CSTATE[0] = PosControl_Sim_B.Constant1[0];
    PosControl_Sim_X.xe_CSTATE[1] = PosControl_Sim_B.Constant1[1];
    PosControl_Sim_X.xe_CSTATE[2] = PosControl_Sim_B.Constant1[2];
  }

  PosControl_Sim_B.xe[0] = PosControl_Sim_X.xe_CSTATE[0];
  PosControl_Sim_B.xe[1] = PosControl_Sim_X.xe_CSTATE[1];
  PosControl_Sim_B.xe[2] = PosControl_Sim_X.xe_CSTATE[2];

  /* End of Integrator: '<S50>/xe' */

  /* Sum: '<S63>/Sum8' incorporates:
   *  Constant: '<S63>/Constant2'
   *  Gain: '<S63>/Gain1'
   */
  PosControl_Sim_B.Sum8 = PosControl_Sim_P.Gain1_Gain_l * PosControl_Sim_B.xe[1]
    + PosControl_Sim_P.Constant2_Value;

  /* Sum: '<S63>/Sum9' incorporates:
   *  Constant: '<S63>/Constant3'
   *  Gain: '<S63>/Gain2'
   *  Gain: '<S63>/Gain4'
   */
  PosControl_Sim_B.Sum9 = PosControl_Sim_P.Gain4_Gain * PosControl_Sim_B.xe[0] *
    PosControl_Sim_P.Gain2_Gain + PosControl_Sim_P.Constant3_Value;

  /* Sum: '<S63>/Sum10' incorporates:
   *  Constant: '<S63>/Constant1'
   *  Gain: '<S63>/Gain8'
   */
  PosControl_Sim_B.Sum10 = PosControl_Sim_P.Gain8_Gain * PosControl_Sim_B.xe[2]
    + PosControl_Sim_P.Constant1_Value;
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[3] == 0) {
    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet_fd[0] = 0U;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__i[0] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__h[0] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__g[0] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet_gm[0] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__f[0] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__o[0] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__n[0] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__a[0] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet_nd[0] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet_fd[1] = 0U;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__i[1] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__h[1] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__g[1] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet_gm[1] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__f[1] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__o[1] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__n[1] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__a[1] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet_nd[1] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet_fd[2] = 0U;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__i[2] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__h[2] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__g[2] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet_gm[2] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__f[2] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__o[2] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__n[2] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__a[2] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet_nd[2] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet_fd[3] = 0U;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__i[3] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__h[3] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__g[3] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet_gm[3] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__f[3] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__o[3] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__n[3] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet__a[3] = 0.0F;

    /* SignalConversion generated from: '<S3>/Pack net_fdm Packet for FlightGear' */
    PosControl_Sim_B.TmpSignalConversionAtPacknet_nd[3] = 0.0F;

    /* S-Function (saerofgpacknetfdm): '<S3>/Pack net_fdm Packet for FlightGear' */
    /*
     * Get the selected/unique ascii value of the first s-function parameter
     */
    /* Block: <S3>/Pack net_fdm Packet for FlightGear
     * Build a net_fdm data packet for FlightGear
     */
    asbFg24net_fdm(&PosControl_Sim_B.Packnet_fdmPacketforFlightGear[0]);
    asbFg24SetGrp1(&PosControl_Sim_B.Packnet_fdmPacketforFlightGear[0],
                   PosControl_Sim_B.Sum8,
                   PosControl_Sim_B.Sum9,
                   PosControl_Sim_B.Sum10,
                   PosControl_Sim_B.DataTypeConversion6[0],
                   PosControl_Sim_B.DataTypeConversion6[1],
                   PosControl_Sim_B.DataTypeConversion6[2] );
    asbFg24SetGrp3(&PosControl_Sim_B.Packnet_fdmPacketforFlightGear[0],
                   PosControl_Sim_B.DataTypeConversion7,
                   0.0F,
                   0.0F,
                   0.0F,
                   0.0F,
                   0.0F,
                   0.0F,
                   0.0F,
                   0.0F,
                   0.0F );
    asbFg24SetGrp4(&PosControl_Sim_B.Packnet_fdmPacketforFlightGear[0],
                   PosControl_Sim_B.DataTypeConversion4,
                   &PosControl_Sim_B.TmpSignalConversionAtPacknet_fd[0],
                   &PosControl_Sim_B.DataTypeConversion5[0],
                   &PosControl_Sim_B.TmpSignalConversionAtPacknet__i[0],
                   &PosControl_Sim_B.TmpSignalConversionAtPacknet__h[0],
                   &PosControl_Sim_B.TmpSignalConversionAtPacknet__g[0],
                   &PosControl_Sim_B.TmpSignalConversionAtPacknet_gm[0],
                   &PosControl_Sim_B.TmpSignalConversionAtPacknet__f[0],
                   &PosControl_Sim_B.TmpSignalConversionAtPacknet__o[0],
                   &PosControl_Sim_B.TmpSignalConversionAtPacknet__n[0],
                   &PosControl_Sim_B.TmpSignalConversionAtPacknet__a[0],
                   0U,
                   &PosControl_Sim_B.TmpSignalConversionAtPacknet_nd[0] );
  }

  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (saeroclockpacer): '<S3>/Simulation Pace' */
    /*
     * The Clock Pacer generates no code, it is only active in
     * interpreted simulation.
     */
  }

  /* Clock: '<S1>/Clock' */
  PosControl_Sim_B.Clock = PosControl_Sim_M->Timing.t[0];
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* DataTypeConversion: '<S1>/Conversion1' incorporates:
     *  Constant: '<Root>/   2'
     */
    rtb_Conversion1 = PosControl_Sim_P.u_Value_o;

    /* DataTypeConversion: '<S1>/Conversion2' incorporates:
     *  Constant: '<Root>/   1'
     */
    rtb_Conversion2 = PosControl_Sim_P.u_Value;

    /* DataTypeConversion: '<S1>/Conversion3' incorporates:
     *  Constant: '<Root>/   3'
     */
    rtb_Conversion3 = PosControl_Sim_P.u_Value_b;

    /* DataTypeConversion: '<S1>/Conversion4' incorporates:
     *  Constant: '<Root>/          '
     */
    rtb_Conversion4 = PosControl_Sim_P._Value;

    /* MATLAB Function: '<S5>/deadzone1' */
    PosControl_Sim_deadzone1(rtb_Conversion1, &PosControl_Sim_B.sf_deadzone1);

    /* MATLAB Function: '<S5>/deadzone2' */
    PosControl_Sim_deadzone1(rtb_Conversion2, &PosControl_Sim_B.sf_deadzone2);

    /* Saturate: '<S5>/Saturation8' */
    if (PosControl_Sim_B.sf_deadzone1.y > PosControl_Sim_P.Saturation8_UpperSat)
    {
      rtb_ixj = PosControl_Sim_P.Saturation8_UpperSat;
    } else if (PosControl_Sim_B.sf_deadzone1.y <
               PosControl_Sim_P.Saturation8_LowerSat) {
      rtb_ixj = PosControl_Sim_P.Saturation8_LowerSat;
    } else {
      rtb_ixj = PosControl_Sim_B.sf_deadzone1.y;
    }

    /* End of Saturate: '<S5>/Saturation8' */

    /* Gain: '<S5>/max_vx' */
    rtb_max_vx = -PosControl_Sim_P.MAX_CONTROL_VELOCITY_XY * rtb_ixj;

    /* Saturate: '<S5>/Saturation9' */
    if (PosControl_Sim_B.sf_deadzone2.y > PosControl_Sim_P.Saturation9_UpperSat)
    {
      rtb_ixj = PosControl_Sim_P.Saturation9_UpperSat;
    } else if (PosControl_Sim_B.sf_deadzone2.y <
               PosControl_Sim_P.Saturation9_LowerSat) {
      rtb_ixj = PosControl_Sim_P.Saturation9_LowerSat;
    } else {
      rtb_ixj = PosControl_Sim_B.sf_deadzone2.y;
    }

    /* End of Saturate: '<S5>/Saturation9' */

    /* Gain: '<S5>/max_vy' */
    rtb_max_vy = PosControl_Sim_P.MAX_CONTROL_VELOCITY_XY * rtb_ixj;

    /* MATLAB Function: '<S5>/deadzone3' */
    PosControl_Sim_deadzone1(rtb_Conversion3, &PosControl_Sim_B.sf_deadzone3);

    /* Saturate: '<S5>/Saturation10' */
    if (PosControl_Sim_B.sf_deadzone3.y > PosControl_Sim_P.Saturation10_UpperSat)
    {
      rtb_ixj = PosControl_Sim_P.Saturation10_UpperSat;
    } else if (PosControl_Sim_B.sf_deadzone3.y <
               PosControl_Sim_P.Saturation10_LowerSat) {
      rtb_ixj = PosControl_Sim_P.Saturation10_LowerSat;
    } else {
      rtb_ixj = PosControl_Sim_B.sf_deadzone3.y;
    }

    /* End of Saturate: '<S5>/Saturation10' */

    /* Gain: '<S5>/max_vz' */
    rtb_max_vz = -PosControl_Sim_P.MAX_CONTROL_VELOCITY_Z * rtb_ixj;

    /* MATLAB Function: '<S6>/deadzone3' */
    PosControl_Sim_deadzone1(rtb_Conversion3, &PosControl_Sim_B.sf_deadzone3_g);

    /* Gain: '<S6>/Gain' incorporates:
     *  Constant: '<S6>/Constant'
     *  Sum: '<S6>/Sum'
     */
    rtb_ixj = (PosControl_Sim_B.sf_deadzone3_g.y +
               PosControl_Sim_P.Constant_Value_hk) *
      PosControl_Sim_P.Gain_Gain_e;

    /* MATLAB Function: '<S6>/f1' incorporates:
     *  Constant: '<S6>/thr_hover '
     */
    /* MATLAB Function 'Control System/InputConditioning1/f1': '<S18>:1' */
    if (rtb_ixj < 0.5) {
      /* '<S18>:1:10' */
      /* '<S18>:1:11' */
      rtb_ixj = 2.0 * rtb_ixj * PosControl_Sim_P.THR_HOVER;
    } else {
      /* '<S18>:1:13' */
      rtb_ixj = (rtb_ixj - 0.5) * 2.0 * (1.0 - PosControl_Sim_P.THR_HOVER) +
        PosControl_Sim_P.THR_HOVER;
    }

    /* End of MATLAB Function: '<S6>/f1' */

    /* Saturate: '<S6>/Saturation10' */
    if (rtb_ixj > PosControl_Sim_P.Saturation10_UpperSat_e) {
      PosControl_Sim_B.Saturation10 = PosControl_Sim_P.Saturation10_UpperSat_e;
    } else if (rtb_ixj < PosControl_Sim_P.Saturation10_LowerSat_g) {
      PosControl_Sim_B.Saturation10 = PosControl_Sim_P.Saturation10_LowerSat_g;
    } else {
      PosControl_Sim_B.Saturation10 = rtb_ixj;
    }

    /* End of Saturate: '<S6>/Saturation10' */

    /* MATLAB Function: '<S6>/deadzone1' */
    PosControl_Sim_deadzone1(rtb_Conversion2, &PosControl_Sim_B.sf_deadzone1_d);

    /* Saturate: '<S6>/Saturation9' */
    if (PosControl_Sim_B.sf_deadzone1_d.y >
        PosControl_Sim_P.Saturation9_UpperSat_o) {
      rtb_ixj = PosControl_Sim_P.Saturation9_UpperSat_o;
    } else if (PosControl_Sim_B.sf_deadzone1_d.y <
               PosControl_Sim_P.Saturation9_LowerSat_l) {
      rtb_ixj = PosControl_Sim_P.Saturation9_LowerSat_l;
    } else {
      rtb_ixj = PosControl_Sim_B.sf_deadzone1_d.y;
    }

    /* End of Saturate: '<S6>/Saturation9' */

    /* Gain: '<S6>/deg2rad1' incorporates:
     *  Gain: '<S6>/Gain1'
     */
    PosControl_Sim_B.deg2rad1 = PosControl_Sim_P.MAX_CONTROL_ANGLE_ROLL *
      rtb_ixj * PosControl_Sim_P.DEG2RAD;

    /* MATLAB Function: '<S6>/deadzone2' */
    PosControl_Sim_deadzone1(rtb_Conversion1, &PosControl_Sim_B.sf_deadzone2_b);

    /* Saturate: '<S6>/Saturation8' */
    if (PosControl_Sim_B.sf_deadzone2_b.y >
        PosControl_Sim_P.Saturation8_UpperSat_d) {
      rtb_ixj = PosControl_Sim_P.Saturation8_UpperSat_d;
    } else if (PosControl_Sim_B.sf_deadzone2_b.y <
               PosControl_Sim_P.Saturation8_LowerSat_c) {
      rtb_ixj = PosControl_Sim_P.Saturation8_LowerSat_c;
    } else {
      rtb_ixj = PosControl_Sim_B.sf_deadzone2_b.y;
    }

    /* End of Saturate: '<S6>/Saturation8' */

    /* Gain: '<S6>/deg2rad2' incorporates:
     *  Gain: '<S6>/Gain3'
     */
    PosControl_Sim_B.deg2rad2 = PosControl_Sim_P.MAX_CONTROL_ANGLE_PITCH *
      rtb_ixj * PosControl_Sim_P.DEG2RAD;

    /* MATLAB Function: '<S6>/deadzone4' */
    PosControl_Sim_deadzone1(rtb_Conversion4, &PosControl_Sim_B.sf_deadzone4);

    /* Saturate: '<S6>/Saturation7' */
    if (PosControl_Sim_B.sf_deadzone4.y > PosControl_Sim_P.Saturation7_UpperSat)
    {
      rtb_ixj = PosControl_Sim_P.Saturation7_UpperSat;
    } else if (PosControl_Sim_B.sf_deadzone4.y <
               PosControl_Sim_P.Saturation7_LowerSat) {
      rtb_ixj = PosControl_Sim_P.Saturation7_LowerSat;
    } else {
      rtb_ixj = PosControl_Sim_B.sf_deadzone4.y;
    }

    /* End of Saturate: '<S6>/Saturation7' */

    /* Gain: '<S6>/deg2rad3' incorporates:
     *  Gain: '<S6>/Gain2'
     */
    PosControl_Sim_B.deg2rad3 = PosControl_Sim_P.MAX_CONTROL_ANGLE_RATE_Y *
      rtb_ixj * PosControl_Sim_P.DEG2RAD;

    /* MATLAB Function: '<S1>/MATLAB Function1' */
    /* MATLAB Function 'Control System/MATLAB Function1': '<S8>:1' */
    if (rtb_Conversion3 < 1250.0) {
      /* '<S8>:1:4' */
      /* '<S8>:1:5' */
      PosControl_Sim_B.reset_i[0] = 1.0;
      PosControl_Sim_B.reset_i[1] = 1.0;
      PosControl_Sim_B.reset_i[2] = 1.0;
      PosControl_Sim_B.reset_i[3] = 1.0;
    } else {
      /* '<S8>:1:7' */
      PosControl_Sim_B.reset_i[0] = 0.0;
      PosControl_Sim_B.reset_i[1] = 0.0;
      PosControl_Sim_B.reset_i[2] = 0.0;
      PosControl_Sim_B.reset_i[3] = 0.0;
    }

    /* End of MATLAB Function: '<S1>/MATLAB Function1' */

    /* Constant: '<S49>/Constant2' */
    PosControl_Sim_B.Constant2[0] = PosControl_Sim_P.ModelInit_VelB[0];
    PosControl_Sim_B.Constant2[1] = PosControl_Sim_P.ModelInit_VelB[1];
    PosControl_Sim_B.Constant2[2] = PosControl_Sim_P.ModelInit_VelB[2];
  }

  /* MATLAB Function: '<S50>/Quaternion to rotation matrix' */
  /* MATLAB Function 'Quadcopter Dynamics/Rigid Model/Rigid-Body Kinematic Model /Quaternion to rotation matrix': '<S60>:1' */
  /* '<S60>:1:5' */
  rtb_ixj = rtb_Divide_idx_0 * rtb_Divide_idx_0;
  rtb_Reb[0] = ((rtb_ixj + rtb_ixk) - rtb_Sum_p) - y_tmp;
  rtb_Reb[3] = (y_tmp_0 - y_tmp_1) * 2.0;
  rtb_Reb[6] = (y_tmp_2 + y_tmp_3) * 2.0;
  rtb_Reb[1] = y_tmp_4;
  rtb_ixj -= rtb_ixk;
  rtb_Reb[4] = (rtb_ixj + rtb_Sum_p) - y_tmp;
  rtb_Reb[7] = (rtb_jxi - rtb_kxi) * 2.0;
  rtb_Reb[2] = (y_tmp_2 - y_tmp_3) * 2.0;
  rtb_Reb[5] = y_tmp_5;
  rtb_Reb[8] = (rtb_ixj - rtb_Sum_p) + y_tmp;

  /* Integrator: '<S49>/vb' */
  if (PosControl_Sim_DW.vb_IWORK != 0) {
    PosControl_Sim_X.vb_CSTATE[0] = PosControl_Sim_B.Constant2[0];
    PosControl_Sim_X.vb_CSTATE[1] = PosControl_Sim_B.Constant2[1];
    PosControl_Sim_X.vb_CSTATE[2] = PosControl_Sim_B.Constant2[2];
  }

  /* Product: '<S49>/Product1' incorporates:
   *  Integrator: '<S49>/vb'
   */
  for (x1 = 0; x1 < 3; x1++) {
    PosControl_Sim_B.Product1[x1] = 0.0;
    PosControl_Sim_B.Product1[x1] += rtb_Reb[x1] * PosControl_Sim_X.vb_CSTATE[0];
    PosControl_Sim_B.Product1[x1] += rtb_Reb[x1 + 3] *
      PosControl_Sim_X.vb_CSTATE[1];
    PosControl_Sim_B.Product1[x1] += rtb_Reb[x1 + 6] *
      PosControl_Sim_X.vb_CSTATE[2];
  }

  /* End of Product: '<S49>/Product1' */
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S1>/MATLAB Function2' */
    /* MATLAB Function 'Control System/MATLAB Function2': '<S9>:1' */
    /* '<S9>:1:15' */
    if ((PosControl_Sim_B.xe[0] > -0.25) && (PosControl_Sim_B.xe[0] < 0.25) &&
        (PosControl_Sim_B.xe[1] > -0.25) && (PosControl_Sim_B.xe[1] < 0.25)) {
      /* '<S9>:1:16' */
      /* '<S9>:1:17' */
      x1 = 0;

      /* '<S9>:1:18' */
      b_y1 = -2;
    } else if ((PosControl_Sim_B.xe[0] > -0.25) && (PosControl_Sim_B.xe[0] <
                0.25) && (PosControl_Sim_B.xe[1] > -2.25) &&
               (PosControl_Sim_B.xe[1] < -1.75)) {
      /* '<S9>:1:19' */
      /* '<S9>:1:20' */
      x1 = 2;

      /* '<S9>:1:21' */
      b_y1 = -2;
    } else if ((PosControl_Sim_B.xe[0] > 1.75) && (PosControl_Sim_B.xe[0] < 2.25)
               && (PosControl_Sim_B.xe[1] > -2.25) && (PosControl_Sim_B.xe[1] <
                -1.75)) {
      /* '<S9>:1:22' */
      /* '<S9>:1:23' */
      x1 = 2;

      /* '<S9>:1:24' */
      b_y1 = 0;
    } else {
      /* '<S9>:1:26' */
      x1 = 0;

      /* '<S9>:1:27' */
      b_y1 = 0;
    }

    /* '<S9>:1:34' */
    /* '<S9>:1:36' */
    PosControl_Sim_B.vx_d = rtb_max_vx;

    /* '<S9>:1:36' */
    PosControl_Sim_B.vy_d = rtb_max_vy;

    /* '<S9>:1:36' */
    PosControl_Sim_B.vz_d = rtb_max_vz;

    /* '<S9>:1:38' */
    PosControl_Sim_B.x_d = x1;

    /* '<S9>:1:39' */
    PosControl_Sim_B.y_d = b_y1;

    /* '<S9>:1:40' */
    PosControl_Sim_B.z_d = PosControl_Sim_B.xe[2];

    /* End of MATLAB Function: '<S1>/MATLAB Function2' */

    /* Constant: '<S49>/Constant1' */
    PosControl_Sim_B.Constant1_d[0] = PosControl_Sim_P.ModelInit_RateB[0];
    PosControl_Sim_B.Constant1_d[1] = PosControl_Sim_P.ModelInit_RateB[1];
    PosControl_Sim_B.Constant1_d[2] = PosControl_Sim_P.ModelInit_RateB[2];
  }

  /* Integrator: '<S49>/p,q,r' */
  if (PosControl_Sim_DW.pqr_IWORK != 0) {
    PosControl_Sim_X.pqr_CSTATE[0] = PosControl_Sim_B.Constant1_d[0];
    PosControl_Sim_X.pqr_CSTATE[1] = PosControl_Sim_B.Constant1_d[1];
    PosControl_Sim_X.pqr_CSTATE[2] = PosControl_Sim_B.Constant1_d[2];
  }

  /* Gain: '<S32>/Gain1' incorporates:
   *  Sum: '<S21>/Sum1'
   */
  y_tmp_2 = (PosControl_Sim_B.x_d - PosControl_Sim_B.xe[0]) *
    PosControl_Sim_P.Kpxp;

  /* Saturate: '<S32>/Saturation3' */
  if (y_tmp_2 > PosControl_Sim_P.MAX_CONTROL_VELOCITY_XY) {
    y_tmp_2 = PosControl_Sim_P.MAX_CONTROL_VELOCITY_XY;
  } else {
    if (y_tmp_2 < -PosControl_Sim_P.MAX_CONTROL_VELOCITY_XY) {
      y_tmp_2 = -PosControl_Sim_P.MAX_CONTROL_VELOCITY_XY;
    }
  }

  /* End of Saturate: '<S32>/Saturation3' */

  /* Sum: '<S21>/Sum7' */
  rtb_ixj = (PosControl_Sim_B.vx_d + y_tmp_2) - PosControl_Sim_B.Product1[0];

  /* Gain: '<S35>/Derivative Gain' */
  PosControl_Sim_B.DerivativeGain = PosControl_Sim_P.Kvxd * rtb_ixj;
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* DiscreteIntegrator: '<S35>/Discrete-Time Integrator' */
    if ((PosControl_Sim_B.reset_i[2] != 0.0) ||
        (PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRese != 0)) {
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE =
        PosControl_Sim_P.DiscreteTimeIntegrator_IC;
    }

    if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE >=
        PosControl_Sim_P.Saturation_I_ah) {
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE =
        PosControl_Sim_P.Saturation_I_ah;
    } else {
      if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE <=
          -PosControl_Sim_P.Saturation_I_ah) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE =
          -PosControl_Sim_P.Saturation_I_ah;
      }
    }

    PosControl_Sim_B.DiscreteTimeIntegrator =
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE;

    /* End of DiscreteIntegrator: '<S35>/Discrete-Time Integrator' */

    /* SampleTimeMath: '<S38>/TSamp'
     *
     * About '<S38>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     */
    rtb_TSamp = PosControl_Sim_B.DerivativeGain * PosControl_Sim_P.TSamp_WtEt;

    /* Sum: '<S38>/Diff' incorporates:
     *  UnitDelay: '<S38>/UD'
     */
    PosControl_Sim_B.Diff = rtb_TSamp - PosControl_Sim_DW.UD_DSTATE;
  }

  /* Sum: '<S35>/Sum' incorporates:
   *  Gain: '<S35>/Gain'
   */
  rtb_max_vz = (PosControl_Sim_P.Kvxp * rtb_ixj +
                PosControl_Sim_B.DiscreteTimeIntegrator) + PosControl_Sim_B.Diff;

  /* Gain: '<S33>/Gain1' incorporates:
   *  Sum: '<S21>/Sum3'
   */
  y_tmp_2 = (PosControl_Sim_B.y_d - PosControl_Sim_B.xe[1]) *
    PosControl_Sim_P.Kpyp;

  /* Saturate: '<S33>/Saturation3' */
  if (y_tmp_2 > PosControl_Sim_P.MAX_CONTROL_VELOCITY_XY) {
    y_tmp_2 = PosControl_Sim_P.MAX_CONTROL_VELOCITY_XY;
  } else {
    if (y_tmp_2 < -PosControl_Sim_P.MAX_CONTROL_VELOCITY_XY) {
      y_tmp_2 = -PosControl_Sim_P.MAX_CONTROL_VELOCITY_XY;
    }
  }

  /* End of Saturate: '<S33>/Saturation3' */

  /* Sum: '<S21>/Sum2' */
  rtb_max_vy = (PosControl_Sim_B.vy_d + y_tmp_2) - PosControl_Sim_B.Product1[1];

  /* Gain: '<S36>/Derivative Gain' */
  PosControl_Sim_B.DerivativeGain_g = PosControl_Sim_P.Kvyd * rtb_max_vy;
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* DiscreteIntegrator: '<S36>/Discrete-Time Integrator' */
    if ((PosControl_Sim_B.reset_i[2] != 0.0) ||
        (PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_n != 0)) {
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_f =
        PosControl_Sim_P.DiscreteTimeIntegrator_IC_e;
    }

    if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_f >=
        PosControl_Sim_P.Saturation_I_ah) {
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_f =
        PosControl_Sim_P.Saturation_I_ah;
    } else {
      if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_f <=
          -PosControl_Sim_P.Saturation_I_ah) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_f =
          -PosControl_Sim_P.Saturation_I_ah;
      }
    }

    PosControl_Sim_B.DiscreteTimeIntegrator_o =
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_f;

    /* End of DiscreteIntegrator: '<S36>/Discrete-Time Integrator' */

    /* SampleTimeMath: '<S39>/TSamp'
     *
     * About '<S39>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     */
    rtb_TSamp_j = PosControl_Sim_B.DerivativeGain_g *
      PosControl_Sim_P.TSamp_WtEt_n;

    /* Sum: '<S39>/Diff' incorporates:
     *  UnitDelay: '<S39>/UD'
     */
    PosControl_Sim_B.Diff_h = rtb_TSamp_j - PosControl_Sim_DW.UD_DSTATE_g;
  }

  /* Sum: '<S36>/Sum' incorporates:
   *  Gain: '<S36>/Gain'
   */
  rtb_Sum_p = (PosControl_Sim_P.Kvyp * rtb_max_vy +
               PosControl_Sim_B.DiscreteTimeIntegrator_o) +
    PosControl_Sim_B.Diff_h;

  /* Switch generated from: '<S10>/Switch1' incorporates:
   *  Constant: '<S1>/Constant'
   */
  /* MATLAB Function 'Control System/Subsystem/position_control/MATLAB Function': '<S31>:1' */
  /* '<S31>:1:10' */
  /* '<S31>:1:11' */
  if (PosControl_Sim_P.Constant_Value_h >= PosControl_Sim_P.Switch1_1_Threshold)
  {
    /* MATLAB Function: '<S21>/MATLAB Function' */
    rtb_ixk = (-sin(PosControl_Sim_B.y[2]) * rtb_max_vz + cos
               (PosControl_Sim_B.y[2]) * rtb_Sum_p) / 9.8;

    /* Saturate: '<S21>/Saturation1' */
    rtb_kxi = -PosControl_Sim_P.MAX_CONTROL_ANGLE_ROLL *
      PosControl_Sim_P.DEG2RAD;
    y_tmp_5 = PosControl_Sim_P.MAX_CONTROL_ANGLE_ROLL * PosControl_Sim_P.DEG2RAD;
    if (rtb_ixk > y_tmp_5) {
      rtb_ixk = y_tmp_5;
    } else {
      if (rtb_ixk < rtb_kxi) {
        rtb_ixk = rtb_kxi;
      }
    }

    /* End of Saturate: '<S21>/Saturation1' */
  } else {
    rtb_ixk = PosControl_Sim_B.deg2rad1;
  }

  /* Sum: '<S20>/Sum18' */
  rtb_ixk -= PosControl_Sim_B.y[0];

  /* Gain: '<S25>/Gain' */
  rtb_ixk *= PosControl_Sim_P.Kp_ROLL_ANGLE;

  /* Saturate: '<S20>/Saturation5' */
  if (rtb_ixk > PosControl_Sim_P.MAX_CONTROL_ANGLE_RATE_ROLL) {
    rtb_ixk = PosControl_Sim_P.MAX_CONTROL_ANGLE_RATE_ROLL;
  } else {
    if (rtb_ixk < -PosControl_Sim_P.MAX_CONTROL_ANGLE_RATE_ROLL) {
      rtb_ixk = -PosControl_Sim_P.MAX_CONTROL_ANGLE_RATE_ROLL;
    }
  }

  /* End of Saturate: '<S20>/Saturation5' */

  /* Sum: '<S20>/Sum21' incorporates:
   *  Integrator: '<S49>/p,q,r'
   */
  rtb_ixk -= PosControl_Sim_X.pqr_CSTATE[0];

  /* Gain: '<S26>/Derivative Gain' */
  PosControl_Sim_B.DerivativeGain_m = PosControl_Sim_P.Kd_ROLL_AngleRate *
    rtb_ixk;
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* DiscreteIntegrator: '<S26>/Discrete-Time Integrator' */
    if ((PosControl_Sim_B.reset_i[0] != 0.0) ||
        (PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_m != 0)) {
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_k =
        PosControl_Sim_P.DiscreteTimeIntegrator_IC_m;
    }

    if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_k >=
        PosControl_Sim_P.Saturation_I_RP_Max) {
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_k =
        PosControl_Sim_P.Saturation_I_RP_Max;
    } else {
      if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_k <=
          PosControl_Sim_P.Saturation_I_RP_Min) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_k =
          PosControl_Sim_P.Saturation_I_RP_Min;
      }
    }

    PosControl_Sim_B.DiscreteTimeIntegrator_e =
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_k;

    /* End of DiscreteIntegrator: '<S26>/Discrete-Time Integrator' */

    /* SampleTimeMath: '<S29>/TSamp'
     *
     * About '<S29>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     */
    rtb_TSamp_k = PosControl_Sim_B.DerivativeGain_m *
      PosControl_Sim_P.TSamp_WtEt_k;

    /* Sum: '<S29>/Diff' incorporates:
     *  UnitDelay: '<S29>/UD'
     */
    PosControl_Sim_B.Diff_o = rtb_TSamp_k - PosControl_Sim_DW.UD_DSTATE_p;
  }

  /* Sum: '<S26>/Sum' incorporates:
   *  Gain: '<S26>/Gain'
   */
  rtb_max_vx = (PosControl_Sim_P.Kp_ROLL_AngleRate * rtb_ixk +
                PosControl_Sim_B.DiscreteTimeIntegrator_e) +
    PosControl_Sim_B.Diff_o;

  /* Saturate: '<S20>/Saturation' */
  if (rtb_max_vx > PosControl_Sim_P.Saturation_UpperSat_f) {
    rtb_max_vx = PosControl_Sim_P.Saturation_UpperSat_f;
  } else {
    if (rtb_max_vx < PosControl_Sim_P.Saturation_LowerSat_h) {
      rtb_max_vx = PosControl_Sim_P.Saturation_LowerSat_h;
    }
  }

  /* End of Saturate: '<S20>/Saturation' */

  /* Switch generated from: '<S10>/Switch1' incorporates:
   *  Constant: '<S1>/Constant'
   */
  if (PosControl_Sim_P.Constant_Value_h >= PosControl_Sim_P.Switch1_2_Threshold)
  {
    /* MATLAB Function: '<S21>/MATLAB Function' */
    rtb_jxi = (-cos(PosControl_Sim_B.y[2]) * rtb_max_vz - sin
               (PosControl_Sim_B.y[2]) * rtb_Sum_p) / 9.8;

    /* Saturate: '<S21>/Saturation2' */
    rtb_kxi = -PosControl_Sim_P.MAX_CONTROL_ANGLE_PITCH *
      PosControl_Sim_P.DEG2RAD / 2.0;
    y_tmp_5 = PosControl_Sim_P.MAX_CONTROL_ANGLE_PITCH *
      PosControl_Sim_P.DEG2RAD / 2.0;
    if (rtb_jxi > y_tmp_5) {
      rtb_jxi = y_tmp_5;
    } else {
      if (rtb_jxi < rtb_kxi) {
        rtb_jxi = rtb_kxi;
      }
    }

    /* End of Saturate: '<S21>/Saturation2' */
  } else {
    rtb_jxi = PosControl_Sim_B.deg2rad2;
  }

  /* Sum: '<S20>/Sum19' */
  rtb_jxi -= PosControl_Sim_B.y[1];

  /* Gain: '<S23>/Gain' */
  rtb_jxi *= PosControl_Sim_P.Kp_PITCH_ANGLE;

  /* Saturate: '<S20>/Saturation3' */
  if (rtb_jxi > PosControl_Sim_P.MAX_CONTROL_ANGLE_RATE_PITCH) {
    rtb_jxi = PosControl_Sim_P.MAX_CONTROL_ANGLE_RATE_PITCH;
  } else {
    if (rtb_jxi < -PosControl_Sim_P.MAX_CONTROL_ANGLE_RATE_PITCH) {
      rtb_jxi = -PosControl_Sim_P.MAX_CONTROL_ANGLE_RATE_PITCH;
    }
  }

  /* End of Saturate: '<S20>/Saturation3' */

  /* Sum: '<S20>/Sum22' incorporates:
   *  Integrator: '<S49>/p,q,r'
   */
  rtb_jxi -= PosControl_Sim_X.pqr_CSTATE[1];

  /* Gain: '<S24>/Derivative Gain' */
  PosControl_Sim_B.DerivativeGain_j = PosControl_Sim_P.Kd_PITCH_AngleRate *
    rtb_jxi;
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* DiscreteIntegrator: '<S24>/Discrete-Time Integrator' */
    if ((PosControl_Sim_B.reset_i[0] != 0.0) ||
        (PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_p != 0)) {
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_o =
        PosControl_Sim_P.DiscreteTimeIntegrator_IC_ek;
    }

    if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_o >=
        PosControl_Sim_P.Saturation_I_RP_Max) {
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_o =
        PosControl_Sim_P.Saturation_I_RP_Max;
    } else {
      if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_o <=
          PosControl_Sim_P.Saturation_I_RP_Min) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_o =
          PosControl_Sim_P.Saturation_I_RP_Min;
      }
    }

    PosControl_Sim_B.DiscreteTimeIntegrator_er =
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_o;

    /* End of DiscreteIntegrator: '<S24>/Discrete-Time Integrator' */

    /* SampleTimeMath: '<S28>/TSamp'
     *
     * About '<S28>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     */
    rtb_TSamp_a = PosControl_Sim_B.DerivativeGain_j *
      PosControl_Sim_P.TSamp_WtEt_d;

    /* Sum: '<S28>/Diff' incorporates:
     *  UnitDelay: '<S28>/UD'
     */
    PosControl_Sim_B.Diff_i = rtb_TSamp_a - PosControl_Sim_DW.UD_DSTATE_m;
  }

  /* Sum: '<S24>/Sum' incorporates:
   *  Gain: '<S24>/Gain'
   */
  rtb_max_vz = (PosControl_Sim_P.Kp_PITCH_AngleRate * rtb_jxi +
                PosControl_Sim_B.DiscreteTimeIntegrator_er) +
    PosControl_Sim_B.Diff_i;

  /* Saturate: '<S20>/Saturation1' */
  if (rtb_max_vz > PosControl_Sim_P.Saturation1_UpperSat) {
    rtb_max_vz = PosControl_Sim_P.Saturation1_UpperSat;
  } else {
    if (rtb_max_vz < PosControl_Sim_P.Saturation1_LowerSat) {
      rtb_max_vz = PosControl_Sim_P.Saturation1_LowerSat;
    }
  }

  /* End of Saturate: '<S20>/Saturation1' */

  /* Sum: '<S20>/Sum1' incorporates:
   *  Integrator: '<S49>/p,q,r'
   */
  rtb_kxi = PosControl_Sim_B.deg2rad3 - PosControl_Sim_X.pqr_CSTATE[2];

  /* Gain: '<S27>/Derivative Gain' */
  PosControl_Sim_B.DerivativeGain_n = PosControl_Sim_P.Kd_YAW_AngleRate *
    rtb_kxi;
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* DiscreteIntegrator: '<S27>/Discrete-Time Integrator' */
    if ((PosControl_Sim_B.reset_i[1] != 0.0) ||
        (PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_i != 0)) {
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_l =
        PosControl_Sim_P.DiscreteTimeIntegrator_IC_f;
    }

    if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_l >=
        PosControl_Sim_P.Saturation_I_Y_Max) {
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_l =
        PosControl_Sim_P.Saturation_I_Y_Max;
    } else {
      if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_l <=
          PosControl_Sim_P.Saturation_I_Y_Min) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_l =
          PosControl_Sim_P.Saturation_I_Y_Min;
      }
    }

    PosControl_Sim_B.DiscreteTimeIntegrator_m =
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_l;

    /* End of DiscreteIntegrator: '<S27>/Discrete-Time Integrator' */

    /* SampleTimeMath: '<S30>/TSamp'
     *
     * About '<S30>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     */
    rtb_TSamp_e = PosControl_Sim_B.DerivativeGain_n *
      PosControl_Sim_P.TSamp_WtEt_kk;

    /* Sum: '<S30>/Diff' incorporates:
     *  UnitDelay: '<S30>/UD'
     */
    PosControl_Sim_B.Diff_d = rtb_TSamp_e - PosControl_Sim_DW.UD_DSTATE_h;
  }

  /* Sum: '<S27>/Sum' incorporates:
   *  Gain: '<S27>/Gain'
   */
  rtb_Sum_p = (PosControl_Sim_P.Kp_YAW_AngleRate * rtb_kxi +
               PosControl_Sim_B.DiscreteTimeIntegrator_m) +
    PosControl_Sim_B.Diff_d;

  /* Saturate: '<S20>/Saturation2' */
  if (rtb_Sum_p > PosControl_Sim_P.Saturation2_UpperSat) {
    rtb_Sum_p = PosControl_Sim_P.Saturation2_UpperSat;
  } else {
    if (rtb_Sum_p < PosControl_Sim_P.Saturation2_LowerSat) {
      rtb_Sum_p = PosControl_Sim_P.Saturation2_LowerSat;
    }
  }

  /* End of Saturate: '<S20>/Saturation2' */

  /* Gain: '<S24>/Integral Gain' */
  PosControl_Sim_B.IntegralGain = PosControl_Sim_P.Ki_PITCH_AngleRate * rtb_jxi;

  /* Gain: '<S26>/Integral Gain' */
  PosControl_Sim_B.IntegralGain_m = PosControl_Sim_P.Ki_ROLL_AngleRate * rtb_ixk;

  /* Gain: '<S27>/Integral Gain' */
  PosControl_Sim_B.IntegralGain_p = PosControl_Sim_P.Ki_YAW_AngleRate * rtb_kxi;

  /* Gain: '<S34>/Gain1' incorporates:
   *  Sum: '<S21>/Sum4'
   */
  y_tmp_2 = (PosControl_Sim_B.z_d - PosControl_Sim_B.xe[2]) *
    PosControl_Sim_P.Kpzp;

  /* Saturate: '<S34>/Saturation3' */
  if (y_tmp_2 > PosControl_Sim_P.MAX_CONTROL_VELOCITY_Z) {
    y_tmp_2 = PosControl_Sim_P.MAX_CONTROL_VELOCITY_Z;
  } else {
    if (y_tmp_2 < -PosControl_Sim_P.MAX_CONTROL_VELOCITY_Z) {
      y_tmp_2 = -PosControl_Sim_P.MAX_CONTROL_VELOCITY_Z;
    }
  }

  /* End of Saturate: '<S34>/Saturation3' */

  /* Sum: '<S21>/Sum' */
  rtb_jxi = (PosControl_Sim_B.vz_d + y_tmp_2) - PosControl_Sim_B.Product1[2];

  /* Gain: '<S37>/Derivative Gain' */
  PosControl_Sim_B.DerivativeGain_d = PosControl_Sim_P.Kvzd * rtb_jxi;
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* DiscreteIntegrator: '<S37>/Discrete-Time Integrator' */
    if ((PosControl_Sim_B.reset_i[3] != 0.0) ||
        (PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_b != 0)) {
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_j =
        PosControl_Sim_P.DiscreteTimeIntegrator_IC_g;
    }

    if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_j >=
        PosControl_Sim_P.Saturation_I_az) {
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_j =
        PosControl_Sim_P.Saturation_I_az;
    } else {
      if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_j <=
          -PosControl_Sim_P.Saturation_I_az) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_j =
          -PosControl_Sim_P.Saturation_I_az;
      }
    }

    PosControl_Sim_B.DiscreteTimeIntegrator_b =
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_j;

    /* End of DiscreteIntegrator: '<S37>/Discrete-Time Integrator' */

    /* SampleTimeMath: '<S40>/TSamp'
     *
     * About '<S40>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     */
    rtb_TSamp_b = PosControl_Sim_B.DerivativeGain_d *
      PosControl_Sim_P.TSamp_WtEt_b;

    /* Sum: '<S40>/Diff' incorporates:
     *  UnitDelay: '<S40>/UD'
     */
    PosControl_Sim_B.Diff_e = rtb_TSamp_b - PosControl_Sim_DW.UD_DSTATE_b;
  }

  /* Switch: '<S10>/Switch2' incorporates:
   *  Constant: '<S1>/Constant'
   */
  if (PosControl_Sim_P.Constant_Value_h >= PosControl_Sim_P.Switch2_Threshold) {
    /* Sum: '<S37>/Sum' incorporates:
     *  Gain: '<S37>/Gain'
     */
    y_tmp_2 = (PosControl_Sim_P.Kvzp * rtb_jxi +
               PosControl_Sim_B.DiscreteTimeIntegrator_b) +
      PosControl_Sim_B.Diff_e;

    /* Saturate: '<S21>/az' */
    if (y_tmp_2 > PosControl_Sim_P.az_UpperSat) {
      y_tmp_2 = PosControl_Sim_P.az_UpperSat;
    } else {
      if (y_tmp_2 < PosControl_Sim_P.az_LowerSat) {
        y_tmp_2 = PosControl_Sim_P.az_LowerSat;
      }
    }

    /* End of Saturate: '<S21>/az' */

    /* Gain: '<S21>/Gain' incorporates:
     *  Constant: '<S21>/Constant'
     *  Sum: '<S21>/Sum6'
     */
    rtb_ixk = (y_tmp_2 - PosControl_Sim_P.THR_HOVER) *
      PosControl_Sim_P.Gain_Gain;

    /* Saturate: '<S21>/Saturation' */
    if (rtb_ixk > PosControl_Sim_P.Saturation_UpperSat) {
      rtb_ixk = PosControl_Sim_P.Saturation_UpperSat;
    } else {
      if (rtb_ixk < PosControl_Sim_P.Saturation_LowerSat) {
        rtb_ixk = PosControl_Sim_P.Saturation_LowerSat;
      }
    }

    /* End of Saturate: '<S21>/Saturation' */
  } else {
    rtb_ixk = PosControl_Sim_B.Saturation10;
  }

  /* End of Switch: '<S10>/Switch2' */

  /* Gain: '<S35>/Integral Gain' */
  /* MATLAB Function 'Control System/Subsystem/pwm_out2': '<S22>:1' */
  /* '<S22>:1:13' */
  /* '<S22>:1:14' */
  /* '<S22>:1:16' */
  /* '<S22>:1:17' */
  /* '<S22>:1:18' */
  /* '<S22>:1:19' */
  PosControl_Sim_B.IntegralGain_e = PosControl_Sim_P.Kvxi * rtb_ixj;

  /* Gain: '<S36>/Integral Gain' */
  PosControl_Sim_B.IntegralGain_i = PosControl_Sim_P.Kvyi * rtb_max_vy;

  /* Gain: '<S37>/Integral Gain' */
  PosControl_Sim_B.IntegralGain_l = PosControl_Sim_P.Kvzi * rtb_jxi;

  /* MATLAB Function: '<S41>/Propeller Model' incorporates:
   *  Constant: '<S41>/Constant3'
   *  Constant: '<S41>/Constant4'
   *  Constant: '<S41>/Constant5'
   *  SignalConversion generated from: '<S44>/ SFunction '
   */
  /* MATLAB Function 'Quadcopter Dynamics/Control Effectiveness Model/Propeller Model': '<S44>:1' */
  /* '<S44>:1:26' */
  /* '<S44>:1:29' */
  rtb_max_vy = rtb_Motor_Dynamics_1 * rtb_Motor_Dynamics_1;
  rtb_jxi = rtb_Motor_Dynamics_2 * rtb_Motor_Dynamics_2;
  rtb_kxi = rtb_Motor_Dynamics_3 * rtb_Motor_Dynamics_3;
  y_tmp_5 = rtb_Motor_Dynamics_4 * rtb_Motor_Dynamics_4;
  rtb_ixj = -0.70710678118654746 * PosControl_Sim_P.ModelParam_uavR *
    PosControl_Sim_P.ModelParam_rotorCt;
  tmp[0] = rtb_ixj;
  y_tmp_2 = 0.70710678118654746 * PosControl_Sim_P.ModelParam_uavR *
    PosControl_Sim_P.ModelParam_rotorCt;
  tmp[3] = y_tmp_2;
  tmp[6] = y_tmp_2;
  tmp[9] = rtb_ixj;
  tmp[1] = y_tmp_2;
  tmp[4] = rtb_ixj;
  tmp[7] = y_tmp_2;
  tmp[10] = rtb_ixj;
  tmp[2] = PosControl_Sim_P.ModelParam_rotorCm;
  tmp[5] = PosControl_Sim_P.ModelParam_rotorCm;
  tmp[8] = -PosControl_Sim_P.ModelParam_rotorCm;
  tmp[11] = -PosControl_Sim_P.ModelParam_rotorCm;
  for (x1 = 0; x1 < 3; x1++) {
    rtb_ixj = tmp[x1 + 9] * y_tmp_5 + (tmp[x1 + 6] * rtb_kxi + (tmp[x1 + 3] *
      rtb_jxi + tmp[x1] * rtb_max_vy));
    rtb_Mp[x1] = rtb_ixj;
  }

  /* '<S44>:1:31' */
  /* '<S44>:1:32' */
  /* '<S44>:1:33' */
  /* '<S44>:1:35' */
  /* '<S44>:1:36' */
  /* '<S44>:1:37' */
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* Product: '<S2>/Product1' incorporates:
     *  Constant: '<S2>/g'
     *  Constant: '<S2>/mass'
     */
    PosControl_Sim_B.Product1_o[0] = PosControl_Sim_P.ModelParam_uavMass *
      PosControl_Sim_P.g_Value[0];
    PosControl_Sim_B.Product1_o[1] = PosControl_Sim_P.ModelParam_uavMass *
      PosControl_Sim_P.g_Value[1];
    PosControl_Sim_B.Product1_o[2] = PosControl_Sim_P.ModelParam_uavMass *
      PosControl_Sim_P.g_Value[2];
  }

  /* MATLAB Function: '<S10>/pwm_out2' */
  y_tmp_3 = rtb_ixk - rtb_max_vx;
  y_tmp_2 = ((y_tmp_3 + rtb_max_vz) + rtb_Sum_p) * 1000.0 + 1000.0;

  /* Saturate: '<S10>/Output_Limits1' */
  if (y_tmp_2 > PosControl_Sim_P.Output_Limits1_UpperSat) {
    y_tmp_2 = PosControl_Sim_P.Output_Limits1_UpperSat;
  } else {
    if (y_tmp_2 < PosControl_Sim_P.Output_Limits1_LowerSat) {
      y_tmp_2 = PosControl_Sim_P.Output_Limits1_LowerSat;
    }
  }

  /* Gain: '<S2>/Gain1' incorporates:
   *  Constant: '<S2>/Constant2'
   *  Sum: '<S2>/Sum'
   */
  rtb_ixj = (y_tmp_2 - PosControl_Sim_P.Constant2_Value_b[0]) *
    PosControl_Sim_P.Gain1_Gain_p;

  /* Saturate: '<S2>/Saturation' */
  if (rtb_ixj > PosControl_Sim_P.Saturation_UpperSat_i) {
    rtb_ixj = PosControl_Sim_P.Saturation_UpperSat_i;
  } else {
    if (rtb_ixj < PosControl_Sim_P.Saturation_LowerSat_k) {
      rtb_ixj = PosControl_Sim_P.Saturation_LowerSat_k;
    }
  }

  /* MATLAB Function: '<S10>/pwm_out2' */
  rtb_ixk += rtb_max_vx;
  y_tmp_2 = ((rtb_ixk - rtb_max_vz) + rtb_Sum_p) * 1000.0 + 1000.0;

  /* Saturate: '<S10>/Output_Limits1' */
  if (y_tmp_2 > PosControl_Sim_P.Output_Limits1_UpperSat) {
    y_tmp_2 = PosControl_Sim_P.Output_Limits1_UpperSat;
  } else {
    if (y_tmp_2 < PosControl_Sim_P.Output_Limits1_LowerSat) {
      y_tmp_2 = PosControl_Sim_P.Output_Limits1_LowerSat;
    }
  }

  /* Gain: '<S2>/Gain1' incorporates:
   *  Constant: '<S2>/Constant2'
   *  Sum: '<S2>/Sum'
   */
  rtb_max_vx = (y_tmp_2 - PosControl_Sim_P.Constant2_Value_b[1]) *
    PosControl_Sim_P.Gain1_Gain_p;

  /* Saturate: '<S2>/Saturation' */
  if (rtb_max_vx > PosControl_Sim_P.Saturation_UpperSat_i) {
    rtb_max_vx = PosControl_Sim_P.Saturation_UpperSat_i;
  } else {
    if (rtb_max_vx < PosControl_Sim_P.Saturation_LowerSat_k) {
      rtb_max_vx = PosControl_Sim_P.Saturation_LowerSat_k;
    }
  }

  /* MATLAB Function: '<S10>/pwm_out2' */
  y_tmp_2 = ((rtb_ixk + rtb_max_vz) - rtb_Sum_p) * 1000.0 + 1000.0;

  /* Saturate: '<S10>/Output_Limits1' */
  if (y_tmp_2 > PosControl_Sim_P.Output_Limits1_UpperSat) {
    y_tmp_2 = PosControl_Sim_P.Output_Limits1_UpperSat;
  } else {
    if (y_tmp_2 < PosControl_Sim_P.Output_Limits1_LowerSat) {
      y_tmp_2 = PosControl_Sim_P.Output_Limits1_LowerSat;
    }
  }

  /* Gain: '<S2>/Gain1' incorporates:
   *  Constant: '<S2>/Constant2'
   *  Sum: '<S2>/Sum'
   */
  rtb_ixk = (y_tmp_2 - PosControl_Sim_P.Constant2_Value_b[2]) *
    PosControl_Sim_P.Gain1_Gain_p;

  /* Saturate: '<S2>/Saturation' */
  if (rtb_ixk > PosControl_Sim_P.Saturation_UpperSat_i) {
    rtb_ixk = PosControl_Sim_P.Saturation_UpperSat_i;
  } else {
    if (rtb_ixk < PosControl_Sim_P.Saturation_LowerSat_k) {
      rtb_ixk = PosControl_Sim_P.Saturation_LowerSat_k;
    }
  }

  /* MATLAB Function: '<S10>/pwm_out2' */
  y_tmp_2 = ((y_tmp_3 - rtb_max_vz) - rtb_Sum_p) * 1000.0 + 1000.0;

  /* Saturate: '<S10>/Output_Limits1' */
  if (y_tmp_2 > PosControl_Sim_P.Output_Limits1_UpperSat) {
    y_tmp_2 = PosControl_Sim_P.Output_Limits1_UpperSat;
  } else {
    if (y_tmp_2 < PosControl_Sim_P.Output_Limits1_LowerSat) {
      y_tmp_2 = PosControl_Sim_P.Output_Limits1_LowerSat;
    }
  }

  /* Gain: '<S2>/Gain1' incorporates:
   *  Constant: '<S2>/Constant2'
   *  Sum: '<S2>/Sum'
   */
  rtb_max_vz = (y_tmp_2 - PosControl_Sim_P.Constant2_Value_b[3]) *
    PosControl_Sim_P.Gain1_Gain_p;

  /* Saturate: '<S2>/Saturation' */
  if (rtb_max_vz > PosControl_Sim_P.Saturation_UpperSat_i) {
    rtb_max_vz = PosControl_Sim_P.Saturation_UpperSat_i;
  } else {
    if (rtb_max_vz < PosControl_Sim_P.Saturation_LowerSat_k) {
      rtb_max_vz = PosControl_Sim_P.Saturation_LowerSat_k;
    }
  }

  /* Saturate: '<S42>/Signal_Saturation_1' */
  if (rtb_ixj > PosControl_Sim_P.Signal_Saturation_1_UpperSat) {
    rtb_ixj = PosControl_Sim_P.Signal_Saturation_1_UpperSat;
  } else {
    if (rtb_ixj < PosControl_Sim_P.Signal_Saturation_1_LowerSat) {
      rtb_ixj = PosControl_Sim_P.Signal_Saturation_1_LowerSat;
    }
  }

  /* End of Saturate: '<S42>/Signal_Saturation_1' */

  /* Sum: '<S45>/Sum1' incorporates:
   *  Constant: '<S45>/Constant'
   *  Gain: '<S45>/Gain'
   */
  PosControl_Sim_B.Sum1 = PosControl_Sim_P.ModelParam_motorCr * rtb_ixj +
    PosControl_Sim_P.ModelParam_motorWb;

  /* Saturate: '<S42>/Signal_Saturation_2' */
  if (rtb_max_vx > PosControl_Sim_P.Signal_Saturation_2_UpperSat) {
    rtb_max_vx = PosControl_Sim_P.Signal_Saturation_2_UpperSat;
  } else {
    if (rtb_max_vx < PosControl_Sim_P.Signal_Saturation_2_LowerSat) {
      rtb_max_vx = PosControl_Sim_P.Signal_Saturation_2_LowerSat;
    }
  }

  /* End of Saturate: '<S42>/Signal_Saturation_2' */

  /* Sum: '<S46>/Sum1' incorporates:
   *  Constant: '<S46>/Constant'
   *  Gain: '<S46>/Gain'
   */
  PosControl_Sim_B.Sum1_k = PosControl_Sim_P.ModelParam_motorCr * rtb_max_vx +
    PosControl_Sim_P.ModelParam_motorWb;

  /* Saturate: '<S42>/Signal_Saturation_3' */
  if (rtb_ixk > PosControl_Sim_P.Signal_Saturation_3_UpperSat) {
    rtb_ixk = PosControl_Sim_P.Signal_Saturation_3_UpperSat;
  } else {
    if (rtb_ixk < PosControl_Sim_P.Signal_Saturation_3_LowerSat) {
      rtb_ixk = PosControl_Sim_P.Signal_Saturation_3_LowerSat;
    }
  }

  /* End of Saturate: '<S42>/Signal_Saturation_3' */

  /* Sum: '<S47>/Sum1' incorporates:
   *  Constant: '<S47>/Constant'
   *  Gain: '<S47>/Gain'
   */
  PosControl_Sim_B.Sum1_k3 = PosControl_Sim_P.ModelParam_motorCr * rtb_ixk +
    PosControl_Sim_P.ModelParam_motorWb;

  /* Saturate: '<S42>/Signal_Saturation_4' */
  if (rtb_max_vz > PosControl_Sim_P.Signal_Saturation_4_UpperSat) {
    rtb_max_vz = PosControl_Sim_P.Signal_Saturation_4_UpperSat;
  } else {
    if (rtb_max_vz < PosControl_Sim_P.Signal_Saturation_4_LowerSat) {
      rtb_max_vz = PosControl_Sim_P.Signal_Saturation_4_LowerSat;
    }
  }

  /* End of Saturate: '<S42>/Signal_Saturation_4' */

  /* Sum: '<S48>/Sum1' incorporates:
   *  Constant: '<S48>/Constant'
   *  Gain: '<S48>/Gain'
   */
  PosControl_Sim_B.Sum1_b = PosControl_Sim_P.ModelParam_motorCr * rtb_max_vz +
    PosControl_Sim_P.ModelParam_motorWb;
  if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
      PosControl_Sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* Product: '<S49>/Divide' incorporates:
     *  Constant: '<S49>/Constant'
     *  Constant: '<S49>/J '
     */
    rt_mrdivided3x3_snf(PosControl_Sim_P.Constant_Value_p,
                        PosControl_Sim_P.ModelParam_uavJ,
                        PosControl_Sim_B.Divide);
  }

  /* Product: '<S51>/J*w' incorporates:
   *  Constant: '<S49>/J '
   *  Integrator: '<S49>/p,q,r'
   */
  for (x1 = 0; x1 < 3; x1++) {
    rtb_Sum_aw[x1] = PosControl_Sim_P.ModelParam_uavJ[x1 + 6] *
      PosControl_Sim_X.pqr_CSTATE[2] + (PosControl_Sim_P.ModelParam_uavJ[x1 + 3]
      * PosControl_Sim_X.pqr_CSTATE[1] + PosControl_Sim_P.ModelParam_uavJ[x1] *
      PosControl_Sim_X.pqr_CSTATE[0]);
  }

  /* End of Product: '<S51>/J*w' */

  /* Product: '<S49>/Product' incorporates:
   *  Constant: '<S41>/Constant1'
   *  Constant: '<S41>/Jrp'
   *  Integrator: '<S49>/p,q,r'
   *  MATLAB Function: '<S41>/Propeller Model'
   *  Product: '<S54>/i x j'
   *  Product: '<S54>/j x k'
   *  Product: '<S54>/k x i'
   *  Product: '<S55>/i x k'
   *  Product: '<S55>/j x i'
   *  Product: '<S55>/k x j'
   *  SignalConversion generated from: '<S44>/ SFunction '
   *  Sum: '<S41>/Add1'
   *  Sum: '<S49>/Sum2'
   *  Sum: '<S52>/Sum'
   */
  rtb_max_vz = ((((rtb_Motor_Dynamics_1 + rtb_Motor_Dynamics_2) -
                  rtb_Motor_Dynamics_3) - rtb_Motor_Dynamics_4) *
                (PosControl_Sim_P.ModelParam_motorJm *
                 PosControl_Sim_X.pqr_CSTATE[1]) +
                (-PosControl_Sim_P.ModelParam_uavCCm[0] *
                 PosControl_Sim_X.pqr_CSTATE[0] * fabs
                 (PosControl_Sim_X.pqr_CSTATE[0]) + rtb_Mp[0])) -
    (PosControl_Sim_X.pqr_CSTATE[1] * rtb_Sum_aw[2] -
     PosControl_Sim_X.pqr_CSTATE[2] * rtb_Sum_aw[1]);
  rtb_Motor_Dynamics_1 = ((((-rtb_Motor_Dynamics_1 - rtb_Motor_Dynamics_2) +
    rtb_Motor_Dynamics_3) + rtb_Motor_Dynamics_4) *
    (PosControl_Sim_P.ModelParam_motorJm * PosControl_Sim_X.pqr_CSTATE[0]) +
    (-PosControl_Sim_P.ModelParam_uavCCm[1] * PosControl_Sim_X.pqr_CSTATE[1] *
     fabs(PosControl_Sim_X.pqr_CSTATE[1]) + rtb_Mp[1])) -
    (PosControl_Sim_X.pqr_CSTATE[2] * rtb_Sum_aw[0] -
     PosControl_Sim_X.pqr_CSTATE[0] * rtb_Sum_aw[2]);
  rtb_Motor_Dynamics_2 = (-PosControl_Sim_P.ModelParam_uavCCm[2] *
    PosControl_Sim_X.pqr_CSTATE[2] * fabs(PosControl_Sim_X.pqr_CSTATE[2]) +
    rtb_Mp[2]) - (PosControl_Sim_X.pqr_CSTATE[0] * rtb_Sum_aw[1] -
                  PosControl_Sim_X.pqr_CSTATE[1] * rtb_Sum_aw[0]);

  /* MATLAB Function: '<S41>/Propeller Model' incorporates:
   *  Constant: '<S41>/Constant3'
   *  Constant: '<S41>/Constant7'
   *  Integrator: '<S49>/vb'
   *  Sum: '<S41>/Add7'
   */
  rtb_Mp[0] = -PosControl_Sim_P.ModelParam_uavCd * PosControl_Sim_X.vb_CSTATE[0]
    * fabs(PosControl_Sim_X.vb_CSTATE[0]) * 0.5;
  rtb_Mp[1] = -PosControl_Sim_P.ModelParam_uavCd * PosControl_Sim_X.vb_CSTATE[1]
    * fabs(PosControl_Sim_X.vb_CSTATE[1]) * 0.5;
  rtb_Mp[2] = -(((rtb_max_vy * PosControl_Sim_P.ModelParam_rotorCt + rtb_jxi *
                  PosControl_Sim_P.ModelParam_rotorCt) + rtb_kxi *
                 PosControl_Sim_P.ModelParam_rotorCt) + y_tmp_5 *
                PosControl_Sim_P.ModelParam_rotorCt) +
    -PosControl_Sim_P.ModelParam_uavCd * PosControl_Sim_X.vb_CSTATE[2] * fabs
    (PosControl_Sim_X.vb_CSTATE[2]) * 0.5;

  /* Sum: '<S53>/Sum' incorporates:
   *  Integrator: '<S49>/p,q,r'
   *  Integrator: '<S49>/vb'
   *  Product: '<S56>/i x j'
   *  Product: '<S56>/j x k'
   *  Product: '<S56>/k x i'
   *  Product: '<S57>/i x k'
   *  Product: '<S57>/j x i'
   *  Product: '<S57>/k x j'
   */
  rtb_Sum_aw[0] = PosControl_Sim_X.pqr_CSTATE[1] * PosControl_Sim_X.vb_CSTATE[2];
  rtb_Sum_aw[1] = PosControl_Sim_X.pqr_CSTATE[2] * PosControl_Sim_X.vb_CSTATE[0];
  rtb_Sum_aw[2] = PosControl_Sim_X.pqr_CSTATE[0] * PosControl_Sim_X.vb_CSTATE[1];
  tmp_0[0] = PosControl_Sim_X.pqr_CSTATE[2] * PosControl_Sim_X.vb_CSTATE[1];
  tmp_0[1] = PosControl_Sim_X.pqr_CSTATE[0] * PosControl_Sim_X.vb_CSTATE[2];
  tmp_0[2] = PosControl_Sim_X.pqr_CSTATE[1] * PosControl_Sim_X.vb_CSTATE[0];
  for (x1 = 0; x1 < 3; x1++) {
    /* Product: '<S49>/Product' */
    PosControl_Sim_B.Product[x1] = 0.0;
    PosControl_Sim_B.Product[x1] += PosControl_Sim_B.Divide[x1] * rtb_max_vz;
    PosControl_Sim_B.Product[x1] += PosControl_Sim_B.Divide[x1 + 3] *
      rtb_Motor_Dynamics_1;
    PosControl_Sim_B.Product[x1] += PosControl_Sim_B.Divide[x1 + 6] *
      rtb_Motor_Dynamics_2;

    /* Sum: '<S49>/Sum1' incorporates:
     *  Constant: '<S49>/ Mass '
     *  Math: '<S41>/Transpose'
     *  Product: '<S41>/Product'
     *  Product: '<S49>/a=f//m'
     *  Sum: '<S41>/Add2'
     *  Sum: '<S53>/Sum'
     */
    PosControl_Sim_B.Sum1_o[x1] = ((rtb_Reb[3 * x1 + 2] *
      PosControl_Sim_B.Product1_o[2] + (rtb_Reb[3 * x1 + 1] *
      PosControl_Sim_B.Product1_o[1] + rtb_Reb[3 * x1] *
      PosControl_Sim_B.Product1_o[0])) + rtb_Mp[x1]) /
      PosControl_Sim_P.ModelParam_uavMass - (rtb_Sum_aw[x1] - tmp_0[x1]);
  }

  /* MATLAB Function: '<S50>/Quaternions model' incorporates:
   *  Integrator: '<S49>/p,q,r'
   */
  /* MATLAB Function 'Quadcopter Dynamics/Rigid Model/Rigid-Body Kinematic Model /Quaternions model': '<S61>:1' */
  /* '<S61>:1:4' */
  tmp_1[0] = 0.0;
  rtb_ixj = 0.5 * -PosControl_Sim_X.pqr_CSTATE[0];
  tmp_1[4] = rtb_ixj;
  y_tmp_2 = 0.5 * -PosControl_Sim_X.pqr_CSTATE[1];
  tmp_1[8] = y_tmp_2;
  rtb_Motor_Dynamics_1 = 0.5 * -PosControl_Sim_X.pqr_CSTATE[2];
  tmp_1[12] = rtb_Motor_Dynamics_1;
  tmp_1[1] = 0.5 * PosControl_Sim_X.pqr_CSTATE[0];
  tmp_1[5] = 0.0;
  tmp_1[9] = 0.5 * PosControl_Sim_X.pqr_CSTATE[2];
  tmp_1[13] = y_tmp_2;
  tmp_1[2] = 0.5 * PosControl_Sim_X.pqr_CSTATE[1];
  tmp_1[6] = rtb_Motor_Dynamics_1;
  tmp_1[10] = 0.0;
  tmp_1[14] = 0.5 * PosControl_Sim_X.pqr_CSTATE[0];
  tmp_1[3] = 0.5 * PosControl_Sim_X.pqr_CSTATE[2];
  tmp_1[7] = 0.5 * PosControl_Sim_X.pqr_CSTATE[1];
  tmp_1[11] = rtb_ixj;
  tmp_1[15] = 0.0;
  for (x1 = 0; x1 < 4; x1++) {
    PosControl_Sim_B.qdot[x1] = 0.0;
    PosControl_Sim_B.qdot[x1] += tmp_1[x1] * rtb_Divide_idx_0;
    PosControl_Sim_B.qdot[x1] += tmp_1[x1 + 4] * rtb_Divide_idx_1;
    PosControl_Sim_B.qdot[x1] += tmp_1[x1 + 8] * rtb_Divide_idx_2;
    PosControl_Sim_B.qdot[x1] += tmp_1[x1 + 12] * rtb_Divide_idx_3;
  }

  /* End of MATLAB Function: '<S50>/Quaternions model' */
  if (rtmIsMajorTimeStep(PosControl_Sim_M)) {
    /* Matfile logging */
    rt_UpdateTXYLogVars(PosControl_Sim_M->rtwLogInfo,
                        (PosControl_Sim_M->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(PosControl_Sim_M)) {
    /* Update for RateTransition: '<S3>/Rate Transition' */
    if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
        PosControl_Sim_M->Timing.TaskCounters.TID[3] == 0) {
      memcpy(&PosControl_Sim_DW.RateTransition_Buffer0[0],
             &PosControl_Sim_B.Packnet_fdmPacketforFlightGear[0], 408U * sizeof
             (uint8_T));
    }

    /* End of Update for RateTransition: '<S3>/Rate Transition' */

    /* Update for Integrator: '<S50>/q' */
    PosControl_Sim_DW.q_IWORK = 0;

    /* Update for Integrator: '<S50>/xe' */
    PosControl_Sim_DW.xe_IWORK = 0;

    /* Update for Integrator: '<S49>/vb' */
    PosControl_Sim_DW.vb_IWORK = 0;

    /* Update for Integrator: '<S49>/p,q,r' */
    PosControl_Sim_DW.pqr_IWORK = 0;
    if (rtmIsMajorTimeStep(PosControl_Sim_M) &&
        PosControl_Sim_M->Timing.TaskCounters.TID[1] == 0) {
      /* Update for DiscreteIntegrator: '<S35>/Discrete-Time Integrator' */
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE +=
        PosControl_Sim_P.DiscreteTimeIntegrator_gainval *
        PosControl_Sim_B.IntegralGain_e;
      if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE >=
          PosControl_Sim_P.Saturation_I_ah) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE =
          PosControl_Sim_P.Saturation_I_ah;
      } else {
        if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE <=
            -PosControl_Sim_P.Saturation_I_ah) {
          PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE =
            -PosControl_Sim_P.Saturation_I_ah;
        }
      }

      if (PosControl_Sim_B.reset_i[2] > 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRese = 1;
      } else if (PosControl_Sim_B.reset_i[2] < 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRese = -1;
      } else if (PosControl_Sim_B.reset_i[2] == 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRese = 0;
      } else {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRese = 2;
      }

      /* End of Update for DiscreteIntegrator: '<S35>/Discrete-Time Integrator' */

      /* Update for UnitDelay: '<S38>/UD' */
      PosControl_Sim_DW.UD_DSTATE = rtb_TSamp;

      /* Update for DiscreteIntegrator: '<S36>/Discrete-Time Integrator' */
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_f +=
        PosControl_Sim_P.DiscreteTimeIntegrator_gainva_m *
        PosControl_Sim_B.IntegralGain_i;
      if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_f >=
          PosControl_Sim_P.Saturation_I_ah) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_f =
          PosControl_Sim_P.Saturation_I_ah;
      } else {
        if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_f <=
            -PosControl_Sim_P.Saturation_I_ah) {
          PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_f =
            -PosControl_Sim_P.Saturation_I_ah;
        }
      }

      if (PosControl_Sim_B.reset_i[2] > 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_n = 1;
      } else if (PosControl_Sim_B.reset_i[2] < 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_n = -1;
      } else if (PosControl_Sim_B.reset_i[2] == 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_n = 0;
      } else {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_n = 2;
      }

      /* End of Update for DiscreteIntegrator: '<S36>/Discrete-Time Integrator' */

      /* Update for UnitDelay: '<S39>/UD' */
      PosControl_Sim_DW.UD_DSTATE_g = rtb_TSamp_j;

      /* Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator' */
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_k +=
        PosControl_Sim_P.DiscreteTimeIntegrator_gainva_h *
        PosControl_Sim_B.IntegralGain_m;
      if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_k >=
          PosControl_Sim_P.Saturation_I_RP_Max) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_k =
          PosControl_Sim_P.Saturation_I_RP_Max;
      } else {
        if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_k <=
            PosControl_Sim_P.Saturation_I_RP_Min) {
          PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_k =
            PosControl_Sim_P.Saturation_I_RP_Min;
        }
      }

      if (PosControl_Sim_B.reset_i[0] > 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_m = 1;
      } else if (PosControl_Sim_B.reset_i[0] < 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_m = -1;
      } else if (PosControl_Sim_B.reset_i[0] == 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_m = 0;
      } else {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_m = 2;
      }

      /* End of Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator' */

      /* Update for UnitDelay: '<S29>/UD' */
      PosControl_Sim_DW.UD_DSTATE_p = rtb_TSamp_k;

      /* Update for DiscreteIntegrator: '<S24>/Discrete-Time Integrator' */
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_o +=
        PosControl_Sim_P.DiscreteTimeIntegrator_gainva_g *
        PosControl_Sim_B.IntegralGain;
      if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_o >=
          PosControl_Sim_P.Saturation_I_RP_Max) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_o =
          PosControl_Sim_P.Saturation_I_RP_Max;
      } else {
        if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_o <=
            PosControl_Sim_P.Saturation_I_RP_Min) {
          PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_o =
            PosControl_Sim_P.Saturation_I_RP_Min;
        }
      }

      if (PosControl_Sim_B.reset_i[0] > 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_p = 1;
      } else if (PosControl_Sim_B.reset_i[0] < 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_p = -1;
      } else if (PosControl_Sim_B.reset_i[0] == 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_p = 0;
      } else {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_p = 2;
      }

      /* End of Update for DiscreteIntegrator: '<S24>/Discrete-Time Integrator' */

      /* Update for UnitDelay: '<S28>/UD' */
      PosControl_Sim_DW.UD_DSTATE_m = rtb_TSamp_a;

      /* Update for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' */
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_l +=
        PosControl_Sim_P.DiscreteTimeIntegrator_gainva_c *
        PosControl_Sim_B.IntegralGain_p;
      if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_l >=
          PosControl_Sim_P.Saturation_I_Y_Max) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_l =
          PosControl_Sim_P.Saturation_I_Y_Max;
      } else {
        if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_l <=
            PosControl_Sim_P.Saturation_I_Y_Min) {
          PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_l =
            PosControl_Sim_P.Saturation_I_Y_Min;
        }
      }

      if (PosControl_Sim_B.reset_i[1] > 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_i = 1;
      } else if (PosControl_Sim_B.reset_i[1] < 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_i = -1;
      } else if (PosControl_Sim_B.reset_i[1] == 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_i = 0;
      } else {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_i = 2;
      }

      /* End of Update for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' */

      /* Update for UnitDelay: '<S30>/UD' */
      PosControl_Sim_DW.UD_DSTATE_h = rtb_TSamp_e;

      /* Update for DiscreteIntegrator: '<S37>/Discrete-Time Integrator' */
      PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_j +=
        PosControl_Sim_P.DiscreteTimeIntegrator_gainva_j *
        PosControl_Sim_B.IntegralGain_l;
      if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_j >=
          PosControl_Sim_P.Saturation_I_az) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_j =
          PosControl_Sim_P.Saturation_I_az;
      } else {
        if (PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_j <=
            -PosControl_Sim_P.Saturation_I_az) {
          PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_j =
            -PosControl_Sim_P.Saturation_I_az;
        }
      }

      if (PosControl_Sim_B.reset_i[3] > 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_b = 1;
      } else if (PosControl_Sim_B.reset_i[3] < 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_b = -1;
      } else if (PosControl_Sim_B.reset_i[3] == 0.0) {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_b = 0;
      } else {
        PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_b = 2;
      }

      /* End of Update for DiscreteIntegrator: '<S37>/Discrete-Time Integrator' */

      /* Update for UnitDelay: '<S40>/UD' */
      PosControl_Sim_DW.UD_DSTATE_b = rtb_TSamp_b;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(PosControl_Sim_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(PosControl_Sim_M)!=-1) &&
          !((rtmGetTFinal(PosControl_Sim_M)-
             (((PosControl_Sim_M->Timing.clockTick1+
                PosControl_Sim_M->Timing.clockTickH1* 4294967296.0)) * 0.001)) >
            (((PosControl_Sim_M->Timing.clockTick1+
               PosControl_Sim_M->Timing.clockTickH1* 4294967296.0)) * 0.001) *
            (DBL_EPSILON))) {
        rtmSetErrorStatus(PosControl_Sim_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&PosControl_Sim_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++PosControl_Sim_M->Timing.clockTick0)) {
      ++PosControl_Sim_M->Timing.clockTickH0;
    }

    PosControl_Sim_M->Timing.t[0] = rtsiGetSolverStopTime
      (&PosControl_Sim_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      PosControl_Sim_M->Timing.clockTick1++;
      if (!PosControl_Sim_M->Timing.clockTick1) {
        PosControl_Sim_M->Timing.clockTickH1++;
      }
    }

    rate_scheduler();
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void PosControl_Sim_derivatives(void)
{
  XDot_PosControl_Sim_T *_rtXdot;
  _rtXdot = ((XDot_PosControl_Sim_T *) PosControl_Sim_M->derivs);

  /* Derivatives for StateSpace: '<S42>/Motor_Dynamics_1' */
  _rtXdot->Motor_Dynamics_1_CSTATE = 0.0;
  _rtXdot->Motor_Dynamics_1_CSTATE += -1.0 / PosControl_Sim_P.ModelParam_motorT *
    PosControl_Sim_X.Motor_Dynamics_1_CSTATE;
  _rtXdot->Motor_Dynamics_1_CSTATE += PosControl_Sim_P.Motor_Dynamics_1_B *
    PosControl_Sim_B.Sum1;

  /* Derivatives for StateSpace: '<S42>/Motor_Dynamics_2' */
  _rtXdot->Motor_Dynamics_2_CSTATE = 0.0;
  _rtXdot->Motor_Dynamics_2_CSTATE += -1.0 / PosControl_Sim_P.ModelParam_motorT *
    PosControl_Sim_X.Motor_Dynamics_2_CSTATE;
  _rtXdot->Motor_Dynamics_2_CSTATE += PosControl_Sim_P.Motor_Dynamics_2_B *
    PosControl_Sim_B.Sum1_k;

  /* Derivatives for StateSpace: '<S42>/Motor_Dynamics_3' */
  _rtXdot->Motor_Dynamics_3_CSTATE = 0.0;
  _rtXdot->Motor_Dynamics_3_CSTATE += -1.0 / PosControl_Sim_P.ModelParam_motorT *
    PosControl_Sim_X.Motor_Dynamics_3_CSTATE;
  _rtXdot->Motor_Dynamics_3_CSTATE += PosControl_Sim_P.Motor_Dynamics_3_B *
    PosControl_Sim_B.Sum1_k3;

  /* Derivatives for StateSpace: '<S42>/Motor_Dynamics_4' */
  _rtXdot->Motor_Dynamics_4_CSTATE = 0.0;
  _rtXdot->Motor_Dynamics_4_CSTATE += -1.0 / PosControl_Sim_P.ModelParam_motorT *
    PosControl_Sim_X.Motor_Dynamics_4_CSTATE;
  _rtXdot->Motor_Dynamics_4_CSTATE += PosControl_Sim_P.Motor_Dynamics_4_B *
    PosControl_Sim_B.Sum1_b;

  /* Derivatives for Integrator: '<S50>/q' */
  _rtXdot->q_CSTATE[0] = PosControl_Sim_B.qdot[0];
  _rtXdot->q_CSTATE[1] = PosControl_Sim_B.qdot[1];
  _rtXdot->q_CSTATE[2] = PosControl_Sim_B.qdot[2];
  _rtXdot->q_CSTATE[3] = PosControl_Sim_B.qdot[3];

  /* Derivatives for Integrator: '<S50>/xe' */
  _rtXdot->xe_CSTATE[0] = PosControl_Sim_B.Product1[0];

  /* Derivatives for Integrator: '<S49>/vb' */
  _rtXdot->vb_CSTATE[0] = PosControl_Sim_B.Sum1_o[0];

  /* Derivatives for Integrator: '<S49>/p,q,r' */
  _rtXdot->pqr_CSTATE[0] = PosControl_Sim_B.Product[0];

  /* Derivatives for Integrator: '<S50>/xe' */
  _rtXdot->xe_CSTATE[1] = PosControl_Sim_B.Product1[1];

  /* Derivatives for Integrator: '<S49>/vb' */
  _rtXdot->vb_CSTATE[1] = PosControl_Sim_B.Sum1_o[1];

  /* Derivatives for Integrator: '<S49>/p,q,r' */
  _rtXdot->pqr_CSTATE[1] = PosControl_Sim_B.Product[1];

  /* Derivatives for Integrator: '<S50>/xe' */
  _rtXdot->xe_CSTATE[2] = PosControl_Sim_B.Product1[2];

  /* Derivatives for Integrator: '<S49>/vb' */
  _rtXdot->vb_CSTATE[2] = PosControl_Sim_B.Sum1_o[2];

  /* Derivatives for Integrator: '<S49>/p,q,r' */
  _rtXdot->pqr_CSTATE[2] = PosControl_Sim_B.Product[2];
}

/* Model initialize function */
void PosControl_Sim_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)PosControl_Sim_M, 0,
                sizeof(RT_MODEL_PosControl_Sim_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&PosControl_Sim_M->solverInfo,
                          &PosControl_Sim_M->Timing.simTimeStep);
    rtsiSetTPtr(&PosControl_Sim_M->solverInfo, &rtmGetTPtr(PosControl_Sim_M));
    rtsiSetStepSizePtr(&PosControl_Sim_M->solverInfo,
                       &PosControl_Sim_M->Timing.stepSize0);
    rtsiSetdXPtr(&PosControl_Sim_M->solverInfo, &PosControl_Sim_M->derivs);
    rtsiSetContStatesPtr(&PosControl_Sim_M->solverInfo, (real_T **)
                         &PosControl_Sim_M->contStates);
    rtsiSetNumContStatesPtr(&PosControl_Sim_M->solverInfo,
      &PosControl_Sim_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&PosControl_Sim_M->solverInfo,
      &PosControl_Sim_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&PosControl_Sim_M->solverInfo,
      &PosControl_Sim_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&PosControl_Sim_M->solverInfo,
      &PosControl_Sim_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&PosControl_Sim_M->solverInfo, (&rtmGetErrorStatus
      (PosControl_Sim_M)));
    rtsiSetRTModelPtr(&PosControl_Sim_M->solverInfo, PosControl_Sim_M);
  }

  rtsiSetSimTimeStep(&PosControl_Sim_M->solverInfo, MAJOR_TIME_STEP);
  PosControl_Sim_M->intgData.y = PosControl_Sim_M->odeY;
  PosControl_Sim_M->intgData.f[0] = PosControl_Sim_M->odeF[0];
  PosControl_Sim_M->intgData.f[1] = PosControl_Sim_M->odeF[1];
  PosControl_Sim_M->intgData.f[2] = PosControl_Sim_M->odeF[2];
  PosControl_Sim_M->intgData.f[3] = PosControl_Sim_M->odeF[3];
  PosControl_Sim_M->contStates = ((X_PosControl_Sim_T *) &PosControl_Sim_X);
  rtsiSetSolverData(&PosControl_Sim_M->solverInfo, (void *)
                    &PosControl_Sim_M->intgData);
  rtsiSetSolverName(&PosControl_Sim_M->solverInfo,"ode4");
  rtmSetTPtr(PosControl_Sim_M, &PosControl_Sim_M->Timing.tArray[0]);
  rtmSetTFinal(PosControl_Sim_M, 3.8000000000000003);
  PosControl_Sim_M->Timing.stepSize0 = 0.001;
  rtmSetFirstInitCond(PosControl_Sim_M, 1);

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = NULL;
    PosControl_Sim_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(PosControl_Sim_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(PosControl_Sim_M->rtwLogInfo, (NULL));
    rtliSetLogT(PosControl_Sim_M->rtwLogInfo, "tout");
    rtliSetLogX(PosControl_Sim_M->rtwLogInfo, "");
    rtliSetLogXFinal(PosControl_Sim_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(PosControl_Sim_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(PosControl_Sim_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(PosControl_Sim_M->rtwLogInfo, 0);
    rtliSetLogDecimation(PosControl_Sim_M->rtwLogInfo, 1);
    rtliSetLogY(PosControl_Sim_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(PosControl_Sim_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(PosControl_Sim_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &PosControl_Sim_B), 0,
                sizeof(B_PosControl_Sim_T));

  /* states (continuous) */
  {
    (void) memset((void *)&PosControl_Sim_X, 0,
                  sizeof(X_PosControl_Sim_T));
  }

  /* states (dwork) */
  (void) memset((void *)&PosControl_Sim_DW, 0,
                sizeof(DW_PosControl_Sim_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(PosControl_Sim_M->rtwLogInfo, 0.0,
    rtmGetTFinal(PosControl_Sim_M), PosControl_Sim_M->Timing.stepSize0,
    (&rtmGetErrorStatus(PosControl_Sim_M)));

  {
    int32_T i;

    /* Start for RateTransition: '<S3>/Rate Transition' */
    for (i = 0; i < 408; i++) {
      PosControl_Sim_B.RateTransition[i] =
        PosControl_Sim_P.RateTransition_InitialCondition;
    }

    /* End of Start for RateTransition: '<S3>/Rate Transition' */

    /* Start for Constant: '<S50>/Constant1' */
    PosControl_Sim_B.Constant1[0] = PosControl_Sim_P.ModelInit_PosE[0];

    /* Start for Constant: '<S49>/Constant2' */
    PosControl_Sim_B.Constant2[0] = PosControl_Sim_P.ModelInit_VelB[0];

    /* Start for Constant: '<S49>/Constant1' */
    PosControl_Sim_B.Constant1_d[0] = PosControl_Sim_P.ModelInit_RateB[0];

    /* Start for Constant: '<S50>/Constant1' */
    PosControl_Sim_B.Constant1[1] = PosControl_Sim_P.ModelInit_PosE[1];

    /* Start for Constant: '<S49>/Constant2' */
    PosControl_Sim_B.Constant2[1] = PosControl_Sim_P.ModelInit_VelB[1];

    /* Start for Constant: '<S49>/Constant1' */
    PosControl_Sim_B.Constant1_d[1] = PosControl_Sim_P.ModelInit_RateB[1];

    /* Start for Constant: '<S50>/Constant1' */
    PosControl_Sim_B.Constant1[2] = PosControl_Sim_P.ModelInit_PosE[2];

    /* Start for Constant: '<S49>/Constant2' */
    PosControl_Sim_B.Constant2[2] = PosControl_Sim_P.ModelInit_VelB[2];

    /* Start for Constant: '<S49>/Constant1' */
    PosControl_Sim_B.Constant1_d[2] = PosControl_Sim_P.ModelInit_RateB[2];
  }

  {
    int32_T i;
    real_T Motor_Dynamics_1_CSTATE_tmp;

    /* InitializeConditions for RateTransition: '<S3>/Rate Transition' */
    for (i = 0; i < 408; i++) {
      PosControl_Sim_DW.RateTransition_Buffer0[i] =
        PosControl_Sim_P.RateTransition_InitialCondition;
    }

    /* End of InitializeConditions for RateTransition: '<S3>/Rate Transition' */

    /* InitializeConditions for StateSpace: '<S42>/Motor_Dynamics_1' incorporates:
     *  StateSpace: '<S42>/Motor_Dynamics_2'
     *  StateSpace: '<S42>/Motor_Dynamics_3'
     *  StateSpace: '<S42>/Motor_Dynamics_4'
     */
    Motor_Dynamics_1_CSTATE_tmp = PosControl_Sim_P.ModelInit_Rads *
      PosControl_Sim_P.ModelParam_motorT;
    PosControl_Sim_X.Motor_Dynamics_1_CSTATE = Motor_Dynamics_1_CSTATE_tmp;

    /* InitializeConditions for StateSpace: '<S42>/Motor_Dynamics_2' */
    PosControl_Sim_X.Motor_Dynamics_2_CSTATE = Motor_Dynamics_1_CSTATE_tmp;

    /* InitializeConditions for StateSpace: '<S42>/Motor_Dynamics_3' */
    PosControl_Sim_X.Motor_Dynamics_3_CSTATE = Motor_Dynamics_1_CSTATE_tmp;

    /* InitializeConditions for StateSpace: '<S42>/Motor_Dynamics_4' */
    PosControl_Sim_X.Motor_Dynamics_4_CSTATE = Motor_Dynamics_1_CSTATE_tmp;

    /* InitializeConditions for Integrator: '<S50>/q' incorporates:
     *  Integrator: '<S50>/xe'
     */
    if (rtmIsFirstInitCond(PosControl_Sim_M)) {
      PosControl_Sim_X.q_CSTATE[0] = 0.0;
      PosControl_Sim_X.q_CSTATE[1] = 0.0;
      PosControl_Sim_X.q_CSTATE[2] = 0.0;
      PosControl_Sim_X.q_CSTATE[3] = 0.0;
      PosControl_Sim_X.xe_CSTATE[0] = 0.0;
      PosControl_Sim_X.xe_CSTATE[1] = 0.0;
      PosControl_Sim_X.xe_CSTATE[2] = -100.0;
    }

    PosControl_Sim_DW.q_IWORK = 1;

    /* End of InitializeConditions for Integrator: '<S50>/q' */

    /* InitializeConditions for Integrator: '<S50>/xe' */
    PosControl_Sim_DW.xe_IWORK = 1;

    /* InitializeConditions for Integrator: '<S49>/vb' incorporates:
     *  Integrator: '<S49>/p,q,r'
     */
    if (rtmIsFirstInitCond(PosControl_Sim_M)) {
      PosControl_Sim_X.vb_CSTATE[0] = 0.0;
      PosControl_Sim_X.vb_CSTATE[1] = 0.0;
      PosControl_Sim_X.vb_CSTATE[2] = 0.0;
      PosControl_Sim_X.pqr_CSTATE[0] = 0.0;
      PosControl_Sim_X.pqr_CSTATE[1] = 0.0;
      PosControl_Sim_X.pqr_CSTATE[2] = 0.0;
    }

    PosControl_Sim_DW.vb_IWORK = 1;

    /* End of InitializeConditions for Integrator: '<S49>/vb' */

    /* InitializeConditions for Integrator: '<S49>/p,q,r' */
    PosControl_Sim_DW.pqr_IWORK = 1;

    /* InitializeConditions for DiscreteIntegrator: '<S35>/Discrete-Time Integrator' */
    PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE =
      PosControl_Sim_P.DiscreteTimeIntegrator_IC;
    PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRese = 0;

    /* InitializeConditions for UnitDelay: '<S38>/UD' */
    PosControl_Sim_DW.UD_DSTATE =
      PosControl_Sim_P.DiscreteDerivative_ICPrevScaled;

    /* InitializeConditions for DiscreteIntegrator: '<S36>/Discrete-Time Integrator' */
    PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_f =
      PosControl_Sim_P.DiscreteTimeIntegrator_IC_e;
    PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_n = 0;

    /* InitializeConditions for UnitDelay: '<S39>/UD' */
    PosControl_Sim_DW.UD_DSTATE_g =
      PosControl_Sim_P.DiscreteDerivative_ICPrevScal_f;

    /* InitializeConditions for DiscreteIntegrator: '<S26>/Discrete-Time Integrator' */
    PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_k =
      PosControl_Sim_P.DiscreteTimeIntegrator_IC_m;
    PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_m = 0;

    /* InitializeConditions for UnitDelay: '<S29>/UD' */
    PosControl_Sim_DW.UD_DSTATE_p =
      PosControl_Sim_P.DiscreteDerivative_ICPrevScal_i;

    /* InitializeConditions for DiscreteIntegrator: '<S24>/Discrete-Time Integrator' */
    PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_o =
      PosControl_Sim_P.DiscreteTimeIntegrator_IC_ek;
    PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_p = 0;

    /* InitializeConditions for UnitDelay: '<S28>/UD' */
    PosControl_Sim_DW.UD_DSTATE_m =
      PosControl_Sim_P.DiscreteDerivative_ICPrevScal_c;

    /* InitializeConditions for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' */
    PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_l =
      PosControl_Sim_P.DiscreteTimeIntegrator_IC_f;
    PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_i = 0;

    /* InitializeConditions for UnitDelay: '<S30>/UD' */
    PosControl_Sim_DW.UD_DSTATE_h =
      PosControl_Sim_P.DiscreteDerivative_ICPrevSca_iz;

    /* InitializeConditions for DiscreteIntegrator: '<S37>/Discrete-Time Integrator' */
    PosControl_Sim_DW.DiscreteTimeIntegrator_DSTATE_j =
      PosControl_Sim_P.DiscreteTimeIntegrator_IC_g;
    PosControl_Sim_DW.DiscreteTimeIntegrator_PrevRe_b = 0;

    /* InitializeConditions for UnitDelay: '<S40>/UD' */
    PosControl_Sim_DW.UD_DSTATE_b =
      PosControl_Sim_P.DiscreteDerivative_ICPrevSca_f3;

    /* set "at time zero" to false */
    if (rtmIsFirstInitCond(PosControl_Sim_M)) {
      rtmSetFirstInitCond(PosControl_Sim_M, 0);
    }
  }
}

/* Model terminate function */
void PosControl_Sim_terminate(void)
{
  /* (no terminate code required) */
}
