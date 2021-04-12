/*
 * PosControl_Sim_private.h
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

#ifndef RTW_HEADER_PosControl_Sim_private_h_
#define RTW_HEADER_PosControl_Sim_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "PosControl_Sim.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetFirstInitCond
# define rtmSetFirstInitCond(rtm, val) ((rtm)->Timing.firstInitCondFlag = (val))
#endif

#ifndef rtmIsFirstInitCond
# define rtmIsFirstInitCond(rtm)       ((rtm)->Timing.firstInitCondFlag)
#endif

#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTFinal
# define rtmSetTFinal(rtm, val)        ((rtm)->Timing.tFinal = (val))
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

#define rt_htondbe(dout,x)             { uint32_T *dest = (uint32_T *) dout; real_T tmpx = x; uint32_T *src = (uint32_T *) &tmpx; dest[0] = *((uint32_T *)src++); dest[1] = *((uint32_T *)src);}
#define rt_htonfbe(dout,x)             { uint32_T *dest = (uint32_T *)(dout); real32_T tmpx = x; *dest = *((uint32_T *)&tmpx);}
#define rt_htonu32be(dout,x)           { uint32_T *dest = (uint32_T *)(dout); *dest = x;}
#define rt_htoni32be(dout,x)           { uint32_T *dest = (uint32_T *)(dout); *dest = x;}
#define rt_htonbbe(dest,x)             { *dest = (uint8_T)(x);}
#define rt_htond(dout,x)               { uint32_T *dest = (uint32_T *) dout; real_T tmpx = x; uint32_T *src = (uint32_T *) &tmpx; dest[1] = rt_htonl(*((uint32_T *)src++)); dest[0] = rt_htonl(*((uint32_T *)src));}
#define rt_htonf(dout,x)               { uint32_T *dest = (uint32_T *)(dout); real32_T tmpx = x; *dest = rt_htonl(*((uint32_T *)&tmpx));}
#define rt_htonu32(dout,x)             { uint32_T *dest = (uint32_T *)(dout); *dest = rt_htonl(x);}
#define rt_htoni32(dout,x)             { uint32_T *dest = (uint32_T *)(dout); *dest = rt_htonl(x);}
#define rt_htonb(dest,x)               { *dest = (uint8_T)(x);}

extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void rt_mrdivided3x3_snf(const real_T u0[9], const real_T u1[9], real_T
  y[9]);
void asbFg24net_fdm( uint8_T *out );
void asbFg24SetGrp1( uint8_T *out, const real_T longitudeVal, const real_T
                    latitudeVal, const real_T altitudeVal, const real32_T phiVal,
                    const real32_T thetaVal, const real32_T psiVal );
void asbFg24SetGrp3( uint8_T *out, const real32_T elevatorVal, const real32_T
                    elevator_trim_tabVal, const real32_T left_flapVal, const
                    real32_T right_flapVal, const real32_T left_aileronVal,
                    const real32_T right_aileronVal, const real32_T rudderVal,
                    const real32_T nose_wheelVal, const real32_T speedbrakeVal,
                    const real32_T spoilersVal );
void asbFg24SetGrp4( uint8_T *out, const uint32_T num_enginesVal, const uint32_T
                    *eng_stateVal, const real32_T *rpmVal, const real32_T
                    *fuel_flowVal, const real32_T *fuel_pxVal, const real32_T
                    *egtVal, const real32_T *chtVal, const real32_T *mp_osiVal,
                    const real32_T *titVal, const real32_T *oil_tempVal, const
                    real32_T *oil_pxVal, const uint32_T num_tanksVal, const
                    real32_T *fuel_quantityVal );
extern void PosControl_Sim_deadzone1(real_T rtu_u, B_deadzone1_PosControl_Sim_T *
  localB);

/* private model entry point functions */
extern void PosControl_Sim_derivatives(void);

#endif                                /* RTW_HEADER_PosControl_Sim_private_h_ */
