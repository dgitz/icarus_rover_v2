/*
 * Pose_AutoCode_types.h
 *
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * Code generation for model "Pose_AutoCode".
 *
 * Model version              : 1.41
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C++ source code generated on : Fri Feb 14 07:35:24 2020
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Pose_AutoCode_types_h_
#define RTW_HEADER_Pose_AutoCode_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SignalState_
#define DEFINED_TYPEDEF_FOR_SignalState_

typedef uint8_T SignalState;

/* enum SignalState */
const SignalState SIGSTATE_UNDEFINED = 0U;/* Default value */
const SignalState SIGSTATE_INVALID = 1U;
const SignalState SIGSTATE_INITIALIZING = 2U;
const SignalState SIGSTATE_SIGNALSTATE_UPDATED = 3U;
const SignalState SIGSTATE_SIGNALSTATE_EXTRAPOLATED = 4U;
const SignalState SIGTATE_SIGNALSTATE_HOLD = 5U;
const SignalState SIGSTATE_SIGNALSTATE_CALIBRATING = 6U;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_Pose_AutoCode_T RT_MODEL_Pose_AutoCode_T;

#endif                                 /* RTW_HEADER_Pose_AutoCode_types_h_ */
