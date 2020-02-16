/*
 * Pose_AutoCode_types.h
 *
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * Code generation for model "Pose_AutoCode".
 *
 * Model version              : 1.97
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C++ source code generated on : Sun Feb 16 09:36:46 2020
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
#include "multiword_types.h"
#ifndef DEFINED_TYPEDEF_FOR_InputSignalObject_
#define DEFINED_TYPEDEF_FOR_InputSignalObject_

typedef struct {
  real_T value;
  uint8_T status;
  real_T rms;
  uint32_T sequence_number;
} InputSignalObject;

#endif

#ifndef DEFINED_TYPEDEF_FOR_OutputSignalObject_
#define DEFINED_TYPEDEF_FOR_OutputSignalObject_

typedef struct {
  real_T value;
  uint8_T status;
  real_T rms;
} OutputSignalObject;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TimeCompensatorObjectState_
#define DEFINED_TYPEDEF_FOR_TimeCompensatorObjectState_

typedef struct {
  uint8_T initialized;
  uint32_T signal1_update_counter;
  uint32_T signal2_update_counter;
  uint32_T signal3_update_counter;
} TimeCompensatorObjectState;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SignalState_
#define DEFINED_TYPEDEF_FOR_SignalState_

typedef uint8_T SignalState;

/* enum SignalState */
const SignalState SIGSTATE_UNDEFINED = 0U;/* Default value */
const SignalState SIGSTATE_INVALID = 1U;
const SignalState SIGSTATE_INITIALIZING = 2U;
const SignalState SIGSTATE_UPDATED = 3U;
const SignalState SIGSTATE_EXTRAPOLATED = 4U;
const SignalState SIGSTATE_HOLD = 5U;
const SignalState SIGSTATE_CALIBRATING = 6U;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SamplingMethod_
#define DEFINED_TYPEDEF_FOR_SamplingMethod_

typedef uint8_T SamplingMethod;

/* enum SamplingMethod */
const SamplingMethod SAMPLEMETHOD_UNKNOWN = 0U;/* Default value */
const SamplingMethod SAMPLEMETHOD_SAMPLEHOLD = 1U;
const SamplingMethod SAMPLEMETHOD_LINEAREXTRAPOLATE = 2U;

#endif

/* Parameters (default storage) */
typedef struct P_Pose_AutoCode_T_ P_Pose_AutoCode_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_Pose_AutoCode_T RT_MODEL_Pose_AutoCode_T;

#endif                                 /* RTW_HEADER_Pose_AutoCode_types_h_ */
