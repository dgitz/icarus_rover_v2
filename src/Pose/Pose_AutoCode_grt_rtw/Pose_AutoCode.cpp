/*
 * Pose_AutoCode.cpp
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

#include "Pose_AutoCode.h"
#include "Pose_AutoCode_private.h"

/* Model step function */
void Pose_AutoCodeModelClass::step()
{
  /* Outport: '<Root>/pose_linearacceleration_signals' */
  Pose_AutoCode_Y.pose_linearacceleration_signals = 0.0;

  /* Outport: '<Root>/Out2' */
  Pose_AutoCode_Y.Out2 = 0.0;

  /* Outport: '<Root>/Out3' */
  Pose_AutoCode_Y.Out3 = 0.0;

  /* Outport: '<Root>/Out4' */
  Pose_AutoCode_Y.Out4 = 0.0;

  /* Outport: '<Root>/Out5' */
  Pose_AutoCode_Y.Out5 = 0.0;

  /* Outport: '<Root>/Out6' */
  Pose_AutoCode_Y.Out6 = 0.0;
}

/* Model initialize function */
void Pose_AutoCodeModelClass::initialize()
{
  /* Registration code */

  /* external inputs */
  (void)std::memset(&Pose_AutoCode_U, 0, sizeof(ExtU_Pose_AutoCode_T));

  /* external outputs */
  (void) std::memset(static_cast<void *>(&Pose_AutoCode_Y), 0,
                     sizeof(ExtY_Pose_AutoCode_T));
}

/* Model terminate function */
void Pose_AutoCodeModelClass::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
Pose_AutoCodeModelClass::Pose_AutoCodeModelClass() : Pose_AutoCode_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
Pose_AutoCodeModelClass::~Pose_AutoCodeModelClass()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_Pose_AutoCode_T * Pose_AutoCodeModelClass::getRTM()
{
  return (&Pose_AutoCode_M);
}
