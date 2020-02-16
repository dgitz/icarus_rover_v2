/*
 * Pose_AutoCode.h
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

#ifndef RTW_HEADER_Pose_AutoCode_h_
#define RTW_HEADER_Pose_AutoCode_h_
#include <cstring>
#ifndef Pose_AutoCode_COMMON_INCLUDES_
# define Pose_AutoCode_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* Pose_AutoCode_COMMON_INCLUDES_ */

#include "Pose_AutoCode_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  TimeCompensatorObjectState Memory_PreviousInput;/* '<S5>/Memory' */
  real_T Memory1_PreviousInput[24];    /* '<S5>/Memory1' */
  real_T Memory2_PreviousInput[24];    /* '<S5>/Memory2' */
} DW_Pose_AutoCode_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T current_time;                 /* '<Root>/current_time' */
  InputSignalObject accel1x_in;        /* '<Root>/accel1x_in' */
  InputSignalObject accel1y_in;        /* '<Root>/accel1y_in' */
  InputSignalObject accel1z_in;        /* '<Root>/accel1z_in' */
} ExtU_Pose_AutoCode_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  OutputSignalObject timed_signals_output[3];/* '<Root>/timed_signals_output' */
} ExtY_Pose_AutoCode_T;

/* Parameters (default storage) */
struct P_Pose_AutoCode_T_ {
  TimeCompensatorObjectState Memory_InitialCondition;
                                  /* Computed Parameter: Memory_InitialCondition
                                   * Referenced by: '<S5>/Memory'
                                   */
  real_T Memory1_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S5>/Memory1'
                                        */
  real_T Memory2_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S5>/Memory2'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_Pose_AutoCode_T {
  const char_T *errorStatus;
};

/* External data declarations for dependent source files */
extern const OutputSignalObject Pose_AutoCode_rtZOutputSignalObject;/* OutputSignalObject ground */

/* Class declaration for model Pose_AutoCode */
class Pose_AutoCodeModelClass {
  /* public data and function members */
 public:
  /* External inputs */
  ExtU_Pose_AutoCode_T Pose_AutoCode_U;

  /* External outputs */
  ExtY_Pose_AutoCode_T Pose_AutoCode_Y;

  /* model initialize function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  void terminate();

  /* Constructor */
  Pose_AutoCodeModelClass();

  /* Destructor */
  ~Pose_AutoCodeModelClass();

  /* Real-Time Model get method */
  RT_MODEL_Pose_AutoCode_T * getRTM();

  /* private data and function members */
 private:
  /* Tunable parameters */
  static P_Pose_AutoCode_T Pose_AutoCode_P;

  /* Block states */
  DW_Pose_AutoCode_T Pose_AutoCode_DW;

  /* Real-Time Model */
  RT_MODEL_Pose_AutoCode_T Pose_AutoCode_M;

  /* private member function(s) for subsystem '<Root>'*/
  void Pose_Auto_timecompensate_signal(real_T current_time, uint32_T
    update_count_in, real_T b_index, real_T value_in, uint8_T status_in, real_T
    rms_in, const real_T buffers_x_in[24], const real_T buffers_t_in[24], real_T
    *value, uint8_T *status, real_T *rms, uint32_T *update_count, real_T
    buffers_x[24], real_T buffers_t[24]);
};

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
 * '<Root>' : 'Pose_AutoCode'
 * '<S1>'   : 'Pose_AutoCode/PoseModel'
 * '<S2>'   : 'Pose_AutoCode/PoseModel/SensorPostProcessing'
 * '<S3>'   : 'Pose_AutoCode/PoseModel/SignalLinker'
 * '<S4>'   : 'Pose_AutoCode/PoseModel/TimeCompensate'
 * '<S5>'   : 'Pose_AutoCode/PoseModel/TimeCompensate/Time Compensate Signal x12'
 * '<S6>'   : 'Pose_AutoCode/PoseModel/TimeCompensate/Time Compensate Signal x12/TimeCompensator1'
 */
#endif                                 /* RTW_HEADER_Pose_AutoCode_h_ */
