/*
 * Pose_AutoCode.h
 *
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * Code generation for model "Pose_AutoCode".
 *
 * Model version              : 1.102
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C++ source code generated on : Fri Feb 28 20:45:21 2020
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

/* Block signals for system '<S2>/KalmanFilter' */
typedef struct {
  real_T xhat;                         /* '<S2>/KalmanFilter' */
  real_T P;                            /* '<S2>/KalmanFilter' */
  KalmanFilterObjectState state;       /* '<S2>/KalmanFilter' */
} B_KalmanFilter_Pose_AutoCode_T;

/* Block signals (default storage) */
typedef struct {
  B_KalmanFilter_Pose_AutoCode_T sf_KalmanFilter2;/* '<S2>/KalmanFilter2' */
  B_KalmanFilter_Pose_AutoCode_T sf_KalmanFilter1;/* '<S2>/KalmanFilter1' */
  B_KalmanFilter_Pose_AutoCode_T sf_KalmanFilter;/* '<S2>/KalmanFilter' */
} B_Pose_AutoCode_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  TimeCompensatorObjectState Memory_PreviousInput;/* '<S15>/Memory' */
  real_T Memory1_PreviousInput;        /* '<S2>/Memory1' */
  real_T Memory3_PreviousInput;        /* '<S2>/Memory3' */
  real_T Memory4_PreviousInput;        /* '<S2>/Memory4' */
  real_T Memory6_PreviousInput;        /* '<S2>/Memory6' */
  real_T Memory7_PreviousInput;        /* '<S2>/Memory7' */
  real_T Memory9_PreviousInput;        /* '<S2>/Memory9' */
  real_T Memory1_PreviousInput_m[24];  /* '<S15>/Memory1' */
  real_T Memory2_PreviousInput[24];    /* '<S15>/Memory2' */
  KalmanFilterObjectState Memory2_PreviousInput_j;/* '<S2>/Memory2' */
  KalmanFilterObjectState Memory5_PreviousInput;/* '<S2>/Memory5' */
  KalmanFilterObjectState Memory8_PreviousInput;/* '<S2>/Memory8' */
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
                                   * Referenced by: '<S15>/Memory'
                                   */
  KalmanFilterObjectState Memory2_InitialCondition;
                                 /* Computed Parameter: Memory2_InitialCondition
                                  * Referenced by: '<S2>/Memory2'
                                  */
  KalmanFilterObjectState Memory5_InitialCondition;
                                 /* Computed Parameter: Memory5_InitialCondition
                                  * Referenced by: '<S2>/Memory5'
                                  */
  KalmanFilterObjectState Memory8_InitialCondition;
                                 /* Computed Parameter: Memory8_InitialCondition
                                  * Referenced by: '<S2>/Memory8'
                                  */
  real_T Memory1_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S15>/Memory1'
                                        */
  real_T Memory2_InitialCondition_o;   /* Expression: 0
                                        * Referenced by: '<S15>/Memory2'
                                        */
  real_T Constant_Value;               /* Expression: 1
                                        * Referenced by: '<S1>/Constant'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S1>/Constant1'
                                        */
  real_T Memory1_InitialCondition_e;   /* Expression: 0
                                        * Referenced by: '<S2>/Memory1'
                                        */
  real_T Memory3_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S2>/Memory3'
                                        */
  real_T Memory4_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S2>/Memory4'
                                        */
  real_T Memory6_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S2>/Memory6'
                                        */
  real_T Memory7_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S2>/Memory7'
                                        */
  real_T Memory9_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S2>/Memory9'
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

  /* Block signals */
  B_Pose_AutoCode_T Pose_AutoCode_B;

  /* Block states */
  DW_Pose_AutoCode_T Pose_AutoCode_DW;

  /* Real-Time Model */
  RT_MODEL_Pose_AutoCode_T Pose_AutoCode_M;

  /* private member function(s) for subsystem '<S2>/KalmanFilter'*/
  void Pose_AutoCode_KalmanFilter(real_T rtu_reset, real_T rtu_enable, real_T
    rtu_z, real_T rtu_R, real_T rtu_Q, real_T rtu_C, const
    KalmanFilterObjectState *rtu_state_in, B_KalmanFilter_Pose_AutoCode_T
    *localB);

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
 * '<S2>'   : 'Pose_AutoCode/PoseModel/PoseAccelerationBlock'
 * '<S3>'   : 'Pose_AutoCode/PoseModel/SensorPostProcessing'
 * '<S4>'   : 'Pose_AutoCode/PoseModel/SignalLinker'
 * '<S5>'   : 'Pose_AutoCode/PoseModel/TimeCompensate'
 * '<S6>'   : 'Pose_AutoCode/PoseModel/PoseAccelerationBlock/KalmanFilter'
 * '<S7>'   : 'Pose_AutoCode/PoseModel/PoseAccelerationBlock/KalmanFilter1'
 * '<S8>'   : 'Pose_AutoCode/PoseModel/PoseAccelerationBlock/KalmanFilter2'
 * '<S9>'   : 'Pose_AutoCode/PoseModel/PoseAccelerationBlock/PostKalmanFilter'
 * '<S10>'  : 'Pose_AutoCode/PoseModel/PoseAccelerationBlock/PostKalmanFilter1'
 * '<S11>'  : 'Pose_AutoCode/PoseModel/PoseAccelerationBlock/PostKalmanFilter2'
 * '<S12>'  : 'Pose_AutoCode/PoseModel/PoseAccelerationBlock/PreKalmanFilterXAcceleration'
 * '<S13>'  : 'Pose_AutoCode/PoseModel/PoseAccelerationBlock/PreKalmanFilterYAcceleration'
 * '<S14>'  : 'Pose_AutoCode/PoseModel/PoseAccelerationBlock/PreKalmanFilterZAcceleration'
 * '<S15>'  : 'Pose_AutoCode/PoseModel/TimeCompensate/Time Compensate Signal x12'
 * '<S16>'  : 'Pose_AutoCode/PoseModel/TimeCompensate/Time Compensate Signal x12/TimeCompensator1'
 */
#endif                                 /* RTW_HEADER_Pose_AutoCode_h_ */
