/*
 * Pose_AutoCode.cpp
 *
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * Code generation for model "Pose_AutoCode".
 *
 * Model version              : 1.98
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C++ source code generated on : Tue Feb 18 05:27:33 2020
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Pose_AutoCode.h"
#include "Pose_AutoCode_private.h"

const OutputSignalObject Pose_AutoCode_rtZOutputSignalObject = {
  0.0,                                 /* value */
  0U,                                  /* status */
  0.0                                  /* rms */
} ;                                    /* OutputSignalObject ground */

/* Function for MATLAB Function: '<S5>/TimeCompensator1' */
void Pose_AutoCodeModelClass::Pose_Auto_timecompensate_signal(real_T
  current_time, uint32_T update_count_in, real_T b_index, real_T value_in,
  uint8_T status_in, real_T rms_in, const real_T buffers_x_in[24], const real_T
  buffers_t_in[24], real_T *value, uint8_T *status, real_T *rms, uint32_T
  *update_count, real_T buffers_x[24], real_T buffers_t[24])
{
  int32_T i;
  int32_T b_index_tmp;
  int32_T b_index_tmp_0;
  int32_T buffers_x_tmp;
  *update_count = update_count_in;
  std::memcpy(&buffers_x[0], &buffers_x_in[0], 24U * sizeof(real_T));
  std::memcpy(&buffers_t[0], &buffers_t_in[0], 24U * sizeof(real_T));
  if (status_in == SIGNALSTATE_UPDATED_) {
    *value = value_in;
    *status = 3U;
    *rms = rms_in;
    if (update_count_in <= 8U) {
      buffers_x_tmp = (static_cast<int32_T>(b_index) + 3 * (static_cast<int32_T>
        (update_count_in) - 1)) - 1;
      buffers_x[buffers_x_tmp] = value_in;
      buffers_t[buffers_x_tmp] = current_time;
    } else {
      b_index_tmp = static_cast<int32_T>(b_index);
      b_index_tmp_0 = b_index_tmp - 1;
      buffers_x[b_index_tmp_0 + 21] = value_in;
      for (i = 0; i < 7; i++) {
        buffers_x_tmp = (i + 1) * 3;
        buffers_x[b_index_tmp_0 + 3 * i] = buffers_x_in[(buffers_x_tmp +
          b_index_tmp) - 1];
        buffers_t[b_index_tmp_0 + 3 * i] = buffers_t_in[(buffers_x_tmp +
          b_index_tmp) - 1];
      }

      buffers_t[b_index_tmp_0 + 21] = current_time;
    }

    *update_count = update_count_in + 1U;
    if (*update_count < update_count_in) {
      *update_count = MAX_uint32_T;
    }
  } else {
    *value = value_in;
    *status = SIGNALSTATE_HOLD_;
    *rms = rms_in;
  }
}

/* Model step function */
void Pose_AutoCodeModelClass::step()
{
  uint8_T rtb_state_initialized;
  uint32_T rtb_state_signal1_update_counte;
  uint32_T rtb_state_signal2_update_counte;
  uint32_T rtb_state_signal3_update_counte;
  real_T rtb_buffers_x[24];
  real_T rtb_buffers_t[24];

  /* Outputs for Atomic SubSystem: '<S1>/TimeCompensate' */
  /* MATLAB Function: '<S5>/TimeCompensator1' incorporates:
   *  Memory: '<S5>/Memory'
   */
  rtb_state_initialized = Pose_AutoCode_DW.Memory_PreviousInput.initialized;
  rtb_state_signal1_update_counte =
    Pose_AutoCode_DW.Memory_PreviousInput.signal1_update_counter;
  rtb_state_signal2_update_counte =
    Pose_AutoCode_DW.Memory_PreviousInput.signal2_update_counter;
  rtb_state_signal3_update_counte =
    Pose_AutoCode_DW.Memory_PreviousInput.signal3_update_counter;
  if (Pose_AutoCode_DW.Memory_PreviousInput.initialized == 0) {
    rtb_state_signal1_update_counte = 1U;
    rtb_state_signal2_update_counte = 1U;
    rtb_state_signal3_update_counte = 1U;
    rtb_state_initialized = 1U;
  }

  /* Update for Memory: '<S5>/Memory' incorporates:
   *  BusCreator: '<S5>/Bus Creator1'
   *  BusCreator: '<S5>/Bus Creator4'
   *  Inport: '<Root>/accel1x_in'
   *  Inport: '<Root>/accel1y_in'
   *  Inport: '<Root>/current_time'
   *  MATLAB Function: '<S5>/TimeCompensator1'
   *  Memory: '<S5>/Memory1'
   *  Memory: '<S5>/Memory2'
   *  Outport: '<Root>/timed_signals_output'
   */
  Pose_Auto_timecompensate_signal(Pose_AutoCode_U.current_time,
    rtb_state_signal1_update_counte, 1.0, Pose_AutoCode_U.accel1x_in.value,
    Pose_AutoCode_U.accel1x_in.status, Pose_AutoCode_U.accel1x_in.rms,
    Pose_AutoCode_DW.Memory2_PreviousInput,
    Pose_AutoCode_DW.Memory1_PreviousInput,
    &Pose_AutoCode_Y.timed_signals_output[0].value,
    &Pose_AutoCode_Y.timed_signals_output[0].status,
    &Pose_AutoCode_Y.timed_signals_output[0].rms,
    &Pose_AutoCode_DW.Memory_PreviousInput.signal1_update_counter, rtb_buffers_x,
    rtb_buffers_t);
  Pose_Auto_timecompensate_signal(Pose_AutoCode_U.current_time,
    rtb_state_signal2_update_counte, 2.0, Pose_AutoCode_U.accel1y_in.value,
    Pose_AutoCode_U.accel1y_in.status, Pose_AutoCode_U.accel1y_in.rms,
    Pose_AutoCode_DW.Memory2_PreviousInput,
    Pose_AutoCode_DW.Memory1_PreviousInput,
    &Pose_AutoCode_Y.timed_signals_output[1].value,
    &Pose_AutoCode_Y.timed_signals_output[1].status,
    &Pose_AutoCode_Y.timed_signals_output[1].rms,
    &Pose_AutoCode_DW.Memory_PreviousInput.signal2_update_counter, rtb_buffers_x,
    rtb_buffers_t);

  /* Memory: '<S5>/Memory2' */
  std::memcpy(&rtb_buffers_x[0], &Pose_AutoCode_DW.Memory2_PreviousInput[0], 24U
              * sizeof(real_T));

  /* Memory: '<S5>/Memory1' */
  std::memcpy(&rtb_buffers_t[0], &Pose_AutoCode_DW.Memory1_PreviousInput[0], 24U
              * sizeof(real_T));

  /* Update for Memory: '<S5>/Memory' incorporates:
   *  BusCreator: '<S5>/Bus Creator2'
   *  Inport: '<Root>/accel1z_in'
   *  Inport: '<Root>/current_time'
   *  MATLAB Function: '<S5>/TimeCompensator1'
   *  Memory: '<S5>/Memory1'
   *  Memory: '<S5>/Memory2'
   *  Outport: '<Root>/timed_signals_output'
   */
  Pose_Auto_timecompensate_signal(Pose_AutoCode_U.current_time,
    rtb_state_signal3_update_counte, 3.0, Pose_AutoCode_U.accel1z_in.value,
    Pose_AutoCode_U.accel1z_in.status, Pose_AutoCode_U.accel1z_in.rms,
    rtb_buffers_x, rtb_buffers_t, &Pose_AutoCode_Y.timed_signals_output[2].value,
    &Pose_AutoCode_Y.timed_signals_output[2].status,
    &Pose_AutoCode_Y.timed_signals_output[2].rms,
    &Pose_AutoCode_DW.Memory_PreviousInput.signal3_update_counter,
    Pose_AutoCode_DW.Memory2_PreviousInput,
    Pose_AutoCode_DW.Memory1_PreviousInput);
  Pose_AutoCode_DW.Memory_PreviousInput.initialized = rtb_state_initialized;

  /* End of Outputs for SubSystem: '<S1>/TimeCompensate' */
}

/* Model initialize function */
void Pose_AutoCodeModelClass::initialize()
{
  /* Registration code */

  /* states (dwork) */
  (void) std::memset(static_cast<void *>(&Pose_AutoCode_DW), 0,
                     sizeof(DW_Pose_AutoCode_T));

  /* external inputs */
  (void)std::memset(&Pose_AutoCode_U, 0, sizeof(ExtU_Pose_AutoCode_T));

  /* external outputs */
  (void) std::memset(&Pose_AutoCode_Y.timed_signals_output[0], 0,
                     3U*sizeof(OutputSignalObject));

  {
    int32_T i;

    /* SystemInitialize for Atomic SubSystem: '<S1>/TimeCompensate' */
    /* InitializeConditions for Memory: '<S5>/Memory' */
    Pose_AutoCode_DW.Memory_PreviousInput =
      Pose_AutoCode_P.Memory_InitialCondition;
    for (i = 0; i < 24; i++) {
      /* InitializeConditions for Memory: '<S5>/Memory1' */
      Pose_AutoCode_DW.Memory1_PreviousInput[i] =
        Pose_AutoCode_P.Memory1_InitialCondition;

      /* InitializeConditions for Memory: '<S5>/Memory2' */
      Pose_AutoCode_DW.Memory2_PreviousInput[i] =
        Pose_AutoCode_P.Memory2_InitialCondition;
    }

    /* End of SystemInitialize for SubSystem: '<S1>/TimeCompensate' */
  }
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
