/*
 * Pose_AutoCode.cpp
 *
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * Code generation for model "Pose_AutoCode".
 *
 * Model version              : 1.117
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C++ source code generated on : Sun Mar  1 16:57:12 2020
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Pose_AutoCode.h"
#include "Pose_AutoCode_private.h"

/*
 * Output and update for atomic system:
 *    '<S2>/KalmanFilter'
 *    '<S2>/KalmanFilter1'
 *    '<S2>/KalmanFilter2'
 */
void Pose_AutoCodeModelClass::Pose_AutoCode_KalmanFilter(real_T rtu_reset,
  real_T rtu_enable, real_T rtu_z, real_T rtu_R, real_T rtu_Q, real_T rtu_C,
  const KalmanFilterObjectState *rtu_state_in, B_KalmanFilter_Pose_AutoCode_T
  *localB)
{
  KalmanFilterObjectState state;
  real_T xhat;
  real_T P;
  uint32_T qY;
  real_T G_tmp;
  P = localB->P;
  xhat = localB->xhat;
  state = *rtu_state_in;
  if (rtu_reset == 1.0) {
    state.initialized = 0U;
    state.update_counter = 0U;
  }

  if (rtu_enable == 1.0) {
    if (state.initialized == 0) {
      xhat = rtu_z;
      state.initialized = 1U;
      P = 1.0;
    }

    qY = state.update_counter + 1U;
    if (qY < state.update_counter) {
      qY = MAX_uint32_T;
    }

    state.update_counter = qY;
    P += rtu_Q;
    G_tmp = rtu_C * P;
    G_tmp *= 1.0 / (G_tmp * rtu_C + rtu_R);
    P *= 1.0 - G_tmp * rtu_C;
    xhat += (rtu_z - rtu_C * xhat) * G_tmp;
  }

  localB->xhat = xhat;
  localB->P = P;
  localB->state = state;
}

/*
 * Output and update for atomic system:
 *    '<S2>/PostKalmanFilter'
 *    '<S2>/PostKalmanFilter1'
 *    '<S2>/PostKalmanFilter2'
 */
void Pose_AutoCodeModelClass::Pose_AutoCode_PostKalmanFilter(real_T rtu_xhat,
  real_T rtu_P, B_PostKalmanFilter_Pose_AutoC_T *localB)
{
  localB->value = rtu_xhat;
  localB->status = SIGNALSTATE_UPDATED_;
  localB->rms = rtu_P;
}

/* Function for MATLAB Function: '<S17>/TimeCompensator1' */
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
  real_T value1;
  real_T value2;
  real_T value3;
  uint8_T rtb_state_initialized;
  uint32_T rtb_state_signal1_update_counte;
  uint32_T rtb_state_signal2_update_counte;
  uint32_T rtb_state_signal3_update_counte;
  real_T rtb_buffers_x[24];
  real_T rtb_buffers_t[24];
  KalmanFilterObjectState *rtb_Memory2_a_0;

  /* Outputs for Atomic SubSystem: '<S1>/TimeCompensate' */
  /* MATLAB Function: '<S17>/TimeCompensator1' incorporates:
   *  Memory: '<S17>/Memory'
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

  /* Update for Memory: '<S17>/Memory' incorporates:
   *  BusCreator: '<S17>/Bus Creator1'
   *  BusCreator: '<S17>/Bus Creator4'
   *  Inport: '<Root>/accel1x_in'
   *  Inport: '<Root>/accel1y_in'
   *  Inport: '<Root>/current_time'
   *  MATLAB Function: '<S17>/TimeCompensator1'
   *  Memory: '<S17>/Memory1'
   *  Memory: '<S17>/Memory2'
   *  Outport: '<Root>/timed_signals_output'
   */
  Pose_Auto_timecompensate_signal(Pose_AutoCode_U.current_time,
    rtb_state_signal1_update_counte, 1.0, Pose_AutoCode_U.accel1x_in.value,
    Pose_AutoCode_U.accel1x_in.status, Pose_AutoCode_U.accel1x_in.rms,
    Pose_AutoCode_DW.Memory2_PreviousInput,
    Pose_AutoCode_DW.Memory1_PreviousInput_m, &value1,
    &Pose_AutoCode_Y.timed_signals_output[0].status,
    &Pose_AutoCode_Y.timed_signals_output[0].rms,
    &Pose_AutoCode_DW.Memory_PreviousInput.signal1_update_counter, rtb_buffers_x,
    rtb_buffers_t);
  Pose_Auto_timecompensate_signal(Pose_AutoCode_U.current_time,
    rtb_state_signal2_update_counte, 2.0, Pose_AutoCode_U.accel1y_in.value,
    Pose_AutoCode_U.accel1y_in.status, Pose_AutoCode_U.accel1y_in.rms,
    Pose_AutoCode_DW.Memory2_PreviousInput,
    Pose_AutoCode_DW.Memory1_PreviousInput_m, &value2,
    &Pose_AutoCode_Y.timed_signals_output[1].status,
    &Pose_AutoCode_Y.timed_signals_output[1].rms,
    &Pose_AutoCode_DW.Memory_PreviousInput.signal2_update_counter, rtb_buffers_x,
    rtb_buffers_t);

  /* Memory: '<S17>/Memory2' */
  std::memcpy(&rtb_buffers_x[0], &Pose_AutoCode_DW.Memory2_PreviousInput[0], 24U
              * sizeof(real_T));

  /* Memory: '<S17>/Memory1' */
  std::memcpy(&rtb_buffers_t[0], &Pose_AutoCode_DW.Memory1_PreviousInput_m[0],
              24U * sizeof(real_T));

  /* Update for Memory: '<S17>/Memory' incorporates:
   *  BusCreator: '<S17>/Bus Creator2'
   *  Inport: '<Root>/accel1z_in'
   *  Inport: '<Root>/current_time'
   *  MATLAB Function: '<S17>/TimeCompensator1'
   *  Memory: '<S17>/Memory1'
   *  Memory: '<S17>/Memory2'
   *  Outport: '<Root>/timed_signals_output'
   */
  Pose_Auto_timecompensate_signal(Pose_AutoCode_U.current_time,
    rtb_state_signal3_update_counte, 3.0, Pose_AutoCode_U.accel1z_in.value,
    Pose_AutoCode_U.accel1z_in.status, Pose_AutoCode_U.accel1z_in.rms,
    rtb_buffers_x, rtb_buffers_t, &value3,
    &Pose_AutoCode_Y.timed_signals_output[2].status,
    &Pose_AutoCode_Y.timed_signals_output[2].rms,
    &Pose_AutoCode_DW.Memory_PreviousInput.signal3_update_counter,
    Pose_AutoCode_DW.Memory2_PreviousInput,
    Pose_AutoCode_DW.Memory1_PreviousInput_m);

  /* BusCreator: '<S17>/Bus Creator1' incorporates:
   *  MATLAB Function: '<S17>/TimeCompensator1'
   *  Outport: '<Root>/timed_signals_output'
   */
  Pose_AutoCode_Y.timed_signals_output[1].value = value2;

  /* BusCreator: '<S17>/Bus Creator2' incorporates:
   *  MATLAB Function: '<S17>/TimeCompensator1'
   *  Outport: '<Root>/timed_signals_output'
   */
  Pose_AutoCode_Y.timed_signals_output[2].value = value3;

  /* BusCreator: '<S17>/Bus Creator4' incorporates:
   *  MATLAB Function: '<S17>/TimeCompensator1'
   *  Outport: '<Root>/timed_signals_output'
   */
  Pose_AutoCode_Y.timed_signals_output[0].value = value1;

  /* Update for Memory: '<S17>/Memory' */
  Pose_AutoCode_DW.Memory_PreviousInput.initialized = rtb_state_initialized;

  /* End of Outputs for SubSystem: '<S1>/TimeCompensate' */

  /* Memory: '<S2>/Memory2' */
  rtb_Memory2_a_0 = &Pose_AutoCode_DW.Memory2_PreviousInput_j;

  /* MATLAB Function: '<S2>/KalmanFilter' incorporates:
   *  Constant: '<S1>/enable_poseaccelerationfilter'
   *  Constant: '<S1>/reset_poseaccelerationfilter'
   *  MATLAB Function: '<S17>/TimeCompensator1'
   *  MATLAB Function: '<S2>/PreKalmanFilterXAcceleration'
   *  Memory: '<S2>/Memory1'
   *  Memory: '<S2>/Memory3'
   */
  Pose_AutoCode_B.sf_KalmanFilter.P = Pose_AutoCode_DW.Memory3_PreviousInput;
  Pose_AutoCode_B.sf_KalmanFilter.xhat = Pose_AutoCode_DW.Memory1_PreviousInput;

  /* Outputs for Atomic SubSystem: '<S1>/TimeCompensate' */
  Pose_AutoCode_KalmanFilter(Pose_AutoCode_P.reset_poseaccelerationfilter_Va,
    Pose_AutoCode_P.enable_poseaccelerationfilter_V, value1, 0.64, 0.05, 1.0,
    rtb_Memory2_a_0, &Pose_AutoCode_B.sf_KalmanFilter);

  /* End of Outputs for SubSystem: '<S1>/TimeCompensate' */

  /* MATLAB Function: '<S2>/PostKalmanFilter' */
  Pose_AutoCode_PostKalmanFilter(Pose_AutoCode_B.sf_KalmanFilter.xhat,
    Pose_AutoCode_B.sf_KalmanFilter.P, &Pose_AutoCode_B.sf_PostKalmanFilter);

  /* SignalConversion generated from: '<S2>/Vector Concatenate' incorporates:
   *  Outport: '<Root>/pose_linearacceleration_signals_output'
   */
  Pose_AutoCode_Y.pose_linearacceleration_signals[0] =
    Pose_AutoCode_B.sf_PostKalmanFilter.value;

  /* Memory: '<S2>/Memory5' */
  rtb_Memory2_a_0 = &Pose_AutoCode_DW.Memory5_PreviousInput;

  /* MATLAB Function: '<S2>/KalmanFilter1' incorporates:
   *  Constant: '<S1>/enable_poseaccelerationfilter'
   *  Constant: '<S1>/reset_poseaccelerationfilter'
   *  MATLAB Function: '<S17>/TimeCompensator1'
   *  MATLAB Function: '<S2>/PreKalmanFilterYAcceleration'
   *  Memory: '<S2>/Memory4'
   *  Memory: '<S2>/Memory6'
   */
  Pose_AutoCode_B.sf_KalmanFilter1.P = Pose_AutoCode_DW.Memory6_PreviousInput;
  Pose_AutoCode_B.sf_KalmanFilter1.xhat = Pose_AutoCode_DW.Memory4_PreviousInput;

  /* Outputs for Atomic SubSystem: '<S1>/TimeCompensate' */
  Pose_AutoCode_KalmanFilter(Pose_AutoCode_P.reset_poseaccelerationfilter_Va,
    Pose_AutoCode_P.enable_poseaccelerationfilter_V, value2, 0.64, 0.05, 1.0,
    rtb_Memory2_a_0, &Pose_AutoCode_B.sf_KalmanFilter1);

  /* End of Outputs for SubSystem: '<S1>/TimeCompensate' */

  /* MATLAB Function: '<S2>/PostKalmanFilter1' */
  Pose_AutoCode_PostKalmanFilter(Pose_AutoCode_B.sf_KalmanFilter1.xhat,
    Pose_AutoCode_B.sf_KalmanFilter1.P, &Pose_AutoCode_B.sf_PostKalmanFilter1);

  /* SignalConversion generated from: '<S2>/Vector Concatenate' incorporates:
   *  Outport: '<Root>/pose_linearacceleration_signals_output'
   */
  Pose_AutoCode_Y.pose_linearacceleration_signals[1] =
    Pose_AutoCode_B.sf_PostKalmanFilter1.value;

  /* Memory: '<S2>/Memory8' */
  rtb_Memory2_a_0 = &Pose_AutoCode_DW.Memory8_PreviousInput;

  /* MATLAB Function: '<S2>/KalmanFilter2' incorporates:
   *  Constant: '<S1>/enable_poseaccelerationfilter'
   *  Constant: '<S1>/reset_poseaccelerationfilter'
   *  MATLAB Function: '<S17>/TimeCompensator1'
   *  MATLAB Function: '<S2>/PreKalmanFilterZAcceleration'
   *  Memory: '<S2>/Memory7'
   *  Memory: '<S2>/Memory9'
   */
  Pose_AutoCode_B.sf_KalmanFilter2.P = Pose_AutoCode_DW.Memory9_PreviousInput;
  Pose_AutoCode_B.sf_KalmanFilter2.xhat = Pose_AutoCode_DW.Memory7_PreviousInput;

  /* Outputs for Atomic SubSystem: '<S1>/TimeCompensate' */
  Pose_AutoCode_KalmanFilter(Pose_AutoCode_P.reset_poseaccelerationfilter_Va,
    Pose_AutoCode_P.enable_poseaccelerationfilter_V, value3, 0.64, 0.05, 1.0,
    rtb_Memory2_a_0, &Pose_AutoCode_B.sf_KalmanFilter2);

  /* End of Outputs for SubSystem: '<S1>/TimeCompensate' */

  /* MATLAB Function: '<S2>/PostKalmanFilter2' */
  Pose_AutoCode_PostKalmanFilter(Pose_AutoCode_B.sf_KalmanFilter2.xhat,
    Pose_AutoCode_B.sf_KalmanFilter2.P, &Pose_AutoCode_B.sf_PostKalmanFilter2);

  /* SignalConversion generated from: '<S2>/Vector Concatenate' incorporates:
   *  Outport: '<Root>/pose_linearacceleration_signals_output'
   */
  Pose_AutoCode_Y.pose_linearacceleration_signals[2] =
    Pose_AutoCode_B.sf_PostKalmanFilter2.value;

  /* Outport: '<Root>/pose_orientation_signals_output' */
  Pose_AutoCode_Y.pose_orientation_signals_output = 0.0;

  /* Update for Memory: '<S2>/Memory1' */
  Pose_AutoCode_DW.Memory1_PreviousInput = Pose_AutoCode_B.sf_KalmanFilter.xhat;

  /* Update for Memory: '<S2>/Memory3' */
  Pose_AutoCode_DW.Memory3_PreviousInput = Pose_AutoCode_B.sf_KalmanFilter.P;

  /* Update for Memory: '<S2>/Memory2' */
  Pose_AutoCode_DW.Memory2_PreviousInput_j =
    Pose_AutoCode_B.sf_KalmanFilter.state;

  /* Update for Memory: '<S2>/Memory4' */
  Pose_AutoCode_DW.Memory4_PreviousInput = Pose_AutoCode_B.sf_KalmanFilter1.xhat;

  /* Update for Memory: '<S2>/Memory6' */
  Pose_AutoCode_DW.Memory6_PreviousInput = Pose_AutoCode_B.sf_KalmanFilter1.P;

  /* Update for Memory: '<S2>/Memory5' */
  Pose_AutoCode_DW.Memory5_PreviousInput =
    Pose_AutoCode_B.sf_KalmanFilter1.state;

  /* Update for Memory: '<S2>/Memory7' */
  Pose_AutoCode_DW.Memory7_PreviousInput = Pose_AutoCode_B.sf_KalmanFilter2.xhat;

  /* Update for Memory: '<S2>/Memory9' */
  Pose_AutoCode_DW.Memory9_PreviousInput = Pose_AutoCode_B.sf_KalmanFilter2.P;

  /* Update for Memory: '<S2>/Memory8' */
  Pose_AutoCode_DW.Memory8_PreviousInput =
    Pose_AutoCode_B.sf_KalmanFilter2.state;
}

/* Model initialize function */
void Pose_AutoCodeModelClass::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* block I/O */
  (void) std::memset((static_cast<void *>(&Pose_AutoCode_B)), 0,
                     sizeof(B_Pose_AutoCode_T));

  /* states (dwork) */
  (void) std::memset(static_cast<void *>(&Pose_AutoCode_DW), 0,
                     sizeof(DW_Pose_AutoCode_T));

  /* external inputs */
  (void)std::memset(&Pose_AutoCode_U, 0, sizeof(ExtU_Pose_AutoCode_T));

  /* external outputs */
  (void) std::memset(static_cast<void *>(&Pose_AutoCode_Y), 0,
                     sizeof(ExtY_Pose_AutoCode_T));

  {
    int32_T i;

    /* InitializeConditions for Memory: '<S2>/Memory1' */
    Pose_AutoCode_DW.Memory1_PreviousInput =
      Pose_AutoCode_P.Memory1_InitialCondition_e;

    /* InitializeConditions for Memory: '<S2>/Memory3' */
    Pose_AutoCode_DW.Memory3_PreviousInput =
      Pose_AutoCode_P.Memory3_InitialCondition;

    /* InitializeConditions for Memory: '<S2>/Memory2' */
    Pose_AutoCode_DW.Memory2_PreviousInput_j =
      Pose_AutoCode_P.Memory2_InitialCondition;

    /* InitializeConditions for Memory: '<S2>/Memory4' */
    Pose_AutoCode_DW.Memory4_PreviousInput =
      Pose_AutoCode_P.Memory4_InitialCondition;

    /* InitializeConditions for Memory: '<S2>/Memory6' */
    Pose_AutoCode_DW.Memory6_PreviousInput =
      Pose_AutoCode_P.Memory6_InitialCondition;

    /* InitializeConditions for Memory: '<S2>/Memory5' */
    Pose_AutoCode_DW.Memory5_PreviousInput =
      Pose_AutoCode_P.Memory5_InitialCondition;

    /* InitializeConditions for Memory: '<S2>/Memory7' */
    Pose_AutoCode_DW.Memory7_PreviousInput =
      Pose_AutoCode_P.Memory7_InitialCondition;

    /* InitializeConditions for Memory: '<S2>/Memory9' */
    Pose_AutoCode_DW.Memory9_PreviousInput =
      Pose_AutoCode_P.Memory9_InitialCondition;

    /* InitializeConditions for Memory: '<S2>/Memory8' */
    Pose_AutoCode_DW.Memory8_PreviousInput =
      Pose_AutoCode_P.Memory8_InitialCondition;

    /* SystemInitialize for Atomic SubSystem: '<S1>/TimeCompensate' */
    /* InitializeConditions for Memory: '<S17>/Memory' */
    Pose_AutoCode_DW.Memory_PreviousInput =
      Pose_AutoCode_P.Memory_InitialCondition;
    for (i = 0; i < 24; i++) {
      /* InitializeConditions for Memory: '<S17>/Memory1' */
      Pose_AutoCode_DW.Memory1_PreviousInput_m[i] =
        Pose_AutoCode_P.Memory1_InitialCondition;

      /* InitializeConditions for Memory: '<S17>/Memory2' */
      Pose_AutoCode_DW.Memory2_PreviousInput[i] =
        Pose_AutoCode_P.Memory2_InitialCondition_o;
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
