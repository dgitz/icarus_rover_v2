/*
 * Pose_AutoCode.cpp
 *
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * Code generation for model "Pose_AutoCode".
 *
 * Model version              : 1.137
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C++ source code generated on : Wed Mar  4 21:29:51 2020
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
 *    '<S3>/KalmanFilter'
 *    '<S3>/KalmanFilter1'
 *    '<S4>/KalmanFilter'
 *    '<S4>/KalmanFilter1'
 *    '<S4>/KalmanFilter2'
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
 *    '<S4>/PostKalmanFilter'
 *    '<S4>/PostKalmanFilter1'
 *    '<S4>/PostKalmanFilter2'
 */
void Pose_AutoCodeModelClass::Pose_AutoCode_PostKalmanFilter(real_T rtu_xhat,
  real_T rtu_P, B_PostKalmanFilter_Pose_AutoC_T *localB)
{
  localB->value = rtu_xhat;
  localB->status = SIGNALSTATE_UPDATED_;
  localB->rms = rtu_P;
}

/* Function for MATLAB Function: '<S32>/TimeCompensator_x6' */
void Pose_AutoCodeModelClass::Pose_Auto_timecompensate_signal(real_T
  current_time, uint32_T update_count_in, real_T b_index, real_T value_in,
  uint8_T status_in, real_T rms_in, const real_T buffers_x_in[48], const real_T
  buffers_t_in[48], real_T *value, uint8_T *status, real_T *rms, uint32_T
  *update_count, real_T buffers_x[48], real_T buffers_t[48])
{
  int32_T i;
  int32_T b_index_tmp;
  int32_T b_index_tmp_0;
  int32_T buffers_x_tmp;
  *update_count = update_count_in;
  std::memcpy(&buffers_x[0], &buffers_x_in[0], 48U * sizeof(real_T));
  std::memcpy(&buffers_t[0], &buffers_t_in[0], 48U * sizeof(real_T));
  if (status_in == SIGNALSTATE_UPDATED_) {
    *value = value_in;
    *status = 3U;
    *rms = rms_in;
    if (update_count_in <= 8U) {
      buffers_x_tmp = (static_cast<int32_T>(b_index) + 6 * (static_cast<int32_T>
        (update_count_in) - 1)) - 1;
      buffers_x[buffers_x_tmp] = value_in;
      buffers_t[buffers_x_tmp] = current_time;
    } else {
      b_index_tmp = static_cast<int32_T>(b_index);
      b_index_tmp_0 = b_index_tmp - 1;
      buffers_x[b_index_tmp_0 + 42] = value_in;
      for (i = 0; i < 7; i++) {
        buffers_x_tmp = (i + 1) * 6;
        buffers_x[b_index_tmp_0 + 6 * i] = buffers_x_in[(buffers_x_tmp +
          b_index_tmp) - 1];
        buffers_t[b_index_tmp_0 + 6 * i] = buffers_t_in[(buffers_x_tmp +
          b_index_tmp) - 1];
      }

      buffers_t[b_index_tmp_0 + 42] = current_time;
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

    y = atan2(static_cast<real_T>(u0_0), static_cast<real_T>(u1_0));
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

/* Model step function */
void Pose_AutoCodeModelClass::step()
{
  real_T value1;
  real_T value2;
  real_T value3;
  real_T value4;
  real_T value5;
  real_T value6;
  uint8_T rtb_state_initialized;
  uint32_T rtb_state_signal1_update_counte;
  uint32_T rtb_state_signal2_update_counte;
  uint32_T rtb_state_signal3_update_counte;
  uint32_T rtb_state_signal4_update_counte;
  uint32_T rtb_state_signal5_update_counte;
  uint32_T rtb_state_signal6_update_counte;
  real_T rtb_buffers_x[48];
  real_T rtb_buffers_t[48];
  KalmanFilterObjectState *rtb_Memory2_a_0;

  /* Outputs for Atomic SubSystem: '<S1>/TimeCompensate' */
  /* MATLAB Function: '<S32>/TimeCompensator_x6' incorporates:
   *  Memory: '<S32>/Memory'
   */
  rtb_state_initialized = Pose_AutoCode_DW.Memory_PreviousInput.initialized;
  rtb_state_signal1_update_counte =
    Pose_AutoCode_DW.Memory_PreviousInput.signal1_update_counter;
  rtb_state_signal2_update_counte =
    Pose_AutoCode_DW.Memory_PreviousInput.signal2_update_counter;
  rtb_state_signal3_update_counte =
    Pose_AutoCode_DW.Memory_PreviousInput.signal3_update_counter;
  rtb_state_signal4_update_counte =
    Pose_AutoCode_DW.Memory_PreviousInput.signal4_update_counter;
  rtb_state_signal5_update_counte =
    Pose_AutoCode_DW.Memory_PreviousInput.signal5_update_counter;
  rtb_state_signal6_update_counte =
    Pose_AutoCode_DW.Memory_PreviousInput.signal6_update_counter;
  if (Pose_AutoCode_DW.Memory_PreviousInput.initialized == 0) {
    rtb_state_signal1_update_counte = 1U;
    rtb_state_signal2_update_counte = 1U;
    rtb_state_signal3_update_counte = 1U;
    rtb_state_signal4_update_counte = 1U;
    rtb_state_signal5_update_counte = 1U;
    rtb_state_signal6_update_counte = 1U;
    rtb_state_initialized = 1U;
  }

  /* Update for Memory: '<S32>/Memory' incorporates:
   *  BusCreator: '<S32>/Bus Creator1'
   *  BusCreator: '<S32>/Bus Creator2'
   *  BusCreator: '<S32>/Bus Creator3'
   *  BusCreator: '<S32>/Bus Creator4'
   *  BusCreator: '<S32>/Bus Creator6'
   *  Inport: '<Root>/accel1x_in'
   *  Inport: '<Root>/accel1y_in'
   *  Inport: '<Root>/accel1z_in'
   *  Inport: '<Root>/current_time'
   *  Inport: '<Root>/rotationrate1x_in'
   *  Inport: '<Root>/rotationrate1y_in'
   *  MATLAB Function: '<S32>/TimeCompensator_x6'
   *  Memory: '<S32>/Memory1'
   *  Memory: '<S32>/Memory2'
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
  Pose_Auto_timecompensate_signal(Pose_AutoCode_U.current_time,
    rtb_state_signal3_update_counte, 3.0, Pose_AutoCode_U.accel1z_in.value,
    Pose_AutoCode_U.accel1z_in.status, Pose_AutoCode_U.accel1z_in.rms,
    Pose_AutoCode_DW.Memory2_PreviousInput,
    Pose_AutoCode_DW.Memory1_PreviousInput_m, &value3,
    &Pose_AutoCode_Y.timed_signals_output[2].status,
    &Pose_AutoCode_Y.timed_signals_output[2].rms,
    &Pose_AutoCode_DW.Memory_PreviousInput.signal3_update_counter, rtb_buffers_x,
    rtb_buffers_t);
  Pose_Auto_timecompensate_signal(Pose_AutoCode_U.current_time,
    rtb_state_signal4_update_counte, 3.0,
    Pose_AutoCode_U.rotationrate1x_in.value,
    Pose_AutoCode_U.rotationrate1x_in.status,
    Pose_AutoCode_U.rotationrate1x_in.rms,
    Pose_AutoCode_DW.Memory2_PreviousInput,
    Pose_AutoCode_DW.Memory1_PreviousInput_m, &value4,
    &Pose_AutoCode_Y.timed_signals_output[3].status,
    &Pose_AutoCode_Y.timed_signals_output[3].rms,
    &Pose_AutoCode_DW.Memory_PreviousInput.signal4_update_counter, rtb_buffers_x,
    rtb_buffers_t);
  Pose_Auto_timecompensate_signal(Pose_AutoCode_U.current_time,
    rtb_state_signal5_update_counte, 4.0,
    Pose_AutoCode_U.rotationrate1y_in.value,
    Pose_AutoCode_U.rotationrate1y_in.status,
    Pose_AutoCode_U.rotationrate1y_in.rms,
    Pose_AutoCode_DW.Memory2_PreviousInput,
    Pose_AutoCode_DW.Memory1_PreviousInput_m, &value5,
    &Pose_AutoCode_Y.timed_signals_output[4].status,
    &Pose_AutoCode_Y.timed_signals_output[4].rms,
    &Pose_AutoCode_DW.Memory_PreviousInput.signal5_update_counter, rtb_buffers_x,
    rtb_buffers_t);

  /* Memory: '<S32>/Memory2' */
  std::memcpy(&rtb_buffers_x[0], &Pose_AutoCode_DW.Memory2_PreviousInput[0], 48U
              * sizeof(real_T));

  /* Memory: '<S32>/Memory1' */
  std::memcpy(&rtb_buffers_t[0], &Pose_AutoCode_DW.Memory1_PreviousInput_m[0],
              48U * sizeof(real_T));

  /* Update for Memory: '<S32>/Memory' incorporates:
   *  BusCreator: '<S32>/Bus Creator5'
   *  Inport: '<Root>/current_time'
   *  Inport: '<Root>/rotationrate1z_in'
   *  MATLAB Function: '<S32>/TimeCompensator_x6'
   *  Memory: '<S32>/Memory1'
   *  Memory: '<S32>/Memory2'
   *  Outport: '<Root>/timed_signals_output'
   */
  Pose_Auto_timecompensate_signal(Pose_AutoCode_U.current_time,
    rtb_state_signal6_update_counte, 5.0,
    Pose_AutoCode_U.rotationrate1z_in.value,
    Pose_AutoCode_U.rotationrate1z_in.status,
    Pose_AutoCode_U.rotationrate1z_in.rms, rtb_buffers_x, rtb_buffers_t, &value6,
    &Pose_AutoCode_Y.timed_signals_output[5].status,
    &Pose_AutoCode_Y.timed_signals_output[5].rms,
    &Pose_AutoCode_DW.Memory_PreviousInput.signal6_update_counter,
    Pose_AutoCode_DW.Memory2_PreviousInput,
    Pose_AutoCode_DW.Memory1_PreviousInput_m);

  /* BusCreator: '<S32>/Bus Creator1' incorporates:
   *  MATLAB Function: '<S32>/TimeCompensator_x6'
   *  Outport: '<Root>/timed_signals_output'
   */
  Pose_AutoCode_Y.timed_signals_output[1].value = value2;

  /* BusCreator: '<S32>/Bus Creator2' incorporates:
   *  MATLAB Function: '<S32>/TimeCompensator_x6'
   *  Outport: '<Root>/timed_signals_output'
   */
  Pose_AutoCode_Y.timed_signals_output[2].value = value3;

  /* BusCreator: '<S32>/Bus Creator3' incorporates:
   *  MATLAB Function: '<S32>/TimeCompensator_x6'
   *  Outport: '<Root>/timed_signals_output'
   */
  Pose_AutoCode_Y.timed_signals_output[4].value = value5;

  /* BusCreator: '<S32>/Bus Creator4' incorporates:
   *  MATLAB Function: '<S32>/TimeCompensator_x6'
   *  Outport: '<Root>/timed_signals_output'
   */
  Pose_AutoCode_Y.timed_signals_output[0].value = value1;

  /* BusCreator: '<S32>/Bus Creator5' incorporates:
   *  MATLAB Function: '<S32>/TimeCompensator_x6'
   *  Outport: '<Root>/timed_signals_output'
   */
  Pose_AutoCode_Y.timed_signals_output[5].value = value6;

  /* BusCreator: '<S32>/Bus Creator6' incorporates:
   *  MATLAB Function: '<S32>/TimeCompensator_x6'
   *  Outport: '<Root>/timed_signals_output'
   */
  Pose_AutoCode_Y.timed_signals_output[3].value = value4;

  /* Update for Memory: '<S32>/Memory' */
  Pose_AutoCode_DW.Memory_PreviousInput.initialized = rtb_state_initialized;

  /* End of Outputs for SubSystem: '<S1>/TimeCompensate' */

  /* Memory: '<S2>/Memory2' */
  rtb_Memory2_a_0 = &Pose_AutoCode_DW.Memory2_PreviousInput_j;

  /* MATLAB Function: '<S2>/KalmanFilter' incorporates:
   *  Constant: '<S1>/enable_poseaccelerationfilter'
   *  Constant: '<S1>/reset_poseaccelerationfilter'
   *  MATLAB Function: '<S2>/PreKalmanFilterXAcceleration'
   *  MATLAB Function: '<S32>/TimeCompensator_x6'
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

  /* BusCreator: '<S2>/Bus Creator1' incorporates:
   *  Outport: '<Root>/pose_linearacceleration_signals_output'
   */
  Pose_AutoCode_Y.pose_linearacceleration_signals[0].value =
    Pose_AutoCode_B.sf_PostKalmanFilter.value;
  Pose_AutoCode_Y.pose_linearacceleration_signals[0].status =
    Pose_AutoCode_B.sf_PostKalmanFilter.status;
  Pose_AutoCode_Y.pose_linearacceleration_signals[0].rms =
    Pose_AutoCode_B.sf_PostKalmanFilter.rms;

  /* Memory: '<S2>/Memory5' */
  rtb_Memory2_a_0 = &Pose_AutoCode_DW.Memory5_PreviousInput;

  /* MATLAB Function: '<S2>/KalmanFilter1' incorporates:
   *  Constant: '<S1>/enable_poseaccelerationfilter'
   *  Constant: '<S1>/reset_poseaccelerationfilter'
   *  MATLAB Function: '<S2>/PreKalmanFilterYAcceleration'
   *  MATLAB Function: '<S32>/TimeCompensator_x6'
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

  /* BusCreator: '<S2>/Bus Creator3' incorporates:
   *  Outport: '<Root>/pose_linearacceleration_signals_output'
   */
  Pose_AutoCode_Y.pose_linearacceleration_signals[1].value =
    Pose_AutoCode_B.sf_PostKalmanFilter1.value;
  Pose_AutoCode_Y.pose_linearacceleration_signals[1].status =
    Pose_AutoCode_B.sf_PostKalmanFilter1.status;
  Pose_AutoCode_Y.pose_linearacceleration_signals[1].rms =
    Pose_AutoCode_B.sf_PostKalmanFilter1.rms;

  /* Memory: '<S2>/Memory8' */
  rtb_Memory2_a_0 = &Pose_AutoCode_DW.Memory8_PreviousInput;

  /* MATLAB Function: '<S2>/KalmanFilter2' incorporates:
   *  Constant: '<S1>/enable_poseaccelerationfilter'
   *  Constant: '<S1>/reset_poseaccelerationfilter'
   *  MATLAB Function: '<S2>/PreKalmanFilterZAcceleration'
   *  MATLAB Function: '<S32>/TimeCompensator_x6'
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

  /* BusCreator: '<S2>/Bus Creator4' incorporates:
   *  Outport: '<Root>/pose_linearacceleration_signals_output'
   */
  Pose_AutoCode_Y.pose_linearacceleration_signals[2].value =
    Pose_AutoCode_B.sf_PostKalmanFilter2.value;
  Pose_AutoCode_Y.pose_linearacceleration_signals[2].status =
    Pose_AutoCode_B.sf_PostKalmanFilter2.status;
  Pose_AutoCode_Y.pose_linearacceleration_signals[2].rms =
    Pose_AutoCode_B.sf_PostKalmanFilter2.rms;

  /* Memory: '<S3>/Memory2' */
  rtb_Memory2_a_0 = &Pose_AutoCode_DW.Memory2_PreviousInput_m;

  /* MATLAB Function: '<S3>/KalmanFilter' incorporates:
   *  Memory: '<S3>/Memory1'
   *  Memory: '<S3>/Memory3'
   */
  Pose_AutoCode_B.sf_KalmanFilter_f.P = Pose_AutoCode_DW.Memory3_PreviousInput_o;
  Pose_AutoCode_B.sf_KalmanFilter_f.xhat =
    Pose_AutoCode_DW.Memory1_PreviousInput_e;

  /* MATLAB Function: '<S3>/PreKalmanFilterXOrientation' incorporates:
   *  BusCreator: '<S2>/Bus Creator4'
   */
  if (Pose_AutoCode_B.sf_PostKalmanFilter2.value < 0.0) {
    value1 = -1.0;
  } else if (Pose_AutoCode_B.sf_PostKalmanFilter2.value > 0.0) {
    value1 = 1.0;
  } else if (Pose_AutoCode_B.sf_PostKalmanFilter2.value == 0.0) {
    value1 = 0.0;
  } else {
    value1 = (rtNaN);
  }

  /* MATLAB Function: '<S3>/KalmanFilter' incorporates:
   *  BusCreator: '<S2>/Bus Creator1'
   *  BusCreator: '<S2>/Bus Creator3'
   *  BusCreator: '<S2>/Bus Creator4'
   *  Constant: '<S1>/enable_poseorientationblock'
   *  Constant: '<S1>/reset_poseorientationblock'
   *  MATLAB Function: '<S3>/PreKalmanFilterXOrientation'
   */
  Pose_AutoCode_KalmanFilter(Pose_AutoCode_P.reset_poseorientationblock_Valu,
    Pose_AutoCode_P.enable_poseorientationblock_Val, rt_atan2d_snf
    (Pose_AutoCode_B.sf_PostKalmanFilter1.value, value1 * std::sqrt
     (Pose_AutoCode_B.sf_PostKalmanFilter2.value *
      Pose_AutoCode_B.sf_PostKalmanFilter2.value + 0.01 *
      (Pose_AutoCode_B.sf_PostKalmanFilter.value *
       Pose_AutoCode_B.sf_PostKalmanFilter.value))), 0.64, 0.05, 1.0,
    rtb_Memory2_a_0, &Pose_AutoCode_B.sf_KalmanFilter_f);

  /* BusCreator: '<S3>/Bus Creator1' incorporates:
   *  MATLAB Function: '<S3>/PostKalmanFilterXOrientation'
   *  Outport: '<Root>/pose_orientation_signals_output'
   */
  Pose_AutoCode_Y.pose_orientation_signals_output[0].value =
    Pose_AutoCode_B.sf_KalmanFilter_f.xhat * 180.0 / 3.1415926535897931;
  Pose_AutoCode_Y.pose_orientation_signals_output[0].status =
    SIGNALSTATE_UPDATED_;
  Pose_AutoCode_Y.pose_orientation_signals_output[0].rms =
    Pose_AutoCode_B.sf_KalmanFilter_f.P;

  /* Memory: '<S3>/Memory5' */
  rtb_Memory2_a_0 = &Pose_AutoCode_DW.Memory5_PreviousInput_p;

  /* MATLAB Function: '<S3>/KalmanFilter1' incorporates:
   *  BusCreator: '<S2>/Bus Creator1'
   *  BusCreator: '<S2>/Bus Creator3'
   *  BusCreator: '<S2>/Bus Creator4'
   *  Constant: '<S1>/enable_poseorientationblock'
   *  Constant: '<S1>/reset_poseorientationblock'
   *  MATLAB Function: '<S3>/PreKalmanFilterYOrientation'
   *  Memory: '<S3>/Memory4'
   *  Memory: '<S3>/Memory6'
   */
  Pose_AutoCode_B.sf_KalmanFilter1_c.P =
    Pose_AutoCode_DW.Memory6_PreviousInput_d;
  Pose_AutoCode_B.sf_KalmanFilter1_c.xhat =
    Pose_AutoCode_DW.Memory4_PreviousInput_h;
  Pose_AutoCode_KalmanFilter(Pose_AutoCode_P.reset_poseorientationblock_Valu,
    Pose_AutoCode_P.enable_poseorientationblock_Val, rt_atan2d_snf
    (-Pose_AutoCode_B.sf_PostKalmanFilter.value, std::sqrt
     (Pose_AutoCode_B.sf_PostKalmanFilter1.value *
      Pose_AutoCode_B.sf_PostKalmanFilter1.value +
      Pose_AutoCode_B.sf_PostKalmanFilter2.value *
      Pose_AutoCode_B.sf_PostKalmanFilter2.value)), 0.64, 0.05, 1.0,
    rtb_Memory2_a_0, &Pose_AutoCode_B.sf_KalmanFilter1_c);

  /* BusCreator: '<S3>/Bus Creator3' incorporates:
   *  MATLAB Function: '<S3>/PostKalmanFilterYOrientation'
   *  Outport: '<Root>/pose_orientation_signals_output'
   */
  Pose_AutoCode_Y.pose_orientation_signals_output[1].value =
    Pose_AutoCode_B.sf_KalmanFilter1_c.xhat * 180.0 / 3.1415926535897931;
  Pose_AutoCode_Y.pose_orientation_signals_output[1].status =
    SIGNALSTATE_UPDATED_;
  Pose_AutoCode_Y.pose_orientation_signals_output[1].rms =
    Pose_AutoCode_B.sf_KalmanFilter1_c.P;

  /* Memory: '<S4>/Memory2' */
  rtb_Memory2_a_0 = &Pose_AutoCode_DW.Memory2_PreviousInput_d;

  /* MATLAB Function: '<S4>/KalmanFilter' incorporates:
   *  Constant: '<S1>/enable_poserotationratefilter'
   *  Constant: '<S1>/reset_poserotationratefilter'
   *  MATLAB Function: '<S32>/TimeCompensator_x6'
   *  MATLAB Function: '<S4>/PreKalmanFilterXRotationRate'
   *  Memory: '<S4>/Memory1'
   *  Memory: '<S4>/Memory3'
   */
  Pose_AutoCode_B.sf_KalmanFilter_i.P = Pose_AutoCode_DW.Memory3_PreviousInput_e;
  Pose_AutoCode_B.sf_KalmanFilter_i.xhat =
    Pose_AutoCode_DW.Memory1_PreviousInput_i;

  /* Outputs for Atomic SubSystem: '<S1>/TimeCompensate' */
  Pose_AutoCode_KalmanFilter(Pose_AutoCode_P.reset_poserotationratefilter_Va,
    Pose_AutoCode_P.enable_poserotationratefilter_V, value4, 0.64, 0.05, 1.0,
    rtb_Memory2_a_0, &Pose_AutoCode_B.sf_KalmanFilter_i);

  /* End of Outputs for SubSystem: '<S1>/TimeCompensate' */

  /* MATLAB Function: '<S4>/PostKalmanFilter' */
  Pose_AutoCode_PostKalmanFilter(Pose_AutoCode_B.sf_KalmanFilter_i.xhat,
    Pose_AutoCode_B.sf_KalmanFilter_i.P, &Pose_AutoCode_B.sf_PostKalmanFilter_c);

  /* BusCreator: '<S4>/Bus Creator1' incorporates:
   *  Outport: '<Root>/pose_rotationrate_signals_output'
   */
  Pose_AutoCode_Y.pose_rotationrate_signals_outpu[0].value =
    Pose_AutoCode_B.sf_PostKalmanFilter_c.value;
  Pose_AutoCode_Y.pose_rotationrate_signals_outpu[0].status =
    Pose_AutoCode_B.sf_PostKalmanFilter_c.status;
  Pose_AutoCode_Y.pose_rotationrate_signals_outpu[0].rms =
    Pose_AutoCode_B.sf_PostKalmanFilter_c.rms;

  /* Memory: '<S4>/Memory5' */
  rtb_Memory2_a_0 = &Pose_AutoCode_DW.Memory5_PreviousInput_f;

  /* MATLAB Function: '<S4>/KalmanFilter1' incorporates:
   *  Constant: '<S1>/enable_poserotationratefilter'
   *  Constant: '<S1>/reset_poserotationratefilter'
   *  MATLAB Function: '<S32>/TimeCompensator_x6'
   *  MATLAB Function: '<S4>/PreKalmanFilterYRotationRate'
   *  Memory: '<S4>/Memory4'
   *  Memory: '<S4>/Memory6'
   */
  Pose_AutoCode_B.sf_KalmanFilter1_f.P =
    Pose_AutoCode_DW.Memory6_PreviousInput_j;
  Pose_AutoCode_B.sf_KalmanFilter1_f.xhat =
    Pose_AutoCode_DW.Memory4_PreviousInput_e;

  /* Outputs for Atomic SubSystem: '<S1>/TimeCompensate' */
  Pose_AutoCode_KalmanFilter(Pose_AutoCode_P.reset_poserotationratefilter_Va,
    Pose_AutoCode_P.enable_poserotationratefilter_V, value5, 0.64, 0.05, 1.0,
    rtb_Memory2_a_0, &Pose_AutoCode_B.sf_KalmanFilter1_f);

  /* End of Outputs for SubSystem: '<S1>/TimeCompensate' */

  /* MATLAB Function: '<S4>/PostKalmanFilter1' */
  Pose_AutoCode_PostKalmanFilter(Pose_AutoCode_B.sf_KalmanFilter1_f.xhat,
    Pose_AutoCode_B.sf_KalmanFilter1_f.P,
    &Pose_AutoCode_B.sf_PostKalmanFilter1_n);

  /* BusCreator: '<S4>/Bus Creator3' incorporates:
   *  Outport: '<Root>/pose_rotationrate_signals_output'
   */
  Pose_AutoCode_Y.pose_rotationrate_signals_outpu[1].value =
    Pose_AutoCode_B.sf_PostKalmanFilter1_n.value;
  Pose_AutoCode_Y.pose_rotationrate_signals_outpu[1].status =
    Pose_AutoCode_B.sf_PostKalmanFilter1_n.status;
  Pose_AutoCode_Y.pose_rotationrate_signals_outpu[1].rms =
    Pose_AutoCode_B.sf_PostKalmanFilter1_n.rms;

  /* Memory: '<S4>/Memory8' */
  rtb_Memory2_a_0 = &Pose_AutoCode_DW.Memory8_PreviousInput_g;

  /* MATLAB Function: '<S4>/KalmanFilter2' incorporates:
   *  Constant: '<S1>/enable_poserotationratefilter'
   *  Constant: '<S1>/reset_poserotationratefilter'
   *  MATLAB Function: '<S32>/TimeCompensator_x6'
   *  MATLAB Function: '<S4>/PreKalmanFilterZRotationRate'
   *  Memory: '<S4>/Memory7'
   *  Memory: '<S4>/Memory9'
   */
  Pose_AutoCode_B.sf_KalmanFilter2_o.P =
    Pose_AutoCode_DW.Memory9_PreviousInput_m;
  Pose_AutoCode_B.sf_KalmanFilter2_o.xhat =
    Pose_AutoCode_DW.Memory7_PreviousInput_m;

  /* Outputs for Atomic SubSystem: '<S1>/TimeCompensate' */
  Pose_AutoCode_KalmanFilter(Pose_AutoCode_P.reset_poserotationratefilter_Va,
    Pose_AutoCode_P.enable_poserotationratefilter_V, value6, 0.64, 0.05, 1.0,
    rtb_Memory2_a_0, &Pose_AutoCode_B.sf_KalmanFilter2_o);

  /* End of Outputs for SubSystem: '<S1>/TimeCompensate' */

  /* MATLAB Function: '<S4>/PostKalmanFilter2' */
  Pose_AutoCode_PostKalmanFilter(Pose_AutoCode_B.sf_KalmanFilter2_o.xhat,
    Pose_AutoCode_B.sf_KalmanFilter2_o.P,
    &Pose_AutoCode_B.sf_PostKalmanFilter2_m);

  /* BusCreator: '<S4>/Bus Creator4' incorporates:
   *  Outport: '<Root>/pose_rotationrate_signals_output'
   */
  Pose_AutoCode_Y.pose_rotationrate_signals_outpu[2].value =
    Pose_AutoCode_B.sf_PostKalmanFilter2_m.value;
  Pose_AutoCode_Y.pose_rotationrate_signals_outpu[2].status =
    Pose_AutoCode_B.sf_PostKalmanFilter2_m.status;
  Pose_AutoCode_Y.pose_rotationrate_signals_outpu[2].rms =
    Pose_AutoCode_B.sf_PostKalmanFilter2_m.rms;

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

  /* Update for Memory: '<S3>/Memory1' */
  Pose_AutoCode_DW.Memory1_PreviousInput_e =
    Pose_AutoCode_B.sf_KalmanFilter_f.xhat;

  /* Update for Memory: '<S3>/Memory3' */
  Pose_AutoCode_DW.Memory3_PreviousInput_o = Pose_AutoCode_B.sf_KalmanFilter_f.P;

  /* Update for Memory: '<S3>/Memory2' */
  Pose_AutoCode_DW.Memory2_PreviousInput_m =
    Pose_AutoCode_B.sf_KalmanFilter_f.state;

  /* Update for Memory: '<S3>/Memory4' */
  Pose_AutoCode_DW.Memory4_PreviousInput_h =
    Pose_AutoCode_B.sf_KalmanFilter1_c.xhat;

  /* Update for Memory: '<S3>/Memory6' */
  Pose_AutoCode_DW.Memory6_PreviousInput_d =
    Pose_AutoCode_B.sf_KalmanFilter1_c.P;

  /* Update for Memory: '<S3>/Memory5' */
  Pose_AutoCode_DW.Memory5_PreviousInput_p =
    Pose_AutoCode_B.sf_KalmanFilter1_c.state;

  /* Update for Memory: '<S4>/Memory1' */
  Pose_AutoCode_DW.Memory1_PreviousInput_i =
    Pose_AutoCode_B.sf_KalmanFilter_i.xhat;

  /* Update for Memory: '<S4>/Memory3' */
  Pose_AutoCode_DW.Memory3_PreviousInput_e = Pose_AutoCode_B.sf_KalmanFilter_i.P;

  /* Update for Memory: '<S4>/Memory2' */
  Pose_AutoCode_DW.Memory2_PreviousInput_d =
    Pose_AutoCode_B.sf_KalmanFilter_i.state;

  /* Update for Memory: '<S4>/Memory4' */
  Pose_AutoCode_DW.Memory4_PreviousInput_e =
    Pose_AutoCode_B.sf_KalmanFilter1_f.xhat;

  /* Update for Memory: '<S4>/Memory6' */
  Pose_AutoCode_DW.Memory6_PreviousInput_j =
    Pose_AutoCode_B.sf_KalmanFilter1_f.P;

  /* Update for Memory: '<S4>/Memory5' */
  Pose_AutoCode_DW.Memory5_PreviousInput_f =
    Pose_AutoCode_B.sf_KalmanFilter1_f.state;

  /* Update for Memory: '<S4>/Memory7' */
  Pose_AutoCode_DW.Memory7_PreviousInput_m =
    Pose_AutoCode_B.sf_KalmanFilter2_o.xhat;

  /* Update for Memory: '<S4>/Memory9' */
  Pose_AutoCode_DW.Memory9_PreviousInput_m =
    Pose_AutoCode_B.sf_KalmanFilter2_o.P;

  /* Update for Memory: '<S4>/Memory8' */
  Pose_AutoCode_DW.Memory8_PreviousInput_g =
    Pose_AutoCode_B.sf_KalmanFilter2_o.state;
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

    /* InitializeConditions for Memory: '<S3>/Memory1' */
    Pose_AutoCode_DW.Memory1_PreviousInput_e =
      Pose_AutoCode_P.Memory1_InitialCondition_b;

    /* InitializeConditions for Memory: '<S3>/Memory3' */
    Pose_AutoCode_DW.Memory3_PreviousInput_o =
      Pose_AutoCode_P.Memory3_InitialCondition_h;

    /* InitializeConditions for Memory: '<S3>/Memory2' */
    Pose_AutoCode_DW.Memory2_PreviousInput_m =
      Pose_AutoCode_P.Memory2_InitialCondition_e;

    /* InitializeConditions for Memory: '<S3>/Memory4' */
    Pose_AutoCode_DW.Memory4_PreviousInput_h =
      Pose_AutoCode_P.Memory4_InitialCondition_p;

    /* InitializeConditions for Memory: '<S3>/Memory6' */
    Pose_AutoCode_DW.Memory6_PreviousInput_d =
      Pose_AutoCode_P.Memory6_InitialCondition_i;

    /* InitializeConditions for Memory: '<S3>/Memory5' */
    Pose_AutoCode_DW.Memory5_PreviousInput_p =
      Pose_AutoCode_P.Memory5_InitialCondition_e;

    /* InitializeConditions for Memory: '<S4>/Memory1' */
    Pose_AutoCode_DW.Memory1_PreviousInput_i =
      Pose_AutoCode_P.Memory1_InitialCondition_c;

    /* InitializeConditions for Memory: '<S4>/Memory3' */
    Pose_AutoCode_DW.Memory3_PreviousInput_e =
      Pose_AutoCode_P.Memory3_InitialCondition_g;

    /* InitializeConditions for Memory: '<S4>/Memory2' */
    Pose_AutoCode_DW.Memory2_PreviousInput_d =
      Pose_AutoCode_P.Memory2_InitialCondition_m;

    /* InitializeConditions for Memory: '<S4>/Memory4' */
    Pose_AutoCode_DW.Memory4_PreviousInput_e =
      Pose_AutoCode_P.Memory4_InitialCondition_c;

    /* InitializeConditions for Memory: '<S4>/Memory6' */
    Pose_AutoCode_DW.Memory6_PreviousInput_j =
      Pose_AutoCode_P.Memory6_InitialCondition_m;

    /* InitializeConditions for Memory: '<S4>/Memory5' */
    Pose_AutoCode_DW.Memory5_PreviousInput_f =
      Pose_AutoCode_P.Memory5_InitialCondition_es;

    /* InitializeConditions for Memory: '<S4>/Memory7' */
    Pose_AutoCode_DW.Memory7_PreviousInput_m =
      Pose_AutoCode_P.Memory7_InitialCondition_e;

    /* InitializeConditions for Memory: '<S4>/Memory9' */
    Pose_AutoCode_DW.Memory9_PreviousInput_m =
      Pose_AutoCode_P.Memory9_InitialCondition_f;

    /* InitializeConditions for Memory: '<S4>/Memory8' */
    Pose_AutoCode_DW.Memory8_PreviousInput_g =
      Pose_AutoCode_P.Memory8_InitialCondition_d;

    /* SystemInitialize for Atomic SubSystem: '<S1>/TimeCompensate' */
    /* InitializeConditions for Memory: '<S32>/Memory' */
    Pose_AutoCode_DW.Memory_PreviousInput =
      Pose_AutoCode_P.Memory_InitialCondition;
    for (i = 0; i < 48; i++) {
      /* InitializeConditions for Memory: '<S32>/Memory1' */
      Pose_AutoCode_DW.Memory1_PreviousInput_m[i] =
        Pose_AutoCode_P.Memory1_InitialCondition;

      /* InitializeConditions for Memory: '<S32>/Memory2' */
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
