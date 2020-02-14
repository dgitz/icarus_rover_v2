/*
 * Pose_AutoCode.h
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

/* External inputs (root inport signals with default storage) */
typedef struct {
  uint8_T type;                        /* '<Root>/accel1x_in_type' */
  real_T value;                        /* '<Root>/accel1x_in_value' */
  uint8_T status;                      /* '<Root>/accel1x_in_status' */
  real_T rms;                          /* '<Root>/accel1x_in_rms' */
  uint8_T sequence_number;             /* '<Root>/accel1x_in_sequence_number' */
  uint8_T type_a;                      /* '<Root>/accel1y_in_type' */
  real_T value_j;                      /* '<Root>/accel1y_in_value' */
  uint8_T status_d;                    /* '<Root>/accel1y_in_status' */
  real_T rms_h;                        /* '<Root>/accel1y_in_rms' */
  uint8_T sequence_number_p;           /* '<Root>/accel1y_in_sequence_number' */
  uint8_T type_p;                      /* '<Root>/accel1z_in_type' */
  real_T value_d;                      /* '<Root>/accel1z_in_value' */
  uint8_T status_p;                    /* '<Root>/accel1z_in_status' */
  real_T rms_l;                        /* '<Root>/accel1z_in_rms' */
  uint8_T sequence_number_e;           /* '<Root>/accel1z_in_sequence_number' */
  uint8_T type_o;                      /* '<Root>/accel2x_in_type' */
  real_T value_h;                      /* '<Root>/accel2x_in_value' */
  uint8_T status_k;                    /* '<Root>/accel2x_in_status' */
  real_T rms_m;                        /* '<Root>/accel2x_in_rms' */
  uint8_T sequence_number_a;           /* '<Root>/accel2x_in_sequence_number' */
  uint8_T type_am;                     /* '<Root>/accel2y_in_type' */
  real_T value_i;                      /* '<Root>/accel2y_in_value' */
  uint8_T status_n;                    /* '<Root>/accel2y_in_status' */
  real_T rms_g;                        /* '<Root>/accel2y_in_rms' */
  uint8_T sequence_number_m;           /* '<Root>/accel2y_in_sequence_number' */
  uint8_T type_g;                      /* '<Root>/accel2z_in_type' */
  real_T value_b;                      /* '<Root>/accel2z_in_value' */
  uint8_T status_l;                    /* '<Root>/accel2z_in_status' */
  real_T rms_k;                        /* '<Root>/accel2z_in_rms' */
  uint8_T sequence_number_f;           /* '<Root>/accel2z_in_sequence_number' */
  uint8_T type_c;                      /* '<Root>/accel3x_in_type' */
  real_T value_ji;                     /* '<Root>/accel3x_in_value' */
  uint8_T status_b;                    /* '<Root>/accel3x_in_status' */
  real_T rms_a;                        /* '<Root>/accel3x_in_rms' */
  uint8_T sequence_number_d;           /* '<Root>/accel3x_in_sequence_number' */
  uint8_T type_pi;                     /* '<Root>/accel3y_in_type' */
  real_T value_i0;                     /* '<Root>/accel3y_in_value' */
  uint8_T status_a;                    /* '<Root>/accel3y_in_status' */
  real_T rms_kj;                       /* '<Root>/accel3y_in_rms' */
  uint8_T sequence_number_l;           /* '<Root>/accel3y_in_sequence_number' */
  uint8_T type_f;                      /* '<Root>/accel3z_in_type' */
  real_T value_n;                      /* '<Root>/accel3z_in_value' */
  uint8_T status_h;                    /* '<Root>/accel3z_in_status' */
  real_T rms_c;                        /* '<Root>/accel3z_in_rms' */
  uint8_T sequence_number_o;           /* '<Root>/accel3z_in_sequence_number' */
  uint8_T type_n;                      /* '<Root>/accel4x_in_type' */
  real_T value_k;                      /* '<Root>/accel4x_in_value' */
  uint8_T status_m;                    /* '<Root>/accel4x_in_status' */
  real_T rms_ml;                       /* '<Root>/accel4x_in_rms' */
  uint8_T sequence_number_g;           /* '<Root>/accel4x_in_sequence_number' */
  uint8_T type_e;                      /* '<Root>/accel4y_in_type' */
  real_T value_h2;                     /* '<Root>/accel4y_in_value' */
  uint8_T status_nm;                   /* '<Root>/accel4y_in_status' */
  real_T rms_ck;                       /* '<Root>/accel4y_in_rms' */
  uint8_T sequence_number_h;           /* '<Root>/accel4y_in_sequence_number' */
  uint8_T type_k;                      /* '<Root>/accel4z_in_type' */
  real_T value_hg;                     /* '<Root>/accel4z_in_value' */
  uint8_T status_o;                    /* '<Root>/accel4z_in_status' */
  real_T rms_i;                        /* '<Root>/accel4z_in_rms' */
  uint8_T sequence_number_ap;          /* '<Root>/accel4z_in_sequence_number' */
  uint8_T type_py;                     /* '<Root>/rotationrate1x_in_type' */
  real_T value_hgi;                    /* '<Root>/rotationrate1x_in_value' */
  uint8_T status_pc;                   /* '<Root>/rotationrate1x_in_status' */
  real_T rms_cw;                       /* '<Root>/rotationrate1x_in_rms' */
  uint8_T sequence_number_i;    /* '<Root>/rotationrate1x_in_sequence_number' */
  uint8_T type_pf;                     /* '<Root>/rotationrate1y_in_type' */
  real_T value_p;                      /* '<Root>/rotationrate1y_in_value' */
  uint8_T status_e;                    /* '<Root>/rotationrate1y_in_status' */
  real_T rms_mc;                       /* '<Root>/rotationrate1y_in_rms' */
  uint8_T sequence_number_m1;   /* '<Root>/rotationrate1y_in_sequence_number' */
  uint8_T type_m;                      /* '<Root>/rotationrate1z_in_type' */
  real_T value_o;                      /* '<Root>/rotationrate1z_in_value' */
  uint8_T status_i;                    /* '<Root>/rotationrate1z_in_status' */
  real_T rms_b;                        /* '<Root>/rotationrate1z_in_rms' */
  uint8_T sequence_number_n;    /* '<Root>/rotationrate1z_in_sequence_number' */
  uint8_T type_pm;                     /* '<Root>/rotationrate2x_in_type1' */
  real_T value_kb;                     /* '<Root>/rotationrate2x_in_value' */
  uint8_T status_ic;                   /* '<Root>/rotationrate2x_in_status' */
  real_T rms_n;                        /* '<Root>/rotationrate2x_in_rms' */
  uint8_T sequence_number_fx;   /* '<Root>/rotationrate2x_in_sequence_number' */
  uint8_T type_d;                      /* '<Root>/rotationrate2y_in_type' */
  real_T value_il;                     /* '<Root>/rotationrate2y_in_value' */
  uint8_T status_f;                    /* '<Root>/rotationrate2y_in_status' */
  real_T rms_o;                        /* '<Root>/rotationrate2y_in_rms' */
  uint8_T sequence_number_ez;   /* '<Root>/rotationrate2y_in_sequence_number' */
  uint8_T type_ge;                     /* '<Root>/rotationrate2z_in_type' */
  real_T value_f;                      /* '<Root>/rotationrate2z_in_value' */
  uint8_T status_fn;                   /* '<Root>/rotationrate2z_in_status1' */
  real_T rms_ca;                       /* '<Root>/rotationrate2z_in_rms' */
  uint8_T sequence_number_hk;   /* '<Root>/rotationrate2z_in_sequence_number' */
  uint8_T type_j;                      /* '<Root>/rotationrate3x_in_type' */
  real_T value_br;                     /* '<Root>/rotationrate3x_in_value' */
  uint8_T status_p4;                   /* '<Root>/rotationrate3x_in_status' */
  real_T rms_lw;                       /* '<Root>/rotationrate3x_in_rms' */
  uint8_T sequence_number_fu;   /* '<Root>/rotationrate3x_in_sequence_number' */
  uint8_T type_h;                      /* '<Root>/rotationrate3y_in_type' */
  real_T value_kg;                     /* '<Root>/rotationrate3y_in_value' */
  uint8_T status_lk;                   /* '<Root>/rotationrate3y_in_status' */
  real_T rms_f;                        /* '<Root>/rotationrate3y_in_rms' */
  uint8_T sequence_number_d2;   /* '<Root>/rotationrate3y_in_sequence_number' */
  uint8_T type_jo;                     /* '<Root>/rotationrate3z_in_type' */
  real_T value_a;                      /* '<Root>/rotationrate3z_in_value' */
  uint8_T status_mm;                   /* '<Root>/rotationrate3z_in_status' */
  real_T rms_gk;                       /* '<Root>/rotationrate3z_in_rms' */
  uint8_T sequence_number_m4;   /* '<Root>/rotationrate3z_in_sequence_number' */
  uint8_T type_m3;                     /* '<Root>/rotationrate4x_in_type' */
  real_T value_a5;                     /* '<Root>/rotationrate4x_in_value' */
  uint8_T status_e1;                   /* '<Root>/rotationrate4x_in_status' */
  real_T rms_ho;                       /* '<Root>/rotationrate4x_in_rms' */
  uint8_T sequence_number_nj;   /* '<Root>/rotationrate4x_in_sequence_number' */
  uint8_T type_l;                      /* '<Root>/rotationrate4y_in_type' */
  real_T value_od;                     /* '<Root>/rotationrate4y_in_value' */
  uint8_T status_bf;                   /* '<Root>/rotationrate4y_in_status' */
  real_T rms_i3;                       /* '<Root>/rotationrate4y_in_rms' */
  uint8_T sequence_number_h4;   /* '<Root>/rotationrate4y_in_sequence_number' */
  uint8_T type_b;                      /* '<Root>/rotationrate4z_in_type' */
  real_T value_d5;                     /* '<Root>/rotationrate4z_in_value' */
  uint8_T status_m3;                   /* '<Root>/rotationrate4z_in_status' */
  real_T rms_f3;                       /* '<Root>/rotationrate4z_in_rms' */
  uint8_T sequence_number_d5;   /* '<Root>/rotationrate4z_in_sequence_number' */
  uint8_T type_bi;                     /* '<Root>/mag1x_in_type' */
  real_T value_c;                      /* '<Root>/mag1x_in_value' */
  uint8_T status_i1;                   /* '<Root>/mag1x_in_status' */
  real_T rms_mm;                       /* '<Root>/mag1x_in_rms' */
  uint8_T sequence_number_c;           /* '<Root>/mag1x_in_sequence_number' */
  uint8_T type_my;                     /* '<Root>/mag1y_in_type' */
  real_T value_jt;                     /* '<Root>/mag1y_in_value' */
  uint8_T status_dg;                   /* '<Root>/mag1y_in_status' */
  real_T rms_cm;                       /* '<Root>/mag1y_in_rms' */
  uint8_T sequence_number_fp;          /* '<Root>/mag1y_in_sequence_number' */
  uint8_T type_hy;                     /* '<Root>/mag1z_in_type' */
  real_T value_je;                     /* '<Root>/mag1z_in_value' */
  uint8_T status_ao;                   /* '<Root>/mag1z_in_status' */
  real_T rms_p;                        /* '<Root>/mag1z_in_rms' */
  uint8_T sequence_number_es;          /* '<Root>/mag1z_in_sequence_number' */
  uint8_T type_gg;                     /* '<Root>/mag2x_in_type' */
  real_T value_m;                      /* '<Root>/mag2x_in_value' */
  uint8_T status_g;                    /* '<Root>/mag2x_in_status' */
  real_T rms_fj;                       /* '<Root>/mag2x_in_rms' */
  uint8_T sequence_number_hg;          /* '<Root>/mag2x_in_sequence_number' */
  uint8_T type_ni;                     /* '<Root>/mag2y_in_type' */
  real_T value_ps;                     /* '<Root>/mag2y_in_value' */
  uint8_T status_gt;                   /* '<Root>/mag2y_in_status' */
  real_T rms_j;                        /* '<Root>/mag2y_in_rms' */
  uint8_T sequence_number_pq;          /* '<Root>/mag2y_in_sequence_number' */
  uint8_T type_kb;                     /* '<Root>/mag2z_in_type' */
  real_T value_hu;                     /* '<Root>/mag2z_in_value' */
  uint8_T status_j;                    /* '<Root>/mag2z_in_status' */
  real_T rms_d;                        /* '<Root>/mag2z_in_rms' */
  uint8_T sequence_number_lo;          /* '<Root>/mag2z_in_sequence_number' */
  uint8_T type_ci;                     /* '<Root>/mag3x_in_type' */
  real_T value_dz;                     /* '<Root>/mag3x_in_value' */
  uint8_T status_k0;                   /* '<Root>/mag3x_in_status' */
  real_T rms_pj;                       /* '<Root>/mag3x_in_rms' */
  uint8_T sequence_number_g0;          /* '<Root>/mag3x_in_sequence_number' */
  uint8_T type_ob;                     /* '<Root>/mag3y_in_type' */
  real_T value_g;                      /* '<Root>/mag3y_in_value' */
  uint8_T status_bd;                   /* '<Root>/mag3y_in_status' */
  real_T rms_io;                       /* '<Root>/mag3y_in_rms' */
  uint8_T sequence_number_d0;          /* '<Root>/mag3y_in_sequence_number' */
  uint8_T type_jp;                     /* '<Root>/mag3z_in_type' */
  real_T value_md;                     /* '<Root>/mag3z_in_value' */
  uint8_T status_ha;                   /* '<Root>/mag3z_in_status' */
  real_T rms_dd;                       /* '<Root>/mag3z_in_rms' */
  uint8_T sequence_number_id;          /* '<Root>/mag3z_in_sequence_number' */
  uint8_T type_gd;                     /* '<Root>/mag4x_in_type' */
  real_T value_pn;                     /* '<Root>/mag4x_in_value' */
  uint8_T status_ew;                   /* '<Root>/mag4x_in_status' */
  real_T rms_e;                        /* '<Root>/mag4x_in_rms' */
  uint8_T sequence_number_h1;          /* '<Root>/mag4x_in_sequence_number' */
  uint8_T type_bz;                     /* '<Root>/mag4y_in_type' */
  real_T value_h2p;                    /* '<Root>/mag4y_in_value' */
  uint8_T status_jo;                   /* '<Root>/mag4y_in_status' */
  real_T rms_cl;                       /* '<Root>/mag4y_in_rms' */
  uint8_T sequence_number_oy;          /* '<Root>/mag4y_in_sequence_number' */
  uint8_T type_i;                      /* '<Root>/mag4z_in_type' */
  real_T value_n4;                     /* '<Root>/mag4z_in_value' */
  uint8_T status_gl;                   /* '<Root>/mag4z_in_status' */
  real_T rms_n2;                       /* '<Root>/mag4z_in_rms' */
  uint8_T sequence_number_fk;          /* '<Root>/mag4z_in_sequence_number' */
} ExtU_Pose_AutoCode_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T pose_linearacceleration_signals;
                                  /* '<Root>/pose_linearacceleration_signals' */
  real_T Out2;                         /* '<Root>/Out2' */
  real_T Out3;                         /* '<Root>/Out3' */
  real_T Out4;                         /* '<Root>/Out4' */
  real_T Out5;                         /* '<Root>/Out5' */
  real_T Out6;                         /* '<Root>/Out6' */
} ExtY_Pose_AutoCode_T;

/* Real-time Model Data Structure */
struct tag_RTM_Pose_AutoCode_T {
  const char_T *errorStatus;
};

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
  /* Real-Time Model */
  RT_MODEL_Pose_AutoCode_T Pose_AutoCode_M;
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
 * '<S1>'   : 'Pose_AutoCode/PoseSystem'
 * '<S2>'   : 'Pose_AutoCode/PoseSystem/PoseAccelerationBlock'
 * '<S3>'   : 'Pose_AutoCode/PoseSystem/SensorPostProcessing'
 * '<S4>'   : 'Pose_AutoCode/PoseSystem/SignalLinker'
 * '<S5>'   : 'Pose_AutoCode/PoseSystem/TimeCompensate'
 * '<S6>'   : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x1'
 * '<S7>'   : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x12'
 * '<S8>'   : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x2'
 * '<S9>'   : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x1/TimeCompensator1'
 * '<S10>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x1/TimeCompensator10'
 * '<S11>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x1/TimeCompensator11'
 * '<S12>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x1/TimeCompensator12'
 * '<S13>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x1/TimeCompensator2'
 * '<S14>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x1/TimeCompensator3'
 * '<S15>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x1/TimeCompensator4'
 * '<S16>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x1/TimeCompensator5'
 * '<S17>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x1/TimeCompensator6'
 * '<S18>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x1/TimeCompensator7'
 * '<S19>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x1/TimeCompensator8'
 * '<S20>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x1/TimeCompensator9'
 * '<S21>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x12/TimeCompensator1'
 * '<S22>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x12/TimeCompensator10'
 * '<S23>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x12/TimeCompensator11'
 * '<S24>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x12/TimeCompensator12'
 * '<S25>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x12/TimeCompensator2'
 * '<S26>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x12/TimeCompensator3'
 * '<S27>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x12/TimeCompensator4'
 * '<S28>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x12/TimeCompensator5'
 * '<S29>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x12/TimeCompensator6'
 * '<S30>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x12/TimeCompensator7'
 * '<S31>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x12/TimeCompensator8'
 * '<S32>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x12/TimeCompensator9'
 * '<S33>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x2/TimeCompensator1'
 * '<S34>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x2/TimeCompensator10'
 * '<S35>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x2/TimeCompensator11'
 * '<S36>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x2/TimeCompensator12'
 * '<S37>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x2/TimeCompensator2'
 * '<S38>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x2/TimeCompensator3'
 * '<S39>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x2/TimeCompensator4'
 * '<S40>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x2/TimeCompensator5'
 * '<S41>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x2/TimeCompensator6'
 * '<S42>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x2/TimeCompensator7'
 * '<S43>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x2/TimeCompensator8'
 * '<S44>'  : 'Pose_AutoCode/PoseSystem/TimeCompensate/Time Compensate Signal x2/TimeCompensator9'
 */
#endif                                 /* RTW_HEADER_Pose_AutoCode_h_ */
