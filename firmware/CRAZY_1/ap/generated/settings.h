/* This file has been generated from /home/gautier/dev/paparazzi/conf/modules/stabilization_int_quat.xml /home/gautier/dev/paparazzi/conf/modules/ins_extended.xml /home/gautier/dev/paparazzi/conf/modules/ahrs_madgwick.xml /home/gautier/dev/paparazzi/conf/modules/guidance_pid_rotorcraft.xml /home/gautier/dev/paparazzi/conf/modules/guidance_rotorcraft.xml /home/gautier/dev/paparazzi/conf/modules/imu_common.xml /home/gautier/dev/paparazzi/conf/modules/electrical.xml /home/gautier/dev/paparazzi/conf/modules/air_data.xml /home/gautier/dev/paparazzi/conf/modules/gps.xml /home/gautier/dev/paparazzi/conf/modules/nav_rotorcraft.xml /home/gautier/dev/paparazzi/conf/settings/rotorcraft_basic.xml */
/* Version v7.0_unstable-15-g5c1c35e859-dirty */
/* Please DO NOT EDIT */

#ifndef SETTINGS_H
#define SETTINGS_H


#include "generated/flight_plan.h"
#include "generated/periodic_telemetry.h"
#include "mcu.h"
#include "autopilot.h"
#include "navigation.h"
#include "modules/gps/gps.h"
#include "air_data/air_data.h"
#include "modules/energy/electrical.h"
#include "modules/imu/imu.h"
#include "guidance/guidance_h.h"
#include "guidance/guidance_pid.h"
#include "modules/ins/vf_extended_float.h"
#include "stabilization/stabilization_attitude_quat_int.h"
#include "stabilization/stabilization_attitude_common_int.h"
#include "generated/modules.h"

#define SETTINGS_NAMES { \
 { "telemetry_mode_Main" }, \
 { "nominal_alt" }, \
 { "desired_heading" }, \
 { "gps_datalink_gps_datalink_periodic_check_status" }, \
 { "autopilot_mode_auto2" }, \
 { "autopilot.kill_throttle" }, \
 { "autopilot.mode" }, \
 { "mcu_reboot" }, \
 { "flight_altitude" }, \
 { "nav.heading" }, \
 { "nav.radius" }, \
 { "nav.climb_vspeed" }, \
 { "nav.descend_vspeed" }, \
 { "multi_gps_mode" }, \
 { "air_data.qnh" }, \
 { "air_data.tas_factor" }, \
 { "air_data.calc_qnh_once" }, \
 { "air_data.calc_airspeed" }, \
 { "air_data.calc_tas_factor" }, \
 { "air_data.calc_amsl_baro" }, \
 { "avg_reset" }, \
 { "imu.body_to_imu.eulers_f.phi" }, \
 { "imu.body_to_imu.eulers_f.theta" }, \
 { "imu.body_to_imu.eulers_f.psi" }, \
 { "imu.b2i_set_current" }, \
 { "imu.gyro_abi_send_id" }, \
 { "imu.accel_abi_send_id" }, \
 { "imu.mag_abi_send_id" }, \
 { "guidance_h.use_ref" }, \
 { "gh_ref.max_speed" }, \
 { "gh_ref.tau" }, \
 { "gh_ref.omega" }, \
 { "gh_ref.zeta" }, \
 { "guidance_h.sp.pos.x" }, \
 { "guidance_h.sp.pos.y" }, \
 { "guidance_v.nominal_throttle" }, \
 { "guidance_v.z_sp" }, \
 { "guidance_pid.approx_force_by_thrust" }, \
 { "guidance_pid.kp" }, \
 { "guidance_pid.kd" }, \
 { "guidance_pid.ki" }, \
 { "guidance_pid.kv" }, \
 { "guidance_pid.ka" }, \
 { "guidance_pid.v_kp" }, \
 { "guidance_pid.v_kd" }, \
 { "guidance_pid.v_ki" }, \
 { "guidance_pid.adapt_throttle_enabled" }, \
 { "ahrs_madgwick.reset" }, \
 { "vff.accel_noise" }, \
 { "vff.r_baro" }, \
 { "vff.r_alt" }, \
 { "vff.r_obs_height" }, \
 { "stabilization_gains.p.x" }, \
 { "stabilization_gains.d.x" }, \
 { "stabilization_gains.i.x" }, \
 { "stabilization_gains.dd.x" }, \
 { "stabilization_gains.p.y" }, \
 { "stabilization_gains.d.y" }, \
 { "stabilization_gains.i.y" }, \
 { "stabilization_gains.dd.y" }, \
 { "stabilization_gains.p.z" }, \
 { "stabilization_gains.d.z" }, \
 { "stabilization_gains.i.z" }, \
 { "stabilization_gains.dd.z" }, \
 { "att_ref_quat_i.model.omega.p" }, \
 { "att_ref_quat_i.model.zeta.p" }, \
 { "att_ref_quat_i.model.omega.q" }, \
 { "att_ref_quat_i.model.zeta.q" }, \
 { "att_ref_quat_i.model.omega.r" }, \
 { "att_ref_quat_i.model.zeta.r" }, \
};
#define SETTINGS_NAMES_SHORT { \
 "tel_mod_Mai" , \
 "nom_alt" , \
 "des_hea" , \
 "gps_dat_gps_dat_" , \
 "aut_mod_aut" , \
 "aut_kil_thr" , \
 "aut_mod" , \
 "mcu_reb" , \
 "fli_alt" , \
 "nav_hea" , \
 "nav_rad" , \
 "nav_cli_vsp" , \
 "nav_des_vsp" , \
 "mul_gps_mod" , \
 "air_dat_qnh" , \
 "air_dat_tas_fac" , \
 "air_dat_cal_qnh_" , \
 "air_dat_cal_air" , \
 "air_dat_cal_tas_" , \
 "air_dat_cal_ams_" , \
 "avg_res" , \
 "imu_bod_to_imu_e" , \
 "imu_bod_to_imu_e" , \
 "imu_bod_to_imu_e" , \
 "imu_b2i_set_cur" , \
 "imu_gyr_abi_sen_" , \
 "imu_acc_abi_sen_" , \
 "imu_mag_abi_sen_" , \
 "gui_h_use_ref" , \
 "gh_ref_max_spe" , \
 "gh_ref_tau" , \
 "gh_ref_ome" , \
 "gh_ref_zet" , \
 "gui_h_sp_pos_x" , \
 "gui_h_sp_pos_y" , \
 "gui_v_nom_thr" , \
 "gui_v_z_sp" , \
 "gui_pid_app_for_" , \
 "gui_pid_kp" , \
 "gui_pid_kd" , \
 "gui_pid_ki" , \
 "gui_pid_kv" , \
 "gui_pid_ka" , \
 "gui_pid_v_kp" , \
 "gui_pid_v_kd" , \
 "gui_pid_v_ki" , \
 "gui_pid_ada_thr_" , \
 "ahr_mad_res" , \
 "vff_acc_noi" , \
 "vff_r_bar" , \
 "vff_r_alt" , \
 "vff_r_obs_hei" , \
 "sta_gai_p_x" , \
 "sta_gai_d_x" , \
 "sta_gai_i_x" , \
 "sta_gai_dd_x" , \
 "sta_gai_p_y" , \
 "sta_gai_d_y" , \
 "sta_gai_i_y" , \
 "sta_gai_dd_y" , \
 "sta_gai_p_z" , \
 "sta_gai_d_z" , \
 "sta_gai_i_z" , \
 "sta_gai_dd_z" , \
 "att_ref_qua_i_mo" , \
 "att_ref_qua_i_mo" , \
 "att_ref_qua_i_mo" , \
 "att_ref_qua_i_mo" , \
 "att_ref_qua_i_mo" , \
 "att_ref_qua_i_mo" , \
};
#define NB_SETTING 70
#define DlSetting(_idx, _value) { \
  switch (_idx) { \
    case 0: telemetry_mode_Main = _value; break;\
    case 1: nominal_alt = _value; break;\
    case 2: desired_heading = _value; break;\
    case 3: gps_datalink_gps_datalink_periodic_check_status = _value; break;\
    case 4: autopilot_mode_auto2 = _value; break;\
    case 5: autopilot_KillThrottle( _value ); _value = autopilot.kill_throttle; break;\
    case 6: autopilot_SetModeHandler( _value ); _value = autopilot.mode; break;\
    case 7: mcu_reboot( _value ); break;\
    case 8: navigation_SetFlightAltitude( _value ); _value = flight_altitude; break;\
    case 9: nav.heading = _value; break;\
    case 10: nav.radius = _value; break;\
    case 11: nav.climb_vspeed = _value; break;\
    case 12: nav.descend_vspeed = _value; break;\
    case 13: multi_gps_mode = _value; break;\
    case 14: air_data.qnh = _value; break;\
    case 15: air_data.tas_factor = _value; break;\
    case 16: air_data.calc_qnh_once = _value; break;\
    case 17: air_data.calc_airspeed = _value; break;\
    case 18: air_data.calc_tas_factor = _value; break;\
    case 19: air_data.calc_amsl_baro = _value; break;\
    case 20: electrical_avg_reset( _value ); break;\
    case 21: imu_SetBodyToImuPhi( _value ); _value = imu.body_to_imu.eulers_f.phi; break;\
    case 22: imu_SetBodyToImuTheta( _value ); _value = imu.body_to_imu.eulers_f.theta; break;\
    case 23: imu_SetBodyToImuPsi( _value ); _value = imu.body_to_imu.eulers_f.psi; break;\
    case 24: imu_SetBodyToImuCurrent( _value ); _value = imu.b2i_set_current; break;\
    case 25: imu.gyro_abi_send_id = _value; break;\
    case 26: imu.accel_abi_send_id = _value; break;\
    case 27: imu.mag_abi_send_id = _value; break;\
    case 28: guidance_h_SetUseRef( _value ); _value = guidance_h.use_ref; break;\
    case 29: guidance_h_SetMaxSpeed( _value ); _value = gh_ref.max_speed; break;\
    case 30: guidance_h_SetTau( _value ); _value = gh_ref.tau; break;\
    case 31: guidance_h_SetOmega( _value ); _value = gh_ref.omega; break;\
    case 32: guidance_h_SetZeta( _value ); _value = gh_ref.zeta; break;\
    case 33: guidance_h.sp.pos.x = _value; break;\
    case 34: guidance_h.sp.pos.y = _value; break;\
    case 35: guidance_v.nominal_throttle = _value; break;\
    case 36: guidance_v.z_sp = _value; break;\
    case 37: guidance_pid.approx_force_by_thrust = _value; break;\
    case 38: guidance_pid.kp = _value; break;\
    case 39: guidance_pid.kd = _value; break;\
    case 40: guidance_pid_set_h_igain( _value ); _value = guidance_pid.ki; break;\
    case 41: guidance_pid.kv = _value; break;\
    case 42: guidance_pid.ka = _value; break;\
    case 43: guidance_pid.v_kp = _value; break;\
    case 44: guidance_pid.v_kd = _value; break;\
    case 45: guidance_pid_set_v_igain( _value ); _value = guidance_pid.v_ki; break;\
    case 46: guidance_pid.adapt_throttle_enabled = _value; break;\
    case 47: ahrs_madgwick.reset = _value; break;\
    case 48: vff.accel_noise = _value; break;\
    case 49: vff.r_baro = _value; break;\
    case 50: vff.r_alt = _value; break;\
    case 51: vff.r_obs_height = _value; break;\
    case 52: stabilization_gains.p.x = _value; break;\
    case 53: stabilization_gains.d.x = _value; break;\
    case 54: stabilization_gains.i.x = _value; break;\
    case 55: stabilization_gains.dd.x = _value; break;\
    case 56: stabilization_gains.p.y = _value; break;\
    case 57: stabilization_gains.d.y = _value; break;\
    case 58: stabilization_gains.i.y = _value; break;\
    case 59: stabilization_gains.dd.y = _value; break;\
    case 60: stabilization_gains.p.z = _value; break;\
    case 61: stabilization_gains.d.z = _value; break;\
    case 62: stabilization_gains.i.z = _value; break;\
    case 63: stabilization_gains.dd.z = _value; break;\
    case 64: stabilization_attitude_quat_int_SetOmegaP( _value ); _value = att_ref_quat_i.model.omega.p; break;\
    case 65: stabilization_attitude_quat_int_SetZetaP( _value ); _value = att_ref_quat_i.model.zeta.p; break;\
    case 66: stabilization_attitude_quat_int_SetOmegaQ( _value ); _value = att_ref_quat_i.model.omega.q; break;\
    case 67: stabilization_attitude_quat_int_SetZetaQ( _value ); _value = att_ref_quat_i.model.zeta.q; break;\
    case 68: stabilization_attitude_quat_int_SetOmegaR( _value ); _value = att_ref_quat_i.model.omega.r; break;\
    case 69: stabilization_attitude_quat_int_SetZetaR( _value ); _value = att_ref_quat_i.model.zeta.r; break;\
    default: break;\
  }\
}
#define PeriodicSendDlValue(_trans, _dev) { \
  static uint8_t i;\
  float var;\
  if (i >= 70) i = 0;\
  switch (i) { \
    case 0: var = telemetry_mode_Main; break;\
    case 1: var = nominal_alt; break;\
    case 2: var = desired_heading; break;\
    case 3: var = gps_datalink_gps_datalink_periodic_check_status; break;\
    case 4: var = autopilot_mode_auto2; break;\
    case 5: var = autopilot.kill_throttle; break;\
    case 6: var = autopilot.mode; break;\
    case 7: var = 0; break;\
    case 8: var = flight_altitude; break;\
    case 9: var = nav.heading; break;\
    case 10: var = nav.radius; break;\
    case 11: var = nav.climb_vspeed; break;\
    case 12: var = nav.descend_vspeed; break;\
    case 13: var = multi_gps_mode; break;\
    case 14: var = air_data.qnh; break;\
    case 15: var = air_data.tas_factor; break;\
    case 16: var = air_data.calc_qnh_once; break;\
    case 17: var = air_data.calc_airspeed; break;\
    case 18: var = air_data.calc_tas_factor; break;\
    case 19: var = air_data.calc_amsl_baro; break;\
    case 20: var = 0; break;\
    case 21: var = imu.body_to_imu.eulers_f.phi; break;\
    case 22: var = imu.body_to_imu.eulers_f.theta; break;\
    case 23: var = imu.body_to_imu.eulers_f.psi; break;\
    case 24: var = imu.b2i_set_current; break;\
    case 25: var = imu.gyro_abi_send_id; break;\
    case 26: var = imu.accel_abi_send_id; break;\
    case 27: var = imu.mag_abi_send_id; break;\
    case 28: var = guidance_h.use_ref; break;\
    case 29: var = gh_ref.max_speed; break;\
    case 30: var = gh_ref.tau; break;\
    case 31: var = gh_ref.omega; break;\
    case 32: var = gh_ref.zeta; break;\
    case 33: var = guidance_h.sp.pos.x; break;\
    case 34: var = guidance_h.sp.pos.y; break;\
    case 35: var = guidance_v.nominal_throttle; break;\
    case 36: var = guidance_v.z_sp; break;\
    case 37: var = guidance_pid.approx_force_by_thrust; break;\
    case 38: var = guidance_pid.kp; break;\
    case 39: var = guidance_pid.kd; break;\
    case 40: var = guidance_pid.ki; break;\
    case 41: var = guidance_pid.kv; break;\
    case 42: var = guidance_pid.ka; break;\
    case 43: var = guidance_pid.v_kp; break;\
    case 44: var = guidance_pid.v_kd; break;\
    case 45: var = guidance_pid.v_ki; break;\
    case 46: var = guidance_pid.adapt_throttle_enabled; break;\
    case 47: var = ahrs_madgwick.reset; break;\
    case 48: var = vff.accel_noise; break;\
    case 49: var = vff.r_baro; break;\
    case 50: var = vff.r_alt; break;\
    case 51: var = vff.r_obs_height; break;\
    case 52: var = stabilization_gains.p.x; break;\
    case 53: var = stabilization_gains.d.x; break;\
    case 54: var = stabilization_gains.i.x; break;\
    case 55: var = stabilization_gains.dd.x; break;\
    case 56: var = stabilization_gains.p.y; break;\
    case 57: var = stabilization_gains.d.y; break;\
    case 58: var = stabilization_gains.i.y; break;\
    case 59: var = stabilization_gains.dd.y; break;\
    case 60: var = stabilization_gains.p.z; break;\
    case 61: var = stabilization_gains.d.z; break;\
    case 62: var = stabilization_gains.i.z; break;\
    case 63: var = stabilization_gains.dd.z; break;\
    case 64: var = att_ref_quat_i.model.omega.p; break;\
    case 65: var = att_ref_quat_i.model.zeta.p; break;\
    case 66: var = att_ref_quat_i.model.omega.q; break;\
    case 67: var = att_ref_quat_i.model.zeta.q; break;\
    case 68: var = att_ref_quat_i.model.omega.r; break;\
    case 69: var = att_ref_quat_i.model.zeta.r; break;\
    default: var = 0.; break;\
  }\
  pprz_msg_send_DL_VALUE(_trans, _dev, AC_ID, &i, &var);\
  i++;\
}
static inline float settings_get_value(uint8_t i) {
  switch (i) {
    case 0: return telemetry_mode_Main;
    case 1: return nominal_alt;
    case 2: return desired_heading;
    case 3: return gps_datalink_gps_datalink_periodic_check_status;
    case 4: return autopilot_mode_auto2;
    case 5: return autopilot.kill_throttle;
    case 6: return autopilot.mode;
    case 7: return 0;
    case 8: return flight_altitude;
    case 9: return nav.heading;
    case 10: return nav.radius;
    case 11: return nav.climb_vspeed;
    case 12: return nav.descend_vspeed;
    case 13: return multi_gps_mode;
    case 14: return air_data.qnh;
    case 15: return air_data.tas_factor;
    case 16: return air_data.calc_qnh_once;
    case 17: return air_data.calc_airspeed;
    case 18: return air_data.calc_tas_factor;
    case 19: return air_data.calc_amsl_baro;
    case 20: return 0;
    case 21: return imu.body_to_imu.eulers_f.phi;
    case 22: return imu.body_to_imu.eulers_f.theta;
    case 23: return imu.body_to_imu.eulers_f.psi;
    case 24: return imu.b2i_set_current;
    case 25: return imu.gyro_abi_send_id;
    case 26: return imu.accel_abi_send_id;
    case 27: return imu.mag_abi_send_id;
    case 28: return guidance_h.use_ref;
    case 29: return gh_ref.max_speed;
    case 30: return gh_ref.tau;
    case 31: return gh_ref.omega;
    case 32: return gh_ref.zeta;
    case 33: return guidance_h.sp.pos.x;
    case 34: return guidance_h.sp.pos.y;
    case 35: return guidance_v.nominal_throttle;
    case 36: return guidance_v.z_sp;
    case 37: return guidance_pid.approx_force_by_thrust;
    case 38: return guidance_pid.kp;
    case 39: return guidance_pid.kd;
    case 40: return guidance_pid.ki;
    case 41: return guidance_pid.kv;
    case 42: return guidance_pid.ka;
    case 43: return guidance_pid.v_kp;
    case 44: return guidance_pid.v_kd;
    case 45: return guidance_pid.v_ki;
    case 46: return guidance_pid.adapt_throttle_enabled;
    case 47: return ahrs_madgwick.reset;
    case 48: return vff.accel_noise;
    case 49: return vff.r_baro;
    case 50: return vff.r_alt;
    case 51: return vff.r_obs_height;
    case 52: return stabilization_gains.p.x;
    case 53: return stabilization_gains.d.x;
    case 54: return stabilization_gains.i.x;
    case 55: return stabilization_gains.dd.x;
    case 56: return stabilization_gains.p.y;
    case 57: return stabilization_gains.d.y;
    case 58: return stabilization_gains.i.y;
    case 59: return stabilization_gains.dd.y;
    case 60: return stabilization_gains.p.z;
    case 61: return stabilization_gains.d.z;
    case 62: return stabilization_gains.i.z;
    case 63: return stabilization_gains.dd.z;
    case 64: return att_ref_quat_i.model.omega.p;
    case 65: return att_ref_quat_i.model.zeta.p;
    case 66: return att_ref_quat_i.model.omega.q;
    case 67: return att_ref_quat_i.model.zeta.q;
    case 68: return att_ref_quat_i.model.omega.r;
    case 69: return att_ref_quat_i.model.zeta.r;
    default: return 0.;
  }
}

/* Persistent Settings */
struct PersistentSettings {
  float s_0; /* air_data.tas_factor */
  uint8_t s_1; /* air_data.calc_airspeed */
  uint8_t s_2; /* air_data.calc_tas_factor */
  uint8_t s_3; /* air_data.calc_amsl_baro */
  float s_4; /* imu.body_to_imu.eulers_f.phi */
  float s_5; /* imu.body_to_imu.eulers_f.theta */
  float s_6; /* imu.body_to_imu.eulers_f.psi */
  float s_7; /* guidance_h.use_ref */
  float s_8; /* gh_ref.max_speed */
  float s_9; /* gh_ref.tau */
  float s_10; /* gh_ref.omega */
  float s_11; /* gh_ref.zeta */
  float s_12; /* guidance_v.nominal_throttle */
  uint8_t s_13; /* guidance_pid.approx_force_by_thrust */
  int32_t s_14; /* guidance_pid.kp */
  int32_t s_15; /* guidance_pid.kd */
  int32_t s_16; /* guidance_pid.ki */
  int32_t s_17; /* guidance_pid.kv */
  int32_t s_18; /* guidance_pid.ka */
  float s_19; /* guidance_pid.v_kp */
  float s_20; /* guidance_pid.v_kd */
  float s_21; /* guidance_pid.v_ki */
  float s_22; /* guidance_pid.adapt_throttle_enabled */
  int32_t s_23; /* stabilization_gains.p.x */
  int32_t s_24; /* stabilization_gains.d.x */
  int32_t s_25; /* stabilization_gains.i.x */
  int32_t s_26; /* stabilization_gains.dd.x */
  int32_t s_27; /* stabilization_gains.p.y */
  int32_t s_28; /* stabilization_gains.d.y */
  int32_t s_29; /* stabilization_gains.i.y */
  int32_t s_30; /* stabilization_gains.dd.y */
  int32_t s_31; /* stabilization_gains.p.z */
  int32_t s_32; /* stabilization_gains.d.z */
  int32_t s_33; /* stabilization_gains.i.z */
  int32_t s_34; /* stabilization_gains.dd.z */
};

extern struct PersistentSettings pers_settings;

static inline void persistent_settings_store( void ) {
  pers_settings.s_0 = air_data.tas_factor;
  pers_settings.s_1 = air_data.calc_airspeed;
  pers_settings.s_2 = air_data.calc_tas_factor;
  pers_settings.s_3 = air_data.calc_amsl_baro;
  pers_settings.s_4 = imu.body_to_imu.eulers_f.phi;
  pers_settings.s_5 = imu.body_to_imu.eulers_f.theta;
  pers_settings.s_6 = imu.body_to_imu.eulers_f.psi;
  pers_settings.s_7 = guidance_h.use_ref;
  pers_settings.s_8 = gh_ref.max_speed;
  pers_settings.s_9 = gh_ref.tau;
  pers_settings.s_10 = gh_ref.omega;
  pers_settings.s_11 = gh_ref.zeta;
  pers_settings.s_12 = guidance_v.nominal_throttle;
  pers_settings.s_13 = guidance_pid.approx_force_by_thrust;
  pers_settings.s_14 = guidance_pid.kp;
  pers_settings.s_15 = guidance_pid.kd;
  pers_settings.s_16 = guidance_pid.ki;
  pers_settings.s_17 = guidance_pid.kv;
  pers_settings.s_18 = guidance_pid.ka;
  pers_settings.s_19 = guidance_pid.v_kp;
  pers_settings.s_20 = guidance_pid.v_kd;
  pers_settings.s_21 = guidance_pid.v_ki;
  pers_settings.s_22 = guidance_pid.adapt_throttle_enabled;
  pers_settings.s_23 = stabilization_gains.p.x;
  pers_settings.s_24 = stabilization_gains.d.x;
  pers_settings.s_25 = stabilization_gains.i.x;
  pers_settings.s_26 = stabilization_gains.dd.x;
  pers_settings.s_27 = stabilization_gains.p.y;
  pers_settings.s_28 = stabilization_gains.d.y;
  pers_settings.s_29 = stabilization_gains.i.y;
  pers_settings.s_30 = stabilization_gains.dd.y;
  pers_settings.s_31 = stabilization_gains.p.z;
  pers_settings.s_32 = stabilization_gains.d.z;
  pers_settings.s_33 = stabilization_gains.i.z;
  pers_settings.s_34 = stabilization_gains.dd.z;
}

static inline void persistent_settings_load( void ) {
  air_data.tas_factor = pers_settings.s_0;
  air_data.calc_airspeed = pers_settings.s_1;
  air_data.calc_tas_factor = pers_settings.s_2;
  air_data.calc_amsl_baro = pers_settings.s_3;
  imu_SetBodyToImuPhi( pers_settings.s_4 );
  imu_SetBodyToImuTheta( pers_settings.s_5 );
  imu_SetBodyToImuPsi( pers_settings.s_6 );
  guidance_h_SetUseRef( pers_settings.s_7 );
  guidance_h_SetMaxSpeed( pers_settings.s_8 );
  guidance_h_SetTau( pers_settings.s_9 );
  guidance_h_SetOmega( pers_settings.s_10 );
  guidance_h_SetZeta( pers_settings.s_11 );
  guidance_v.nominal_throttle = pers_settings.s_12;
  guidance_pid.approx_force_by_thrust = pers_settings.s_13;
  guidance_pid.kp = pers_settings.s_14;
  guidance_pid.kd = pers_settings.s_15;
  guidance_pid_set_h_igain( pers_settings.s_16 );
  guidance_pid.kv = pers_settings.s_17;
  guidance_pid.ka = pers_settings.s_18;
  guidance_pid.v_kp = pers_settings.s_19;
  guidance_pid.v_kd = pers_settings.s_20;
  guidance_pid_set_v_igain( pers_settings.s_21 );
  guidance_pid.adapt_throttle_enabled = pers_settings.s_22;
  stabilization_gains.p.x = pers_settings.s_23;
  stabilization_gains.d.x = pers_settings.s_24;
  stabilization_gains.i.x = pers_settings.s_25;
  stabilization_gains.dd.x = pers_settings.s_26;
  stabilization_gains.p.y = pers_settings.s_27;
  stabilization_gains.d.y = pers_settings.s_28;
  stabilization_gains.i.y = pers_settings.s_29;
  stabilization_gains.dd.y = pers_settings.s_30;
  stabilization_gains.p.z = pers_settings.s_31;
  stabilization_gains.d.z = pers_settings.s_32;
  stabilization_gains.i.z = pers_settings.s_33;
  stabilization_gains.dd.z = pers_settings.s_34;
}

#endif // SETTINGS_H
