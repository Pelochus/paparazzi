/* This file has been generated from  */
/* Version v7.0_unstable-15-g5c1c35e859-dirty */
/* Please DO NOT EDIT */

#ifndef MODULES_H
#define MODULES_H

#define MODULES_IDLE  0
#define MODULES_RUN   1
#define MODULES_START 2
#define MODULES_STOP  3

#ifndef MODULES_FREQUENCY
#ifdef PERIODIC_FREQUENCY
#define MODULES_FREQUENCY PERIODIC_FREQUENCY
#else
#error "neither MODULES_FREQUENCY or PERIODIC_FREQUENCY are defined"
#endif
#endif

#ifdef MODULES_C
#define EXTERN_MODULES
#else
#define EXTERN_MODULES extern
#endif
#include "std.h"
#include "datalink/bitcraze/syslink_dl.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "stabilization/stabilization_none.h"
#include "core/settings.h"
#include "radio_control/radio_control.h"
#include "firmwares/rotorcraft/navigation.h"
#include "./mcu.h"
#include "mcu_periph/gpio.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_trig_int.h"
#include "math/pprz_orientation_conversion.h"
#include "math/pprz_stat.h"
#include "mcu_periph/i2c.h"
#include "gps/gps.h"
#include "datalink/telemetry.h"
#include "datalink/datalink.h"
#include "datalink/downlink.h"
#include "sensors/baro_board_common.h"
#include "air_data/air_data.h"
#include "ahrs/ahrs.h"
#include "actuators/actuators.h"
#include "actuators/actuators_pwm.h"
#include "mcu_periph/adc.h"
#include "sensors/baro_bmp3.h"
#include "energy/electrical.h"
#include "imu/imu.h"
#include "nav/nav_rotorcraft_base.h"
#include "radio_control/rc_datalink.h"
#include "./state.h"
#include "core/sys_mon.h"
#include "mcu_periph/uart.h"
#include "datalink/pprz_dl.h"
#include "core/commands.h"
#include "./led.h"
#include "imu/imu_bmi088_i2c.h"
#include "guidance/guidance_h.h"
#include "guidance/guidance_v.h"
#include "guidance/guidance_pid.h"
#include "gps/gps.h"
#include "./autopilot_guided.h"
#include "ahrs/ahrs.h"
#include "ins/ins_int.h"
#include "stabilization/stabilization_attitude.h"
#include "stabilization/stabilization_attitude_quat_int.h"
#include "actuators/motor_mixing.h"
#include "./autopilot.h"

#define SYSLINK_DL_PERIODIC_PERIOD (0.500000f)
#define SYSLINK_DL_PERIODIC_FREQ (2.000000f)
#define RADIO_CONTROL_PERIODIC_TASK_PERIOD (1.f / MODULES_FREQUENCY)
#define RADIO_CONTROL_PERIODIC_TASK_FREQ (MODULES_FREQUENCY)
#define DATALINK_PERIODIC_PERIOD (1.000000f)
#define DATALINK_PERIODIC_FREQ (1.000000f)
#define TELEMETRY_REPORTING_TASK_PERIOD (1.f / MODULES_FREQUENCY)
#define TELEMETRY_REPORTING_TASK_FREQ (MODULES_FREQUENCY)
#define BARO_BOARD_PERIODIC_PERIOD (1.f / (BARO_PERIODIC_FREQUENCY))
#define BARO_BOARD_PERIODIC_FREQ (BARO_PERIODIC_FREQUENCY)
#define AIR_DATA_PERIODIC_PERIOD (0.100000f)
#define AIR_DATA_PERIODIC_FREQ (10.000000f)
#define ACTUATORS_PERIODIC_PERIOD (1.f / MODULES_FREQUENCY)
#define ACTUATORS_PERIODIC_FREQ (MODULES_FREQUENCY)
#define BARO_BMP3_PERIODIC_PERIOD (0.020000f)
#define BARO_BMP3_PERIODIC_FREQ (50.000000f)
#define ELECTRICAL_PERIODIC_PERIOD (0.100000f)
#define ELECTRICAL_PERIODIC_FREQ (10.000000f)
#define PERIODIC_SYSMON_PERIOD (1.f / MODULES_FREQUENCY)
#define PERIODIC_SYSMON_FREQ (MODULES_FREQUENCY)
#define PERIODIC_REPORT_SYSMON_PERIOD (1.000000f)
#define PERIODIC_REPORT_SYSMON_FREQ (1.000000f)
#define LED_PERIODIC_PERIOD (0.020000f)
#define LED_PERIODIC_FREQ (50.000000f)
#define IMU_BMI088_PERIODIC_PERIOD (1.f / MODULES_FREQUENCY)
#define IMU_BMI088_PERIODIC_FREQ (MODULES_FREQUENCY)
#define GPS_DATALINK_PERIODIC_CHECK_PERIOD (1.000000f)
#define GPS_DATALINK_PERIODIC_CHECK_FREQ (1.000000f)
#define AUTOPILOT_PERIODIC_PERIOD (1.f / MODULES_FREQUENCY)
#define AUTOPILOT_PERIODIC_FREQ (MODULES_FREQUENCY)

#define PRESCALER_1 (uint32_t)(MODULES_FREQUENCY * (0.500000f))
#define PRESCALER_2 (uint32_t)(MODULES_FREQUENCY * (1.f / MODULES_FREQUENCY))
#define PRESCALER_3 (uint32_t)(MODULES_FREQUENCY * (1.000000f))
#define PRESCALER_4 (uint32_t)(MODULES_FREQUENCY * (1.f / (BARO_PERIODIC_FREQUENCY)))
#define PRESCALER_5 (uint32_t)(MODULES_FREQUENCY * (0.100000f))
#define PRESCALER_6 (uint32_t)(MODULES_FREQUENCY * (0.020000f))

EXTERN_MODULES uint8_t gps_datalink_gps_datalink_periodic_check_status;


static inline void modules_mcu_init(void) {
  mcu_init();
}

static inline void modules_core_init(void) {
  settings_init();
  pprz_trig_int_init();
  air_data_init();
  electrical_init();
  stateInit();
  init_sysmon();
  commands_init();
}

static inline void modules_sensors_init(void) {
  gps_init();
  baro_board_init();
  baro_bmp3_init();
  imu_init();
  imu_bmi088_init();
  gps_datalink_init();
  gps_datalink_gps_datalink_periodic_check_status = MODULES_START;
}

static inline void modules_estimation_init(void) {
  ahrs_init();
  ins_int_init();
}

static inline void modules_radio_control_init(void) {
  radio_control_init();
  rc_datalink_init();
}

static inline void modules_control_init(void) {
  stabilization_init();
  stabilization_none_init();
  nav_init();
  nav_rotorcraft_init();
  guidance_h_init();
  guidance_v_init();
  guidance_pid_init();
  stabilization_attitude_init();
  autopilot_init();
}

static inline void modules_actuators_init(void) {
  actuators_init();
  motor_mixing_init();
}

static inline void modules_datalink_init(void) {
  syslink_dl_init();
  datalink_init();
  downlink_init();
  pprz_dl_init();
}

static inline void modules_default_init(void) {
}

static inline void modules_init(void) {
  modules_mcu_init();
  modules_core_init();
  modules_sensors_init();
  modules_estimation_init();
  modules_radio_control_init();
  modules_control_init();
  modules_actuators_init();
  modules_datalink_init();
  modules_default_init();
}

static inline void modules_mcu_periodic_task(void) {

}

static inline void modules_core_periodic_task(void) {
  static uint32_t i2; i2++; if (i2>=PRESCALER_2) i2=0;
  static uint32_t i3; i3++; if (i3>=PRESCALER_3) i3=0;
  static uint32_t i5; i5++; if (i5>=PRESCALER_5) i5=0;
  static uint32_t i6; i6++; if (i6>=PRESCALER_6) i6=0;

  if (i6 == (uint32_t)(0.200000f * PRESCALER_6)) {
    LED_PERIODIC();
  }
  if (i5 == (uint32_t)(0.600000f * PRESCALER_5)) {
    air_data_periodic();
  }
  if (i5 == (uint32_t)(0.900000f * PRESCALER_5)) {
    electrical_periodic();
  }
  if (i3 == (uint32_t)(0.100000f * PRESCALER_3)) {
    periodic_report_sysmon();
  }
  periodic_sysmon();
}

static inline void modules_sensors_periodic_task(void) {
  static uint32_t i2; i2++; if (i2>=PRESCALER_2) i2=0;
  static uint32_t i3; i3++; if (i3>=PRESCALER_3) i3=0;
  static uint32_t i4; i4++; if (i4>=PRESCALER_4) i4=0;
  static uint32_t i6; i6++; if (i6>=PRESCALER_6) i6=0;

  if (gps_datalink_gps_datalink_periodic_check_status == MODULES_START) {
    gps_datalink_gps_datalink_periodic_check_status = MODULES_RUN;
  }
  if (gps_datalink_gps_datalink_periodic_check_status == MODULES_STOP) {
    gps_datalink_gps_datalink_periodic_check_status = MODULES_IDLE;
  }

  if (i6 == (uint32_t)(0.800000f * PRESCALER_6)) {
    baro_bmp3_periodic();
  }
  if (i3 == (uint32_t)(0.400000f * PRESCALER_3) && gps_datalink_gps_datalink_periodic_check_status == MODULES_RUN) {
    gps_datalink_periodic_check();
  }
  if (i4 == (uint32_t)(0.500000f * PRESCALER_4)) {
    baro_board_periodic();
  }
  imu_bmi088_periodic();
}

static inline void modules_estimation_periodic_task(void) {

}

static inline void modules_radio_control_periodic_task(void) {
  static uint32_t i2; i2++; if (i2>=PRESCALER_2) i2=0;

  radio_control_periodic_task();
}

static inline void modules_control_periodic_task(void) {
  static uint32_t i2; i2++; if (i2>=PRESCALER_2) i2=0;

  autopilot_periodic();
}

static inline void modules_actuators_periodic_task(void) {
  static uint32_t i2; i2++; if (i2>=PRESCALER_2) i2=0;

  actuators_periodic();
}

static inline void modules_datalink_periodic_task(void) {
  static uint32_t i1; i1++; if (i1>=PRESCALER_1) i1=0;
  static uint32_t i2; i2++; if (i2>=PRESCALER_2) i2=0;
  static uint32_t i3; i3++; if (i3>=PRESCALER_3) i3=0;

  if (i1 == (uint32_t)(0.100000f * PRESCALER_1)) {
    syslink_dl_periodic();
  }
  if (i3 == (uint32_t)(0.300000f * PRESCALER_3)) {
    datalink_periodic();
  }
  telemetry_reporting_task();
}

static inline void modules_default_periodic_task(void) {

}

static inline void modules_periodic_task(void) {
  modules_mcu_periodic_task();
  modules_core_periodic_task();
  modules_sensors_periodic_task();
  modules_estimation_periodic_task();
  modules_radio_control_periodic_task();
  modules_control_periodic_task();
  modules_actuators_periodic_task();
  modules_datalink_periodic_task();
  modules_default_periodic_task();
}

static inline void modules_mcu_event_task(void) {
  mcu_event();
}

static inline void modules_core_event_task(void) {
  event_sysmon();
}

static inline void modules_sensors_event_task(void) {
  baro_board_event();
  baro_bmp3_event();
  imu_bmi088_event();
}

static inline void modules_estimation_event_task(void) {
}

static inline void modules_radio_control_event_task(void) {
  rc_datalink_event();
}

static inline void modules_control_event_task(void) {
  autopilot_event();
}

static inline void modules_actuators_event_task(void) {
}

static inline void modules_datalink_event_task(void) {
  syslink_dl_event();
#if !(TELEMETRY_DISABLE_RX)
  pprz_dl_event();
#endif
}

static inline void modules_default_event_task(void) {
}

static inline void modules_event_task(void) {
  modules_mcu_event_task();
  modules_core_event_task();
  modules_sensors_event_task();
  modules_estimation_event_task();
  modules_radio_control_event_task();
  modules_control_event_task();
  modules_actuators_event_task();
  modules_datalink_event_task();
  modules_default_event_task();
}

#ifdef MODULES_DATALINK_C

#include "pprzlink/messages.h"

#include "pprzlink/dl_protocol.h"

#include "pprzlink/intermcu_msg.h"
#include "generated/airframe.h"
static inline void modules_parse_datalink(uint8_t msg_id __attribute__((unused)),
                                          uint8_t class_id __attribute__((unused)),
                                          struct link_device *dev __attribute__((unused)),
                                          struct transport_tx *trans __attribute__((unused)),
                                          uint8_t *buf __attribute__((unused))) {
  switch (class_id) {
    case DL_datalink_CLASS_ID: {
      switch (msg_id) {
        case DL_MOVE_WP: {
          nav_parse_MOVE_WP(buf);
          break;
        }
        case DL_GUIDED_SETPOINT_NED: {
#if AP_MODE_GUIDED
          autopilot_guided_parse_GUIDED(buf);
#endif
#if AP_MODE_GUIDED
          autopilot_guided_parse_GUIDED(buf);
#endif
          break;
        }
        case DL_GET_SETTING: {
          settings_parse_msg_GET_SETTING(dev,trans,buf);
          break;
        }
        case DL_RC_4CH: {
          rc_datalink_parse_RC_4CH(buf);
          break;
        }
        case DL_RC_3CH: {
          rc_datalink_parse_RC_3CH(buf);
          break;
        }
        case DL_BLOCK: {
          nav_parse_BLOCK(buf);
          break;
        }
        case DL_REMOTE_GPS_LOCAL: {
          gps_datalink_parse_REMOTE_GPS_LOCAL(buf);
          break;
        }
        case DL_PING: {
          datalink_parse_PING(dev, trans, buf);
          break;
        }
        case DL_EXTERNAL_POSE: {
          gps_datalink_parse_EXTERNAL_POSE(buf);
          break;
        }
        case DL_RTCM_INJECT: {
          gps_parse_RTCM_INJECT(buf);
          break;
        }
        case DL_EXTERNAL_POSE_SMALL: {
          gps_datalink_parse_EXTERNAL_POSE_SMALL(buf);
          break;
        }
        case DL_WIND_INFO: {
          air_data_parse_WIND_INFO(dev,trans,buf);
          break;
        }
        case DL_GPS_INJECT: {
          gps_parse_GPS_INJECT(buf);
          break;
        }
        case DL_SETTING: {
          settings_parse_msg_SETTING(dev,trans,buf);
          break;
        }
        default: break;
      }
      break;
    }
    default: break;
  }
}

#endif // MODULES_DATALINK_C

#endif // MODULES_H
