#include "nps_ivy.h"

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <Ivy/ivy.h>

#include <Ivy/ivyloop.h>
#include <pthread.h>

#include "generated/airframe.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_double.h"
#include "nps_main.h"
#include "nps_autopilot.h"
#include "nps_fdm.h"
#include "nps_sensors.h"
#include "nps_atmosphere.h"

#include "generated/settings.h"
#include "pprzlink/dl_protocol.h"
#include "modules/datalink/downlink.h"

#if USE_GPS
#include "modules/gps/gps.h"
#endif

bool nps_ivy_send_world_env = false;
pthread_t th_ivy_main; // runs main Ivy loop
static MsgRcvPtr ivyPtr = NULL;
static int seq = 1;
static int ap_launch_index;
static pthread_mutex_t ivy_mutex; // mutex for ivy send

/* Gaia Ivy functions */
static void on_WORLD_ENV(IvyClientPtr app __attribute__((unused)),
                         void *user_data __attribute__((unused)),
                         int argc, char *argv[]);

/* Datalink Ivy functions */
static void on_DL_SETTING(IvyClientPtr app __attribute__((unused)),
                          void *user_data __attribute__((unused)),
                          int argc, char *argv[]);

void* ivy_main_loop(void* data __attribute__((unused)));

int find_launch_index(void);


void* ivy_main_loop(void* data __attribute__((unused)))
{
  IvyMainLoop();

  return NULL;
}

void nps_ivy_init(char *ivy_bus)
{
  const char *agent_name = AIRFRAME_NAME"_NPS";
  const char *ready_msg = AIRFRAME_NAME"_NPS Ready";
  IvyInit(agent_name, ready_msg, NULL, NULL, NULL, NULL);

  // bind on a general WORLD_ENV (not a reply to request)
  IvyBindMsg(on_WORLD_ENV, NULL, "^(\\S*) WORLD_ENV (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");

  // to be able to change datalink_enabled setting back on
  IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");

#ifdef __APPLE__
  const char *default_ivy_bus = "224.255.255.255";
#else
  const char *default_ivy_bus = "127.255.255.255";
#endif
  if (ivy_bus == NULL) {
    IvyStart(default_ivy_bus);
  } else {
    IvyStart(ivy_bus);
  }

  nps_ivy_send_world_env = false;

  ap_launch_index = find_launch_index();

  // Launch separate thread with IvyMainLoop()
  pthread_create(&th_ivy_main, NULL, ivy_main_loop, NULL);

}

/*
 * Parse WORLD_ENV message from gaia.
 *
 */
static void on_WORLD_ENV(IvyClientPtr app __attribute__((unused)),
                         void *user_data __attribute__((unused)),
                         int argc, char *argv[])
{
  if (argc < 6) { return; }

  // wind speed in m/s
  struct FloatVect3 wind;
  wind.x = atof(argv[1]); //east
  wind.y = atof(argv[2]); //north
  wind.z = atof(argv[3]); //up

  /* set wind speed in NED */
  nps_atmosphere_set_wind_ned(wind.y, wind.x, -wind.z);

  /* not used so far */
  //float ir_contrast = atof(argv[4]);

  /* set new time factor */
  nps_set_time_factor(atof(argv[5]));

#if USE_GPS
  // directly set gps fix in modules/gps/gps_sim_nps.h
  gps_has_fix = atoi(argv[6]); // gps_availability
#endif

}

/*
 * Send a WORLD_ENV_REQ message
 */


void nps_ivy_send_WORLD_ENV_REQ(void)
{
  pthread_mutex_lock(&ivy_mutex);

  // First unbind from previous request if needed
  if (ivyPtr != NULL) {
    IvyUnbindMsg(ivyPtr);
    ivyPtr = NULL;
  }

  int pid = (int)getpid();

  // Bind to the reply
  ivyPtr = IvyBindMsg(on_WORLD_ENV, NULL, "^%d_%d (\\S*) WORLD_ENV (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)", pid, seq);

  // Send actual request
  struct NpsFdm fdm_ivy;
  memcpy(&fdm_ivy, &fdm, sizeof(struct NpsFdm));

  IvySendMsg("nps %d_%d WORLD_ENV_REQ %f %f %f %f %f %f",
      pid, seq,
      DegOfRad(fdm_ivy.lla_pos_pprz.lat),
      DegOfRad(fdm_ivy.lla_pos_pprz.lon),
      (fdm_ivy.hmsl),
      (fdm_ivy.ltpprz_pos.x),
      (fdm_ivy.ltpprz_pos.y),
      (fdm_ivy.ltpprz_pos.z));
  seq++;

  nps_ivy_send_world_env = false;

  pthread_mutex_unlock(&ivy_mutex);
}

int find_launch_index(void)
{
#ifdef AP_LAUNCH
  return AP_LAUNCH - 1; // index of AP_LAUNCH starts at 1, but it should be 0 here
#else
#if NB_SETTING > 0
  static const char ap_launch[] = "aut_lau"; // short name
  char *ap_settings[NB_SETTING] = SETTINGS_NAMES_SHORT;

  // list through the settings
  // TODO: maybe search for a substring with if(strstr(sent, word) != NULL)
  for (uint8_t idx=0;idx<NB_SETTING;idx++) {
   if (strcmp(ap_settings[idx],ap_launch) == 0) {
     return (int)idx;
    }
  }
#endif
  return -1;
#endif
}

static void on_DL_SETTING(IvyClientPtr app __attribute__((unused)),
                          void *user_data __attribute__((unused)),
                          int argc, char *argv[])
{
  if (argc < 3) { return; }

  if (atoi(argv[1]) != AC_ID) {
    return;
  }

  /* HACK:
   * we actually don't want to allow changing settings if datalink is disabled,
   * but since we currently change this variable via settings we have to allow it
   * TODO: only allow changing the datalink_enabled setting
   */
  uint8_t index = atoi(argv[2]);
  float value = atof(argv[3]);
#ifndef HITL
  if (!datalink_enabled) {
    DlSetting(index, value);
    DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &index, &value);
  }
#endif
  printf("setting %d %f\n", index, value);

  /*
   * In case of HITL, update nps_autopilot.launch from DL_SETTINGS
   * so the plane can be properly launched.
   *
   * In case of STIL nps_update_launch_from_dl() is an empty function
   */
  if ((ap_launch_index >= 0) || (ap_launch_index < NB_SETTING)) {
    if (index==ap_launch_index){
      nps_update_launch_from_dl(value);
    }
  }
}


void nps_ivy_display(struct NpsFdm* fdm_data, struct NpsSensors* sensors_data)
{
  struct NpsFdm fdm_ivy;
  struct NpsSensors sensors_ivy;

  // make a local copy with mutex
  pthread_mutex_lock(&fdm_mutex);
  memcpy(&fdm_ivy, fdm_data, sizeof(fdm));
  memcpy(&sensors_ivy, sensors_data, sizeof(sensors));
  pthread_mutex_unlock(&fdm_mutex);

  // protect Ivy thread
  pthread_mutex_lock(&ivy_mutex);

  IvySendMsg("%d NPS_RATE_ATTITUDE %f %f %f %f %f %f",
             AC_ID,
             DegOfRad(fdm_ivy.body_ecef_rotvel.p),
             DegOfRad(fdm_ivy.body_ecef_rotvel.q),
             DegOfRad(fdm_ivy.body_ecef_rotvel.r),
             DegOfRad(fdm_ivy.ltp_to_body_eulers.phi),
             DegOfRad(fdm_ivy.ltp_to_body_eulers.theta),
             DegOfRad(fdm_ivy.ltp_to_body_eulers.psi));
  IvySendMsg("%d NPS_POS_LLH %f %f %f %f %f %f %f %f %f",
             AC_ID,
             (fdm_ivy.lla_pos_pprz.lat),
             (fdm_ivy.lla_pos_geod.lat),
             (fdm_ivy.lla_pos_geoc.lat),
             (fdm_ivy.lla_pos_pprz.lon),
             (fdm_ivy.lla_pos_geod.lon),
             (fdm_ivy.lla_pos_pprz.alt),
             (fdm_ivy.lla_pos_geod.alt),
             (fdm_ivy.agl),
             (fdm_ivy.hmsl));
  IvySendMsg("%d NPS_SPEED_POS %f %f %f %f %f %f %f %f %f",
             AC_ID,
             (fdm_ivy.ltpprz_ecef_accel.x),
             (fdm_ivy.ltpprz_ecef_accel.y),
             (fdm_ivy.ltpprz_ecef_accel.z),
             (fdm_ivy.ltpprz_ecef_vel.x),
             (fdm_ivy.ltpprz_ecef_vel.y),
             (fdm_ivy.ltpprz_ecef_vel.z),
             (fdm_ivy.ltpprz_pos.x),
             (fdm_ivy.ltpprz_pos.y),
             (fdm_ivy.ltpprz_pos.z));
  IvySendMsg("%d NPS_GYRO_BIAS %f %f %f",
             AC_ID,
             DegOfRad(RATE_FLOAT_OF_BFP(sensors_ivy.gyro.bias_random_walk_value.x) + sensors_ivy.gyro.bias_initial.x),
             DegOfRad(RATE_FLOAT_OF_BFP(sensors_ivy.gyro.bias_random_walk_value.y) + sensors_ivy.gyro.bias_initial.y),
             DegOfRad(RATE_FLOAT_OF_BFP(sensors_ivy.gyro.bias_random_walk_value.z) + sensors_ivy.gyro.bias_initial.z));

  /* transform magnetic field to body frame */
  struct DoubleVect3 h_body;
  double_quat_vmult(&h_body, &fdm_ivy.ltp_to_body_quat, &fdm_ivy.ltp_h);

  IvySendMsg("%d NPS_SENSORS_SCALED %f %f %f %f %f %f",
             AC_ID,
             ((sensors_ivy.accel.value.x - sensors_ivy.accel.neutral.x) / NPS_ACCEL_SENSITIVITY_XX),
             ((sensors_ivy.accel.value.y - sensors_ivy.accel.neutral.y) / NPS_ACCEL_SENSITIVITY_YY),
             ((sensors_ivy.accel.value.z - sensors_ivy.accel.neutral.z) / NPS_ACCEL_SENSITIVITY_ZZ),
             h_body.x,
             h_body.y,
             h_body.z);

  IvySendMsg("%d NPS_WIND %f %f %f",
             AC_ID,
             fdm_ivy.wind.x,
             fdm_ivy.wind.y,
             fdm_ivy.wind.z);

  pthread_mutex_unlock(&ivy_mutex);

  if (nps_ivy_send_world_env) {
    nps_ivy_send_WORLD_ENV_REQ();
  }
}
