/* Standard libraries */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* Snapdragon Navigator API */
#include "snapdragon_navigator.h"

static const float time_step = 0.01;

int set_props(SnavCachedData* snav_data, bool spin_props)
{
  SnPropsState props_state = (SnPropsState) snav_data->general_status.props_state;
  if (props_state == SN_PROPS_STATE_NOT_SPINNING && spin_props)
  {
    return sn_spin_props();
  }
  else
  {
    return sn_stop_props();
  }
}

void heartbeat()
{
  sn_send_rc_command(SN_RC_VIO_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING, 0, 0, 0, 0);
}

typedef struct Pose
{
  float pos[3];
  float yaw;
} Pose;

typedef struct TrajCmd
{
  float pos[3];
  float vel[3];
  float acc[3];
  float yaw[2];
} TrajCmd;

float ct_to_time(unsigned int cntr)
{
  return (float) cntr*time_step;
}

void bezier_mission(TrajCmd* cmd, SnavCachedData* snav_data, Pose* origin, float time)
{
  const float omega = 0.5f;
  const float radius = 1.0f;

  cmd->pos[0] = radius*(cos(omega*time) - 1.0f);
  cmd->pos[1] = radius*sin(omega*time);
}

void circle_mission(TrajCmd* cmd, SnavCachedData* snav_data, Pose* origin, float time)
{
  const float omega = 0.5f;
  const float radius = 1.0f;

  cmd->pos[0] = radius*(cos(omega*time) - 1.0f);
  cmd->pos[1] = radius*sin(omega*time);
}

float wrap_angle(float angle)
{
  if (angle > 3.14)
    angle = angle - 6.28;
  else if (angle < -3.14)
    angle = angle + 6.28;
  return angle;
}

void spin360_mission(TrajCmd* cmd, SnavCachedData* snav_data, Pose* origin, float time)
{
  // cmd->pos[2] = origin->pos[2] + 1.0f;
  cmd->yaw[0] = wrap_angle(origin->yaw + time);
  cmd->yaw[1] = 1.0f;
}

void land_mission(TrajCmd* cmd, SnavCachedData* snav_data, Pose* origin, float time)
{
  const float landing_speed = 0.5f;

  cmd->pos[2] -= time_step*landing_speed;
  cmd->vel[2] = -landing_speed;
}

void init_mission(TrajCmd* cmd, SnavCachedData* snav_data, Pose* origin, float time)
{
  cmd->pos[0] = origin->pos[0];
  cmd->pos[1] = origin->pos[1];
  cmd->pos[2] = origin->pos[2];
  cmd->yaw[0] = origin->yaw;
}

void takeoff_mission(TrajCmd* cmd, SnavCachedData* snav_data, Pose* origin, float time)
{
  const float takeoff_height = 2.0f;
  const float takeoff_speed = 0.30f;

  if (cmd->pos[2] - origin->pos[2] < takeoff_height)
  {
    cmd->pos[2] += time_step*takeoff_speed;
    cmd->vel[2] = takeoff_speed;
  }
  else
  {
    cmd->vel[2] = 0.0f;
  }
}


void mission(SnavCachedData* snav_data, Pose* origin, unsigned int cntr)
{
  // TrajCmd cmd = {};
  static TrajCmd cmd;
  float time = ct_to_time(cntr);

  if (time < 0.05f)
  {
    init_mission(&cmd, snav_data, origin, time);
  }
  else if (time < 10.0f)
  {
    takeoff_mission(&cmd, snav_data, origin, time);
  }
  else if (time < 10.0f + 3.14f*4.0f)
  {
    // spin360_mission(&cmd, snav_data, origin, time - 10.0f);
    circle_mission(&cmd, snav_data, origin, time - 10.0f);
  }
  else
  {
    land_mission(&cmd, snav_data, origin, time - 12.56f);
  }

  sn_send_trajectory_tracking_command(SN_POSITION_CONTROL_VIO, SN_TRAJ_DEFAULT, cmd.pos[0], cmd.pos[1], cmd.pos[2], cmd.vel[0], cmd.vel[1], cmd.vel[2], cmd.acc[0], cmd.acc[1], cmd.acc[2], cmd.yaw[0], cmd.yaw[1]);
}

int main(int argc, char* argv[])
{
  // loop counter
  unsigned int cntr = 0;
  static Pose origin;

  // Attempt to initialize pointer to snav data structure
  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return -1;
  }

  // update cached data within Snapdragon Navigator
  int update_ret = sn_update_data();
  if (update_ret != 0)
  {
    printf("\nDetected likely failure in SN. Ensure it is running\n");
  }

  // start props
  if (set_props(snav_data, true) != 0)
  {
    printf("\nFlight software not functional\n");
    return -2;
  }
  heartbeat();

  memcpy(origin.pos, snav_data->pos_vel.position_estimated, sizeof(float)*3);
  origin.yaw = snav_data->pos_vel.yaw_estimated;

  while (true)
  {
    // update cached data within Snapdragon Navigator
    int update_ret = sn_update_data();
    if (update_ret != 0)
    {
      printf("\nDetected likely failure in SN. Ensure it is running\n");
      break;
    }

    mission(snav_data, &origin, cntr);

    // safety kill mode
    if (snav_data->spektrum_rc_0_raw.vals[5] > 512)
    {
      if (set_props(snav_data, false) != 0)
      {
        printf("\nFlight software not functional\n");
        return -2;
      }
      return 0;
    }

    usleep(10000);
    cntr++;
  }

  return 0;
}


