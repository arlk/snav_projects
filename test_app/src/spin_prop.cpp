/* Standard libraries */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/* Snapdragon Navigator API */
#include "snapdragon_navigator.h"

int main(int argc, char* argv[])
{
  // loop counter
  unsigned int cntr = 0;

  // Attempt to initialize pointer to snav data structure
  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return -1;
  }

  // update LED colors in infinite loop
  bool keep_going = true;
  while ( keep_going )
  {
    // update cached data within Snapdragon Navigator
    int update_ret = sn_update_data();
    if (update_ret != 0)
    {
      printf("\nDetected likely failure in SN. Ensure it is running\n");
      break;
    }

    int32_t props_state = snav_data->general_status.props_state;
    if (props_state == SN_PROPS_STATE_NOT_SPINNING)
	{
            printf("SPINNING PROPS... \n");
	    int ret = sn_spin_props();
	    if (ret != 0)
	    {
	      printf("Flight software not functional. \n");
	      keep_going = false;
	    }
	    printf("State: %d \n", props_state);
	}
   else if (cntr > 1000)
	{
            printf("STOPPING PROPS... \n");
	    int ret = sn_stop_props();
	    if (ret != 0)
	    {
	      printf("Flight software not functional. \n");
	    }
            keep_going = false;
	    printf("State: %d", props_state);
	}

    sn_send_rc_command(SN_RC_THRUST_ANGLE_CMD, RC_OPT_LINEAR_MAPPING, 0, 0, 0, 0);

    // note that commands should only be sent as often as needed (minimize message traffic)
    usleep(10000);
    cntr++;
  }

  return 0;
}


