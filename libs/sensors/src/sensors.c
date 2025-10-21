#include "tkjhat/sdk.h"
#include <sensors/sensors.h>

int hello_sensors(void) { return 666; }

void read_motion_data(motion_data_t *data) {
  // TODO: Check the return code for errors.
  ICM42670_read_sensor_data(&(data->ax), &(data->ay), &(data->az), &(data->gx),
                            &(data->gy), &(data->gz), &(data->t));
}
