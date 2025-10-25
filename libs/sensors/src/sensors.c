#include "tkjhat/sdk.h"
#include <sensors/sensors.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

int hello_sensors(void) { return 666; }

void read_motion_data(motion_data_t *data) {
  if (data == NULL) {
    return; // TODO: an error code?
  }
  // TODO: Check the return code for errors.
  uint8_t error = ICM42670_read_sensor_data(&(data->ax), &(data->ay), &(data->az), &(data->gx),
                            &(data->gy), &(data->gz), &(data->t));

  data->error = error;
}

void format_motion_csv(const motion_data_t *data, char *buf, size_t buf_size) {
  if (data == NULL || buf == NULL || buf_size < 1) {
    return; // TODO: an error code?
  }
  snprintf(buf, buf_size, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", data->ax,
           data->ay, data->az, data->gx, data->gy, data->gz, data->t);
}
