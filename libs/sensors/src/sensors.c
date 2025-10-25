#include "tkjhat/sdk.h"
#include <sensors/sensors.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// Field names for printing out in csv-form.
const char IMU_FIELD_NAMES[] =
    "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,tmp\r\n";

int hello_sensors(void) { return 666; }

void read_motion_data(motion_data_t *data) {
  if (data == NULL) {
    return; // TODO: an error code?
  }
  // TODO: Check the return code for errors.
  uint8_t error = ICM42670_read_sensor_data(
      &(data->ax), &(data->ay), &(data->az), &(data->gx), &(data->gy),
      &(data->gz), &(data->t));

  data->error = error;
}

void format_motion_csv(const motion_data_t *data, char *buf, size_t buf_size) {
  if (data == NULL || buf == NULL || buf_size < 1) {
    return; // TODO: an error code?
  }
  snprintf(buf, buf_size, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", data->ax,
           data->ay, data->az, data->gx, data->gy, data->gz, data->t);
}

// Calculates the exponential moving average of two floats
float exp_moving_avg(float current, float next, float alpha) {
  return (next - current) * alpha + current;
}

static motion_data_t _imu_data_filter;

void read_filtered_motion_data(motion_data_t *data, float alpha) {
  if (alpha <= 0 || alpha > 1) {
    return;
  }
  read_motion_data(&_imu_data_filter);
  data->ax = exp_moving_avg(data->ax, _imu_data_filter.ax, alpha);
  data->ay = exp_moving_avg(data->ay, _imu_data_filter.ay, alpha);
  data->az = exp_moving_avg(data->az, _imu_data_filter.az, alpha);
  data->gx = exp_moving_avg(data->gx, _imu_data_filter.gx, alpha);
  data->gy = exp_moving_avg(data->gy, _imu_data_filter.gy, alpha);
  data->gz = exp_moving_avg(data->gz, _imu_data_filter.gz, alpha);
  data->t = exp_moving_avg(data->t, _imu_data_filter.t, alpha);
}

uint8_t get_position(const motion_data_t *data) {
  // TODO: Away with magic numbers and such.
  // TODO: Works pretty well for something so simple. Could be smoother, though...
  if (data->ay < -0.7) {
    return WHITESPACE_STATE;
  }
  if (data->ax < -0.7) {
    return DASH_STATE;
  }
  return DOT_STATE;
}
