#include "tkjhat/sdk.h"
#include <sensors/sensors.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define ABS(x) (((x) < (0)) ? (-x) : (x))

// Gesture thresholds
#define READY_G_SUM 20.0f
#define READY_AZ 0.7f
#define READY_AY -0.3f
#define SEND_AZ 1.8f
#define SEND_GX -180.0f
#define SPACE_AZ 0.35f
#define SPACE_GX 120.0f
#define DOT_AX 0.4f
#define DOT_GY_GZ 100.0f
#define DASH_AX -0.5f
#define DASH_GY_GZ -100.0f

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
  // TODO: Works pretty well for something so simple. Could be smoother,
  // though...
  if (data->ay < -0.7) {
    return WHITESPACE_STATE;
  }
  if (data->ax < -0.7) {
    return DASH_STATE;
  }
  return DOT_STATE;
}

// TODO: We could have handled theIMU readings as multidimensional direction
// vectors and just check how far the vectors land from defined gesture
// points... That's heavier to calculate though so let's not fix it when
// it's not broken?
Gesture_t detect_gesture(const motion_data_t *data) {
  float gyro_sum = ABS(data->gx) + ABS(data->gy) + ABS(data->gz);

  // Minimal movement and a neutral position.
  if (gyro_sum < READY_G_SUM && data->az > READY_AZ && data->ay < READY_AY) {
    return GESTURE_READY;
  }

  // Up flick
  if (data->az > SEND_AZ && data->gx < SEND_GX) {
    return GESTURE_SEND;
  }

  // Down flick
  if (data->az < SPACE_AZ && data->gx > SPACE_GX) {
    return GESTURE_SPACE;
  }

  // Left flick.
  if (data->ax > DOT_AX && data->gy + data->gz > DOT_GY_GZ) {
    return GESTURE_DOT;
  }

  // Right flick.
  if (data->ax < DASH_AX && data->gy + data->gz < DASH_GY_GZ) {
    return GESTURE_DASH;
  }

  return GESTURE_NONE;
}
