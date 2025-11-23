#include "tkjhat/sdk.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <tkj_utils/tkj_utils.h>

#define ABS(x) (((x) < (0)) ? (-x) : (x))

// Gesture thresholds
// READY
#define READY_G_SUM 20.0f
#define READY_AZ 0.7f
#define READY_AY -0.3f

// SEND
#define SEND_AZ 1.8f
#define SEND_GX -180.0f

// SPACE
#define SPACE_AZ 0.35f
#define SPACE_GX 120.0f

// DOT
#define DOT_AX 0.4f
#define DOT_GY_GZ 100.0f

// DASH
#define DASH_AX -0.5f
#define DASH_GY_GZ -100.0f

// Field names in order for printing out in csv-form.
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

/**
 * Tried to implement a sensor fusion to combine accelometer and gyro data
 * into pitch and roll of the device. Pitch calculation works quite fine
 * but roll calculation has some issues with latency and quick movements.
 * We ended up scrapping the idea of using sensor fusion.
 *  */

/*
// Sensor fusion
void sensor_fusion(motion_data_t *data, float alpha, float sample_time) {
  // Check that given alpha is acceptable
  if (alpha <= 0 || alpha > 1) {
    return;
  }

  float roll_rad = data->roll * DEG_TO_RAD;
  float pitch_rad = data->pitch * DEG_TO_RAD;

  // Complimentary filter estimates for phi (pitch) and theta (roll)
  // Roll axis is y-axis and pitch axis is x-axis.
  // Calculate angle estimates based on accelerometer data:
  float phi_acc = atan2(data->ax, data->az);
  float theta_acc = asinf(data->ay);
  //float theta_acc = atan2(-data->ax, sqrtf(data->ay * data->ay + data->az * data->az));

  // Gyro data in rad/s
  float p_rps = data->gy * DEG_TO_RAD;
  float q_rps = data->gx * DEG_TO_RAD;
  float r_rps = data->gz * DEG_TO_RAD;

  // Transform gyro measurements from body rates to Euler rates
  float phi_gyro_rps = p_rps + sinf(roll_rad) * tanf(pitch_rad) * q_rps 
                      + cosf(roll_rad) * tanf(pitch_rad) * r_rps;
  float theta_gyro_rps = cosf(roll_rad) * q_rps - sin(roll_rad) * r_rps;

  // Combine accelerometer estimates with integral of gyro readings
  roll_rad = alpha * phi_acc 
              + (1.0f - alpha) * (roll_rad + (sample_time / 1000.0f) * phi_gyro_rps);
  pitch_rad = alpha * theta_acc 
              + (1.0f - alpha) * (pitch_rad + (sample_time / 1000.0f) * theta_gyro_rps);

  data->roll = roll_rad * RAD_TO_DEG;
  data->pitch = pitch_rad * RAD_TO_DEG;
}

*/
