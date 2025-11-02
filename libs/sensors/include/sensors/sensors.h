#pragma once

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

enum position { DOT_STATE, DASH_STATE, WHITESPACE_STATE };
enum motion { WAITING, MOVING, COOLDOWN };

typedef struct {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float t;
  float pitch;
  float roll;
  uint8_t error;
} motion_data_t;

extern const char IMU_FIELD_NAMES[];

int hello_sensors(void);

// A wrapper for ICM42670_read_sensor_data. Reads sensor values into a struct.
void read_motion_data(motion_data_t *data);

// Read IMU sensor data and filter it with exponential moving average. Alpha
// must be ]0, 1].
void read_filtered_motion_data(motion_data_t *data, float alpha);

// Formats the motion data into a given buffer in csv form.
void format_motion_csv(const motion_data_t *data, char *buf, size_t buf_size);

// Calculates the exponential moving average of two floats
float exp_moving_avg(float current, float next, float alpha);

// Calculates the current position state of the device.
// Returns
// 0 : Neutral position (dot)
// 1 : Sideways position (dash)
// 2 : Upwards position (whitespace)
uint8_t get_position(const motion_data_t *data);

// Detect flicking motion
void detect_flicking(motion_data_t *data);

// Combine accelerometer and gyro data.
// Uses sensor fusion with simple complimentary filter.
// Based on a python complimentary filter by Philip Salmony:
// https://github.com/pms67/Attitude-Estimation/blob/master/complimentary_imu.py
void sensor_fusion(motion_data_t *data, float alpha, float sample_time);
