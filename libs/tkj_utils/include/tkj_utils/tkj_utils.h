#pragma once

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#define ABS(x) (((x) < (0)) ? (-x) : (x))

typedef struct {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float t;
  uint8_t error;
} motion_data_t;

// TODO: have gesture enums and such typedeffed?
enum Gesture_State { STATE_READY, STATE_COOLDOWN };

typedef enum {
  GESTURE_NONE,
  GESTURE_READY,
  GESTURE_DOT,
  GESTURE_DASH,
  GESTURE_SPACE,
  GESTURE_SEND,
} Gesture_t;

// A string of the field names for convenient csv output
// of IMU values from the a motion_data_t.
extern const char IMU_FIELD_NAMES[];

// A wrapper for ICM42670_read_sensor_data. 
// Reads sensor values directly into a motion_data_t struct.
void read_motion_data(motion_data_t *data);

// Read IMU sensor data and filter it with exponential moving average. 
// Alpha must be ]0, 1].
// A bigger alpha means smoother but less responsive readings.
void read_filtered_motion_data(motion_data_t *data, float alpha);

// Formats the motion data into a given buffer in csv form.
// Print out IMU_FIELD_NAMES first to get the field names:
//
// "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,tmp\r\n";
void format_motion_csv(const motion_data_t *data, char *buf, size_t buf_size);

// Calculates the exponential moving average of two floats
float exp_moving_avg(float current, float next, float alpha);

// Evaluates the motion data values to detect gestures.
// Returns a Gesture_t type:
// GESTURE_NONE: No gesture detected
// GESTURE_DOT: Dot (left flick)
// GESTURE_DASH: Dash (right flick)
Gesture_t detect_gesture(const motion_data_t *data);

// Combine accelerometer and gyro data.
// Uses sensor fusion with simple complimentary filter.
// Based on a python complimentary filter by Philip Salmony:
// https://github.com/pms67/Attitude-Estimation/blob/master/complimentary_imu.py
// We ended up scrapping the idea of using sensor fusion.
// void sensor_fusion(motion_data_t *data, float alpha, float sample_time);