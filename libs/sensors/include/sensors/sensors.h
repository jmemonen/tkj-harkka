#pragma once

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#define ABS(x) (((x) < (0)) ? (-x) : (x))

enum position { DOT_STATE, DASH_STATE, WHITESPACE_STATE };

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
  GESTURE_DASH
} Gesture_t;

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

// Evaluates the motion data values to detect gestures.
// Returns a Gesture_t type:
// GESTURE_NONE: No gesture detected
// GESTURE_DOT: Dot (left flick)
// GESTURE_DASH: Dash (right flick)
Gesture_t detect_gesture(const motion_data_t *data);
