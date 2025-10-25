#pragma once

#include <stddef.h>
#include <stdint.h>

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

extern const char IMU_FIELD_NAMES[];

int hello_sensors(void);

// A wrapper for ICM42670_read_sensor_data. Reads sensor values into a struct.
void read_motion_data(motion_data_t *data);

// Read IMU sensor data and filter it with exponential moving average. Alpha must be ]0, 1].
void read_filtered_motion_data(motion_data_t *data, float alpha);

// Formats the motion data into a given buffer in csv form.
void format_motion_csv(const motion_data_t *data, char *buf, size_t buf_size);

// Calculates the exponential moving average of two floats
float exp_moving_avg(float current, float next, float alpha);
