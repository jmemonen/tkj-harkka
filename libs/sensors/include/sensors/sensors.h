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

int hello_sensors(void);

// A wrapper for ICM42670_read_sensor_data. Reads sensor values into a struct.
void read_motion_data(motion_data_t *data);

// Formats the motion data into a given buffer in csv form.
void format_motion_csv(const motion_data_t *data, char *buf, size_t buf_size);
