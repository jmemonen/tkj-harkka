#pragma once

typedef struct {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float t;
} motion_data_t;

int hello_sensors(void);

void read_motion_data(motion_data_t *data);
