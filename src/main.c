#include <pico/time.h>
#include <stdint.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <tusb.h>

#include "portmacro.h"
#include "projdefs.h"
#include "sensors/sensors.h"
#include "tkjhat/sdk.h"
#include "usbSerialDebug/helper.h"

// Default stack size for the tasks. It can be reduced to 1024 if task is not
// using lot of memory.
#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX 1
#define MOTION_BUF_SIZE 128
#define EXP_MOV_AVG_ALPHA 0.25

static motion_data_t motion_data;
static uint8_t position_state = DASH_STATE;
static uint8_t motion_state = WAITING;

// Activates the TinyUSB library.
static void usbTask(void *arg) {
  (void)arg;
  while (1) {
    tud_task(); // With FreeRTOS wait for events
                // Do not add vTaskDelay.
  }
}

// Reads the sensors.
static void sensor_task(void *arg) {
  (void)arg;
  char buf[MOTION_BUF_SIZE];

  while (!tud_mounted() || !tud_cdc_n_connected(1)) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (usb_serial_connected()) {
    usb_serial_print("sensorTask started...\r\n");
    usb_serial_flush();
  }

  if (init_ICM42670() == 0) {
    usb_serial_print("ICM-42670P initialized successfully!\r\n");
    ICM42670_start_with_default_values(); // TODO: Handle error values?
  } else {
    usb_serial_print("Failed to initialize ICM-42670P.\r\n");
  }

  motion_data.error = 0;
  usb_serial_print(IMU_FIELD_NAMES);

  while (1) {
    read_filtered_motion_data(&motion_data, EXP_MOV_AVG_ALPHA);
    if (motion_data.error) {
      usb_serial_print("There was an error reading motion data!\r\n");
      usb_serial_flush();
      motion_data.error = 0;
      continue;
    }

    sensor_fusion(&motion_data, 0.02, 10);
    // Prints for dev and debug
    // format_motion_csv(&motion_data, buf, MOTION_BUF_SIZE);
    // usb_serial_print(buf);
    // usb_serial_flush();
    snprintf(buf, MOTION_BUF_SIZE,"%.3f,%.3f,45,-45\r\n", motion_data.pitch, motion_data.roll);
    usb_serial_print(buf);
    usb_serial_flush();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void position_task(void *arg) {
  (void)arg;

  while (!tud_mounted() || !tud_cdc_n_connected(1)) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (usb_serial_connected()) {
    usb_serial_print("position_task started...\r\n");
    usb_serial_flush();
  }

  for (;;) {
    uint8_t new_position = get_position(&motion_data);
    if (new_position != position_state) {
      position_state = new_position;
      switch (position_state) {
      case DOT_STATE:
        // usb_serial_print("New state: DOT\r\n");
        break;
      case DASH_STATE:
        // usb_serial_print("New state: DASH\r\n");
        break;
      case WHITESPACE_STATE:
        // usb_serial_print("New state: WHITESPACE\r\n");
      }
      usb_serial_flush();
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void motion_task(void *arg) {
  (void)arg;

  TickType_t last_event_tick = 0;

  while(1) {
    TickType_t current_tick = xTaskGetTickCount();
    TickType_t time_difference = (current_tick - last_event_tick) * portTICK_PERIOD_MS;

    switch (motion_state) {
      case WAITING:
        if (motion_data.ax < -0.8) {
          usb_serial_print("Flicked left\r\n");
          motion_state = MOVING;
          usb_serial_print("Motion state: MOVING\r\n");
          last_event_tick = current_tick;
        } 
        else if (motion_data.ax > 0.8) {
          usb_serial_print("Flicked right\r\n");
          motion_state = MOVING;
          usb_serial_print("Motion state: MOVING\r\n");
          last_event_tick = current_tick;
        }
        break;
      case MOVING:
        if (motion_data.ax > -0.2 && motion_data.ax < 0.2) {
          motion_state = COOLDOWN;
          usb_serial_print("Motion state: COOLDOWN\r\n");
          last_event_tick = current_tick;
        }
        break;
      case COOLDOWN:
        if (time_difference > 200) {
          motion_state = WAITING;
          usb_serial_print("Motion state: WAITING\r\n");
        }
        break;
    }

    /*
    if (time_difference > 100) {
      if (motion_data.ax < -0.7) {
        usb_serial_print("Flicked left\r\n");
        last_event_tick = current_tick;
      } else if (motion_data.ax > 0.7) {
        usb_serial_print("Flicked right\r\n");
        last_event_tick = current_tick;
      }
    } */

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

int main() {
  // stdio_init_all();
  // Uncomment this lines if you want to wait till the serial monitor is
  // connected
  /*while (!stdio_usb_connected()){
      sleep_ms(10);
  }*/
  init_hat_sdk();
  sleep_ms(1000); // Wait some time so initialization of USB and hat is done.
  init_i2c_default();
  sleep_ms(1000); // Wait some time so initialization of USB and hat is done.
  init_red_led();

  // init_ICM42670(); // TODO: check return value for errors...
  // ICM42670_start_with_default_values();

  TaskHandle_t positionTask, motionTask, hUSB, sensorTask = NULL;

  xTaskCreate(usbTask, "usb", 2048, NULL, 3, &hUSB);
#if (configNUMBER_OF_CORES > 1)
  vTaskCoreAffinitySet(hUSB, 1u << 0);
#endif

  // Create the tasks with xTaskCreate
  BaseType_t result = xTaskCreate(
      position_task,      // (en) Task function
      "position",         // (en) Name of the task
      DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words).
                          // Generally 1024 or 2048
      NULL,               // (en) Arguments of the task
      1,                  // (en) Priority of this task
      &positionTask);     // (en) A handle to control the execution of this task

  if (result != pdPASS) {
    usb_serial_print("Example Task creation failed\n");
    return 0;
  }

  result =
      xTaskCreate(sensor_task, "sensor", DEFAULT_STACK_SIZE, NULL, 2, &sensorTask);

/*
  result =
      xTaskCreate(motion_task, "motion", DEFAULT_STACK_SIZE, NULL, 2, &motionTask);
*/

// These have to be right before the scheduler.
  tusb_init();
  usb_serial_init();
  // Start the scheduler (never returns)
  vTaskStartScheduler();

  // Never reach this line.
  return 0;
}
