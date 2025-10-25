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
#define POSITION_BUF_SIZE 128
#define EXP_MOV_AVG_ALPHA 0.25

static motion_data_t motion_data;
static uint8_t position_state = DASH_STATE;
static float d_pos = 0.0f;

// Activates the TinyUSB library.
static void usbTask(void *arg) {
  (void)arg;
  while (1) {
    tud_task(); // With FreeRTOS wait for events
                // Do not add vTaskDelay.
  }
}

// Reads the sensors.
static void sensorTask(void *arg) {
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

    // Prints for dev and debug
    // format_motion_csv(&motion_data, buf, MOTION_BUF_SIZE);
    // usb_serial_print(buf);
    // usb_serial_flush();

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void position_task(void *arg) {
  (void)arg;
  char buf[POSITION_BUF_SIZE];

  while (!tud_mounted() || !tud_cdc_n_connected(1)) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (usb_serial_connected()) {
    usb_serial_print("position_task started...\r\n");
    usb_serial_flush();
  }

  for (;;) {
    update_da(&motion_data);
    // snprintf(buf, POSITION_BUF_SIZE,
    //          "total_accel: %03.3f | delta_a: %03.3f\r\n", motion_data.a_total,
    //          motion_data.da);
    // usb_serial_print(buf);
    uint8_t new_position = get_position(&motion_data);
    if (new_position != position_state) {
      position_state = new_position;
      switch (position_state) {
      case MOVING:
        usb_serial_print("State: MOVING\r\n");
        break;
      case DOT_STATE:
        usb_serial_print("State: DOT\r\n");
        break;
      case DASH_STATE:
        usb_serial_print("State: DASH\r\n");
        break;
      case WHITESPACE_STATE:
        usb_serial_print("State: WHITESPACE\r\n");
        break;
      }
    }
    usb_serial_flush();

    vTaskDelay(pdMS_TO_TICKS(100));
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

  TaskHandle_t positionTask, hUSB, sensor = NULL;

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
      xTaskCreate(sensorTask, "sensor", DEFAULT_STACK_SIZE, NULL, 2, &sensor);

  // These have to be right before the scheduler.
  tusb_init();
  usb_serial_init();
  // Start the scheduler (never returns)
  vTaskStartScheduler();

  // Never reach this line.
  return 0;
}
