#include <stddef.h>
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

#include "morso/morso.h"

// Flags for toggling debug prints.
#define DEBUG_IMU_VALUES 0
#define DEBUG_GESTURE_STATE 1
#define DEBUG_MSG_BUILDER 0

// Default stack size for the tasks. It can be reduced to 1024 if task is not
// using lot of memory.
#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX 1
#define MOTION_BUF_SIZE 128
#define EXP_MOV_AVG_ALPHA 0.25
#define GESTURE_COOLDOWN_DELAY 20
#define MSG_BUILDER_BUF_SIZE 256

static motion_data_t motion_data;
static uint8_t gesture_state = STATE_COOLDOWN;
static msg_builder_t msg_b;

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

    // ******* Prints for dev and debug ********
    if (DEBUG_IMU_VALUES) {
      format_motion_csv(&motion_data, buf, MOTION_BUF_SIZE);
      usb_serial_print(buf);
      usb_serial_flush();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
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

  size_t cooldown_delay = GESTURE_COOLDOWN_DELAY;

  for (;;) {

    Gesture_t gst = detect_gesture(&motion_data);
    uint8_t gst_read = 0;

    // TODO: Is this a bit crude? Could just compare timestamps?
    if (cooldown_delay) {
      if (gst == GESTURE_READY) {
        --cooldown_delay;
      }
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    switch (gst) {

    case GESTURE_READY:
      if (gesture_state == STATE_COOLDOWN) {
        gesture_state = STATE_READY;
        gst_read = 1;
      }
      break;

    case GESTURE_DOT:
    case GESTURE_DASH:
      if (gesture_state == STATE_READY) {
        gesture_state = STATE_COOLDOWN;
        gst_read = 1;
        cooldown_delay = GESTURE_COOLDOWN_DELAY;
      }
      break;

    case GESTURE_SPACE:
      if (gesture_state == STATE_READY) {
        gesture_state = STATE_COOLDOWN;
        gst_read = 1;
        cooldown_delay = GESTURE_COOLDOWN_DELAY;
      }
      break;

    case GESTURE_SEND:
      // Handle sending the msg.
      // Should eventually put the device into some kind of a send state.
      if (gesture_state == STATE_READY) {
        gesture_state = STATE_COOLDOWN;
        gst_read = 1;
        cooldown_delay = GESTURE_COOLDOWN_DELAY;
      }
      break;

    default:
      break;
    }

    if (DEBUG_GESTURE_STATE && gst_read) {
      switch (gst) {
      case GESTURE_DOT:
        usb_serial_print("DOT\r\n");
        break;
      case GESTURE_DASH:
        usb_serial_print("DASH\r\n");
        break;
      case GESTURE_SPACE:
        usb_serial_print("SPACE\r\n");
        break;
      case GESTURE_READY:
        usb_serial_print("READY\r\n");
        break;
      case GESTURE_SEND:
        usb_serial_print("SEND MSG!\r\n");
        break;
      default:
        break;
      }
      gst_read = 0;
    }

    usb_serial_flush();
    vTaskDelay(pdMS_TO_TICKS(5));
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

  if (init_ICM42670() == 0) {
    usb_serial_print("ICM-42670P initialized successfully!\r\n");
    ICM42670_start_with_default_values(); // TODO: Handle error values?
  } else {
    usb_serial_print("Failed to initialize ICM-42670P.\r\n");
  }

  // Init msg builder/buffer
  char msg_buf[MSG_BUILDER_BUF_SIZE];
  msg_init(&msg_b, msg_buf, MSG_BUILDER_BUF_SIZE);

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
    usb_serial_print("Positoin Task creation failed\n");
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
