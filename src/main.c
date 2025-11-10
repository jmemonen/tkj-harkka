#include <pico/time.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <tusb.h>

#include "class/cdc/cdc_device.h"
#include "portmacro.h"
#include "projdefs.h"
#include "sensors/sensors.h"
#include "tkjhat/sdk.h"
#include "usbSerialDebug/helper.h"

#include "morso/morso.h"

// Flags for toggling debug prints.
#define DEBUG_IMU_VALUES 0
#define DEBUG_GESTURE_STATE 0
#define DEBUG_MSG_BUILDER 1

// Default stack size for the tasks. It can be reduced to 1024 if task is not
// using lot of memory.
#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX 1
#define MOTION_BUF_SIZE 128
#define EXP_MOV_AVG_ALPHA 0.25
#define GESTURE_COOLDOWN_DELAY 10
#define MSG_BUILDER_BUF_SIZE 256

static motion_data_t motion_data;
static uint8_t gesture_state = STATE_COOLDOWN;
static char _msg_buf[MSG_BUILDER_BUF_SIZE];
static msg_builder_t msg_b;

void send_msg(void);

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

static void gesture_task(void *arg) {
  (void)arg;

  while (!tud_mounted() || !tud_cdc_n_connected(1)) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (usb_serial_connected()) {
    usb_serial_print("gesture started...\r\n");
    usb_serial_flush();
  }

  size_t cooldown_delay = GESTURE_COOLDOWN_DELAY;

  for (;;) {

    Gesture_t gst = detect_gesture(&motion_data);
    uint8_t gst_read = 0;
    char c = (char)MORSO_INVALID_INPUT;
    const size_t DEBUG_BUF_SIZE = 256;
    char debug_buf[256];

    // TODO: Is this a bit crude? Could just compare timestamps?
    if (cooldown_delay) {
      // Cooldown only in neutral position.
      if (gst == GESTURE_READY) {
        --cooldown_delay;
      } else {
        cooldown_delay = GESTURE_COOLDOWN_DELAY;
      }
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    switch (gst) {
      // TODO: These are kinda redundant cases, BUT the separation could be
      // useful later on if more specific actions are needed on different
      // gestures?
    case GESTURE_READY:
      if (gesture_state == STATE_COOLDOWN) {
        gesture_state = STATE_READY;
        gst_read = 1;
        rgb_led_write(0, 2, 0);
      }
      break;

    case GESTURE_DOT:
      if (gesture_state == STATE_READY) {
        msg_write(&msg_b, DOT);
        gst_read = 1;
        cooldown_delay = GESTURE_COOLDOWN_DELAY;
        gesture_state = STATE_COOLDOWN;
        rgb_led_write(0, 0, 2);
      }
      break;

    case GESTURE_DASH:
      if (gesture_state == STATE_READY) {
        msg_write(&msg_b, DASH);
        gst_read = 1;
        cooldown_delay = GESTURE_COOLDOWN_DELAY;
        gesture_state = STATE_COOLDOWN;
        rgb_led_write(0, 0, 2);
      }
      break;

    case GESTURE_SPACE:
      if (gesture_state == STATE_READY) {
        msg_write(&msg_b, SPACE);
        gst_read = 1;
        cooldown_delay = GESTURE_COOLDOWN_DELAY;
        gesture_state = STATE_COOLDOWN;
        rgb_led_write(0, 0, 2);
      }
      break;

    case GESTURE_SEND:
      // TODO: Should eventually put the device into some kind of a send state?
      if (gesture_state == STATE_READY) {
        int res = msg_ready(&msg_b);

        if (DEBUG_MSG_BUILDER) {
          if (res == MORSO_OK) {
            snprintf(debug_buf, DEBUG_BUF_SIZE, "Sent msg: %s\r",
                     msg_b.msg_buf);
            usb_serial_print(debug_buf);
            usb_serial_flush();
            decode_morse_msg(msg_b.msg_buf, debug_buf, DEBUG_BUF_SIZE);
            usb_serial_print(debug_buf);
            usb_serial_flush();
            usb_serial_print("\r\n");
          } else {
            usb_serial_print("Failed to send message");
          }
          usb_serial_flush();
        }

        send_msg();

        msg_reset(&msg_b);
        gesture_state = STATE_COOLDOWN;
        gst_read = 1;
        cooldown_delay = GESTURE_COOLDOWN_DELAY;
        rgb_led_write(0, 0, 2);
      }
      break;

    default:
      break;
    }

    // DEBUG: Print gesture state.
    if (DEBUG_GESTURE_STATE && gst_read) {
      switch (gst) {
      case GESTURE_DOT:
        usb_serial_print("DOT\r\n");
        usb_serial_flush();
        break;
      case GESTURE_DASH:
        usb_serial_print("DASH\r\n");
        usb_serial_flush();
        break;
      case GESTURE_SPACE:
        usb_serial_print("SPACE\r\n");
        usb_serial_flush();
        break;
      case GESTURE_READY:
        usb_serial_print("READY\r\n");
        usb_serial_flush();
        break;
      case GESTURE_SEND:
        usb_serial_print("SEND MSG!\r\n");
        usb_serial_flush();
        break;
      default:
        break;
      }
    }

    // DEBUG: Print msg builder state.
    if (DEBUG_MSG_BUILDER && gst_read) {
      if (gst != GESTURE_READY) {
        snprintf(debug_buf, DEBUG_BUF_SIZE, "Msg:%s | Inp:%s\r\n",
                 msg_b.msg_buf, msg_b.inp_buf);
        usb_serial_print(debug_buf);
        usb_serial_flush();
      }
    }

    usb_serial_flush();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void send_msg(void) {
  if (msg_b.msg_len) { // Don't send empties.
    tud_cdc_n_write(CDC_ITF_TX, (uint8_t const *)msg_b.msg_buf, msg_b.msg_len);
    tud_cdc_n_write_flush(CDC_ITF_TX);
  }
}

int main() {
  // stdio_init_all();

  // Uncomment this lines if you want to wait till the serial monitor is
  // connected. Messes it up if using the tusb and usb_serial libraries?
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

  // Init the pretty colourful LED
  init_rgb_led();
  sleep_ms(500);
  rgb_led_write(0, 0, 0);

  // Init msg builder/buffer
  msg_init(&msg_b, _msg_buf, MSG_BUILDER_BUF_SIZE);

  TaskHandle_t gesture, hUSB, sensor = NULL;

  xTaskCreate(usbTask, "usb", 2048, NULL, 3, &hUSB);
#if (configNUMBER_OF_CORES > 1)
  vTaskCoreAffinitySet(hUSB, 1u << 0);
#endif

  // Create the tasks with xTaskCreate
  BaseType_t result = xTaskCreate(
      gesture_task,       // (en) Task function
      "gesture",          // (en) Name of the task
      DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words).
                          // Generally 1024 or 2048
      NULL,               // (en) Arguments of the task
      1,                  // (en) Priority of this task
      &gesture);          // (en) A handle to control the execution of this task

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

  // Never reach this line. Sad.
  return 0;
}
