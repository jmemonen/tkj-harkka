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
#include "portable.h"
#include "portmacro.h"
#include "projdefs.h"
#include "sensors/sensors.h"
#include "tkjhat/sdk.h"
#include "usbSerialDebug/helper.h"

#include "morso/morso.h"

// *************** DEBUG **********************************
#define DEBUG_IMU 0
#define DEBUG_GESTURE_STATE 0
#define DEBUG_MSG_BUILDER 0
#define DEBUG_RX 1

#define DEBUG_BUF_SIZE 256
static char debug_buf[DEBUG_BUF_SIZE];

// *********** GLOBAL VARIABLES AND CONSTANTS *************

#define DEFAULT_STACK_SIZE 2048 // Can be reduced to 1024 if using less memory.
#define CDC_ITF_TX 1
#define MOTION_BUF_SIZE 128
#define EXP_MOV_AVG_ALPHA 0.25
#define GESTURE_COOLDOWN_DELAY 5
#define MSG_BUF_SIZE 256

// Sensor data
static motion_data_t motion_data;

// TX and RX buffers and such.
static char _msg_buf[MSG_BUF_SIZE];
static msg_builder_t msg_b;

// Device state
enum comms_state { STANDBY_STATE, RX_STATE };
static uint8_t dev_comms_state = STANDBY_STATE;
static uint8_t gesture_state = STATE_COOLDOWN;

// RX queue for handling received messages.
// Needs some stuff to keep memory allocations static.
#define QUEUE_SIZE 3
static QueueHandle_t rx_queue;
// static uint8_t _rx_queue_storage_buf[MSG_BUF_SIZE * QUEUE_SIZE];
// static StaticQueue_t *_rx_queue_buf;

// *********** Function prototypes ************************

void send_msg(void);
void debug_print_tx(int res);
void debug_print_msg_builder(int gst);
void debug_print_gst_state(int gst);
void debug_print_rx(uint8_t *buf, char *rx_buf);
QueueHandle_t init_rx_queue(void);

// *********** FREERTOS TASKS  ****************************

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

  if (DEBUG_IMU && usb_serial_connected()) {
    usb_serial_print("sensorTask started...\r\n");
    usb_serial_flush();
  }

  motion_data.error = 0;
  if (DEBUG_IMU) {
    usb_serial_print(IMU_FIELD_NAMES);
  }

  while (1) {
    read_filtered_motion_data(&motion_data, EXP_MOV_AVG_ALPHA);
    if (DEBUG_IMU && motion_data.error) {
      usb_serial_print("There was an error reading motion data!\r\n");
      usb_serial_flush();
      motion_data.error = 0;
      continue;
    }

    if (DEBUG_IMU) {
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

  if (DEBUG_GESTURE_STATE && usb_serial_connected()) {
    usb_serial_print("gesture_task started...\r\n");
    usb_serial_flush();
  }

  size_t cooldown_delay = GESTURE_COOLDOWN_DELAY;

  for (;;) {

    // We'll eventually display received messages somehow.
    // This stops gesture input while displaying.
    if (dev_comms_state == RX_STATE) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    Gesture_t gst = detect_gesture(&motion_data);
    uint8_t gst_read = 0;
    char c = (char)MORSO_INVALID_INPUT;

    // TODO: Is this a bit crude? Could just compare timestamps?
    // Then again, this could be used to accumulate extra cooldown
    // for detecting prolonged shaking gestures?
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
        gst_read = 1;
        gesture_state = STATE_READY;
        rgb_led_write(0, 2, 0);
      }
      break;

    case GESTURE_DOT:
      if (gesture_state == STATE_READY) {
        gst_read = 1;
        msg_write(&msg_b, DOT);
        cooldown_delay = GESTURE_COOLDOWN_DELAY;
        gesture_state = STATE_COOLDOWN;
        rgb_led_write(0, 0, 2);
      }
      break;

    case GESTURE_DASH:
      if (gesture_state == STATE_READY) {
        gst_read = 1;
        msg_write(&msg_b, DASH);
        cooldown_delay = GESTURE_COOLDOWN_DELAY;
        gesture_state = STATE_COOLDOWN;
        rgb_led_write(0, 0, 2);
      }
      break;

    case GESTURE_SPACE:
      if (gesture_state == STATE_READY) {
        gst_read = 1;
        msg_write(&msg_b, SPACE);
        cooldown_delay = GESTURE_COOLDOWN_DELAY;
        gesture_state = STATE_COOLDOWN;
        rgb_led_write(0, 0, 2);
      }
      break;

    case GESTURE_SEND:
      // TODO: Should eventually put the device into some kind of a send state?
      // Do we need a state for it?
      if (gesture_state == STATE_READY) {
        gst_read = 1;
        int res = msg_ready(&msg_b);
        if (DEBUG_MSG_BUILDER) {
          debug_print_tx(res);
        }

        send_msg();
        msg_reset(&msg_b);
        gesture_state = STATE_COOLDOWN;
        cooldown_delay = GESTURE_COOLDOWN_DELAY;
        rgb_led_write(0, 0, 2);
      }
      break;

    default:
      break;
    }

    if (DEBUG_GESTURE_STATE && gst_read) {
      debug_print_gst_state(gst);
    }
    if (DEBUG_MSG_BUILDER && gst_read) {
      debug_print_msg_builder(gst);
    }

    usb_serial_flush();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ************************ CALLBACKS *************************************''

// The TinuUSB library declares the callback function without defining?
// We just define it here and the tud_task handles the rest, I guess.
// This is pretty much straight outta the example.
void tud_cdc_rx_cb(uint8_t itf) {
  static char rx_buf[MSG_BUF_SIZE];
  static size_t rx_len;

  // allocate buffer for the data in the stack
  uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE + 1];

  // read the available data
  // | IMPORTANT: also do this for CDC0 because otherwise
  // | you won't be able to print anymore to CDC0
  // | next time this function is called

  uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));
  buf[count] = '\0';

  // Don't process if received on the second cdc interface.
  if (itf != 1) {
    return;
  }

  bool eom = false;
  for (size_t idx = 0; idx < count; idx++) {
    if (rx_len == MSG_BUF_SIZE - 1) {
      rx_buf[rx_len] = '\0'; // Truncate
      continue;
    }
    if (rx_len >= MSG_BUF_SIZE) {
      continue; // Discard overflow until end of message.
    }
    rx_buf[rx_len++] = buf[idx];
    if (buf[idx] == '\n') {
      eom = true;
      rx_buf[rx_len++] = '\0';
    }
  }

  if (DEBUG_RX) {
    debug_print_rx(buf, rx_buf);
  }

  // Unifinished message. Process no further.
  if (!eom) {
    return;
  }

  // Push finished msg to the queue if there's room.
  if (uxQueueSpacesAvailable(rx_queue)) {
    // Expecting only human input so the allocations shouldn't get out of hands.
    // They are also limited by the memory available in the queue.
    char *msg_ptr = pvPortMalloc(rx_len);
    if (msg_ptr) {
      memcpy(msg_ptr, rx_buf, rx_len);
      xQueueSendToBack(rx_queue, msg_ptr, 5);
      tud_cdc_n_write(itf, (uint8_t const *)"Message received!\n", 19);
    } else { // We give up!
      tud_cdc_n_write(itf, (uint8_t const *)"Error receiving message\n", 25);
      if (DEBUG_RX) {
        usb_serial_print("Couldn't allocate memory for received message!\n");
      }
    }
    tud_cdc_n_write_flush(itf);
    usb_serial_flush();
  }

  // Done. Reset buffer.
  rx_buf[0] = '\0';
  rx_len = 0;
}

// ************* HELPER FUNCTIONS ******************

void send_msg(void) {
  static const size_t EOM = 3;
  if (msg_b.msg_len > EOM) { // Don't send empties.
    tud_cdc_n_write(CDC_ITF_TX, (uint8_t const *)msg_b.msg_buf, msg_b.msg_len);
    tud_cdc_n_write_flush(CDC_ITF_TX);
  }
}

QueueHandle_t init_rx_queue(void) {
  // Dynamic allocations, but that's how the settings were.
  // Gets freed when you pull the plug...
  return rx_queue = xQueueCreate(QUEUE_SIZE, sizeof(char *));
}

// ****************** DEBUGGING UTILITIES **********

void debug_print_tx(int res) {
  if (res == MORSO_OK) {
    snprintf(debug_buf, DEBUG_BUF_SIZE, "Sent msg: %s\r", msg_b.msg_buf);
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

void debug_print_msg_builder(int gst) {
  if (gst != GESTURE_READY) {
    snprintf(debug_buf, DEBUG_BUF_SIZE, "Msg:%s | Inp:%s\r\n", msg_b.msg_buf,
             msg_b.inp_buf);
    usb_serial_print(debug_buf);
    usb_serial_flush();
  }
}

void debug_print_gst_state(int gst) {
  char *s;
  switch (gst) {
  case GESTURE_DOT:
    s = "DOT\r\n";
    break;
  case GESTURE_DASH:
    s = "DASH\r\n";
    break;
  case GESTURE_SPACE:
    s = "SPACE\r\n";
    break;
  case GESTURE_READY:
    s = "READY\r\n";
    break;
  case GESTURE_SEND:
    s = "SEND MSG\r\n";
    break;
  default:
    s = "HUHHUH?\r\n";
    break;
  }
  usb_serial_print(s);
  usb_serial_flush();
}

void debug_print_rx(uint8_t *buf, char *rx_buf) {
    usb_serial_print("\r\nReceived on CDC 1:");
    usb_serial_print((char *)buf);
    usb_serial_print("\r\nrx_buf: ");
    usb_serial_print(rx_buf);
    usb_serial_flush();

}

// *********************** MAIN ********************

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

  // Initializations for data structures.
  msg_init(&msg_b, _msg_buf, MSG_BUF_SIZE);
  if (!init_rx_queue()) {
    usb_serial_print("Failed to initieate RX Queue.");
  }

  TaskHandle_t gesture, hUSB, sensor, rx = NULL;

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

  // TODO: Clean these away into a function...
  if (result != pdPASS) {
    usb_serial_print("Position Task creation failed\n");
    return 0;
  }

  if (xTaskCreate(sensorTask, "sensor", DEFAULT_STACK_SIZE, NULL, 2, &sensor) !=
      pdPASS) {
    usb_serial_print("Sensor Task creation failed\n");
    return 0;
  }

  if (xTaskCreate(sensorTask, "rx", DEFAULT_STACK_SIZE, NULL, 2, &rx) !=
      pdPASS) {
    usb_serial_print("RX Task creation failed\n");
    return 0;
  }

  // These have to be right before the scheduler.
  tusb_init();
  usb_serial_init();
  // Start the scheduler (never returns)
  vTaskStartScheduler();

  // Never reach this line. Sad.
  return 0;
}
