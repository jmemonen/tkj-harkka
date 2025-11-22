#include <pico/time.h>
#include <pico/types.h>
#include <stdbool.h>
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
#include "tkj_utils/message_output.h"
#include "tkj_utils/tkj_utils.h"
#include "tkjhat/sdk.h"
#include "usbSerialDebug/helper.h"

#include "morso/morso.h"

// *************** DEBUG **********************************
// Set to 1 to print debug info of different components.
#define DEBUG_IMU 0
#define DEBUG_GESTURE_STATE 0
#define DEBUG_MSG_BUILDER 0
#define DEBUG_RX 1
#define DEBUG_BUZZER 1

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
static char rx_buf[MSG_BUF_SIZE];
static size_t rx_buf_len = 0;

// Device state
enum comms_state { STANDBY_STATE, RX_DISPLAY_STATE };
static uint8_t dev_comms_state = STANDBY_STATE;
static uint8_t gesture_state = STATE_COOLDOWN;

// RX queue for handling received messages.
#define QUEUE_SIZE 3
static QueueHandle_t rx_queue = NULL;
static QueueHandle_t buzzer_queue = NULL;

// Task handles
static TaskHandle_t rx_task_handle = NULL;

// *********** Function prototypes ************************

void send_msg(void);
void display_msg(const char *msg);
void debug_print_tx(int res);
void debug_print_msg_builder(int gst);
void debug_print_gst_state(int gst);
void debug_print_rx(uint8_t *buf, char *rx_buf);
void push_to_buzzer_queue(const char *msg, size_t msg_len);
void reset_rx_buf(void);
bool init_queues(void);

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
    if (dev_comms_state == RX_DISPLAY_STATE) {
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
      //       Do we need a state for it?
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

// Handles the buzzer.
// Consumes and frees messages from the buzzer_queue.
static void buzzer_task(void *arg) {
  (void)arg;
  while (!tud_mounted() || !tud_cdc_n_connected(1)) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (DEBUG_BUZZER && usb_serial_connected()) {
    usb_serial_print("Buzzer Task started...\r\n");
    usb_serial_flush();
  }
  char *msg;

  for (;;) {
    if (xQueueReceive(buzzer_queue, &msg, portMAX_DELAY) == pdTRUE) {
      if (DEBUG_BUZZER) {
        usb_serial_print("Buzzer playing a msg from queue.\r\n");
        usb_serial_print(msg);
        usb_serial_print("\r\n");
        usb_serial_flush();
      }
      buzzer_play_message(msg);
      if (DEBUG_BUZZER) {
        usb_serial_print("Buzzer done playing. Freed msg*\r\n");
      }
      vPortFree(msg);
    }
  }
}

// A dispatcher task for received complete messages.
static void rx_dispatch_task(void *arg) {
  (void)arg;

  while (!tud_mounted() || !tud_cdc_n_connected(1)) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (DEBUG_RX && usb_serial_connected()) {
    usb_serial_print("RX Task started...\r\n");
    usb_serial_flush();
  }

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Blocked until notice.
    dev_comms_state =
        RX_DISPLAY_STATE; // Block others from using I2C and stuff.
    if (DEBUG_RX) {
      usb_serial_print("--> RX_DISPLAY_STATE\r\n");
    }

    tud_cdc_n_write(CDC_ITF_TX, (uint8_t const *)"Message received!\n", 19);
    tud_cdc_write_flush();

    if (rx_buf_len < 1) {
      // Somebody, put nothing, somebody put nothing in my queue.
      reset_rx_buf();
      continue;
    }

    if (DEBUG_RX) {
      usb_serial_print("RX Task processing a msg.\r\n");
      usb_serial_flush();
    }

    // Split the message for the buzzer and the display.
    push_to_buzzer_queue(rx_buf, rx_buf_len);
    char msg[MSG_BUF_SIZE];
    decode_morse_msg(rx_buf, msg, MSG_BUF_SIZE);
    usb_serial_flush();

    reset_rx_buf();

    display_msg(msg);                // TODO: Implement this!
    dev_comms_state = STANDBY_STATE; // Back to business as usual.
    if (DEBUG_RX) {
      usb_serial_print("--> STANDBY_STATE\r\n");
    }
  }
}

// ************************ CALLBACKS *************************************''

// The TinuUSB library declares the callback function without defining?
// We just define it here and the tud_task handles the rest, I guess.
// This is pretty much straight outta the example.
void tud_cdc_rx_cb(uint8_t itf) {
  // allocate buffer for the data in the stack
  uint8_t read_buf[CFG_TUD_CDC_RX_BUFSIZE + 1];

  // read the available data
  // | IMPORTANT: also do this for CDC0 because otherwise
  // | you won't be able to print anymore to CDC0
  // | next time this function is called

  uint32_t count = tud_cdc_n_read(itf, read_buf, sizeof(read_buf));
  read_buf[count] = '\0';

  // Don't process if received on the second cdc interface.
  if (itf != 1) {
    return;
  }

  // Don't mess with buffer if a complete message is being processed.
  if (dev_comms_state == RX_DISPLAY_STATE) {
    return;
  }

  bool end_of_msg = false;
  for (size_t idx = 0; idx < count; idx++) {
    if (rx_buf_len == MSG_BUF_SIZE - 1) {
      rx_buf[rx_buf_len] = '\0'; // Truncate
      continue;
    }
    if (rx_buf_len >= MSG_BUF_SIZE) {
      continue; // Discard overflow until end of message.
    }
    rx_buf[rx_buf_len++] = read_buf[idx];
    if (read_buf[idx] == '\n') {
      end_of_msg = true;
      rx_buf[rx_buf_len++] = '\0';
    }
  }

  if (DEBUG_RX) {
    debug_print_rx(read_buf, rx_buf);
  }

  // Unifinished message. Process no further.
  if (!end_of_msg) {
    return;
  }

  // Notify the dispatcher task a complete message has been received.
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(rx_task_handle, &pxHigherPriorityTaskWoken);
  if (pxHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
  }
}

// ************* HELPER FUNCTIONS ******************
//
void reset_rx_buf(void) {
  rx_buf[0] = '\0';
  rx_buf_len = 0;
}

// Sends the current message to the serial-client if it's ready.
void send_msg(void) {
  static const size_t EOM = 3;
  if (msg_b.msg_len > EOM) { // Don't send empties.
    tud_cdc_n_write(CDC_ITF_TX, (uint8_t const *)msg_b.msg_buf, msg_b.msg_len);
    tud_cdc_n_write_flush(CDC_ITF_TX);
  }
}

// Could actually do more than just display it e.g. buzz the buzzer.
// Could be renamed more relevantly in that case.
void display_msg(const char *msg) {

  // ===== Your code goes here! ========
  usb_serial_print("Seasons greetings from display_msg!\r\n");
  // ===================================
}

bool init_queues(void) {
  // Dynamic allocations, but that's how the settings were.
  // Gets freed when you pull the plug...
  rx_queue = xQueueCreate(QUEUE_SIZE, sizeof(char *));
  buzzer_queue = xQueueCreate(QUEUE_SIZE, sizeof(char *));

  return rx_queue && buzzer_queue;
}

// Push the message to the rx_queue for further processing.
// Sends an ack to the sender (which should be a separate thing tbh).
void push_to_buzzer_queue(const char *msg, size_t msg_len) {
  if (DEBUG_BUZZER) {
    usb_serial_print("@ push_to_buzzer_queue\r\n");
    usb_serial_print(msg);
    usb_serial_print("\r\n");
    usb_serial_flush();
  }

  if (uxQueueSpacesAvailable(rx_queue)) {
    // Expecting only human input so the allocations shouldn't get out of hands.
    // They are also limited by the memory available in the queue.
    char *msg_ptr = pvPortMalloc(msg_len * sizeof(char));

    if (msg_ptr) {
      memcpy(msg_ptr, msg, msg_len);

      if (DEBUG_BUZZER) {
        usb_serial_print("msg_ptr is now:\r\n");
        usb_serial_print(msg_ptr);
        usb_serial_print("\r\n");
        usb_serial_flush();
      }

      xQueueSendToBack(buzzer_queue, &msg_ptr, 0);

      if (DEBUG_RX) {
        usb_serial_print("Pushed msg to buzzer queue\r\n");
      }
    } else { // We give up!
      if (DEBUG_RX) {
        usb_serial_print("Couldn't allocate memory for received message!\n");
      }
    }
    usb_serial_flush();
  }
}

// ****************** DEBUGGING UTILITIES **********
// Extracted from the task handlers to keep them clean.

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
  usb_serial_print("\r\n");
  usb_serial_print("rx_buf: ");
  usb_serial_print(rx_buf);
  usb_serial_print("\r\n");
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

  // Init the pretty colourful LED and the buzzer
  init_rgb_led();
  sleep_ms(500);
  rgb_led_write(0, 0, 0);
  init_buzzer();

  // Initializations for data structures.
  msg_init(&msg_b, _msg_buf, MSG_BUF_SIZE);
  if (!init_queues()) {
    usb_serial_print("Failed to initieate RX Queue.");
  }

  TaskHandle_t gesture, hUSB, sensor, buzzer = NULL;

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
    usb_serial_print("Position Task creation failed\r\n");
    return 0;
  }

  if (xTaskCreate(sensorTask, "sensor", DEFAULT_STACK_SIZE, NULL, 2, &sensor) !=
      pdPASS) {
    usb_serial_print("Sensor Task creation failed\r\n");
    return 0;
  }

  result = xTaskCreate(rx_dispatch_task, "rx_dispatch", DEFAULT_STACK_SIZE,
                       NULL, 2, &rx_task_handle);
  if (result != pdPASS) {
    usb_serial_print("RX Dispatcher Task creation failed\r\n");
    return 0;
  }

  if (xTaskCreate(buzzer_task, "buzzer", DEFAULT_STACK_SIZE, NULL, 3,
                  &buzzer)) {
    usb_serial_print("Buzzer Task creation failed\r\n");
  }

  // These have to be right before the scheduler.
  tusb_init();
  usb_serial_init();
  // Start the scheduler (never returns)
  vTaskStartScheduler();

  // Never reach this line. Sad.
  return 0;
}
