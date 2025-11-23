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
#include "osal/osal_freertos.h"
#include "portable.h"
#include "portmacro.h"
#include "projdefs.h"
#include "tkj_utils/melody.h"
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
#define DEBUG_RX 0
#define DEBUG_BUZZER 0

#define DEBUG_BUF_SIZE 256
static char debug_buf[DEBUG_BUF_SIZE];

// *********** GLOBALS  *************

#define DEFAULT_STACK_SIZE 2048 // Can be reduced to 1024 if using less memory.
#define CDC_ITF_TX 1            // Channel to serial-client
#define DELAY_50_MS 50

// Message buffers etc.
#define MSG_BUF_SIZE 256
#define MSG_ACK ".- -.-. -.-  \n"
#define MSG_ACK_LEN 14
static char _msg_buf[MSG_BUF_SIZE];
static char rx_buf[MSG_BUF_SIZE];
static msg_builder_t msg_b;
static size_t rx_buf_len = 0;

// IMU and gestures
#define EXP_MOV_AVG_ALPHA 0.25
#define MOTION_BUF_SIZE 128
#define GESTURE_COOLDOWN_DELAY 5
static motion_data_t motion_data;

// Buzzer stuff
#define BUZ_MELODY_TEMPO 160
#define BUZ_SEND_TEMPO 260
#define BUZ_GEST_FREQ_LOW 314
#define BUZ_GEST_FREQ_HIGH 420
#define BUZ_GEST_LEN_DOT 25
#define BUZ_GEST_LEN_DASH 150

// Display
#define DISPLAY_EOM_X0 0
#define DISPLAY_EOM_Y0 24

// State
enum comms_state { RX_STANDBY_STATE, RX_PROCESSING_STATE, RX_DISPLAY_STATE };
static uint8_t dev_comms_state = RX_STANDBY_STATE;
static uint8_t gesture_state = STATE_COOLDOWN;

// Output queue for handling received messages.
#define QUEUE_SIZE 3
static QueueHandle_t output_queue = NULL;

// Task handles
static TaskHandle_t rx_dispatch_handle =
    NULL; // For notifying the rx_dispatcher

// Mutexes
static SemaphoreHandle_t I2C_mutex; // A mutex to guard the I2C bus

// *********** Function prototypes ************************

// Tasks
static void sensor_task(void *arg);
static void usb_task(void *arg);
static void gesture_task(void *arg);
static void rx_output_task(void *arg);
static void rx_dispatch_task(void *arg);

// Helpers
static bool init_queues(void);
static void push_to_output_queue(const char *msg, const size_t msg_len);
static void notify_dispatch_task(void);
static void send_msg(void);
static void send_ack(void);
static void reset_rx_buf(void);
static void append_to_rx_buf(const uint8_t *buf, size_t len, bool *eom);
static void refresh_display(void);
static void display_eom(void);
static void wait_for_usb_conn(void);
static void handle_gesture_input(size_t *cooldown_delay);

// Debug prints etc.
static void debug_print_tx(const int res);
static void debug_print_msg_builder(const int gst);
static void debug_print_gst_state(const int gst);
static void debug_print_rx(const uint8_t *buf, const char *rx_buf);

// *********** FREERTOS TASKS  ****************************

// Activates the TinyUSB library.
static void usb_task(void *arg) {
  (void)arg;
  while (1) {
    tud_task(); // With FreeRTOS wait for events
                // Do not add vTaskDelay.
  }
}

// Reads the IMU sensor into the motion_data_t struct.
static void sensor_task(void *arg) {
  (void)arg;
  char buf[MOTION_BUF_SIZE];

  if (DEBUG_IMU && usb_serial_connected()) {
    usb_serial_print("Sensor Task started...\r\n");
    usb_serial_flush();
  }

  motion_data.error = 0;
  if (DEBUG_IMU) {
    usb_serial_print(IMU_FIELD_NAMES);
  }

  while (1) {

    // Blocks until the I2C mutex is acquired.
    xSemaphoreTake(I2C_mutex, portMAX_DELAY);
    read_filtered_motion_data(&motion_data, EXP_MOV_AVG_ALPHA);
    xSemaphoreGive(I2C_mutex);

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

  wait_for_usb_conn();

  if (DEBUG_GESTURE_STATE && usb_serial_connected()) {
    usb_serial_print("Gesture Task started...\r\n");
    usb_serial_flush();
  }

  size_t cooldown_delay = GESTURE_COOLDOWN_DELAY;

  for (;;) {

    // This stops gesture input while displaying.
    while (dev_comms_state != RX_STANDBY_STATE) {
      vTaskDelay(pdMS_TO_TICKS(10));
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
      // These are somewhat redundant, BUT the separation could be useful later
      // on if more specific actions are eventually needed for different
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
        buzzer_play_tone(BUZ_GEST_FREQ_LOW, BUZ_GEST_LEN_DOT);
        handle_gesture_input(&cooldown_delay);
      }
      break;

    case GESTURE_DASH:
      if (gesture_state == STATE_READY) {
        gst_read = 1;
        msg_write(&msg_b, DASH);
        buzzer_play_tone(BUZ_GEST_FREQ_LOW, BUZ_GEST_LEN_DASH);
        handle_gesture_input(&cooldown_delay);
      }
      break;

    case GESTURE_SPACE:
      if (gesture_state == STATE_READY) {
        gst_read = 1;
        msg_write(&msg_b, SPACE);
        buzzer_play_tone(BUZ_GEST_FREQ_HIGH, BUZ_GEST_LEN_DOT);
        handle_gesture_input(&cooldown_delay);
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

        handle_gesture_input(&cooldown_delay);
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

// A consumer of received messages.
// Handles output with buzzer and the display.
static void rx_output_task(void *arg) {
  (void)arg;
  wait_for_usb_conn();

  if (DEBUG_BUZZER && usb_serial_connected()) {
    usb_serial_print("Output Task started...\r\n");
    usb_serial_flush();
  }

  // Play the lil startup jingle
  play_melody(&the_lick, BUZ_MELODY_TEMPO);

  vTaskDelay(pdMS_TO_TICKS(600));
  refresh_display();

  char *msg;

  for (;;) {
    if (xQueueReceive(output_queue, &msg, portMAX_DELAY) == pdTRUE) {
      dev_comms_state = RX_DISPLAY_STATE;
      if (DEBUG_RX) {
        usb_serial_print("--> RX_DISPLAY_STATE\r\n");
        usb_serial_flush();
      }

      char ascii[MSG_BUF_SIZE];
      decode_morse_msg(msg, ascii, MSG_BUF_SIZE);

      // Mutex guarded I2C use.
      xSemaphoreTake(I2C_mutex, portMAX_DELAY);
      write_text_multirow(ascii);
      xSemaphoreGive(I2C_mutex);

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

      display_eom();
      vTaskDelay(pdMS_TO_TICKS(300));

      refresh_display();
      dev_comms_state = RX_STANDBY_STATE;
      if (DEBUG_RX) {
        usb_serial_print("--> RX_STANDBY_STATE\r\n");
        usb_serial_flush();
      }
    }
  }
}

// Dispatcher for processing received messages.
// Grabs the contents of the RX buffer and delegates it forwards.
// This exists mostly to keep the cdc callback interrupt short and simple.
static void rx_dispatch_task(void *arg) {
  (void)arg;

  wait_for_usb_conn();

  if (DEBUG_RX && usb_serial_connected()) {
    usb_serial_print("RX Task started...\r\n");
    usb_serial_flush();
  }

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Sleep until notice.

    // Send an ACK to the serial-client.
    send_ack();
    dev_comms_state = RX_PROCESSING_STATE; // Guard the RX buffer
    if (DEBUG_RX) {
      usb_serial_print("--> RX_DISPLAY_STATE\r\n");
    }

    if (rx_buf_len < 1) {
      // Somebody, put nothing, somebody put nothing in my queue.
      reset_rx_buf();
      dev_comms_state = RX_STANDBY_STATE; // Let others use rx_buf.
      if (DEBUG_RX) {
        usb_serial_print("--> STANDBY_STATE\r\n");
      }
      continue;
    }

    if (DEBUG_RX) {
      usb_serial_print("RX Task processing a msg.\r\n");
      usb_serial_flush();
    }

    push_to_output_queue(rx_buf, rx_buf_len);

    reset_rx_buf();
  }
}

// ************************ CALLBACKS *************************************''

// The TinuUSB library declares the callback function without defining?
// We just define it here and the tud_task handles the rest, I guess.
void tud_cdc_rx_cb(uint8_t itf) {
  // allocate buffer for the data in the stack
  uint8_t read_buf[CFG_TUD_CDC_RX_BUFSIZE + 1];

  // | IMPORTANT: also do this for CDC0 because otherwise
  // | you won't be able to print anymore to CDC0
  // | next time this function is called

  // Don't mess with buffer if a complete message is being processed.
  if (dev_comms_state == RX_PROCESSING_STATE) {
    return;
  }

  size_t count = tud_cdc_n_read(itf, read_buf, sizeof(read_buf));
  read_buf[count] = '\0';

  // Don't process if received on the second cdc interface.
  if (itf != 1) {
    return;
  }

  bool end_of_msg = false;
  append_to_rx_buf(read_buf, count, &end_of_msg);

  if (DEBUG_RX) {
    debug_print_rx(read_buf, rx_buf);
  }

  // Unifinished message. Process no further.
  if (!end_of_msg) {
    return;
  }

  notify_dispatch_task();
}

// ************* HELPER FUNCTIONS ******************

// Waits until an usb connection is established.
void wait_for_usb_conn(void) {
  while (!tud_mounted() || !tud_cdc_n_connected(1)) {
    vTaskDelay(pdMS_TO_TICKS(DELAY_50_MS));
  }
}

// Call this after writing the gesture to the message builder. Handles the
// cooldown, state, led, display, refresh etc. required after most gestures.
void handle_gesture_input(size_t *cooldown_delay) {
  *cooldown_delay = GESTURE_COOLDOWN_DELAY;
  gesture_state = STATE_COOLDOWN;
  rgb_led_write(0, 0, 2);
  refresh_display();
}

// Resets the RX buffer.
void reset_rx_buf(void) {
  rx_buf[0] = '\0';
  rx_buf_len = 0;
}

// Wakes up the dispatch task to process a message in the RX buffer.
static void notify_dispatch_task(void) {
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(rx_dispatch_handle, &pxHigherPriorityTaskWoken);
  if (pxHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
  }
}

// Sends the current message to the serial-client if it's ready.
void send_msg(void) {
  static const size_t EOM = 3;
  if (msg_b.msg_len > EOM) { // Don't send empties.
    tud_cdc_n_write(CDC_ITF_TX, (uint8_t const *)msg_b.msg_buf, msg_b.msg_len);
    tud_cdc_n_write_flush(CDC_ITF_TX);
    play_melody(&victory_theme, BUZ_SEND_TEMPO);
  }
  else {
    buzzer_play_tone(BUZ_GEST_FREQ_HIGH, BUZ_GEST_LEN_DASH);
    buzzer_play_tone(BUZ_GEST_FREQ_LOW, BUZ_GEST_LEN_DASH);
  }
}

// Inits the output queue.
bool init_queues(void) {
  output_queue = xQueueCreate(QUEUE_SIZE, sizeof(char *));

  return output_queue != NULL;
}

// Push the message to the output queue for further processing.
// Expecting only human input from the serial-client so the allocations
// shouldn't get out of hands.
// Allocations are also limited by the memory available in the queue.
void push_to_output_queue(const char *msg, const size_t msg_len) {
  if (uxQueueSpacesAvailable(output_queue)) {
    char *msg_ptr = pvPortMalloc(msg_len * sizeof(char));

    if (msg_ptr) {
      memcpy(msg_ptr, msg, msg_len);
      xQueueSendToBack(output_queue, &msg_ptr, portMAX_DELAY);
      if (DEBUG_RX) {
        usb_serial_print("Pushed msg to RX queue\r\n");
      }
    } else { // We give up!
      if (DEBUG_RX) {
        usb_serial_print("Couldn't allocate memory for received message!\n");
      }
    }
    usb_serial_flush();
  }
}

// Sends an ACK message to the serial-client.
void send_ack(void) {
  tud_cdc_n_write(CDC_ITF_TX, (const uint8_t *)MSG_ACK, MSG_ACK_LEN);
  tud_cdc_n_write_flush(CDC_ITF_TX);
}

// Helper function to update the display during inputting message.
void refresh_display(void) {
  xSemaphoreTake(I2C_mutex, portMAX_DELAY);
  write_message_builder(msg_b.inp_buf, msg_b.msg_buf, msg_b.inp);
  xSemaphoreGive(I2C_mutex);
}

// Displays "End of message"
void display_eom(void) {
  xSemaphoreTake(I2C_mutex, portMAX_DELAY);
  clear_display();
  write_text_xy(DISPLAY_EOM_X0, DISPLAY_EOM_Y0, "End of message");
  xSemaphoreGive(I2C_mutex);
}

// Appends bytes read from USB to the RX buffer.
// Sets the end of message flag when required.
void append_to_rx_buf(const uint8_t *buf, size_t len, bool *eom) {
  for (size_t idx = 0; idx < len; idx++) {
    if (rx_buf_len == MSG_BUF_SIZE - 1) {
      rx_buf[rx_buf_len] = '\0'; // Truncate
      continue;
    }
    if (rx_buf_len >= MSG_BUF_SIZE) {
      continue; // Discard overflow until end of message.
    }
    rx_buf[rx_buf_len++] = buf[idx];
    if (buf[idx] == '\n') {
      *eom = true;
      rx_buf[rx_buf_len++] = '\0';
    }
  }
}

// ****************** DEBUGGING UTILITIES ****************
// Extracted from the task handlers to keep them cleaner.

void debug_print_tx(const int res) {
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

void debug_print_gst_state(const int gst) {
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

void debug_print_rx(const uint8_t *buf, const char *rx_buf) {
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
  // init_i2c_default();
  // sleep_ms(1000); // Wait some time so initialization of USB and hat is done.
  // init_red_led();

  if (init_ICM42670() == 0) {
    usb_serial_print("ICM-42670P initialized successfully!\r\n");
    ICM42670_start_with_default_values(); // TODO: Handle error values?
  } else {
    usb_serial_print("Failed to initialize ICM-42670P.\r\n");
  }

  // Init the pretty colourful LED and the buzzer
  init_rgb_led();
  rgb_led_write(2, 0, 0);
  sleep_ms(500);
  init_buzzer();

  init_display();
  clear_display();
  write_text_xy(0, 24, "Starting up...");

  // Initializations for data structures.
  msg_init(&msg_b, _msg_buf, MSG_BUF_SIZE);
  if (!init_queues()) {
    usb_serial_print("Failed to initieate RX Queue.");
  }

  // Initialize mutexes
  I2C_mutex = xSemaphoreCreateMutex();
  if (I2C_mutex == NULL) {
    usb_serial_print("Failed to create I2C_mutex\r\n");
  }

  TaskHandle_t gesture, hUSB, sensor, buzzer = NULL;

  xTaskCreate(usb_task, "usb", 2048, NULL, 3, &hUSB);
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

  if (xTaskCreate(sensor_task, "sensor", DEFAULT_STACK_SIZE, NULL, 2,
                  &sensor) != pdPASS) {
    usb_serial_print("Sensor Task creation failed\r\n");
    return 0;
  }

  result = xTaskCreate(rx_dispatch_task, "rx_dispatch", DEFAULT_STACK_SIZE,
                       NULL, 2, &rx_dispatch_handle);
  if (result != pdPASS) {
    usb_serial_print("RX Dispatcher Task creation failed\r\n");
    return 0;
  }

  if (xTaskCreate(rx_output_task, "buzzer", DEFAULT_STACK_SIZE, NULL, 3,
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
