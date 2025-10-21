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

static motion_data_t motion_data;

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

  // init_ICM42670(); // TODO: check return value for errors...
  // ICM42670_start_with_default_values();

  while (!tud_mounted() || !tud_cdc_n_connected(1)) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (usb_serial_connected()) {
    usb_serial_print("sensorTask started...");
  }
  usb_serial_flush();

  motion_data.error = 0;

  while (1) {
    usb_serial_print("Hello from sensor task!");
    // int sensorReading = hello_sensors();
    // char s[6];
    // snprintf(s, 6, "%d\n", sensorReading);
    // usb_serial_print(s);
    // read_motion_data(&motion_data);
    // if (motion_data.error) {
    //   usb_serial_print("There was an error reading motion data!");
    // } else {
    //   usb_serial_print("No errors detected reading motion data.");
    // }
    // sprintf(buf, "test:%.3f\n", motion_data.ax);
    // usb_serial_print(buf);
    // snprintf(buf, MOTION_BUF_SIZE, "ax:%.3f\n", motion_data.ax);
    // format_motion_csv(&motion_data, buf, MOTION_BUF_SIZE);
    // usb_serial_print(buf);
    //
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Add here necessary states
enum state { IDLE = 1 };
enum state programState = IDLE;

static void example_task(void *arg) {
  (void)arg;

  for (;;) {
    // tight_loop_contents(); // Modify with application code here.
    // usb_serial_print("TESTI");
    // tud_cdc_n_write(CDC_ITF_TX, (uint_fast8_t const *)"TESTI\n", 7);
    // tud_cdc_n_write_flush(CDC_ITF_TX);
    usb_serial_print("Hello example task!");
    vTaskDelay(pdMS_TO_TICKS(2000));
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
  sleep_ms(300); // Wait some time so initialization of USB and hat is done.
  init_red_led();

  TaskHandle_t myExampleTask, hUSB, sensor = NULL;

  xTaskCreate(usbTask, "usb", 2048, NULL, 3, &hUSB);
#if (configNUMBER_OF_CORES > 1)
  vTaskCoreAffinitySet(hUSB, 1u << 0);
#endif

  // Create the tasks with xTaskCreate
  BaseType_t result = xTaskCreate(
      example_task,       // (en) Task function
      "example",          // (en) Name of the task
      DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words).
                          // Generally 1024 or 2048
      NULL,               // (en) Arguments of the task
      2,                  // (en) Priority of this task
      &myExampleTask);    // (en) A handle to control the execution of this task

  if (result != pdPASS) {
    usb_serial_print("Example Task creation failed\n");
    return 0;
  }

  result =
      xTaskCreate(sensorTask, "sensor", DEFAULT_STACK_SIZE, NULL, 2, &sensor);

  if (result != pdPASS) {
    usb_serial_print("Sensor Task creation failed\n");
    blink_led(6);
    return 0;
  }

  // Apparently these should be right before the scheduler...
  tusb_init();
  usb_serial_init();
  // Start the scheduler (never returns)
  vTaskStartScheduler();

  // Never reach this line.
  return 0;
}
