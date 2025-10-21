
#include <pico/types.h>
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <tusb.h>

#include "portmacro.h"
#include "tkjhat/sdk.h"
#include "usbSerialDebug/helper.h"
#include "sensors/sensors.h"

// Default stack size for the tasks. It can be reduced to 1024 if task is not
// using lot of memory.
#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX 1

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
  while (1) {
    int sensorReading = hello_sensors();
    char s[6];
    snprintf(s, 6, "%d\n", sensorReading);
    usb_serial_print(s);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Add here necessary states
enum state { IDLE = 1 };
enum state programState = IDLE;

static void example_task(void *arg) {
  (void)arg;

  for (;;) {
    tight_loop_contents(); // Modify with application code here.
    usb_serial_print("TESTI");
    tud_cdc_n_write(CDC_ITF_TX, (uint_fast8_t const *)"TESTI\n", 7);
    tud_cdc_n_write_flush(CDC_ITF_TX);
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

  result = xTaskCreate(sensorTask, "sensor", DEFAULT_STACK_SIZE, NULL, 2, &sensor);

  tusb_init();
  usb_serial_init();

  // Start the scheduler (never returns)
  vTaskStartScheduler();

  // Never reach this line.
  return 0;
}
