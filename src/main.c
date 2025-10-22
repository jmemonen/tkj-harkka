#include <pico/time.h>
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
// #include "usbSerialDebug/helper.h"

// Default stack size for the tasks. It can be reduced to 1024 if task is not
// using lot of memory.
#define DEFAULT_STACK_SIZE 2048
// #define CDC_ITF_TX 1
// #define MOTION_BUF_SIZE 128

static motion_data_t motion_data;

static void imu_task(void *arg) {
  (void)arg;
  
  printf("Initializing ICM-42670P...\n");
  //Initialize IMU
  if (init_ICM42670() == 0) {
    printf("ICM-42670P initialized successfully!\n");
    printf("Starting accelometer and gyroscope with default values...\n");
    ICM42670_start_with_default_values();
    /*
    if (ICM42670_startAccel(ICM42670_ACCEL_ODR_DEFAULT, ICM42670_ACCEL_FSR_DEFAULT) != 0){
      printf("Wrong values to init the accelerometer in ICM-42670P.\n");
    }
    if (ICM42670_startGyro(ICM42670_GYRO_ODR_DEFAULT, ICM42670_GYRO_FSR_DEFAULT) != 0){
      printf("Wrong values to init the gyroscope in ICM-42670P.\n");
    };
    ICM42670_enable_accel_gyro_ln_mode();
    */

  } else {
      printf("Failed to initialize ICM-42670P.\n");
  }
  float ax, ay, az, gx, gy, gz, t;

  while(1) {
    ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t);

    printf("Accel: X=%.2f Y=%.2f Z=%.2f | Gyro: X=%.2f Y=%.2f Z=%.2f | Temp: %.2f C\n", ax, ay, az, gx, gy, gz, t);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

}

// Add here necessary states
enum state { IDLE = 1 };
enum state programState = IDLE;

/*
static void example_task(void *arg) {
  (void)arg;

  for (;;) {
    // tight_loop_contents(); // Modify with application code here.
    // usb_serial_print("TESTI");
    // tud_cdc_n_write(CDC_ITF_TX, (uint_fast8_t const *)"TESTI\n", 7);
    // tud_cdc_n_write_flush(CDC_ITF_TX);
    // usb_serial_print("Hello example task!");
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
} */

int main() {
  stdio_init_all();
  // Uncomment this lines if you want to wait till the serial monitor is
  // connected
  while (!stdio_usb_connected()){
      sleep_ms(10);
  }

  init_hat_sdk();
  sleep_ms(1000); // Wait some time so initialization of USB and hat is done.
  init_i2c_default();
  sleep_ms(1000); // Wait some time so initialization of USB and hat is done.

  TaskHandle_t imuTaskHandle = NULL;

  // Create the tasks with xTaskCreate
  BaseType_t result = xTaskCreate(
      imu_task,           // (en) Task function
      "imu",              // (en) Name of the task
      DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words).
                          // Generally 1024 or 2048
      NULL,               // (en) Arguments of the task
      2,                  // (en) Priority of this task
      &imuTaskHandle);    // (en) A handle to control the execution of this task

  if (result != pdPASS) {
    printf("IMU Task creation failed\n");
    return 0;
  }

  // Start the scheduler (never returns)
  vTaskStartScheduler();



  // Never reach this line.
  return 0;
}
