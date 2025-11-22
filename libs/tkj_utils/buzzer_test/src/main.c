#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>

#include <tkjhat/sdk.h>
<<<<<<< HEAD:libs/tkj_utils/buzzer_test/src/main.c
#include <tkj_utils/buzzer.h>
=======
#include <sensors/message_output.h>
>>>>>>> a83f94982b1c2d2c7e0e17a5a42b327058891da2:libs/sensors/buzzer_test/src/main.c



// Buzzer task
static void buzzer_task(void *arg) {
  (void)arg;
  
  char msg[] = "- . .-. ...- .  .--- .-  -.- .. .. - --- ...  -.- .- .-.. --- .. ... - .-   ";

  for (;;) {
    buzzer_play_message(msg);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

int main() {
  stdio_init_all();
  init_buzzer();

  sleep_ms(3000);
  
  TaskHandle_t buzzerTask = NULL;

  BaseType_t result =
      xTaskCreate(buzzer_task, "buzzer", 2048, NULL, 2, &buzzerTask);

  vTaskStartScheduler();

  return 0;
}
