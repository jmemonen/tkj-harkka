#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>

#include <tkjhat/sdk.h>
#include <tkj_utils/message_output.h>
#include <tkj_utils/melody.h>

// Buzzer task
static void buzzer_task(void *arg) {
  (void)arg;
  
  play_melody(&the_lick, 160.0f);
  vTaskDelay(pdMS_TO_TICKS(500));
  
  char msg[] = "- . .-. ...- .  .--- .-  -.- .. .. - --- ...  -.- .- .-.. --- .. ... - .-   ";

  for (;;) {

    buzzer_play_message(msg);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

int main() {
  stdio_init_all();
  init_buzzer();

  sleep_ms(1000);

  TaskHandle_t buzzerTask = NULL;

  BaseType_t result =
      xTaskCreate(buzzer_task, "buzzer", 2048, NULL, 2, &buzzerTask);


  vTaskStartScheduler();

  return 0;
}
