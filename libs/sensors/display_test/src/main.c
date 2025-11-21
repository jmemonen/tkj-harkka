#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>

#include <tkjhat/sdk.h>
#include <sensors/message_output.h>



// Display task
static void display_task(void *arg) {
  (void)arg;
  
  char msg[] = "- . .-. ...- .  .--- .-  -.- .. .. - --- ...  -.- .- .-.. --- .. ... - .-   ";
  char viesti[] = "Terve ja kiitos kaloista!";

  for (;;) {
    clear_display();
    write_text_xy(0, 0, msg);
    vTaskDelay(pdMS_TO_TICKS(5000));
    clear_display();
    write_text_xy(0, 0, viesti);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

int main() {
  stdio_init_all();
  init_hat_sdk();
  init_display();

  clear_display();

  sleep_ms(1000);
  
  TaskHandle_t displayTask = NULL;

  BaseType_t result =
      xTaskCreate(display_task, "buzzer", 2048, NULL, 2, &displayTask);

  vTaskStartScheduler();

  return 0;
}
