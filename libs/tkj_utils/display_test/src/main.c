#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>

#include <tkjhat/sdk.h>
#include <tkj_utils/message_output.h>



// Display task
static void display_task(void *arg) {
  (void)arg;
  
  char msg[] = "- . .-. ...- .  .--- .-  -.- .. .. - --- ...  -.- .- .-.. --- .. ... - .-   ";
  // char msg[] = "... - .-   ";
  // char viesti[] = "Terve ja kiitos kaloista!";

  for (;;) {
    display_morse_message(msg);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

int main() {
  stdio_init_all();
  init_hat_sdk();
  init_buzzer();
  init_display();

  clear_display();

  sleep_ms(3000);
  
  TaskHandle_t displayTask = NULL;

  BaseType_t result =
      xTaskCreate(display_task, "buzzer", 2048, NULL, 2, &displayTask);

  vTaskStartScheduler();

  return 0;
}
