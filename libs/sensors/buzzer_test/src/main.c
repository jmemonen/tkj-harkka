#include <stdio.h>
#include <tkjhat/sdk.h>
// #include <sensors/sensors.h>
#include <sensors/buzzer.h>


int main() {
  stdio_init_all();
  init_buzzer();

  char msg[] = "... --- ...  ... --- ...   ";

  sleep_ms(1000);

  while (true) {
    play_message(msg);
    sleep_ms(2000);
  }
  
  return 0;
}
