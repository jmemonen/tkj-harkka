#include <FreeRTOS.h>
#include <task.h>
#include "tkjhat/sdk.h"
#include "tkj_utils/message_output.h"

#define MORSE_CODE_UNIT       80 // Morse code unit lenght
#define DOT_BUZZ_DURATION     (MORSE_CODE_UNIT * 1)
#define DASH_BUZZ_DURATION    (MORSE_CODE_UNIT * 3)
#define INTRA_CHAR_PAUSE      (MORSE_CODE_UNIT * 1) // Pause duration between dots/dashes inside a character
#define INTER_CHAR_PAUSE      (MORSE_CODE_UNIT * 3) // Pause duration between characters
#define WORD_PAUSE            (MORSE_CODE_UNIT * 7) // Pause duration between words
#define MORSE_TONE_FREQUENCY  600

void buzzer_play_message(char *msg) {
  while (*msg) {
    if (*msg == '.') {
      buzzer_play_tone(MORSE_TONE_FREQUENCY, DOT_BUZZ_DURATION);
      vTaskDelay(pdMS_TO_TICKS(INTRA_CHAR_PAUSE));
      msg++;
    }
    else if (*msg == '-') {
      buzzer_play_tone(MORSE_TONE_FREQUENCY, DASH_BUZZ_DURATION);
      vTaskDelay(pdMS_TO_TICKS(INTRA_CHAR_PAUSE));
      msg++;
    }
    else if (*msg == ' ') {
      int space_count = 0;
      const char *p = msg;
      while (*p == ' ') {
        space_count++;
        p++;
      }

      if (space_count == 1) {
        // Character pause
        vTaskDelay(pdMS_TO_TICKS(INTER_CHAR_PAUSE - INTRA_CHAR_PAUSE));
      }
      else if (space_count == 2) {
        // Word pause
        vTaskDelay(pdMS_TO_TICKS(WORD_PAUSE - INTRA_CHAR_PAUSE));
      }
      else if (space_count == 3) {
        // Message ends
        break;
      }

      msg = (char *)p;
    }
    else {
      msg++;
    }
  }
}

void display_morse_message(char *msg) {
  clear_display();
  int display_x = 0;
  int display_y = 0;

  while (*msg) {
    if (*msg == '.') {
      write_text_xy(display_x, display_y, ".");
      vTaskDelay(pdMS_TO_TICKS(DOT_BUZZ_DURATION + INTRA_CHAR_PAUSE));
      
      display_x += 6;
      msg++;
    }
    else if (*msg == '-') {
      write_text_xy(display_x, display_y, "-");
      vTaskDelay(pdMS_TO_TICKS(DASH_BUZZ_DURATION + INTRA_CHAR_PAUSE));

      display_x += 6;
      msg++;
    }
    else if (*msg == ' ') {
      int space_count = 0;
      const char *p = msg;
      while (*p == ' ') {
        space_count++;
        p++;
      }

      if (space_count == 1) {
        // Character pause
        vTaskDelay(pdMS_TO_TICKS(INTER_CHAR_PAUSE));
        if (display_x > 100) {
          display_x = 0;
          display_y += 20;
        }
        else {
          display_x += 10;
        }
      }
      else if (space_count == 2) {
        // Word pause

        vTaskDelay(pdMS_TO_TICKS(WORD_PAUSE));
        clear_display();
        display_x = 0;
        display_y = 0;
      }
      else if (space_count == 3) {
        // Message ends
        clear_display();
        write_text_xy(7, 28, "Message has ended");
        break;
      }

      msg = (char *)p;
    }
    else {
      msg++;
    }
  }
}
