#include <FreeRTOS.h>
#include <task.h>
#include "tkjhat/sdk.h"
#include <tkj_utils/melody.h>

note_data_t major_c_scale_notes[] = {
  { NOTE_C4, NOTE_QUARTER },
  { NOTE_D4, NOTE_QUARTER },
  { NOTE_E4, NOTE_QUARTER },
  { NOTE_F4, NOTE_QUARTER },
  { NOTE_G4, NOTE_QUARTER },
  { NOTE_A4, NOTE_QUARTER },
  { NOTE_B4, NOTE_QUARTER },
  { NOTE_C5, NOTE_HALF },
};

melody_t major_c_scale = {
  .notes = major_c_scale_notes,
  .length = sizeof(major_c_scale_notes) / sizeof(note_data_t)
};

note_data_t the_lick_notes[] = {
  { NOTE_D4, NOTE_EIGHT },
  { NOTE_E4, NOTE_EIGHT },
  { NOTE_F4, NOTE_EIGHT },
  { NOTE_G4, NOTE_EIGHT },
  { NOTE_E4, NOTE_QUARTER },
  { NOTE_C4, NOTE_EIGHT },
  { NOTE_D4, NOTE_QUARTER + NOTE_HALF },
  { NOTE_REST, NOTE_EIGHT + NOTE_QUARTER}
};

melody_t the_lick = {
  .notes = the_lick_notes,
  .length = sizeof(the_lick_notes) / sizeof(note_data_t)
};

void play_melody(const melody_t *melody, float tempo) {
  float eight_note_length_ms = (60000.0f / tempo) * 0.5f;

  for (size_t i = 0; i < melody->length; i++) {
    uint32_t freq = melody->notes[i].note_freq;
    uint32_t duration_ms = melody->notes[i].note_length * eight_note_length_ms;

    if (freq == 0) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }
    else {
      buzzer_play_tone(freq, duration_ms);
    }
  }
}