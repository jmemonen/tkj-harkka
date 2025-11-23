#pragma once

// ----------------------
// NOTE DEFINITIONS
// ----------------------
#define NOTE_REST   0
#define NOTE_C4     262
#define NOTE_CS4    277
#define NOTE_D4     294
#define NOTE_DS4    311
#define NOTE_E4     330
#define NOTE_F4     349
#define NOTE_FS4    370
#define NOTE_G4     392
#define NOTE_GS4    415
#define NOTE_A4     440
#define NOTE_AS4    466
#define NOTE_B4     494
#define NOTE_C5     523
#define NOTE_CS5    554
#define NOTE_D5     587
#define NOTE_DS5    622
#define NOTE_E5     659
#define NOTE_F5     698
#define NOTE_FS5    740
#define NOTE_G5     784
#define NOTE_GS5    831
#define NOTE_A5     880
#define NOTE_AS5    932
#define NOTE_B5     988
#define NOTE_C6     1047

// ----------------------
// NOTE LENGTHS
// ----------------------
#define NOTE_EIGHT    1
#define NOTE_QUARTER  2
#define NOTE_HALF     4
#define NOTE_WHOLE    8

#define NOTE_GAP      0
#define GAP_DURATION  10 // ms

// ----------------------
// STRUCTS
// ----------------------
typedef struct {
    uint32_t note_freq;
    uint32_t note_length;
} note_data_t;

typedef struct {
  const note_data_t *notes;
  size_t length;
} melody_t;

// extern so these can be used in main.c
// while calling the function to play a melody.
extern note_data_t major_c_scale_notes[];
extern melody_t major_c_scale;

extern note_data_t the_lick_notes[];
extern melody_t the_lick;

extern note_data_t victory_theme_notes[];
extern melody_t victory_theme;

// Plays given melody
void play_melody(const melody_t *melody, float tempo);