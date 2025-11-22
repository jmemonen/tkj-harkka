#pragma once

// Plays the given morse-formatted message with the buzzer
void buzzer_play_message(char *msg);

// Plays the given morse-formatted message with the buzzer
// and writes morse-symbols to OLED display
void display_morse_message(char *msg);
