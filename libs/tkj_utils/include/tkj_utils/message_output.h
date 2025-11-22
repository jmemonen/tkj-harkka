#pragma once

// Plays the given morse-formatted message with the buzzer
void buzzer_play_message(char *msg);

// Plays the given morse-formatted message with the buzzer
// and writes morse-symbols to OLED display
// NOTE: This turned out to be too slow
void display_morse_message(char *msg);

// Show given message on the OLED display, with text wrapping!
void display_message_slow(char *msg);
