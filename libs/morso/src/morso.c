#include "morso/morso.h"
#include <stddef.h>

#define CHAR_AMOUNT 26
#define TO_ASCII_DIFF 32
#define TO_IDX_DIFF 65

static const char *const morse_lookup[CHAR_AMOUNT] = {
    ".-",   // A
    "-...", // B
    "-.-.", // C
    "-..",  // D
    ".",    // E
    "..-.", // F
    "--.",  // G
    "....", // H
    "..",   // I
    ".---", // J
    "-.-",  // K
    ".-..", // L
    "--",   // M
    "-.",   // N
    "---",  // O
    ".--.", // P
    "--.-", // Q
    ".-.",  // R
    "...",  // S
    "-",    // T
    "..-",  // U
    "...-", // V
    ".--",  // W
    "-..-", // X
    "-.--", // Y
    "--.."  // Z
};

const char *char_to_morse(char c) {
  if (c >= 'a' && c <= 'z') {
    c -= TO_ASCII_DIFF;
  }
  if (c < 'A' || c > 'Z') {
    return NULL;
  }
  c -= TO_IDX_DIFF;
  return morse_lookup[c];
}

// TODO: WIP
int str_to_morse(const char *str, char *buf, size_t buf_size) {
  if (str == NULL || buf == NULL) {
    return MORSO_NULL_INPUT;
  }
  if (buf_size < 1) {
    return MORSO_BUF_OVERFLOW;
  }
  return -1;
}
