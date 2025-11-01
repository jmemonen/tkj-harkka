#include "morso/morso.h"
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHAR_AMOUNT 26
#define TO_ASCII_DIFF 32
#define TO_IDX_DIFF 65
#define MORSE_TREE_SIZE 29
#define MAX_MORSE_SYMBOL_LEN 4
#define DOT '.'
#define DASH '-'
#define SPACE ' '

#define DOT_IDX(idx) ((2) * (idx)) + (1)
#define DASH_IDX(idx) ((2) * (idx)) + (2)

// TODO: Add support for numbers?
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

// A lookup binary tree represented by an array beginning with index = 0.
// Dot means the left child: idx * 2 + 1.
// Dash means the right child: idx * 2 + 2.
static const char morse_tree[MORSE_TREE_SIZE] = {(char)MORSO_INVALID_INPUT,
                                                 'e',
                                                 't',
                                                 'i',
                                                 'a',
                                                 'n',
                                                 'm',
                                                 's',
                                                 'u',
                                                 'r',
                                                 'w',
                                                 'd',
                                                 'k',
                                                 'g',
                                                 'o',
                                                 'h',
                                                 'v',
                                                 'f',
                                                 (char)MORSO_INVALID_INPUT,
                                                 'l',
                                                 (char)MORSO_INVALID_INPUT,
                                                 'p',
                                                 'j',
                                                 'b',
                                                 'x',
                                                 'c',
                                                 'y',
                                                 'z',
                                                 'q'};

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

// Parses a morse symbol string using a binary tree. BLAZINGLY fast enough!
char morse_to_char(const char *str) {
  if (str == NULL) {
    return (char)MORSO_NULL_INPUT;
  }

  size_t len = 0;
  size_t idx = 0;

  while (*str && len <= MAX_MORSE_SYMBOL_LEN) {
    if (*str != DOT && *str != DASH) {
      return (char)MORSO_INVALID_INPUT;
    }
    idx = (*str == DOT) ? DOT_IDX(idx) : DASH_IDX(idx);
    str++;
    len++;
  }

  if (idx >= MORSE_TREE_SIZE) {
    return (char)MORSO_INVALID_INPUT;
  }

  return morse_tree[idx];
}

// TODO: Could be nicer...
int encode_morse_msg(const char *str, char *buf, size_t buf_size) {
  char *end = buf + buf_size - 1;
  uint8_t needs_space = 0;

  if (str == NULL || buf == NULL) {
    return MORSO_NULL_INPUT;
  }
  if (buf_size < 1) {
    return MORSO_BUF_OVERFLOW;
  }

  while (*str) {
    char c = *str++;
    if (buf > end) {
      return MORSO_BUF_OVERFLOW;
    }

    // Space in input
    if (c == SPACE) {
      *buf++ = SPACE;
      continue;
    }

    // Space to separate morse symbols.
    if (needs_space) {
      *buf++ = SPACE;
      needs_space = 0;
    }

    const char *morse = char_to_morse(c);
    if (!morse) {
      return MORSO_INVALID_INPUT;
    }
    while (*morse) {
      if (buf > end) {
        return MORSO_BUF_OVERFLOW;
      }
      *buf++ = *morse++;
    }
    needs_space = 1;
  }
  *buf = '\0';

  return MORSO_OK;
}
