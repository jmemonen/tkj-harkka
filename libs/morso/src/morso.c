#include "morso/morso.h"
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Values and limits
#define CHAR_AMOUNT 26
#define TO_ASCII_DIFF 32
#define TO_IDX_DIFF 65
#define MORSE_TREE_SIZE 29
#define MAX_MORSE_SYMBOL_LEN 4
#define MSG_RIGHT_PADDING 4 // Enough memory for '   \0' to end the message.

// Symbols
#define DOT '.'
#define DASH '-'
#define SPACE ' '

// Macros
#define DOT_IDX(idx) ((2) * (idx)) + (1)
#define DASH_IDX(idx) ((2) * (idx)) + (2)

// A helper function to read the next morse symbol from a given string.
// This increments the str pointer as a side effect!
// (Basically a consuming verions of morse to char...)
// int get_morse_symbol(const char **str, char *buf, size_t buf_size);

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

// Looks up morse symbol strings from the morse_lookup array.
// The correct indices are calculated from ASCII values.
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

// It is what it is.
int encode_morse_msg(const char *msg, char *buf, size_t buf_size) {
  char *end = buf + buf_size - 1;
  uint8_t needs_space = 0;

  if (!msg || !buf) {
    *buf = '\0';
    return MORSO_NULL_INPUT;
  }
  if (buf_size < 1) {
    *buf = '\0';
    return MORSO_BUF_OVERFLOW;
  }

  while (*msg) {
    const char c = *msg++;

    if (buf > end) {
      *buf = '\0';
      return MORSO_BUF_OVERFLOW;
    }

    // Handle space in input.
    if (c == SPACE) {
      *buf++ = SPACE;
      continue;
    }

    // Space as a separator between symbols.
    if (needs_space) {
      *buf++ = SPACE;
      needs_space = 0;
    }

    // Copy the corresponding morse symbol string to buf.
    const char *morse = char_to_morse(c);
    if (!morse) {
      *buf = '\0';
      return MORSO_INVALID_INPUT;
    }
    while (*morse) {
      if (buf > end) {
        *buf = '\0';
        return MORSO_BUF_OVERFLOW;
      }
      *buf++ = *morse++;
    }
    needs_space = 1;
  }
  *buf = '\0';

  return MORSO_OK;
}

int decode_morse_msg(const char *msg, char *buf, size_t buf_size) {
  if (!msg || !buf) {
    return MORSO_NULL_INPUT;
  }

  if (buf_size < 1) {
    *buf = '\0';
    return MORSO_BUF_OVERFLOW;
  }

  char *end = buf + buf_size - 2; // Enough memory for the last char and \0
  size_t idx = 0;
  size_t len = 0;
  uint8_t needs_space = 0;

  while (*msg) {
    const char c = *msg++;

    if (buf > end) {
      *buf = '\0';
      return MORSO_BUF_OVERFLOW;
    }

    if (len > MAX_MORSE_SYMBOL_LEN) {
      return MORSO_INVALID_INPUT;
    }

    if (needs_space) {
      *buf++ = SPACE;
      needs_space = 0;
      continue;
    }

    // Handle a symbol separator. Emit the char and reset idx and len.
    if (c == SPACE) {

      // Traversal ended in invalid state.
      if (idx >= MORSE_TREE_SIZE) {
        *buf = '\0';
        return MORSO_INVALID_INPUT;
      }

      char c_out = morse_tree[idx];
      if (c_out == MORSO_INVALID_INPUT) {
        *buf = '\0';
        return MORSO_INVALID_INPUT;
      }
      *buf++ = c_out;
      idx = 0;
      len = 0;
      // Add a space to buf if it was a double space.
      needs_space = (*msg == SPACE) ? 1 : 0;
      continue;
    }

    // Handle invalid sybols in msg.
    if (c != DOT && c != DASH) {
      *buf = '\0';
      return MORSO_INVALID_INPUT;
    }

    // Traverse the tree based on the current dot or dash.
    idx = (c == DOT) ? DOT_IDX(idx) : DASH_IDX(idx);
    len++;
  }

  char c_out = morse_tree[idx];
  if (c_out == MORSO_INVALID_INPUT) {
    *buf = '\0';
    return MORSO_INVALID_INPUT;
  }
  *buf++ = c_out;
  *buf = '\0';

  return MORSO_OK;
}

static inline void reset_inp_buf(msg_builder_t *b) {
  b->inp_buf[0] = '\0';
  b->inp_len = 0;
}

// TODO: Kind of a WIP...
// It would kinda be easier to just store the msg as ASCII where every
// symbol is a single char? Easy to encode that into morse when needed.
int msg_write(msg_builder_t *b, char c) {
  if (!b) {
    return MORSO_NULL_INPUT;
  }

  // Handle any buffer being full.
  if (b->inp_len >= MAX_MORSE_SYMBOL_LEN ||
      b->msg_len > b->msg_size - MSG_RIGHT_PADDING) {
    // Truncate the msg with '\0'.
    b->msg[b->msg_len] = '\0';
    // Reset inp_buf.
    reset_inp_buf(b);
    return MORSO_BUF_OVERFLOW;
  }

  switch (c) {
  case DOT:
  case DASH:
    b->inp_buf[b->inp_len++] = c;
    b->inp_buf[b->inp_len] = '\0';
    b->inp = morse_to_char(b->inp_buf);
    return MORSO_OK;

  case SPACE:
    // No input
    if (b->inp_len == 0) {
      return MORSO_NULL_INPUT;
    }
    if (b->msg_len + b->inp_len + 1 >= b->msg_size) {
      // Not enough memory in the msg buffer.
      b->msg[b->msg_len] = '\0';
      reset_inp_buf(b);
      return MORSO_BUF_OVERFLOW;
    }
    for (size_t idx = 0; idx < b->inp_len; idx++) {
      b->msg[b->msg_len++] = b->inp_buf[idx];
    }
    b->msg[b->msg_len++] = ' ';
    b->msg[b->msg_len] = '\0';
    reset_inp_buf(b);
    return MORSO_OK;

  default:
    // Reset the inp buffer and return an error code.
    reset_inp_buf(b);
    return MORSO_INVALID_INPUT;
  }
}
