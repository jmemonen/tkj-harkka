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
#define MSG_RIGHT_PADDING 3
#define END_OF_MSG " \n"

// Macros
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

  while (*str && len < _MORSO_MAX_SYMBOL_LEN) {
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

// It is what it is. Now uses the message builder thing.
int encode_morse_msg(const char *msg, char *buf, size_t buf_size) {
  if (!msg || !buf) {
    return MORSO_NULL_INPUT;
  }

  msg_builder_t b;

  int res = msg_init(&b, buf, buf_size);
  if (res != MORSO_OK) {
    return res;
  }

  while (*msg) {
    char c = *msg++;

    switch (c) {

    case SPACE:
      if ((res = msg_write(&b, SPACE)) != MORSO_OK) {
        return MORSO_BUF_OVERFLOW;
      }
      break;

    default: {
      const char *morse = char_to_morse(c);
      if (!morse) {
        return MORSO_INVALID_INPUT;
      }
      while (*morse) {
        if ((res = msg_write(&b, *morse++)) != MORSO_OK) {
          return res;
        }
      }
      if ((res = msg_write(&b, SPACE)) != MORSO_OK) {
        return res;
      }
      break;
    }
    }
  }

  return msg_ready(&b);
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

    if (len > _MORSO_MAX_SYMBOL_LEN) {
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

int msg_reset_inp(msg_builder_t *b) {
  if (!b) {
    return MORSO_NULL_INPUT;
  }
  b->inp_buf[0] = '\0';
  b->inp_len = 0;
  b->inp = MORSO_INVALID_INPUT;

  return MORSO_OK;
}

int msg_reset(msg_builder_t *b) {
  if (!b || !b->msg_buf) {
    return MORSO_NULL_INPUT;
  }
  b->msg_buf[0] = '\0';
  b->msg_len = 0;
  b->ready_to_send = false;
  b->word_boundary = false;

  return msg_reset_inp(b);
}

int msg_init(msg_builder_t *b, char *msg_buf, size_t buf_size) {
  if (!msg_buf) {
    return MORSO_NULL_INPUT;
  }
  b->msg_buf = msg_buf;

  int64_t max_len = buf_size - MSG_RIGHT_PADDING;
  if (max_len <= 0) {
    return MORSO_BUF_OVERFLOW;
  }
  b->msg_max_len = max_len;

  msg_reset(b);

  return MORSO_OK;
}

int msg_write(msg_builder_t *b, char c) {
  if (!b) {
    return MORSO_NULL_INPUT;
  }

  if (b->ready_to_send) {
    // Don't write to a finalized msg.
    return MORSO_MSG_READY;
  }

  // Handle msg buffer being full.
  if (b->msg_len > b->msg_max_len) {
    // Truncate the msg with '\0'.
    b->msg_buf[b->msg_len] = '\0'; // TODO: Add end of message sequence?
    return MORSO_BUF_OVERFLOW;
  }

  switch (c) {
  case DOT:
  case DASH:
    // printf("    Adding DOT/DASH when inp_len: %d \n", b->inp_len);
    if (b->inp_len >= _MORSO_MAX_SYMBOL_LEN) {
      msg_reset_inp(b);
      return MORSO_INVALID_INPUT;
    }
    b->inp_buf[b->inp_len++] = c;
    b->inp_buf[b->inp_len] = '\0';
    b->inp = morse_to_char(b->inp_buf);
    b->word_boundary = false;
    return MORSO_OK;

  case SPACE:
    if (b->inp_len == 0) {
      if (b->word_boundary) {
        return MORSO_OK; // Don't write extra spaces.
      }
      b->msg_buf[b->msg_len++] = SPACE; // Write a word boundary.
      b->word_boundary = true;
      return MORSO_OK;
    }

    // Handle invalid morse symbol in input.
    if (b->inp == MORSO_INVALID_INPUT) {
      msg_reset_inp(b);
      return MORSO_INVALID_INPUT;
    }

    // Here the msg_max_len + 1 offsets the max len to account for the
    // space that always follows the last morse symbol.
    // So we need only 3 bytes to properly end the message.
    if (b->msg_len + b->inp_len + 1 > b->msg_max_len + 1) {
      // Not enough memory in the msg buffer.
      b->msg_buf[b->msg_len] = '\0';
      msg_reset_inp(b);
      return MORSO_BUF_OVERFLOW;
    }
    for (size_t idx = 0; idx < b->inp_len; idx++) {
      b->msg_buf[b->msg_len++] = b->inp_buf[idx];
    }
    b->msg_buf[b->msg_len++] = ' ';
    b->msg_buf[b->msg_len] = '\0';
    msg_reset_inp(b);
    return MORSO_OK;

  default:
    // Reset the inp buffer and return an error code.
    msg_reset_inp(b);
    return MORSO_INVALID_INPUT;
  }
}

int msg_ready(msg_builder_t *b) {
  if (!b) {
    return MORSO_NULL_INPUT;
  }

  // TODO: Might be a redundant check, if the write function
  // already enforces proper message length.
  if (b->msg_len > b->msg_max_len) {
    return MORSO_BUF_OVERFLOW;
  }

  const char *eom = END_OF_MSG;
  if (b->word_boundary) {
    eom++; // Skip a space if a double space already exists.
  }
  size_t debug_idx = 0;
  while (*eom) {
    // printf("   @msg_ready, i:%d\n", debug_idx++);
    b->msg_buf[b->msg_len++] = *eom++;
  }
  b->msg_buf[b->msg_len] = '\0';

  b->ready_to_send = true;
  return MORSO_OK;
}
