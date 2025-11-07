#pragma once

#include <stddef.h>

enum morso_status {
  MORSO_OK,
  MORSO_INVALID_INPUT,
  MORSO_BUF_OVERFLOW,
  MORSO_NULL_INPUT
};

// morso_msg_t
// A convenient message buffer for building morse code strings.
typedef struct {
  char *msg;
  size_t msg_size;
  size_t msg_len;

  char inp_buf[5]; //TODO: magic number...
  size_t inp_len;
  char inp;
} msg_builder_t;

// Write a dot, dash or space to a msg_builder_t.
int msg_write(msg_builder_t *b, char c);

// Encodes a alphabetic message as morse code into the given buffer.
// So far supports only alphabetical ASCII strings with aA-zZ
// where words are separated by spaces eg. "my cool message".
//
// Returns a morso_status code:
// MORSO_OK: Ok
// MORSO_INVALID_INPUT: Invalid symbols in input string.
// MORSO_BUF_OVERFLOW: Not enough memory in buffer.
// MORSO_NULL_INPUT: A parameter was NULL.
int encode_morse_msg(const char *msg, char *buf, size_t buf_size);

// Decodes a morse message into the given buffer.
// Message must contain only dots, dashes and spaces and the null terminator.
// Morse symbols are separated with a single space: "--- -.".
// Words are separated with a double space "--- -.  -.- --- ...- .-"
// buf_size must be >= 2.
//
// Returns a morso_status code:
// MORSO_OK: Ok
// MORSO_INVALID_INPUT: Invalid symbols in input string.
// MORSO_BUF_OVERFLOW: Not enough memory in buffer.
// MORSO_NULL_INPUT: A parameter was NULL.
int decode_morse_msg(const char *msg, char *buf, size_t buf_size);

// Returns a pointer to a morse string representation of the input char.
// Input char must be between a-z or A-Z.
//
// Returns NULL if a valid morse is not found.
const char *char_to_morse(char c);

// Parses a null terminated string representing a single morse symbol.
// String must contain only dots and dashes eg. ".--.".
//
// Returns a char or a morso_status code in case of an error:
// MORSO_INVALID_INPUT: Invalid symbols in input string.
// MORSO_BUF_OVERFLOW: Not enough memory in buffer.
// MORSO_NULL_INPUT: A parameter was NULL.
char morse_to_char(const char *str);
