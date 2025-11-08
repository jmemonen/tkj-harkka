#pragma once

#include <stdbool.h>
#include <stddef.h>

// Would prefer to hide this in implementation, but needed
// for the struct...
#define _MORSO_MAX_SYMBOL_LEN 4

enum morso_status {
  MORSO_OK,
  MORSO_INVALID_INPUT,
  MORSO_BUF_OVERFLOW,
  MORSO_NULL_INPUT,
  MORSO_MSG_READY
};

// morso_msg_t
// A convenient message buffer for building morse code strings.
// Helps enforce the protocol so only valid morse messages are built.
typedef struct {
  char *msg_buf;
  size_t msg_max_len;
  size_t msg_len;
  char inp_buf[_MORSO_MAX_SYMBOL_LEN];
  size_t inp_len;
  char inp;
  bool ready_to_send;
} msg_builder_t;

// Initialize a message builder by setting the message buffer and its size.
// Size is calculated so that enough memory is reserved for the message
// ending "  \n\0"
//
// Returns a morso_status code:
// MORSO_OK: Ok
// MORSO_BUF_OVERFLOW: Insufficient memory in a buffer (inp or msg buf).
// MORSO_NULL_INPUT: A parameter was NULL.
int msg_init(msg_builder_t *b, char *msg_buf, size_t buf_size);

// Write a dot, dash or space to a msg_builder_t.
// A dot or a dash is written to the input buffer.
// Write a space to append the current symbol to the message.
// Finalize the msg with msg_ready!
//
// Returns a morso_status code:
// MORSO_OK: Ok
// MORSO_INVALID_INPUT: Invalid symbols in input string.
// MORSO_BUF_OVERFLOW: Insufficient memory in a buffer (inp or msg buf).
// MORSO_NULL_INPUT: A parameter was NULL.
int msg_write(msg_builder_t *b, char c);

// Reset the current symbol input.
// Return MORSO_OK or MORSO_NULL_INPUT if *b is NULL.
int msg_reset_inp(msg_builder_t *b);

// Reset the whole message.
// Return MORSO_OK or MORSO_NULL_INPUT if *b is NULL.
int msg_reset(msg_builder_t *b);

// Finalize the message and mark it ready to send.
//
// Returns a morso_status code:
// MORSO_OK: Ok
// MORSO_BUF_OVERFLOW: Insufficient memory in msg buffer.
// MORSO_NULL_INPUT: A parameter was NULL.
int msg_ready(msg_builder_t *b);

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
