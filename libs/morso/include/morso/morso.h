#pragma once

#include <stddef.h>
enum morso_error {
  MORSO_OK,
  MORSO_INVALID_INPUT,
  MORSO_BUF_OVERFLOW,
  MORSO_NULL_INPUT
};

// Returns the given string as morse code in the given buffer.
// So far supports only alphabetical ASCII strings with aA-zZ
// where words are separated by spaces.
//
// Returns a morso_error code.
// 0: Ok
// 1: Invalid symbols in input string.
// 2: Given buffer was too small for the encoded message.
// 3: A parameter was NULL.
int encode_morse_msg(const char *str, char *buf, size_t buf_size);

// Returns a pointer to a morse string representation of the input char.
// Input char must be between a-z or A-Z. Otherwise the function returns NULL.
const char *char_to_morse(char c);

// Parses a null terminated string representing a single morse symbol.
// String must contain only dots and dashes eg. ".--.".
// Returns a char corresponding to a morso_error in case of an error.
char morse_to_char(const char *str);
