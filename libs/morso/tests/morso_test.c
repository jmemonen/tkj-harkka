#include "morso/morso.h"
#include <stddef.h>
#include <stdio.h>

void test_char_to_morse(void);
void test_morse_to_char(void);
void test_encode_morse_msg(void);
void test_decode_morse_msg(void);

int main(void) {
  // test_char_to_morse();
  // test_morse_to_char();
  // test_encode_morse_msg();
  test_decode_morse_msg();
  return 0;
}

void test_char_to_morse(void) {
  for (char c = 'a'; c <= 'z'; c++) {
    const char *str = char_to_morse(c);
    if (str == NULL) {
      printf("ERROR: couldn't encode char: %c\n", c);
    }
    printf("%c : %s\n", c, str);
  }

  for (char c = 'A'; c <= 'Z'; c++) {
    const char *str = char_to_morse(c);
    if (str == NULL) {
      printf("ERROR: couldn't encode char: %c\n", c);
    }
    printf("%c : %s\n", c, str);
  }

  char invalid[] = " 1234567890";
  char *c_ptr = invalid;
  while (*c_ptr) {
    const char *str = char_to_morse(*c_ptr);
    if (str != NULL) {
      printf("%c should have returned NULL! Returned: %s", *c_ptr, str);
    }
    printf("%d : %s\n", *c_ptr, str);
    ++c_ptr;
  }
}

void test_morse_to_char() {
  printf("\ntest_morse_to_char\n");
  for (char c = 'A'; c <= 'Z'; c++) {
    const char *str = char_to_morse(c);
    char c_result = morse_to_char(str);
    printf("%c\t:\t%s\n", c, str);
  }
}

void test_encode_morse_msg(void) {
  const size_t BUF_SIZE = 128;
  char buf[BUF_SIZE];

  char *str = "abcdefg hijklmnop qrstuvxyz";
  printf("\nEncoding message: %s :\t", str);
  int res = encode_morse_msg(str, buf, BUF_SIZE);
  printf("%s\n", buf);
  printf("return code %d. Should be 0.\n", res);

  buf[0] = '\0';
  str = "a hundred beers every day";
  printf("\nEncoding message: %s :\t", str);
  res = encode_morse_msg(str, buf, BUF_SIZE);
  printf("%s\n", buf);
  printf("return code %d. Should be 0\n", res);

  // Edge case where \0 should come immediately after a symbol.
  buf[0] = '\0';
  char lil_buf[2];
  str = "e";
  printf("\nEncoding message: %s :\t", str);
  res = encode_morse_msg(str, lil_buf, 2);
  printf("%s\n", lil_buf);
  printf("return code %d. Should be 0\n", res);

  // Buffer overflow
  lil_buf[0] = '\0';
  str = "east bound and down loaded up and truckin";
  printf("\nEncoding message: %s :\t", str);
  res = encode_morse_msg(str, lil_buf, 2);
  printf("%s\n", lil_buf);
  printf("return code %d. Should be 2\n", res);

  // Non-alphabetic symbols
  buf[0] = '\0';
  str = "1234";
  printf("\nEncoding message: %s :\t", str);
  res = encode_morse_msg(str, buf, BUF_SIZE);
  printf("%s\n", buf);
  printf("return code %d. Should be 1\n", res);

  // Null in arguments
  buf[0] = '\0';
  str = "1234";
  printf("\nEncoding message: %s :\t", str);
  res = encode_morse_msg(NULL, buf, BUF_SIZE);
  printf("%s\n", buf);
  printf("return code %d. Should be 3\n", res);
}

void test_decode_morse_msg(void) {
  size_t buf_size = 64;
  char buf[buf_size];

  // Basic case
  printf("\n");
  printf("Basic test case\n");
  char *msg = "--. .- --  -... ..  -. .-";
  printf("Decoding: %s\n", msg);
  int res = decode_morse_msg(msg, buf, buf_size);
  printf("Decoded: %s\n", buf);
  printf("Result code: %d. Should be 0.\n", res);

  // A tight fit
  printf("\n");
  printf("Just enough memory\n");
  buf_size = 6;
  buf[0] = '\0';
  msg = "-.- --- .. .-. .-";
  printf("Decoding: %s\n", msg);
  res = decode_morse_msg(msg, buf, buf_size);
  printf("Decoded: %s\n", buf);
  printf("Result code: %d. Should be 0.\n", res);

  // Null arguments
  printf("\n");
  printf("Null arguments\n");
  buf[0] = '\0';
  msg = "-.- --- ..-- .-. .-";
  printf("Decoding: %s\n", msg);
  res = decode_morse_msg(msg, NULL, buf_size);
  printf("Decoded: %s\n", buf);
  printf("Result code: %d. Should be 3.\n", res);

  // Faulty morse
  printf("\n");
  printf("Faulty morse code\n");
  buf[0] = '\0';
  msg = "-.- --- ..-- .-. .-";
  printf("Decoding: %s\n", msg);
  res = decode_morse_msg(msg, buf, buf_size);
  printf("Decoded: %s\n", buf);
  printf("Result code: %d. Should be 1.\n", res);

  // A TOO tight fit
  printf("\n");
  printf("Buffer overflow\n");
  buf_size = 5;
  buf[0] = '\0';
  msg = "-.- --- .. .-. .-";
  printf("Decoding: %s\n", msg);
  res = decode_morse_msg(msg, buf, buf_size);
  printf("Decoded: %s\n", buf);
  printf("Result code: %d. Should be 2.\n", res);
}
