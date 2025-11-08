#include "morso/morso.h"
#include <stddef.h>
#include <stdio.h>

// Not really a proper test suite, but enough to visualize
// what happens in them functions...

void test_char_to_morse(void);
void test_morse_to_char(void);
void test_encode_morse_msg(void);
void test_decode_morse_msg(void);
void test_encode_and_decode(void);
void test_msg_builder(void);

int main(void) {
  // test_char_to_morse();
  // test_morse_to_char();
  // test_encode_morse_msg();
  // test_decode_morse_msg();
  // test_encode_and_decode();
  test_msg_builder();
  return 0;
}

void test_char_to_morse(void) {
  printf("\nchar_to_morse ----------------------------------\n");
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
  printf("\nmorse_to_char ----------------------------------\n");
  printf("\ntest_morse_to_char\n");
  for (char c = 'A'; c <= 'Z'; c++) {
    const char *str = char_to_morse(c);
    char c_result = morse_to_char(str);
    printf("%c\t:\t%s\n", c, str);
  }
}

void test_encode_morse_msg(void) {
  printf("\nencode_morse_msg ----------------------------------\n");
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
  printf("\ndecode_morse_msg ----------------------------------\n");
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

void test_encode_and_decode(void) {
  printf("\nencode_and_decode ----------------------------------\n");
  printf("\nTesting back and forth encoding\n");
  char msg[13] = "aktivoitu ON";
  const size_t BUF_SIZE = 64;
  char buf[BUF_SIZE];

  printf("Original message: %s\n", msg);
  int res = encode_morse_msg(msg, buf, BUF_SIZE);
  printf("Encoded: %s\n", buf);
  printf("Result code: %d. Should be 0.\n", res);
  printf("Decoding back.\n");
  res = decode_morse_msg(buf, msg, 13);
  printf("Decoded: %s\n", msg);
  printf("Result code: %d. Should be 0.\n", res);
}

// Print a ready msg.
static inline void print_msg(msg_builder_t *b) {
  if (!b->ready_to_send) {
    return;
  }
  char *p = b->msg_buf;
  while (*p) {
    switch (*p) {
    case ' ':
      printf("_");
      break;
    case '\n':
      printf("/");
      break;
    default:
      printf("%c", *p);
      break;
    }
    p++;
  }
  printf("EOM\n");
}

// Works as intended so far, though.
void test_msg_builder(void) {
  printf("\nmsg_builder ----------------------------------\n");
  char msg_buf[128];
  msg_builder_t builder;
  msg_init(&builder, msg_buf, 128);
  int result = 0;

  char *msg = {"... "};
  size_t i = 0;
  while (*msg) {
    printf("i:%d\n", i++);
    result = msg_write(&builder, *msg++);
    printf("msg_buf inp symbol: %c\n",
           (builder.inp == MORSO_INVALID_INPUT) ? '?' : builder.inp);
    printf("msg is now: %s\n", builder.msg_buf);
    printf("msg len: %d\n", builder.msg_len);
    printf("result: %d\n\n", result);
    if (builder.ready_to_send) {
      printf("msg ready to send.\n");
    }
  }

  result = msg_ready(&builder);
  printf("msg_buf inp symbol: %c\n",
         (builder.inp == MORSO_INVALID_INPUT) ? '?' : builder.inp);
  printf("final message: ");
  printf("msg:%s\n", builder.msg_buf);
  print_msg(&builder);
  printf("result: %d\n\n", result);
  printf("msg len: %d\n", builder.msg_len);
  if (builder.ready_to_send) {
    printf("msg ready to send.\n");
  }

  // Test a tight fit
  printf("Testing a lil msg buffer...");
  size_t small = 10;
  char small_msg[small];
  msg_init(&builder, small_msg, small);
  msg = "--- -. ";
  i = 0;
  while (*msg) {
    printf("i:%d\n", i++);
    result = msg_write(&builder, *msg++);
    printf("msg_buf inp symbol: %c\n",
           (builder.inp == MORSO_INVALID_INPUT) ? '?' : builder.inp);
    printf("msg is now: %s\n", builder.msg_buf);
    printf("msg len: %d\n", builder.msg_len);
    printf("result: %d\n\n", result);
    if (builder.ready_to_send) {
      printf("msg ready to send.\n");
    }
  }
  result = msg_ready(&builder);
  printf("msg_buf inp symbol: %c\n",
         (builder.inp == MORSO_INVALID_INPUT) ? '?' : builder.inp);
  printf("final message: ");
  printf("msg:%s\n", builder.msg_buf);
  print_msg(&builder);
  printf("result: %d\n\n", result);
  printf("msg len: %d\n", builder.msg_len);
  if (builder.ready_to_send) {
    printf("msg ready to send.\n");
  }

  // Test a too long symbol
  printf("\nTesting a symbol too long...");
  msg_init(&builder, small_msg, small);
  msg = "..... ";
  i = 0;
  while (*msg) {
    printf("i:%d\n", i++);
    result = msg_write(&builder, *msg++);
    printf("msg_buf inp symbol: %c\n",
           (builder.inp == MORSO_INVALID_INPUT) ? '?' : builder.inp);
    printf("msg is now: %s\n", builder.msg_buf);
    printf("msg len: %d\n", builder.msg_len);
    printf("inp len: %d\n", builder.inp_len);
    printf("result: %d\n\n", result);
    printf("inp_buf: %s\n", builder.inp_buf);
    if (builder.ready_to_send) {
      printf("msg ready to send.\n");
    }
  }
  result = msg_ready(&builder);
  printf("msg_buf inp symbol: %c\n",
         (builder.inp == MORSO_INVALID_INPUT) ? '?' : builder.inp);
  printf("final message: ");
  printf("msg:%s\n", builder.msg_buf);
  print_msg(&builder);
  printf("result: %d\n\n", result);
  printf("msg len: %d\n", builder.msg_len);
  printf("inp buf: %s\n", builder.inp_buf);
  if (builder.ready_to_send) {
    printf("msg ready to send.\n");
  }
}
