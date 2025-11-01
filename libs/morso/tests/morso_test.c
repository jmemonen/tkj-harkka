#include "morso/morso.h"
#include <stddef.h>
#include <stdio.h>

void test_char_to_morse(void);
void test_morse_to_char(void);

int main(void) {
  test_char_to_morse();
  test_morse_to_char();
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
