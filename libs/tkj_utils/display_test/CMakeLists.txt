# buzzer tests

cmake_minimum_required(VERSION 3.13)
set (APP_NAME display_test)

set(DEFAULT_TARGET display_test)
add_executable(${DEFAULT_TARGET}
  ${CMAKE_CURRENT_LIST_DIR}/src/main.c
)


target_link_libraries(${DEFAULT_TARGET} PRIVATE
  pico_stdlib
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
  TKJHAT_SDK
  tkj_utils
)

pico_enable_stdio_usb(${DEFAULT_TARGET} 1)
pico_enable_stdio_uart(${DEFAULT_TARGET} 0)

pico_add_extra_outputs(${DEFAULT_TARGET})
