# tkj_utils

set (APP_NAME tkj_utils)

add_library(${APP_NAME} STATIC
  src/tkj_utils.c
  src/message_output.c
)

target_include_directories(${APP_NAME}
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${APP_NAME}
  PUBLIC
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
  TKJHAT_SDK
)

target_compile_features(${APP_NAME} PUBLIC c_std_11)
