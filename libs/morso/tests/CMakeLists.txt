# morso tests

cmake_minimum_required(VERSION 3.13)
set (APP_NAME morso_test)

add_executable(${APP_NAME}
  morso_test.c
)

add_subdirectory(
  .. build
)

target_include_directories(${APP_NAME}
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/build
)

target_link_libraries(${APP_NAME} PRIVATE
  morso
)

target_include_directories(${APP_NAME}
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}../include
)

target_compile_features(${APP_NAME} PUBLIC c_std_11)
