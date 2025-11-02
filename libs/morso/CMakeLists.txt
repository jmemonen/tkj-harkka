# morso add_library

set (APP_NAME morso)

add_library(${APP_NAME} STATIC
  src/morso.c
)

# add_executable(morso_test 
#   tests/morso_test.c
# )

target_include_directories(${APP_NAME}
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# target_link_libraries(morso_test PRIVATE
#   morso
# )

# target_include_directories(${APP_NAME}
#   PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

target_compile_features(${APP_NAME} PUBLIC c_std_11)
