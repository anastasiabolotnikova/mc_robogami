find_description_package(robogami_description)

configure_file(robogami.in.cpp "${CMAKE_CURRENT_BINARY_DIR}/robogami.cpp")

set(module_SRC
  "${CMAKE_CURRENT_BINARY_DIR}/robogami.cpp"
)

set(module_HDR
  robogami.h
)

add_robot(robogami "${module_SRC}" "${module_HDR}")
target_include_directories(robogami PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")