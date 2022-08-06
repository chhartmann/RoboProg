cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_definitions(LUA_32BITS)

if (BUILD_FOR_QEMU)
  add_compile_definitions(BUILD_FOR_QEMU)
endif()

project(main)