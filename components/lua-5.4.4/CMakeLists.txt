FILE(GLOB_RECURSE lua_sources ${CMAKE_SOURCE_DIR}/components/lua-5.4.4/src/*.*)

idf_component_register(SRCS ${lua_sources})
