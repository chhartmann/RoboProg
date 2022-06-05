FILE(GLOB_RECURSE lua_sources ${CMAKE_SOURCE_DIR}/lib/lua-5.4.4/src/*.*)

idf_component_register(SRCS ${lua_sources})

#target_include_directories(lua PUBLIC
#                           ${CMAKE_SOURCE_DIR}/lib/lua-5.4.4/src)