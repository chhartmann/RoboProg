FILE(GLOB_RECURSE lua_sources "src/*.c")
idf_component_register(SRCS ${lua_sources} INCLUDE_DIRS "src")
