FILE(GLOB_RECURSE app_sources "*.cpp")
idf_component_register(SRCS ${app_sources} INCLUDE_DIRS ".")

spiffs_create_partition_image(spiffs ../data FLASH_IN_PROJECT)