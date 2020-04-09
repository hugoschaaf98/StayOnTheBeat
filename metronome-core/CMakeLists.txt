# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(COMPONENTS "esptool_py esp32 main")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(metronome)

#spiffs_create_partition_image(my_spiffs_partition my_folder FLASH_IN_PROJECT)