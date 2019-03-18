cmake_minimum_required (VERSION 2.6)
project(Main NONE)

set(BOARD_TYPE "core2")
set(PORT "stm32")
set(BOARD_VERSION "1.0.0")
include(${HFRAMEWORK_PATH}/hFramework.cmake)

enable_module("hCloudClient" "")
enable_module("hSensors" "")
enable_module("hModules" "")
enable_module("hROS" "")


add_hexecutable(main "main.cpp")
