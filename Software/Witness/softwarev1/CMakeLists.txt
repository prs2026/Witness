# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# Witness is an ESP32-S3 (ESP32-S3FN8). Pin the target so a fresh checkout
# does not fall back to the plain ESP32, which lacks TinyUSB and USB Serial/JTAG.
set(IDF_TARGET "esp32s3" CACHE STRING "ESP-IDF target" FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(softwarev1)
