# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# ESP32-C3FH4 is an ESP32-C3 with 4 MB of in-package flash.
set(IDF_TARGET "esp32c3" CACHE STRING "ESP-IDF target" FORCE)

# Build only the application and its declared dependencies. In particular,
# do not link the Wi-Fi, Bluetooth, IEEE 802.15.4, or PHY stacks.
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(softwarev1)
