# One source tree, two images (see Kconfig.projbuild WITNESS_IMAGE and
# build_images.ps1). The FLIGHT image links no Wi-Fi/lwIP/HTTP/TinyUSB code at
# all (they cost ~27 KB of internal RAM just by being linked); the MAINTENANCE
# image has no sensors or flight logic.
set(witness_common_srcs "boot_manager.cpp" "maintenance_mode.cpp" "wifi_ota_logic.cpp"
                        "w25n01kv.cpp" "ra01.cpp" "flash_logger.cpp")

# The WITNESS_IMAGE environment variable (=maintenance; build_images.ps1 sets
# it; unset = flight) picks the image: the component requirements are evaluated before sdkconfig
# is loaded, so they cannot depend on CONFIG_WITNESS_IMAGE_*. A check below
# makes sure the two agree.
set(WITNESS_IMAGE "$ENV{WITNESS_IMAGE}")
if(WITNESS_IMAGE STREQUAL "maintenance")
    set(witness_srcs ${witness_common_srcs} "maint_main.cpp" "maint_radio.cpp" "maint_usb.cpp"
                     "wifi_ota.cpp" "usb_mass_storage.cpp")
    # An explicit list disables main's implicit dependency on every component,
    # so name each one the sources include.
    set(witness_requires esp_adc esp_driver_gpio esp_driver_spi esp_driver_usb_serial_jtag
                         esp_timer nvs_flash vfs app_update esp_event esp_http_server
                         esp_hw_support esp_netif esp_system esp_wifi
                         espressif__esp_tinyusb espressif__tinyusb)
else()
    set(witness_srcs ${witness_common_srcs}
                     "cpu_load.cpp" "can_link.cpp" "twai_driver.cpp" "command_bridge.cpp"
                     "uart_driver.cpp" "nmea_gps.cpp" "hitl_input.cpp" "gps_receiver.cpp"
                     "gps_assist.cpp" "flight_state_machine.cpp" "attitude_estimator.cpp"
                     "accel_filter.cpp" "position_filter.cpp" "heading_align.cpp"
                     "imu_calibration.cpp" "imu_calibration_store.cpp" "radio_power_store.cpp" "ms5607.cpp"
                     "sensors.cpp" "sensors_landed.cpp" "spi_data_forwarder.cpp" "heartbeat.cpp" "main.cpp"
                     "usb_serial_echo.cpp" "flight_state_store.cpp" "flight_selftest.cpp")
    # (main/idf_component.yml also lists esp_tinyusb, which the component
    # manager therefore builds for this image too, but nothing here references
    # it, so none of it is linked: verified in the map and size-components.)
    set(witness_requires esp_adc esp_driver_gpio esp_driver_ledc esp_driver_spi
                         esp_driver_twai esp_driver_uart esp_driver_usb_serial_jtag
                         esp_timer nvs_flash vfs app_update esp_hw_support esp_system esp_pm)
endif()

idf_component_register(SRCS ${witness_srcs}
                    INCLUDE_DIRS "." "../../../Intercomm"
                    PRIV_REQUIRES ${witness_requires})

# Build time (seconds since the Unix epoch) orders images of the same kind
# (image_info.h build_id).
if(WITNESS_IMAGE STREQUAL "maintenance")
    if(NOT CONFIG_WITNESS_IMAGE_MAINTENANCE)
        message(FATAL_ERROR "WITNESS_IMAGE=maintenance (environment) needs the sdkconfig.maint overlay")
    endif()
elseif(CONFIG_WITNESS_IMAGE_MAINTENANCE)
    message(FATAL_ERROR "sdkconfig selects the maintenance image: set WITNESS_IMAGE=maintenance")
endif()

string(TIMESTAMP witness_build_id "%s" UTC)
target_compile_definitions(${COMPONENT_LIB} PRIVATE WITNESS_BUILD_ID=${witness_build_id})

if(WITNESS_IMAGE STREQUAL "maintenance")
    # Enable TinyUSB's MSC class without its internal-flash/SD backend; this
    # component provides the external-NAND callbacks itself.
    idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
    target_compile_definitions(${tusb_lib} PRIVATE
        CONFIG_TINYUSB_MSC_ENABLED=1 CONFIG_TINYUSB_MSC_BUFSIZE=512)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        CONFIG_TINYUSB_MSC_ENABLED=1 CONFIG_TINYUSB_MSC_BUFSIZE=512)
endif()
