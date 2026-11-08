idf_component_register(SRCS "can_link.cpp" "twai_driver.cpp" "command_bridge.cpp" "uart_driver.cpp" "nmea_gps.cpp" "px1120s_gps.cpp" "flight_state_machine.cpp" "w25n01kv.cpp" "ra01.cpp" "flash_logger.cpp" "ms5607.cpp" "sensors.cpp" "spi_data_forwarder.cpp" "heartbeat.cpp" "main.cpp" "usb_serial_echo.cpp" "usb_mass_storage.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_twai)

# Enable TinyUSB's MSC class without its internal-flash/SD backend; this
# component provides the external-NAND callbacks itself.
idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
target_compile_definitions(${tusb_lib} PRIVATE
    CONFIG_TINYUSB_MSC_ENABLED=1 CONFIG_TINYUSB_MSC_BUFSIZE=512)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    CONFIG_TINYUSB_MSC_ENABLED=1 CONFIG_TINYUSB_MSC_BUFSIZE=512)

