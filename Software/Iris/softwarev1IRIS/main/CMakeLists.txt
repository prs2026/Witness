idf_component_register(
    SRCS "battery_voltage.cpp" "mcp23008.cpp" "pac1931.cpp" "usb_serial_echo.cpp" "twai_driver.cpp" "can_link.cpp" "main.cpp"
    INCLUDE_DIRS "." "../../../Intercomm"
    PRIV_REQUIRES esp_adc esp_driver_gpio esp_driver_i2c esp_driver_uart esp_driver_usb_serial_jtag esp_driver_twai esp_timer vfs)
