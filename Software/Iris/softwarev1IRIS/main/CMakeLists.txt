idf_component_register(
    SRCS "usb_serial_echo.cpp" "main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio esp_driver_usb_serial_jtag esp_timer vfs)
