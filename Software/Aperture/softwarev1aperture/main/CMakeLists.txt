idf_component_register(
    SRCS "ra01.cpp" "main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio esp_driver_spi esp_driver_usb_serial_jtag)
