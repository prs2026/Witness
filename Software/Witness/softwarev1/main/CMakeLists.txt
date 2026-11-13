idf_component_register(SRCS "cpu_load.cpp" "can_link.cpp" "twai_driver.cpp" "command_bridge.cpp" "uart_driver.cpp" "nmea_gps.cpp" "hitl_input.cpp" "gps_receiver.cpp" "gps_assist.cpp" "flight_state_machine.cpp" "attitude_estimator.cpp" "accel_filter.cpp" "position_filter.cpp" "heading_align.cpp" "imu_calibration.cpp" "imu_calibration_store.cpp" "w25n01kv.cpp" "ra01.cpp" "flash_logger.cpp" "ms5607.cpp" "sensors.cpp" "spi_data_forwarder.cpp" "heartbeat.cpp" "main.cpp" "usb_serial_echo.cpp" "usb_mass_storage.cpp" "wifi_ota_logic.cpp" "wifi_ota.cpp" "maintenance_mode.cpp" "flight_state_store.cpp"
                    INCLUDE_DIRS "." "../../../Intercomm"
                    # An explicit list disables main's implicit dependency on
                    # every component, so name each one the sources include.
                    PRIV_REQUIRES esp_adc esp_driver_gpio esp_driver_ledc esp_driver_spi
                                  esp_driver_twai esp_driver_uart esp_driver_usb_serial_jtag
                                  esp_timer nvs_flash vfs app_update esp_event esp_http_server
                                  esp_hw_support esp_netif esp_system esp_wifi espressif__esp_tinyusb espressif__tinyusb)

# Enable TinyUSB's MSC class without its internal-flash/SD backend; this
# component provides the external-NAND callbacks itself.
idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
target_compile_definitions(${tusb_lib} PRIVATE
    CONFIG_TINYUSB_MSC_ENABLED=1 CONFIG_TINYUSB_MSC_BUFSIZE=512)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    CONFIG_TINYUSB_MSC_ENABLED=1 CONFIG_TINYUSB_MSC_BUFSIZE=512)

