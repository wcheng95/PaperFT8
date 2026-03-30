idf_component_register(
    SRCS
        "main_ref.cpp"
        "autoseq.cpp"
        "resample.cpp"
        "stream_uac.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        M5Unified
        M5GFX
        ui
        ft8_lib
        spiffs
        fatfs
        sdmmc
        esp_driver_spi
        esp_driver_sdspi
        esp_driver_gpio
        esp_driver_usb_serial_jtag
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Werror)
