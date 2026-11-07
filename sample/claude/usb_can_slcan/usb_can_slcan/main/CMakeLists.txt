idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_driver_gpio esp_driver_uart
                                     nvs_flash esp_timer)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=cpp)