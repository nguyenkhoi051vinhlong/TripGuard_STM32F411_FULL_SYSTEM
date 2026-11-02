idf_component_register(
    SRCS "main.c" "mesh_link.c" "ld2410.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_driver_gpio esp_driver_uart esp_pm esp_timer
)
