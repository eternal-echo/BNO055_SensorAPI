idf_component_register(SRCS "bno055.c"
                            "bno055_support.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_driver_i2c freertos log esp_timer)

target_compile_definitions(${COMPONENT_LIB} PRIVATE BNO055_API)
