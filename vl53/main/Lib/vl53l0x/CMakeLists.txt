set(req driver esp32 freertos)

idf_component_register(
    SRCS "DistSensor.c" "vl53l0x_api.c" "vl53l0x_api_calibration.c" "vl53l0x_api_core.c" "vl53l0x_api_ranging.c" "vl53l0x_api_strings.c" "vl53l0x_platform.c" "vl53l0x_platform_log.c"
    INCLUDE_DIRS . ../
    REQUIRES ${req}
)