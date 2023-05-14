# Edit following two lines to set component requirements (see docs)
cmake_minimum_required(VERSION 3.5)
#set(COMPONENT_REQUIRES )
#set(COMPONENT_PRIV_REQUIRES )
set(_include "${IDF_PATH}/components/freertos/FreeRTOS-Kernel/include/freertos" 
	         "${IDF_PATH}/components/nvs_flash/include" 
	         "${IDF_PATH}/components/esp_hw_support/include"
			 "./Lib/vl53l0x"
			 "./Lib"
			 "")
			 
set( srcs 	"main.c" 
			"./Lib/vl53l0x/DistSensor.c"
			"./Lib/vl53l0x/vl53l0x_api.c"
			"./Lib/vl53l0x/vl53l0x_api_calibration.c"
			"./Lib/vl53l0x/vl53l0x_api_core.c"
			"./Lib/vl53l0x/vl53l0x_api_ranging.c"
			"./Lib/vl53l0x/vl53l0x_api_strings.c"
			"./Lib/vl53l0x/vl53l0x_platform.c"
			"./Lib/vl53l0x/vl53l0x_platform_log.c" )
			 
set(COMPONENT_SRCS ${srcs})
set(COMPONENT_ADD_INCLUDEDIRS ${_include})

register_component()

