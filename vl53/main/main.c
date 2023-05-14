/*****************************************************************************
 @Project	:
 @File 		: main.c
 @Details  	: All Ports and peripherals configuration
 @Author	: Gigasitron X-Cross
 @Hardware	:

 --------------------------------------------------------------------------
 @Revision	:
  Ver  	Author    	Date        	Changes
 --------------------------------------------------------------------------
   1.0  Name     XXXX-XX-XX  		Initial Release

******************************************************************************/

#include <Common.h>
#include "Hal.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/xtensa_api.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "Hal/gpio_types.h"
#include "driver/i2c.h"

#include "DistSensor.h"


/*****************************************************************************
 Define
******************************************************************************/
#define I2C_MASTER_NUM 				I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 			400000U
#define I2C_MASTER_TX_BUF_DISABLE 	0
#define I2C_MASTER_RX_BUF_DISABLE 	0

#define ESP_INTR_FLAG_DEFAULT 		ESP_INTR_FLAG_LOWMED

/*****************************************************************************
 Type definition
******************************************************************************/


/*****************************************************************************
 Global Variables
******************************************************************************/


/*****************************************************************************
 Local Variables
******************************************************************************/
static VL53L0X_Dev_t 		g_hDistSensor1;
static int					g_nDistData = 0;

static EventGroupHandle_t 	g_evtSensor;
static EventGroupHandle_t 	g_evtTx;

/*****************************************************************************
 callback functions
******************************************************************************/
static void IRAM_ATTR gpio_isr_handler(void* arg);


/*****************************************************************************
 Local functions
******************************************************************************/
static void main_I2cInit( void );

/*****************************************************************************
 Implementation
******************************************************************************/

void app_main(void)
{
    EventBits_t uxBits;
    const TickType_t xTicksToWait = 5000 / portTICK_PERIOD_MS;

    g_evtSensor = xEventGroupCreate();
    g_evtTx = xEventGroupCreate();

    main_I2cInit();

	DistSensorInit( &g_hDistSensor1, NULL, 0x52>>1U );
	DistSensorConfigure( &g_hDistSensor1, TOF_MODE_HIGH_ACCURACY );
	DistSensorStart( &g_hDistSensor1 );
	vTaskDelay(1000 / portTICK_PERIOD_MS);


    while (1)
    {
    	uxBits = xEventGroupWaitBits( g_evtSensor, 0x00000001,  pdTRUE, pdFALSE, xTicksToWait );

		if( 0 == uxBits )
		{
			DistSensorStart( &g_hDistSensor1 );
			printf( "Sensor Task: Timeout\r\n" );
			continue;
		}

		 g_nDistData = DistSensorRead( &g_hDistSensor1 );
		 printf( "dist: %d\r\n", g_nDistData );
		 DistSensorStart( &g_hDistSensor1 );

		 xEventGroupSetBits( g_evtTx, 0x00000001 );
    }
}



/*****************************************************************************
 Callback functions
******************************************************************************/


/*****************************************************************************
 Local functions
******************************************************************************/
static void main_I2cInit( void )
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = TOF_I2C_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = TOF_I2C_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;

    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);


	gpio_config_t io_conf;

	io_conf.intr_type = GPIO_INTR_NEGEDGE; 	//interrupt of rising edge
	io_conf.pin_bit_mask = BIT(TOF_IRQ_IN); 	//bit mask of the pins, use GPIO4/5 here
	io_conf.mode = GPIO_MODE_INPUT; 	//set as input mode
	io_conf.pull_up_en = 1; //enable pull-up mode
	io_conf.pull_down_en = 0;
	gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(TOF_IRQ_IN, gpio_isr_handler, (void*) TOF_IRQ_IN);


}



/*****************************************************************************
 Interrupt functions
******************************************************************************/
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    //uint32_t gpio_num = (uint32_t) arg;
	UNUSE(arg);
    BaseType_t xHigherPriorityTaskWoken, xResult;

      xHigherPriorityTaskWoken = pdFALSE;

      xResult = xEventGroupSetBitsFromISR( g_evtSensor, 0x00000001, &xHigherPriorityTaskWoken );

      /* Was the message posted successfully? */
      if( xResult != pdFAIL )
      {
          /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
          switch should be requested.  The macro used is port specific and will
          be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
          the documentation page for the port being used. */
          portYIELD_FROM_ISR();
      }
}
