/*****************************************************************************
 @Project	: Time of flight sensor
 @File 		: DistSensor.c
 @Details  	:
 @Author	: Gigasitron X-Cross
 @Hardware	: 
 
 --------------------------------------------------------------------------
 @Revision	:
  Ver  	Author    	Date        	Changes
 --------------------------------------------------------------------------
   1.0  Name     XXXX-XX-XX  		Initial Release
   
******************************************************************************/
#include <Common.h>
#include "driver/i2c.h"
#include "DistSensor.h"

/*****************************************************************************
 Define
******************************************************************************/


/*****************************************************************************
 Type definition
******************************************************************************/


/*****************************************************************************
 Global Variables
******************************************************************************/


/*****************************************************************************
 Local Variables
******************************************************************************/


/*****************************************************************************
 Implementation
******************************************************************************/
void DistSensorInit( VL53L0X_DEV pDev, void *pI2cHandle, int nI2cAddr )
{
	ESP_ERROR_CHECK( 0 != pI2cHandle );

	pDev->I2cHandle = pI2cHandle;
	pDev->nI2c = I2C_NUM_0;
	pDev->I2cDevAddr = nI2cAddr;
}


void DistSensorSetDevAddress( VL53L0X_DEV pDev, int nI2cAddr )
{
	VL53L0X_Error error;

	error = VL53L0X_SetDeviceAddress( pDev, nI2cAddr );
	if( VL53L0X_ERROR_NONE == error )
	{
		pDev->I2cDevAddr = nI2cAddr;
	}
}


void DistSensorConfigure( VL53L0X_DEV pDev, TOF_MODE Mode )
{
	uint8_t reva;
	uint8_t revb;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;


	VL53L0X_DataInit( pDev );
	
	VL53L0X_GetProductRevision( pDev, &reva, &revb );
	printf( "TOF Sensor: v:%d r:%d\r\n", reva, revb );

	VL53L0X_StaticInit( pDev );
	
	VL53L0X_PerformRefSpadManagement( pDev, &refSpadCount, &isApertureSpads); // Device Initialization

    switch( Mode )
    {
        case TOF_MODE_LONG_RANGE:
            VL53L0X_SetLimitCheckEnable( pDev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );

            VL53L0X_SetLimitCheckEnable( pDev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);

            VL53L0X_SetLimitCheckValue( pDev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536) );

            VL53L0X_SetLimitCheckValue( pDev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536) );
            VL53L0X_SetMeasurementTimingBudgetMicroSeconds( pDev, 33000 );

            VL53L0X_SetVcselPulsePeriod( pDev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18 );
            VL53L0X_SetVcselPulsePeriod( pDev,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14 );
        break;

        case TOF_MODE_HIGH_SPEED:
            VL53L0X_SetLimitCheckEnable( pDev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );

            VL53L0X_SetLimitCheckEnable( pDev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);

            VL53L0X_SetLimitCheckValue( pDev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536) );

            VL53L0X_SetLimitCheckValue( pDev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32*65536) );
            VL53L0X_SetMeasurementTimingBudgetMicroSeconds( pDev, 20000 );

            VL53L0X_SetVcselPulsePeriod( pDev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 14 );
            VL53L0X_SetVcselPulsePeriod( pDev,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 10 );
        break;

        case TOF_MODE_HIGH_ACCURACY:
            VL53L0X_SetLimitCheckEnable( pDev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );

            VL53L0X_SetLimitCheckEnable( pDev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);

            VL53L0X_SetLimitCheckValue( pDev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536) );//(0.25*65536)

            VL53L0X_SetLimitCheckValue( pDev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536) );
            VL53L0X_SetMeasurementTimingBudgetMicroSeconds( pDev, 50000*10 );

            VL53L0X_SetVcselPulsePeriod( pDev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18 );
            VL53L0X_SetVcselPulsePeriod( pDev,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14 );
        break;

        default:
        break;
    }

#if 1 /* Not required as it is done in static init */

#endif



	VL53L0X_PerformRefCalibration( pDev, &VhvSettings, &PhaseCal );

	VL53L0X_SetDeviceMode( pDev, VL53L0X_DEVICEMODE_SINGLE_RANGING );

    VL53L0X_SetGpioConfig( pDev, 0, VL53L0X_DEVICEMODE_SINGLE_RANGING, VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY, VL53L0X_INTERRUPTPOLARITY_LOW );
}


void DistSensorStart( VL53L0X_DEV pDev )
{
	VL53L0X_StartMeasurement( pDev );
}


int DistSensorRead( VL53L0X_DEV pDev )
{
	VL53L0X_RangingMeasurementData_t read;

    VL53L0X_GetRangingMeasurementData( pDev, &read );

    /* Clear the interrupt */
    VL53L0X_ClearInterruptMask( pDev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY );

    return read.RangeMilliMeter;
}


void DistI2cInterruptHandler( VL53L0X_DEV pDev )
{
    pDev->bI2cReady = TRUE;
}


/*****************************************************************************
 Callback functions
******************************************************************************/


/*****************************************************************************
 Local functions
******************************************************************************/


/*****************************************************************************
 Interrupt functions
******************************************************************************/
 

















