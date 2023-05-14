/* 1. This file is from STM32. It is modified to use our I2c driver.
   2. Most of the code styles still remains from STM32
*/

#include <Common.h>
#include <string.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"


static uint8_t _I2CBuffer[64];


static BOOL _I2cWrite( 	int			nI2c,
						uint8_t 	nSlaveAddr,
						uint8_t 	Register,
						void const 	*pData,
						uint8_t 	nByte );

static BOOL _I2cRead( 	int 		nI2c,
						uint8_t 	nSlaveAddr,
						uint8_t 	Register,
						void 		*pData,
						uint8_t 	nByte );

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    if (count > sizeof(_I2CBuffer) )
    {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }
    
    memcpy( _I2CBuffer, pdata, count );

    //Dev->bI2cReady = FALSE;
    _I2cWrite(
        Dev->nI2c,
        Dev->I2cDevAddr,
        index,
        _I2CBuffer,
        count );
    //while( FALSE == Dev->bI2cReady ){}

    return Status;
}


// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    //Dev->bI2cReady = FALSE;
    _I2cRead(
        Dev->nI2c,
        Dev->I2cDevAddr,
        index,
        _I2CBuffer,
        count );
   // while( FALSE == Dev->bI2cReady ){}

    memcpy( pdata, _I2CBuffer, count );

    return Status;
}


VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    _I2CBuffer[0] = data;

    //Dev->bI2cReady = FALSE;
    _I2cWrite(
        Dev->nI2c,
        Dev->I2cDevAddr,
        index,
        _I2CBuffer,
        1U );
   // while( FALSE == Dev->bI2cReady ){}

    return Status;
}


VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    _I2CBuffer[0] = data >> 8;
    _I2CBuffer[1] = data & 0x00FF;

    //Dev->bI2cReady = FALSE;
    _I2cWrite(
        Dev->nI2c,
        Dev->I2cDevAddr,
        index,
        _I2CBuffer,
        2U );
    //while( FALSE == Dev->bI2cReady ){}

    return Status;
}


VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    _I2CBuffer[0] = (data >> 24) & 0xFF;
    _I2CBuffer[1] = (data >> 16) & 0xFF;
    _I2CBuffer[2] = (data >> 8)  & 0xFF;
    _I2CBuffer[3] = (data >> 0 ) & 0xFF;

    //Dev->bI2cReady = FALSE;
    _I2cWrite(
    	Dev->nI2c,
        Dev->I2cDevAddr,
        index,
        _I2CBuffer,
        4U );
   // while( FALSE == Dev->bI2cReady ){}

    return Status;
}


VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    Status = VL53L0X_RdByte(Dev, index, _I2CBuffer);
    if (Status) 
    {
        return Status;
    }
    _I2CBuffer[0] = (_I2CBuffer[0]  & AndData) | OrData;
    Status = VL53L0X_WrByte(Dev, index, _I2CBuffer[0]);

    return Status;
}


VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;


    //Dev->bI2cReady = FALSE;
    _I2cRead(
    	Dev->nI2c,
        Dev->I2cDevAddr,
        index,
        _I2CBuffer,
        1U );
    //while( FALSE == Dev->bI2cReady ){}

   *data = _I2CBuffer[0];

    return Status;
}


VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    //Dev->bI2cReady = FALSE;
    _I2cRead(
    	Dev->nI2c,
        Dev->I2cDevAddr,
        index,
        _I2CBuffer,
        2U );
    //while( FALSE == Dev->bI2cReady ){}

    *data = ((uint16_t)_I2CBuffer[0]<<8) + (uint16_t)_I2CBuffer[1];

    return Status;
}


VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    //Dev->bI2cReady = FALSE;
    _I2cRead(
    	Dev->nI2c,
        Dev->I2cDevAddr,
        index,
        _I2CBuffer,
        4U );
    //while( FALSE == Dev->bI2cReady ){}

    *data = ((uint32_t)_I2CBuffer[0]<<24) + ((uint32_t)_I2CBuffer[1]<<16) + ((uint32_t)_I2CBuffer[2]<<8) + (uint32_t)_I2CBuffer[3];

    return Status;
}


static BOOL
_I2cWrite(
	int			nI2c,
	uint8_t 	nSlaveAddr,
	uint8_t 	Register,
	void const 	*pData,
	uint8_t 	nByte
	)
{
	int i = 0;
	uint8_t *p = (uint8_t *)pData;
	esp_err_t ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (nSlaveAddr<<1) | I2C_MASTER_WRITE, TRUE);
	i2c_master_write_byte(cmd, Register, TRUE);
	for( i=0; i<nByte; i++ )
	{
		i2c_master_write_byte(cmd, *p++, TRUE);
	}
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(nI2c, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return ((ret==ESP_OK)? TRUE : FALSE);
}

static BOOL
_I2cRead(
	int			nI2c,
	uint8_t 	nSlaveAddr,
	uint8_t 	Register,
	void 		*pData,
	uint8_t 	nByte
	)
{
	int i = 0;
	esp_err_t ret;
	uint8_t *p = (uint8_t *)pData;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (nSlaveAddr<<1U) | I2C_MASTER_WRITE, TRUE);
	i2c_master_write_byte(cmd, Register, TRUE);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (nSlaveAddr<<1U) | I2C_MASTER_READ, TRUE);

	for( i=0; i<(nByte-1); i++ )
	{
		i2c_master_read_byte(cmd, p++, FALSE);
	}

	i2c_master_read_byte(cmd, p, TRUE);

	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(nI2c, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return ((ret==ESP_OK)? TRUE : FALSE);
}


VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    UNUSE( Dev );
  
    esp_rom_delay_us(2000);

    return status;
}

//end of file
