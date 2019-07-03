#include "HTS221.h"

#include <stdio.h>

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#define rCTRL_REG1      (0x20)

extern I2C_HandleTypeDef hi2c2;

static uint8_t rWHO_AM_I = (0x0F); //0 means do not autoincrement

//static const	uint8_t rCTRL_REG1 = (0x20);
static uint8_t m_encender[2] = {rCTRL_REG1, 0x83};

/*recuerda, los bits menos significativos son la dirección del registro. El MSb permite address autoincrement*/
static uint8_t rTEMP_OUT_L = 0x2A | 0x80; //este bitwise or es para setear el MSb, para permitir address autoincrement

/*Valores de los registros de calibracion*/
uint16_t T0_degC_x8, T1_degC_x8;
int16_t T0_OUT, T1_OUT;

/*y = mx + b*/
static float mT, bT;


uint8_t hts221_who_am_i(void)
{
	static uint8_t rxbuf[1];
	HAL_StatusTypeDef res;
	res = HAL_I2C_Master_Transmit(&hi2c2, 0xBF, &rWHO_AM_I, 1, 10);
	res = HAL_I2C_Master_Receive(&hi2c2, 0xBE, rxbuf, 1, 10);
	if(res != HAL_OK)
		return 0xFF;
	return rxbuf[0];
}


uint8_t hts221_encender(void)
{
	static uint8_t rxbuf[1];
	HAL_StatusTypeDef res;
	res = HAL_I2C_Master_Transmit(&hi2c2, 0xBF, m_encender, 2, 10);
	res = HAL_I2C_Master_Receive(&hi2c2, 0xBE, rxbuf, 1, 10);
	if(res != HAL_OK)
		return 0xFF;
	return rxbuf[0];
}

uint8_t hts221_iniciar_calibracion(void)
{
	static uint8_t rxbuf[2];
	const uint8_t rT0_degC_x8 = 0x32 | 0x80;
	const uint8_t rT1_T0_msbs = 0x35;
	const uint8_t rT0_OUT = 0x3C | 0x80;
	const uint8_t rT1_OUT = 0x3E | 0x80;
	HAL_StatusTypeDef res;
        
        res = HAL_OK;
	
	/*leemos los registros 32 y 33, los de temperatura x8*/
	res |= HAL_I2C_Master_Transmit(&hi2c2, 0xBF, (uint8_t*)&rT0_degC_x8, 1, 10);
	res |= HAL_I2C_Master_Receive(&hi2c2, 0xBE, rxbuf, 2, 10);
	
	/*leemos el registro 35, los 2 bits mas significativos de los T* degc*/
	uint8_t msbs=0;
	res |= HAL_I2C_Master_Transmit(&hi2c2, 0xBF, (uint8_t*)&rT1_T0_msbs, 1, 10);
	res |= HAL_I2C_Master_Receive(&hi2c2, 0xBE, &msbs, 1, 10);
	
	T0_degC_x8 = rxbuf[0] + ((msbs & 0x3) << 8);
	T1_degC_x8 = rxbuf[1] + ((msbs & 0xC) << 6); //si es 6, pues están dos bits a la izq, y si es C
	
	/*leemos los registros T0 y T1 de calibración, que están asociados con los T*_x8 que acabamos de leer*/
	rxbuf[0] = rxbuf[1] = 0;
	res |= HAL_I2C_Master_Transmit(&hi2c2, 0xBF, (uint8_t*)&rT0_OUT, 1, 10);
	res |= HAL_I2C_Master_Receive(&hi2c2, 0xBE, rxbuf, 2, 10);
	T0_OUT = rxbuf[0] | (rxbuf[1] << 8);
	
	rxbuf[0] = rxbuf[1] = 0;
	res |= HAL_I2C_Master_Transmit(&hi2c2, 0xBF, (uint8_t*)&rT1_OUT, 1, 10);
	res |= HAL_I2C_Master_Receive(&hi2c2, 0xBE, rxbuf, 2, 10);
	T1_OUT = rxbuf[0] | (rxbuf[1] << 8);
	
	/*calculamos la pendiente y la ordenada al origen de la recta que describe el sensor*/
	mT = (float)(T1_OUT - T0_OUT)/(float)(T1_degC_x8 - T0_degC_x8);
	bT = (float)(T0_OUT - mT*T0_degC_x8);
	
	if( (res | HAL_OK) > 0)
		return 0xFF;
	
	return res;
}

float hts221_leer_temp(void)
{
	static uint8_t rxbuf[2];
	HAL_StatusTypeDef res;
	res = HAL_I2C_Master_Transmit(&hi2c2, 0xBF, &rTEMP_OUT_L, 1, 10);
	res = HAL_I2C_Master_Receive(&hi2c2, 0xBE, rxbuf, 2, 10);
	if(res != HAL_OK)
		return 0xFF;
	int16_t temp = 0; //tiene signo. El MSb es el del signo
	temp = rxbuf[0] | (rxbuf[1] << 8);
	
	/*y = mx + b      =>      x = (y - b)/m*/
	float x = (float)(temp - bT)/mT;
	
	/*recuerda que ese valor simboliza la temperatura en °C * 8*/
	return x/8;
}





