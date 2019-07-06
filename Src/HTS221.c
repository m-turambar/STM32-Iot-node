#include "HTS221.h"

#include <stdio.h>

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

/*Los registros con bitwise or 0x80 son para setear el MSb, para permitir address autoincrement al leer con I2C
*/
#define ODR0 					(1 << 0)
#define ODR1					(1 << 1)
#define PD						(1 << 7)

static int ADDR_W =				(0xBF);
static int ADDR_R =				(0xBE);

#define    rCTRL_REG1			(0x20)
static int rWHO_AM_I =			(0x0F);

static int rTEMP_OUT_L = 		(0x2A | 0x80);
static int rT0_degC_x8 = 		(0x32 | 0x80);
static int rT1_T0_msbs = 		(0x35);
static int rT0_OUT = 			(0x3C | 0x80);
static int rT1_OUT = 			(0x3E | 0x80);

static int rHUM_OUT_L =			(0x2A | 0x80);
static int rH0_x2 =				(0x32 | 0x80);
static int rH0_OUT = 			(0x3C | 0x80);
static int rH1_OUT = 			(0x3E | 0x80);

/*mensaje para encender el HTS221*/
static uint8_t m_encender[2] = {rCTRL_REG1, PD | ODR1 | ODR0};

extern I2C_HandleTypeDef hi2c2;



/*recuerda, los bits menos significativos son la dirección del registro. El MSb permite address autoincrement*/

/*Valores de los registros de calibracion*/
uint16_t T0_degC_x8, T1_degC_x8;
int16_t T0_OUT, T1_OUT;
uint16_t H0_OUT, H1_OUT;
uint8_t H0_x2, H1_x2;

/*y = mx + b*/
float mT, bT, mH, bH;

float temperatura, humedad; //se actualizan con cada lectura


uint8_t who_am_i(uint8_t* who)
{
	int res = HAL_OK;
	res |= HAL_I2C_Master_Transmit(&hi2c2, ADDR_W, (uint8_t*)&rWHO_AM_I, 1, 10);
	res |= HAL_I2C_Master_Receive(&hi2c2, ADDR_R, who, 1, 10);
	return res;
}


uint8_t encender(void)
{
	int res;
	res = HAL_I2C_Master_Transmit(&hi2c2, ADDR_W, m_encender, 2, 10);
	return res;
}

uint8_t calibrar_temp(void)
{
	static uint8_t rxbuf[2] = {0};
	
	int res = HAL_OK;

	/*leemos los registros 32 y 33, los de temperatura x8*/
	res |= HAL_I2C_Master_Transmit(&hi2c2, ADDR_W, (uint8_t*)&rT0_degC_x8, 1, 10);
	res |= HAL_I2C_Master_Receive(&hi2c2, ADDR_R, rxbuf, 2, 10);

	/*leemos el registro 35, los 2 bits mas significativos de los T* degc*/
	uint8_t msbs=0;
	res |= HAL_I2C_Master_Transmit(&hi2c2, ADDR_W, (uint8_t*)&rT1_T0_msbs, 1, 10);
	res |= HAL_I2C_Master_Receive(&hi2c2, ADDR_R, &msbs, 1, 10);

	T0_degC_x8 = rxbuf[0] + ((msbs & 0x3) << 8);
	T1_degC_x8 = rxbuf[1] + ((msbs & 0xC) << 6); //si es 6, pues están dos bits a la izq, y si es C

	/*leemos los registros T0 y T1 de calibración, que están asociados con los T*_x8 que acabamos de leer*/
	rxbuf[0] = rxbuf[1] = 0;
	res |= HAL_I2C_Master_Transmit(&hi2c2, ADDR_W, (uint8_t*)&rT0_OUT, 1, 10);
	res |= HAL_I2C_Master_Receive(&hi2c2, ADDR_R, rxbuf, 2, 10);
	T0_OUT = rxbuf[0] | (rxbuf[1] << 8);

	rxbuf[0] = rxbuf[1] = 0;
	res |= HAL_I2C_Master_Transmit(&hi2c2, ADDR_W, (uint8_t*)&rT1_OUT, 1, 10);
	res |= HAL_I2C_Master_Receive(&hi2c2, ADDR_R, rxbuf, 2, 10);
	T1_OUT = rxbuf[0] | (rxbuf[1] << 8);

	/*calculamos la pendiente y la ordenada al origen de la recta que describe el sensor*/
	mT = (float)(T1_degC_x8 - T0_degC_x8)/(float)(T1_OUT - T0_OUT);
	bT = (float)(T0_degC_x8 - mT*T0_OUT);
	
	return res;
}

int leer_temp(void)
{
	static uint8_t rxbuf[2] = {0};
	int res = HAL_OK;
	res |= HAL_I2C_Master_Transmit(&hi2c2, ADDR_W, (uint8_t*)&rTEMP_OUT_L, 1, 10);
	res |= HAL_I2C_Master_Receive(&hi2c2, ADDR_R, rxbuf, 2, 10);
	int16_t temp = 0; //tiene signo. El MSb es el del signo
	temp = rxbuf[0] | (rxbuf[1] << 8);

	/*y = mx + b      =>      x = (y - b)/m*/
	/*recuerda que ese valor simboliza la temperatura en °C * 8*/
	temperatura = (mT*temp + bT)/8;

	return res;
}

uint8_t calibrar_humedad(void)
{
	static uint8_t rxbuf[2] = {0};
	
	int res = HAL_OK;

	/*leemos los registros 32 y 33, los de temperatura x8*/
	res |= HAL_I2C_Master_Transmit(&hi2c2, ADDR_W, (uint8_t*)&rH0_x2, 1, 10);
	res |= HAL_I2C_Master_Receive(&hi2c2, ADDR_R, rxbuf, 2, 10);

	H0_x2 = rxbuf[0];
	H1_x2 = rxbuf[1];
	
	rxbuf[0] = rxbuf[1] = 0;
	res |= HAL_I2C_Master_Transmit(&hi2c2, ADDR_W, (uint8_t*)&rH0_OUT, 1, 10);
	res |= HAL_I2C_Master_Receive(&hi2c2, ADDR_R, rxbuf, 2, 10);
	H0_OUT = rxbuf[0] | (rxbuf[1] << 8);

	rxbuf[0] = rxbuf[1] = 0;
	res |= HAL_I2C_Master_Transmit(&hi2c2, ADDR_W, (uint8_t*)&rH1_OUT, 1, 10);
	res |= HAL_I2C_Master_Receive(&hi2c2, ADDR_R, rxbuf, 2, 10);
	T1_OUT = rxbuf[0] | (rxbuf[1] << 8);

	/*calculamos la pendiente y la ordenada al origen de la recta que describe el sensor*/
	mH = (float)(H1_x2 - H0_x2)/(float)(H1_OUT - H0_OUT);
	bH = (float)(H0_x2 - mH*H0_OUT);
	
	return res;
}

int leer_humedad(void)
{
	static uint8_t rxbuf[2] = {0};
	int res = HAL_OK;
	
	res |= HAL_I2C_Master_Transmit(&hi2c2, ADDR_W, (uint8_t*)&rHUM_OUT_L, 1, 10);
	res |= HAL_I2C_Master_Receive(&hi2c2, ADDR_R, rxbuf, 2, 10);
	
	int16_t temp = 0;
	temp = rxbuf[0] | (rxbuf[1] << 8);

	/*y = mx + b      =>      x = (y - b)/m*/
	/*recuerda que ese valor simboliza la temperatura en °C * 8*/
	humedad = (mH*temp + bH)/8;

	return res;
}

void imprimir_uart(void* puart)
{
	UART_HandleTypeDef* huart = (UART_HandleTypeDef*)puart;
	short tt = 0;
	short hh = 0;
	tt = (short)temperatura;
	hh = (short)humedad;
	uint8_t txbuf[4];
	txbuf[0] = '\\';
	txbuf[1] = 't';
	txbuf[2] = tt & 0xff00;
	txbuf[3] = tt & 0x00ff;
	HAL_UART_Transmit(huart, txbuf, 4, 10);
	
	txbuf[1] = 'h';
	txbuf[2] = hh & 0xff00;
	txbuf[3] = hh & 0x00ff;
	HAL_UART_Transmit(huart, txbuf, 4, 10);
}

