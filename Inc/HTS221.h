#ifndef HTS221_H
#define HTS221_H

#include <stdint.h>

#include "stm32l4xx_hal.h"

#ifdef __cplusplus
extern "C"{
#endif

struct HTS221
{
    HTS221(I2C_HandleTypeDef* i2c);
    
    uint8_t who_am_i(uint8_t* who);
    uint8_t encender(void);
    uint8_t calibrar_temperatura(void);
    uint8_t calibrar_humedad(void);
    int leer_temp(void);
    int leer_humedad(void);

    /*Valores de los registros de calibracion*/
    uint16_t T0_degC_x8{0}, T1_degC_x8{0}, H0_rH_x2{0}, H1_rH_x2{0};
    uint16_t H0_OUT{0}, H1_OUT{0};
    int16_t T0_OUT{0}, T1_OUT{0};

    /*y = mx + b para las rectas de temperatura y humedad*/
    float mT, bT, mH, bH;
    short temperatura, humedad;
  
    I2C_HandleTypeDef *hi2c;
};

void operator<<(UART_HandleTypeDef huart, HTS221 hts);


#ifdef __cplusplus
}
#endif

#endif
