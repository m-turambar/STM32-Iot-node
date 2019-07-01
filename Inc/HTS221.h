#ifndef HTS221_H
#define HTS221_H

#include <stdint.h>

uint8_t hts221_who_am_i(void);
uint8_t hts221_encender(void);

uint8_t hts221_iniciar_calibracion(void);
float hts221_leer_temp(void);






#endif
