/*
 * tomasitos_library.h
 *
 *  Created on: Jun 12, 2024
 *      Author: Ferreyra
 */

#ifndef INC_TOMASITOS_LIBRARY_H_
#define INC_TOMASITOS_LIBRARY_H_

#include "stm32f1xx_hal.h"
#include "ssd1306.h"
#include "fonts.h"

#define ENCODER1 GPIOA, GPIO_PIN_0
#define ENCODER2 GPIOA, GPIO_PIN_1

typedef enum{Incremented,Decremented,Neutral}Encoder_Status;

Encoder_Status Encoder_Get_Status();
void PrintBufferAsGraph(uint32_t* buffer, size_t length,int amplitud,int frecuencia,uint8_t* x_p);
void ImprimirLinea(char *s, uint8_t linea);
uint32_t antirebote_ftn(uint32_t antirebote_f);
uint32_t set_PWM_period(uint32_t period_P);

#endif /* INC_TOMASITOS_LIBRARY_H_ */
