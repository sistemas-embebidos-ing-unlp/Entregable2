/*
 * tomasitos_library.c
 *
 *  Created on: Jun 12, 2024
 *      Author: Ferreyra
 */

#include "tomasitos_library.h"

uint16_t SW1_ant=1;   //Si o si arranco con pull up
uint16_t SW2_ant=1;   //Si o si arranco con pull up


Encoder_Status Encoder_Get_Status(){
	uint16_t SW1;
	uint16_t SW2;
	int ds1;
	int ds2;
	SW1=HAL_GPIO_ReadPin(ENCODER1);
	SW2=HAL_GPIO_ReadPin(ENCODER2);
	Encoder_Status sts=Neutral;
	ds1=SW1-SW1_ant;
	ds2=SW2-SW2_ant;
	if(ds1==-1 && SW2==0){
		sts=Incremented;
	}
	if(ds2==-1 && SW1==0){
		sts=Decremented;
	}
	SW1_ant=SW1;
	SW2_ant=SW2;
	return sts;
}

void PrintBufferAsGraph(uint32_t* buffer, size_t length,int amplitud,int frecuencia,uint8_t* x_p){
	char buffer_char[10];
	SSD1306_Fill(SSD1306_COLOR_BLACK);
    uint32_t maxValue = 0;
    for (size_t i = 1; i < length; i++) {
        if (buffer[i] > maxValue) {
            maxValue = buffer[i];
            *x_p=i;
        }
    }
    if(maxValue==0){
    	maxValue=1;
    }
    if(maxValue<400){
    	maxValue=400;
    }

    for (size_t i = 0; i < length; i++) {
        uint8_t scaledValue = (uint8_t)((buffer[i] * SSD1306_HEIGHT) / maxValue);
        for (uint8_t j = 0; j < scaledValue; j++) {
            SSD1306_DrawPixel(i, SSD1306_HEIGHT - 1 - j, SSD1306_COLOR_WHITE);
        }
    }

    SSD1306_GotoXY(90, 0);
    sprintf(buffer_char, "%dmV", amplitud);
    SSD1306_Puts(buffer_char, &Font_7x10, SSD1306_COLOR_BLACK);
    SSD1306_GotoXY(90, 10);
    sprintf(buffer_char, "%dHz", frecuencia);
    SSD1306_Puts(buffer_char, &Font_7x10, SSD1306_COLOR_BLACK);

}

void ImprimirLinea(char *s, uint8_t linea) {
SSD1306_GotoXY(0, 0 + 14 * (linea - 1));
SSD1306_Puts("                    ", &Font_7x10, 1);
SSD1306_GotoXY(0, 0 + 14 * (linea - 1));
SSD1306_Puts(s, &Font_7x10, 1);
SSD1306_UpdateScreen();
}

uint32_t antirebote_ftn(uint32_t antirebote_f){
	if(antirebote_f>0){
		antirebote_f++;
		if(antirebote_f>302){
			antirebote_f=0;
		}
	}
	return antirebote_f;
}

uint32_t set_PWM_period(uint32_t period_P){
	if(period_P<1){
		period_P=1;
	}
	if(period_P>40){
		period_P=40;
	}
	if(period_P>0&&period_P<10){
		period_P=47*period_P;
		period_P=450-period_P;
	}
	else{
		period_P=period_P-9;
		period_P = 1.3*period_P*period_P-88.2*period_P+2006.6;
		period_P=period_P/100;
	}
	return period_P;
}
