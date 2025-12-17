/*
 * 003LED.c
 *
 *  Created on: Dec 17, 2025
 *      Author: cebin
 */

#include <stdint.h>
#include "stm32f401xx.h"
#define BTN_PRESSED HIGH
#define HIGH 1
void delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}
int main (void)
{
	GPIO_Handle_t GpioLed, GPIOBtn;
	GpioLed.pGPIOx=GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);

	GPIOBtn.pGPIOx=GPIOB;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB,ENABLE);
	GPIO_Init(&GPIOBtn);
	while(1){
		if(GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_12)==BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_8);
		}
	}
	return 0;
}
