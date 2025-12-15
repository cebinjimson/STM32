/*
 * stm32f401xx_gpio_drivers.h
 *
 *  Created on: Dec 10, 2025
 *      Author: cebin
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_
#include "stm32f401xx.h"

typedef struct
{ 	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;    //possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;   //possible values from @GPIO_OP_MODE
	uint8_t GPIO_PinPuPdControl;  //possible values from @GPIO_SPEED
	uint8_t GPIO_PinOPType;    //possible values from @GPIO_PUPD
	uint8_t GPIO_PinAltFunMode;


}GPIO_PinConfig_t;

typedef struct
{
    //pointer to hold the base address of GPIO peripheral
	GPIO_RegDef_t *pGPIOx; //this holds the base address of gpio port to which the pin belong
	GPIO_PinConfig_t GPIO_PinConfig; //This holds GPIO pin configuration settings
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin Numbers
 */
#define GPIO_PIN_NO_0    0
#define GPIO_PIN_NO_1    1
#define GPIO_PIN_NO_2    2
#define GPIO_PIN_NO_3    3
#define GPIO_PIN_NO_4    4
#define GPIO_PIN_NO_5    5
#define GPIO_PIN_NO_6    6
#define GPIO_PIN_NO_7    7
#define GPIO_PIN_NO_8    8
#define GPIO_PIN_NO_9    9
#define GPIO_PIN_NO_10   10
#define GPIO_PIN_NO_11   11
#define GPIO_PIN_NO_12   12
#define GPIO_PIN_NO_13   13
#define GPIO_PIN_NO_14   14
#define GPIO_PIN_NO_15   15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN     0
#define GPIO_MODE_OUT    1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4
#define GPIO_MODE_IT_RT  5
#define GPIO_MODE_IT_RFT 6

/*
 * @GPIO_OP_MODE
 * GPIO pin possible OUTPUT modes
 */
#define GPIO_OP_TYPE_PP  0
#define GPIO_OP_TYPE_OD  1

/*
 * @GPIO_SPEED
 * GPIO pin possible OUTPUT SPEED
 */
#define GPIO_SPEED_LOW     0
#define GPIO_SPEED_MEDIUM  1
#define GPIO_SPEED_FAST    2
#define GPIO_SPEED_HIGH    3

/*
 * @GPIO_PUPD
 * GPIO pin PULLUP/PULLDOWN configuration macros
 */
#define GPIO_NO_PUPD        0
#define GPIO_PIN_PU         1
#define GPIO_PIN_PD         2


/* Init And Deinit */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
/* Peripheral Clock Setup */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);
/* Data Read and Write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
/* IRQ Configuration and ISR Handling */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

/* Some Generic macros */
#define ENABLE 1
#define DISABLE 0
#define SET   ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
