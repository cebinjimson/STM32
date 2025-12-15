/*
 * st32f4xx_gpio_drivers.c
 *
 *  Created on: Dec 10, 2025
 *      Author: cebin
 */
#include <stdint.h>
#include "stm32f401xx_gpio_driver.h"
/* Init And Deinit*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	/**************************************
	 * @fn         -   GPIO Init
	 * @brief      -
	 * @param[in]  -
	 * @param[in]  -
	 * @param[in]  -
	 *
	 * @return     - 	None
	 *
	 * @note       -    None
	 *
	 */

	//1. Configure the mode of gpio pin

	uint32_t temp=0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG)
	{
		//this part is non-interrupt mode
		temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &=~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER|=temp; //setting
	}else
	{
		//this part is interrupt mode
	}
	temp=0;

	//2. configure the speed

	temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &=~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR|=temp;
	temp=0;

	//3. configure the pupd settings

	temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &=~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->PUPDR|=temp;

	//4. configure the output type

	temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER &=~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER|=temp;

	//5. configure the alt functionality

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1;
		uint8_t temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		(void)temp1;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	/**************************************
		 * @fn         -   GPIO DeInit
		 * @brief      -
		 * @param[in]  -
		 * @param[in]  -
		 * @param[in]  -
		 *
		 * @return     - 	None
		 *
		 * @note       -    None
		 *
		 */
	if(pGPIOx==GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx==GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx==GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx==GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx==GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx==GPIOH)
	{
		GPIOH_REG_RESET();
	}
}
/* Peripheral Clock Setup */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi){
	/**************************************
	 * @fn         -   GPIO_PeriClockControl
	 * @brief      -   This function enables or disables peripheral clock for the given GPIO port
	 * @param[in]  -   Base Address of GPIP Peripheral
	 * @param[in]  -   ENABLE or DISABLE macros
	 * @param[in]  -
	 *
	 * @return     - 	None
	 *
	 * @note       -    None
	 *
	 */
	if(EnorDi==ENABLE){
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}else
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}

}
/* Data Read and Write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR >>PinNumber)& 0x00000001);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value=(uint16_t)pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET){
		//write 1 to output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |=(1<<PinNumber);
	}
	else{
		//write
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}
/* IRQ Configuration and ISR Handling */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnorDi)
{

}
void GPIO_IRQHandling(uint8_t PinNumber)
{

}


