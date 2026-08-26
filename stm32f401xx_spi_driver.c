/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: Dec 23, 2025
 *      Author: cebin
 */
#include <stdint.h>
#include "stm32f401xx_spi_driver.h"
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE){
			if(pSPIx==SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if(pSPIx==SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if(pSPIx==SPI3)
			{
				SPI3_PCLK_EN();
			}
			else if(pSPIx==SPI4)
			{
				SPI4_PCLK_EN();
			}
		}else
		{
			if(pSPIx==SPI1)
			{
				SPI1_PCLK_DI();
			}
			else if(pSPIx==SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if(pSPIx==SPI3)
			{
				SPI3_PCLK_DI();
			}
			else if(pSPIx==SPI4)
			{
				SPI4_PCLK_DI();
			}
		}
}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg=0;
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	//1. configure the device mode
	tempreg |=pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	//2. configure the BUS CONFIG
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//Bidi mode should be cleared
		tempreg &= ~(1<<15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//Bidi mode should be set
		tempreg |= (1<<15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<<15);
		//RXONLY bit must be set
		tempreg |= (1<<10);
	}
	//3.Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;
	//4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;
	//5. CONFIGURE THE CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	//6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
	//7. Configure the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;
	pSPIHandle ->pSPIx->CR1=tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx==SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx==SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx==SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx==SPI4)
	{
		SPI4_REG_RESET();
	}

}
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len)
{
	while(Len>0)
	{
		//1.wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)== FLAG_RESET);
		//2.CHECK THE DFF BIT IN CR1
		if(pSPIx->CR1 & (1 <<SPI_CR1_DFF))
		{
			//16 but DFF
			//1.LOAD THE DATA IN TO THE DR
			pSPIx->DR=*((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR=*pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len)
{
	while(Len>0)
	{
		//1.wait until RXNXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNXE_FLAG)== FLAG_RESET);
		//2.CHECK THE DFF BIT IN CR1
		if(pSPIx->CR1 & (1 <<SPI_CR1_DFF))
		{
			//16 but DFF
			//1.LOAD THE DATA from DR TO RX BUFFER
			*((uint16_t*)pRxBuffer)=pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			//8 bit DFF
			*(pRxBuffer)=pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=(1<<SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=(1<<SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=(1<<SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
	}
}
