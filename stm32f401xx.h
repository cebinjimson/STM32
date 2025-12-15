/*
 * stm32f4xx.h
 *
 *  Created on: Dec 9, 2025
 *      Author: cebin
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_
#include<stdint.h>
/*
base address of SRAM and Flash memories
*/
#define FLASH_BASEADDR  (uint32_t)0x00000000 /*flash address*/
#define SRAM1_BASEADDR  0x20000000U /*SRAM1 address*/
#define SRAM2_BASEADDR  0x20001C00U /*SRAM2 address*/
#define ROM             0x1FFF0000U /*ROM(Programmable memory) address*/
#define SRAM            SRAM1_BASEADDR
/*
base address of buses
*/
#define APB1PERIPH_BASE 0x40000000U /*APB1 address*/
#define APB2PERIPH_BASE 0x40010000U /*APB2 address*/
#define AHB1PERIPH_BASE 0x40020000U /*AHB1 address*/
#define AHB2PERIPH_BASE 0x50000000U /*AHB2 address*/
/*
base address of peripherals hanging on AHB1 bus
*/
#define GPIOA_BASEADDR  (AHB1PERIPH_BASE + 0x0000) /* (AHBI address + offset)*/
#define GPIOB_BASEADDR  (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR  (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR  (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR  (AHB1PERIPH_BASE + 0x1000)
#define GPIOH_BASEADDR  (AHB1PERIPH_BASE + 0x1C00)
#define RCC_BASEADDR    (AHB1PERIPH_BASE + 0x3800)
/*
base address of peripherals hanging on APB1 bus
*/
#define I2C1_BASEADDR   (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR   (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR   (APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR   (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR   (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR (APB1PERIPH_BASE + 0x4400)

/*
base address of peripherals hanging on APB2 bus
*/
#define EXTI_BASEADDR   (APB2PERIPH_BASE + 0x3C00)
#define SPI1_BASEADDR   (APB2PERIPH_BASE + 0x3000)
#define SYSCFG_BASEADDR (APB2PERIPH_BASE + 0x3800)
#define USART1_BASEADDR   (APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR   (APB2PERIPH_BASE + 0x1400)

/*          Peripheral register definition structure
 */
typedef struct
{
  volatile uint32_t MODER;  /* GPIO port mode register   Address Offset:0x00 */
  volatile uint32_t OTYPER; /* GPIO port output type register   Address Offset:0x04 */
  volatile uint32_t OSPEEDR;/* GPIO port output speed register    Address Offset:0x08 */
  volatile uint32_t PUPDR;  /* GPIO port pull-up/pull-down register     Address Offset:0x0C */
  volatile uint32_t IDR;    /* GPIO port input data register    Address Offset:0x10 */
  volatile uint32_t ODR;    /* GPIO port output data register    Address Offset:0x14 */
  volatile uint32_t BSRR;   /* GPIO port bit set/reset register   Address Offset:0x18 */
  volatile uint32_t LCKR;   /* GPIO port configuration lock register   Address Offset:0x1C */
  volatile uint32_t AFR[2];   /* GPIO alternate function register   Address Offset:0x20 */

}GPIO_RegDef_t;

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t PLLCFGR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t AHB1RSTR;
  volatile uint32_t AHB2RSTR;
  uint32_t RESERVED0[2];
  volatile uint32_t APB1RSTR;
  volatile uint32_t APB2RSTR;
  uint32_t RESERVED1[2];
  volatile uint32_t AHB1ENR;
  volatile uint32_t AHB2ENR;
  uint32_t RESERVED2[2];
  volatile uint32_t APB1ENR;
  volatile uint32_t APB2ENR;
  uint32_t RESERVED3[2];
  volatile uint32_t AHB1LPENR;
  volatile uint32_t AHB2LPENR;
  uint32_t RESERVED4[2];
  volatile uint32_t APB1LPENR;
  volatile uint32_t APB2LPENR;
  uint32_t RESERVED5[2];
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
  uint32_t RESERVED6[2];
  volatile uint32_t SSCGR;
  volatile uint32_t PLLI2SCFGR;
  uint32_t RESERVED7;
  volatile uint32_t DCKCFGR;
}RCC_RegDef_t;


/*        Peripheral definitions
 */
#define GPIOA    ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB    ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC    ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD    ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE    ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH    ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC    ((RCC_RegDef_t*)RCC_BASEADDR)

/*        Clock enable for GPIOx Peripherals
*/
#define GPIOA_PCLK_EN()  (RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN()  (RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN()  (RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()  (RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()  (RCC->AHB1ENR |=(1<<4))
#define GPIOH_PCLK_EN()  (RCC->AHB1ENR |=(1<<7))

/*        Clock enable for I2Cx Peripherals
*/

#define I2C1_PCLK_EN()     (RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN()     (RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN()     (RCC->APB1ENR |=(1<<23))

/*        Clock enable for SPIx Peripherals
*/

#define SPI1_PCLK_EN()     (RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN()     (RCC->APB1ENR |=(1<<14))
#define SPI3_PCLK_EN()     (RCC->APB1ENR |=(1<<15))

/*        Clock enable for USARTx Peripherals
*/

#define USART1_PCLK_EN()     (RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN()     (RCC->APB1ENR |=(1<<17))
#define USART6_PCLK_EN()     (RCC->APB2ENR |=(1<<5))

/*        Clock enable for SYSCFG Peripherals
*/
#define SYSCFG_PCLK_EN     (RCC->APB1ENR |=(1<<14))

/*        Clock Disble for GPIOx Peripherals
*/
#define GPIOA_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<4))
#define GPIOH_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<7))

/*        Clock Disble for I2Cx Peripherals
*/
#define I2C1_PCLK_DI()     (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()     (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()     (RCC->APB1ENR &= ~(1<<23))

/*        Clock Disble for SPIx Peripherals
*/
#define SPI1_PCLK_DI()     (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()     (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()     (RCC->APB1ENR &= ~(1<<15))

/*        Clock Disable for USARTx Peripherals
*/
#define USART1_PCLK_DI()     (RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()     (RCC->APB1ENR &= ~(1<<17))
#define USART6_PCLK_DI()     (RCC->APB2ENR &= ~(1<<5))

/*macros to reset GPIOx peripherals
 *
 */
#define GPIOA_REG_RESET()    do{(RCC->AHB1RSTR |=(1<<0));   (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()    do{(RCC->AHB1RSTR |=(1<<0));   (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOC_REG_RESET()    do{(RCC->AHB1RSTR |=(1<<0));   (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOD_REG_RESET()    do{(RCC->AHB1RSTR |=(1<<0));   (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOE_REG_RESET()    do{(RCC->AHB1RSTR |=(1<<0));   (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOH_REG_RESET()    do{(RCC->AHB1RSTR |=(1<<0));   (RCC->AHB1RSTR &= ~(1<<0));}while(0)

/* Some Generic macros */
#define ENABLE 1
#define DISABLE 0
#define SET   ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

#include "stm32f401xx_gpio_driver.h"

#endif /* INC_STM32F401XX_H_ */
