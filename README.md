# STM32
STM32F401RE Bare-Metal Drivers
A repository containing MCU-specific header files and Peripheral Driver APIs written from scratch for the STM32F401RE microcontroller (Nucleo-64 board).

## Overview
This project aims to implement low-level device drivers without relying on standard libraries like STM32 HAL or LL. By interacting directly with the processor's memory-mapped registers, this repository serves as a deep dive into the internal architecture of the ARM Cortex-M4 and the STM32 peripheral system.

## Key Goals:

** Create a complete device header file (stm32f401xx.h) defining base addresses and register bit definitions.

** Implement modular, reusable Driver APIs for key peripherals (GPIO, SPI, I2C, USART).

Optimize for code size and execution speed.

🛠 Hardware Used
Microcontroller: STM32F401RE (ARM Cortex-M4)

Development Board: STM32 Nucleo-64 (NUCLEO-F401RE)

IDE: STM32CubeIDE / Keil uVision / Eclipse
