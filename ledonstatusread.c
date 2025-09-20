#include "stm32f4xx.h"
#include "main.h"
int main (void){
	uint32_t *pClkCtrl=(uint32_t*)0x40023830;
	uint32_t *pPinAMode= (uint32_t*)0x40020000;
	uint32_t *pPinAOUTData= (uint32_t*)0x40020014;
	uint32_t *pPinBMode= (uint32_t*)0x40020400;
	uint32_t *pPinBINData= (uint32_t*)0x40020410;
	*pClkCtrl|=(1<<0); //enabled clock for GPIOA
	*pClkCtrl|=(1<<1); //enabled clock for GPIOB
    *pPinAMode&=~(3<<10); //cleared bits at pin 5 of GPIOA
	*pPinAMode|=(1<<10); //SET 1 at pin 5 of GPIOA(output mode)
	*pPinBMode&=~(3<<10); //cleared bits at pin 5 of GPIOB
	*pPinBMode|=(0<<10); //SET 0 at pin 5 of GPIOB(input mode)
	while(1){
		uint8_t pinStatus = ((*pPinBINData) >> 5) & 0x01; //To read status of pin 5 of portB
		if(pinStatus){
			*pPinAOUTData|=(1<<5);
			}
		else{
			*pPinAOUTData &= ~(1 << 5);;
	}
	}
}
