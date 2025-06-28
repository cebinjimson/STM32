
#include "stm32f4xx.h"
void delayms(int n);
int main(void) {
	RCC->AHB1ENR |=1;
	GPIOA->MODER&=~0x00000C00;
	GPIOA->MODER |=0x00000400;
	while(1){
		delayms(50);
		GPIOA->ODR ^= (1 << 5);
		delayms(50);
	}
}
void delayms(int n){
    int i,j;
    for(i = 0; i < n; i++)
    	for(j=0;j<3195;j++);
}
