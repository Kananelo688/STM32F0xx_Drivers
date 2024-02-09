/************************************************************************************************************
 * Source file with main function for testing GPIO driver functions											*
 * **********************************************************************************************************
 * */

#include "sm32f4xx_gpio_driver.h"
#include "stm32f4xx_drivers.h"



uint32_t var = 0x4800100D;


int  int main(void)
{

	RCC_AHB1_Clock_Enable(RCC_AHB1ENR_GPIOA_EN);
	
	return 0;
}