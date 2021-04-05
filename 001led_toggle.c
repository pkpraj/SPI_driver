/*
 * 001led_toggle.c
 *
 *  Created on: Mar 30, 2021
 *      Author: 12406
 */

#include "stm32f412.h"
#include "stm32f412_gpio_driver.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

int main(void)
{
	GPIO_Handle_t GPIOled;
	GPIOled.pGPIOx = GPIOE;
	GPIOled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	//clock....!!
	GPIO_Init(&GPIOled);

	while(1)
		{
			GPIO_ToggleOutputPin(GPIOE,GPIO_PIN_NO_0);
			delay();
		}

	return 0;
}
