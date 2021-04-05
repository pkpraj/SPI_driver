/*
 * stm32f412_gpio_driver.c
 *
 *  Created on: Mar 30, 2021
 *      Author: 12406
 */

#include "stm32f412_gpio_driver.h"

/*********************************************************************
 * 	  - GPIO_PeriClockControl
 /********************************************************************/

void GPIO_PeriClockEnDi(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
	}
}


/*********************************************************************
 *    		  GPIO_Init
 */
void GPIO_Initialization(GPIO_Handle_t *pGPIOHandle)
{
	 uint32_t temp=0; //temp. register

	 //enable the peripheral clock

	 GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1 . configure the mode of gpio pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode


	}
}

void GPIO_DeInitizalization(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
}
