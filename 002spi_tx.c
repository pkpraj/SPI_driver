/*
 * 002spi_tx.c
 *
 *  Created on: Mar 30, 2021
 *      Author: 12406
 */

#include "stm32f412_spi_driver.h"

/*
 * PA6 --> SPI1_MISO
 * PA7 --> SPI1_MOSI
 * PA5 -> SPI1_SCLK
 * PA4 --> SPI2_NSS
 * ALT function mode
 */

void SPI1_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	//GPIO_Init(&SPIPins);


	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	//GPIO_Init(&SPIPins);


}

void SPI1_Initialization(void)
{
}

int main(void){

	char user_data[] = "Hello world";

		//this function is used to initialize the GPIO pins to behave as SPI1 pins
		SPI1_GPIOInits();

		//This function is used to initialize the SPI1 peripheral parameters
		SPI1_Inits();

		//this makes NSS signal internally high and avoids MODF error
		SPI_SSIConfig(SPI1,ENABLE);

		//enable the SPI1 peripheral
		SPI_PeripheralControl(SPI1,ENABLE);

		//to send data
		SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI1,SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI1,DISABLE);

		while(1);

		return 0;
}
