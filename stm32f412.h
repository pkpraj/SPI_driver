/*
 * stm32f412.h
 *
 *  Created on: Mar 30, 2021
 *      Author: 12406
 */

#ifndef INC_STM32F412_H_
#define INC_STM32F412_H_

#include<stdint.h>

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR				0x08000000U   		/*Base address of FLASH Memory*/
#define SRAM1_BASEADDR				0x20000000U  		/*Base address of SRAM1 Memory*/
#define ROM_BASEADDR				0x1FFF0000U         /*Base address of SYSTEM Memory*/
#define SRAM 						SRAM1_BASEADDR      /*Base Address of SRAM1 is renamed as SRAM*/

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR 	 		0x40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x50000000U

/*
 * Base addresses of peripherals which are connected to AHB1 bus
 */

#define GPIOA_BASEADDR               (AHB1PERIPH_BASEADDR + 0x0000)
#define RCC_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR				 (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR				 (APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR        		 (APB2PERIPH_BASEADDR + 0x3800)

/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * Device RM
 */

typedef struct
{
	volatile uint32_t MODER;                        /*GPIO port mode register Address offset: 0x00 */
	volatile uint32_t OTYPER;                       /*GPIO port output type register Address offset: 0x04 */
	volatile uint32_t OSPEEDR;                      /*GPIO port output speed register Address offset: 0x08 */
	volatile uint32_t PUPDR;						/*GPIO port pull-up/pull-down register Address offset: 0x0C */
	volatile uint32_t IDR;							/*GPIO port input data register Address offset: 0x10 */
	volatile uint32_t ODR;							/*GPIO port output data register Address offset: 0x14 */
	volatile uint32_t BSRR;							/*GPIO port bit set/reset register Address offset: 0x18 */
	volatile uint32_t LCKR;							/*GPIO port configuration lock register Address offset: 0x1C */
	volatile uint32_t AFR[2];					    /* AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register Address offset: 0x20-0x24 */
}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;           		 /*	Address offset: 0x00 */
	volatile uint32_t PLLCFGR;      		 /* Address offset: 0x04 */
	volatile uint32_t CFGR;         		 /* Address offset: 0x08 */
	volatile uint32_t CIR;           		/*	Address offset: 0x0C */
	volatile uint32_t AHB1RSTR;      		/*	Address offset: 0x10 */
	volatile uint32_t AHB2RSTR;     		 /*	Address offset: 0x14 */
	volatile uint32_t AHB3RSTR;     		 /*	Address offset: 0x18 */
	uint32_t      RESERVED0;        		 /* Reserved, 0x1C       */
	volatile uint32_t APB1RSTR;     		 /*	Address offset: 0x20 */
	volatile uint32_t APB2RSTR;      		/*	Address offset: 0x24 */
	uint32_t      RESERVED1[2];      		/* Reserved, 0x28-0x2C  */
	volatile uint32_t AHB1ENR;      		 /* Address offset: 0x30 */
	volatile uint32_t AHB2ENR;       		/*	Address offset: 0x34 */
	volatile uint32_t AHB3ENR;       		/*	Address offset: 0x38 */
	uint32_t      RESERVED2;         		/*Reserved, 0x3C       */
	volatile uint32_t APB1ENR;       		/*	Address offset: 0x40 */
	volatile uint32_t APB2ENR;       		/*	Address offset: 0x44 */
	uint32_t      RESERVED3[2];      		/* Reserved, 0x48-0x4C  */
	volatile uint32_t AHB1LPENR;     		/*	Address offset: 0x50 */
	volatile uint32_t AHB2LPENR;     		/*	Address offset: 0x54 */
	volatile uint32_t AHB3LPENR;     		/*	Address offset: 0x58 */
	uint32_t      RESERVED4;         		/* Reserved, 0x5C      */
	volatile uint32_t APB1LPENR;     		/*	Address offset: 0x60 */
	volatile uint32_t APB2LPENR;     		/*	Address offset: 0x64 */
	uint32_t      RESERVED5[2];      		/* Reserved, 0x68-0x6C  */
	volatile uint32_t BDCR;          		/*	Address offset: 0x70 */
  	volatile uint32_t CSR;           		/* Address offset: 0x74 */
  	uint32_t      RESERVED6[2];      		/* Reserved, 0x78-0x7C    */
  	volatile uint32_t SSCGR;         		/*	Address offset: 0x80 */
    volatile uint32_t PLLI2SCFGR;    		/*	Address offset: 0x84 */
    volatile uint32_t PLLSAICFGR;    		/*	Address offset: 0x88 */
  	volatile uint32_t DCKCFGR;       		/*	Address offset: 0x8C */
  	volatile uint32_t CKGATENR;      		/*	Address offset: 0x90 */
  	volatile uint32_t DCKCFGR2;      		/*	Address offset: 0x94 */

} RCC_RegDef_t;


typedef struct
{
	volatile uint32_t CR1;        /* Address offset: 0x00 */
	volatile uint32_t CR2;        /*Address offset: 0x04 */
	volatile uint32_t SR;         /*Address offset: 0x08 */
	volatile uint32_t DR;         /*Address offset: 0x0C */
	volatile uint32_t CRCPR;      /*Address offset: 0x10 */
	volatile uint32_t RXCRCR;     /*Address offset: 0x14 */
	volatile uint32_t TXCRCR;     /*Address offset: 0x18 */
	volatile uint32_t I2SCFGR;    /*Address offset: 0x1C */
	volatile uint32_t I2SPR;      /*Address offset: 0x20 */
} SPI_RegDef_t;


/*
 * peripheral definitions ( Peripheral base addresses type-casted to xxx_RegDef_t)
 */

#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOE_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))			/*Setting 0th bit to enable peripheral clock for GPIOA */

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))


/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

#endif /* INC_STM32F412_H_ */
