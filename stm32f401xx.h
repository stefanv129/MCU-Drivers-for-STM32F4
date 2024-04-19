/*
 * stm32f401xx.h
 *
 *  Created on: Mar 2, 2024
 *      Author: voine
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_



#include <stddef.h>
#include <stdint.h>
#include <string.h>




//macros
#define __vo volatile



#define APB1_PCLKF	8000000
#define APB2_PCLKF	16000000

//MEMORY
#define FLASH_BASEADDR						0x08000000U
#define SRAM_BASEADDR						0x20000000U
#define ROM_BASEADDR						0x1FFF0000U
//COMPILER DECLARE SIGNED AS DEFAULT =>U
#define PERIPH_BASEADDR						0X40000000U

#define APB1_PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2_PERIPH_BASEADDR				0x40010000U
#define AHB1_PERIPH_BASEADDR				0x40020000U
#define AHB2_PERIPH_BASEADDR				0x50000000U


//AHB1_PERIPHERALS
#define GPIOA_BASEADDR						(AHB1_PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR						(AHB1_PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR						(AHB1_PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR						(AHB1_PERIPH_BASEADDR + 0x0c00)
#define GPIOE_BASEADDR						(AHB1_PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR						(AHB1_PERIPH_BASEADDR + 0x1c00)
#define RCC_BASEADDR						(AHB1_PERIPH_BASEADDR + 0x3800)

//APB1_PERIPHERALS
#define TIM2_BASEADDR						(APB1_PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR						(APB1_PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR						(APB1_PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR						(APB1_PERIPH_BASEADDR + 0x0c00)
#define RTCnBKP_BASEADDR					(APB1_PERIPH_BASEADDR + 0x2800)
#define WWDG_BASEADDR						(APB1_PERIPH_BASEADDR + 0x2c00)
#define IWDG_BASEADDR						(APB1_PERIPH_BASEADDR + 0x3000)
#define I2S2ext_BASEADDR					(APB1_PERIPH_BASEADDR + 0x3400)
#define SPI2orI2S2_BASEADDR					(APB1_PERIPH_BASEADDR + 0x3800)
#define SPI3orI2S3_BASEADDR					(APB1_PERIPH_BASEADDR + 0x3c00)
#define I2S3ext_BASEADDR					(APB1_PERIPH_BASEADDR + 0x4000)
#define USART2_BASEADDR						(APB1_PERIPH_BASEADDR + 0x4400)
#define I2C1_BASEADDR						(APB1_PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1_PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1_PERIPH_BASEADDR + 0x5c00)
#define PWR_BASEADDR						(APB1_PERIPH_BASEADDR + 0x7000)

//APB2_PERIPHERALS
#define TIM1_BASEADDR						(APB2_PERIPH_BASEADDR + 0x0000)
#define USART1_BASEADDR						(APB2_PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2_PERIPH_BASEADDR + 0x1400)
#define ADC1_BASEADDR						(APB2_PERIPH_BASEADDR + 0x2000)
#define SDIO_BASEADDR						(APB2_PERIPH_BASEADDR + 0x2c00)
#define SPI1_BASEADDR						(APB2_PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR						(APB2_PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR						(APB2_PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR						(APB2_PERIPH_BASEADDR + 0x3c00)
#define TIM9_BASEADDR						(APB2_PERIPH_BASEADDR + 0x4000)
#define TIM10_BASEADDR						(APB2_PERIPH_BASEADDR + 0x4400)
#define TIM11_BASEADDR						(APB2_PERIPH_BASEADDR + 0x4800)

//SYSCFG REGS
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)



//EXTI REGS
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)


//SPI REGS
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2orI2S2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3orI2S3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)



//GPIO REGS
typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];

}GPIO_RegDef_t;

//GPIO MACROS FOR STRUCT
#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASEADDR)


extern GPIO_RegDef_t *pGPIOA;



typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	uint32_t reserved1[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t reserved2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	uint32_t reserved3[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t reserved4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	uint32_t reserved5[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t reserved6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t reserved7[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	uint32_t reserved8;
	__vo uint32_t DCKCFGR;

}RCC_RegDef_t;

#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * CLOCK ENABLE MACROS FOR GPIOx PERIPHERALS
 */
#define GPIOA_PCLK_EN()			( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()			( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()			( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()			( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()			( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOH_PCLK_EN()			( RCC->AHB1ENR |= ( 1 << 7 ) )



/*
 * CLOCK ENABLE MACROS FOR SPIx PERIPHERALS
 */
#define SPI1_PCLK_EN()			( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 15 ) )
#define SPI4_PCLK_EN()			( RCC->APB2ENR |= ( 1 << 13 ) )


/*
 * USARTx PERIPHERAL
 */

typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;

}USART_RegDef_t;

#define USART1	((USART_RegDef_t*)USART1_BASEADDR)
#define USART2	((USART_RegDef_t*)USART2_BASEADDR)
#define USART6	((USART_RegDef_t*)USART6_BASEADDR)

/*
 * CLOCK ENABLE MACROS FOR USARTx PERIPHERALS
 */
#define USART1_PCLK_EN()			( RCC->APB2ENR |= ( 1 << 4 ) )
#define USART2_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 17 ) )
#define USART6_PCLK_EN()			( RCC->APB2ENR |= ( 1 << 5 ) )

/*
 * REGISTER RESET MACROS FOR USARTx PERIPHERALS
 */
#define USART1_REG_RESET()			do{ ( RCC->APB2RSTR |= ( 1 << 4 ) ); ( RCC->APB2RSTR &= ~( 1 << 12 ) ); }while(0)
#define USART2_REG_RESET()			do{ ( RCC->APB1RSTR |= ( 1 << 17 ) ); ( RCC->APB1RSTR &= ~( 1 << 14 ) ); }while(0)
#define USART6_REG_RESET()			do{ ( RCC->APB2RSTR |= ( 1 << 5 ) ); ( RCC->APB1RSTR &= ~( 1 << 14 ) ); }while(0)

/*
 * REGISTER RESET MACROS FOR GPIOx PERIPHERALS
 */
#define GPIOA_REG_RESET()			do{ ( RCC->AHB1RSTR |= ( 1 << 0 ) ); ( RCC->AHB1RSTR &= ~( 1 << 0 ) ); }while(0)
#define GPIOB_REG_RESET()			do{ ( RCC->AHB1RSTR |= ( 1 << 1 ) ); ( RCC->AHB1RSTR &= ~( 1 << 1 ) ); }while(0)
#define GPIOC_REG_RESET()			do{ ( RCC->AHB1RSTR |= ( 1 << 2 ) ); ( RCC->AHB1RSTR &= ~( 1 << 2 ) ); }while(0)
#define GPIOD_REG_RESET()			do{ ( RCC->AHB1RSTR |= ( 1 << 3 ) ); ( RCC->AHB1RSTR &= ~( 1 << 3 ) ); }while(0)
#define GPIOE_REG_RESET()			do{ ( RCC->AHB1RSTR |= ( 1 << 4 ) ); ( RCC->AHB1RSTR &= ~( 1 << 4 ) ); }while(0)
#define GPIOH_REG_RESET()			do{ ( RCC->AHB1RSTR |= ( 1 << 7 ) ); ( RCC->AHB1RSTR &= ~( 1 << 7 ) ); }while(0)


/*
 * REGISTER RESET MACROS FOR SPIx PERIPHERALS
 */
#define SPI1_REG_RESET()			do{ ( RCC->APB2RSTR |= ( 1 << 12 ) ); ( RCC->APB2RSTR &= ~( 1 << 12 ) ); }while(0)
#define SPI2_REG_RESET()			do{ ( RCC->APB1RSTR |= ( 1 << 14 ) ); ( RCC->APB1RSTR &= ~( 1 << 14 ) ); }while(0)
#define SPI3_REG_RESET()			do{ ( RCC->APB1RSTR |= ( 1 << 15 ) ); ( RCC->APB1RSTR &= ~( 1 << 15 ) ); }while(0)
#define SPI4_REG_RESET()			do{ ( RCC->APB2RSTR |= ( 1 << 13 ) ); ( RCC->APB2RSTR &= ~( 1 << 13 ) ); }while(0)

#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3

/*
 * CLOCK ENABLE MACROS FOR I2Cx PERIPHERALS
 */
#define I2C1_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 23 ) )

#define I2C1_REG_RESET()				do{ ( RCC->APB1RSTR |= ( 1 << 21 ) ); ( RCC->APB1RSTR &= ~( 1 << 21 ) ); }while(0)
#define I2C2_REG_RESET()				do{ ( RCC->APB1RSTR |= ( 1 << 22 ) ); ( RCC->APB1RSTR &= ~( 1 << 22 ) ); }while(0)
#define I2C3_REG_RESET()			do{ ( RCC->APB1RSTR |= ( 1 << 23 ) ); ( RCC->APB1RSTR &= ~( 1 << 23 ) ); }while(0)
//I2C REGS

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;

}I2C_RegDef_t;

#define I2C1	((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2	((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3	((I2C_RegDef_t*)I2C3_BASEADDR)


#define I2C_CR1_START	8
#define I2C_CR1_STOP	9



/*
 * CLOCK ENABLE MACROS FOR SYSCFGx PERIPHERALS
 */
#define SYSCFG_PCLK_EN()			( RCC->APB2ENR |= ( 1 << 14 ) )


/*
 * CLOCK DISABLE MACROS FOR GPIOx PERIPHERALS
 */
#define GPIOA_PCLK_DS()			( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DS()			( RCC->AHB1ENR &= ~ ( 1 << 1 ) )
#define GPIOC_PCLK_DS()			( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DS()			( RCC->AHB1ENR &= ~ ( 1 << 3 ) )
#define GPIOE_PCLK_DS()			( RCC->AHB1ENR &= ~ ( 1 << 4 ) )
#define GPIOH_PCLK_DS()			( RCC->AHB1ENR &= ~ ( 1 << 7 ) )

/*
 * CLOCK DISABLE MACROS FOR I2Cx PERIPHERALS
 */
#define I2C1_PCLK_DS()			( RCC->APB1ENR &= ~ ( 1 << 21 ) )
#define I2C2_PCLK_DS()			( RCC->APB1ENR &= ~ ( 1 << 22 ) )
#define I2C3_PCLK_DS()			( RCC->APB1ENR &= ~ ( 1 << 23 ) )

/*
 * CLOCK DISABLE MACROS FOR SPIx PERIPHERALS
 */
#define SPI1_PCLK_DS()			( RCC->APB2ENR &= ~ ( 1 << 12 ) )
#define SPI2_PCLK_DS()			( RCC->APB1ENR &= ~ ( 1 << 14 ) )
#define SPI3_PCLK_DS()			( RCC->APB1ENR &= ~ ( 1 << 15 ) )
#define SPI4_PCLK_DS()			( RCC->APB2ENR &= ~ ( 1 << 13 ) )

/*
 * CLOCK DISABLE MACROS FOR USARTx PERIPHERALS
 */
#define USART1_PCLK_DS()			( RCC->APB2ENR &= ~ ( 1 << 4 ) )
#define USART2_PCLK_DS()			( RCC->APB1ENR &= ~ ( 1 << 17 ) )
#define USART6_PCLK_DS()			( RCC->APB2ENR &= ~( 1 << 5 ) )

/*
 * CLOCK DISABLE MACROS FOR SYSCFGx PERIPHERALS
 */
#define SYSCFG_PCLK_DS()			( RCC->APB2ENR &= ~ ( 1 << 14 ) )

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84


#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32

//FROM VECTOR TABLE
//CORTEX M4 REGISTERS
#define NVIC_ISER0			( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1			( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2			( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3			( (__vo uint32_t*)0xE000E10c )

#define NVIC_ICER0			( (__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1			( (__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2			( (__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3			( (__vo uint32_t*)0xE000E18c )

#define NVIC_PR_BASEADDR	( (__vo uint32_t*)0xE000E400 )
//#define NVIC_IPR1			( (__vo uint32_t*)0xE000E404 )
//#define NVIC_IPR2			( (__vo uint32_t*)0xE000E408 )
//#define NVIC_IPR3			( (__vo uint32_t*)0xE000E40c )

#define NO_PR_BITS_IMPLEMENTED	4 //in ST case


#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

//BIT POSITION MACROS
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR		    3
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15
#define SPI_CR1_SPE			6
#define SPI_CR1_SSI			8


#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

#define FLAG_RESET			RESET
#define FLAG_SET			SET

#include "stm32f401xx_gpio_driver.h"
#include "stm32f401xx_spi_driver.h"
#include "stm32f401xx_i2c_driver.h"
#include "stm32f401xx_usart_driver.h"

#endif /* INC_STM32F401XX_H_ */
