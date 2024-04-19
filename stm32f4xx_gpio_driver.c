/*GPIO_PinConfig
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: Mar 3, 2024
 *      Author: voine
 */

#include "stm32f401xx_gpio_driver.h"

void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDS){
	if(ENorDS == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DS();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DS();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DS();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DS();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DS();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DS();
		}
	}
}


void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp;
	uint8_t temp1;
	uint8_t temp2;
	uint8_t value;
	//MODE CONFIG
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		//MODER REGISTER
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
		//pGPIOx is GPIORegDef
	}else
	{
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT )
		{
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		SYSCFG_PCLK_EN();

		if(pGPIOHandle->pGPIOx == GPIOA)
				{
					value= 0x0;
				}else if(pGPIOHandle->pGPIOx == GPIOB)
				{
					value= 0x1;
				}else if(pGPIOHandle->pGPIOx == GPIOC)
				{
					value= 0x2;
				}else if(pGPIOHandle->pGPIOx == GPIOD)
				{
					value= 0x3;
				}else if(pGPIOHandle->pGPIOx == GPIOE)
				{
					value= 0x4;
				}else if(pGPIOHandle->pGPIOx == GPIOH)
				{
					value= 0x7;
				}

		temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinMode / 4;
		temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinMode % 4;

		SYSCFG->EXTICR[temp1] |= value << (4*temp2);

		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	}
	temp = 0;

	//SPEED CONFIG
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//PUPD CONFIG 0/1/2
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp=0;

	//OUTPUT TYPE 0/1
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER |= temp;
		temp=0;
	}

	//AF
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF)
	{
		uint8_t temp1;
		temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - temp1*8)));
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - temp1*8)));
		pGPIOHandle->pGPIOx->AFR[temp1] |= temp;
	}
	temp=0;

	GPIO_PCLKControl(pGPIOHandle->pGPIOx, ENABLE);

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
//RCC_AHB1RSTR
	if(pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();

			//THESE AMCROS IN MCU HEADER FILE
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();

		}else if(pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();

		}else if(pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();

		}else if(pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();

		}else if(pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();

		}
}


uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value= (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);//uint8_t result!!!!
	return value;
}

uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value= (uint16_t) pGPIOx->IDR;
	return value;
}


void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else
	{
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx,  uint16_t Value)
{
	pGPIOx->ODR |= Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR = pGPIOx->ODR ^ ( 1 << PinNumber);//XOR NOT OR
}


void GPIO_IRQITConfig(uint16_t IRQNumber, uint8_t ENorDS)
{

	if(ENorDS == ENABLE)
		{
			if(IRQNumber <=31)
			{
				*NVIC_ISER0	|= (1 << IRQNumber);
			}else if(IRQNumber > 31 && IRQNumber <= 64)
			{
				*NVIC_ISER1	|= (1 << IRQNumber % 32);
			}else if(IRQNumber > 64 && IRQNumber <= 96)
			{
				*NVIC_ISER2	|= (1 << IRQNumber % 64);
			}
		}else
			{
				if(IRQNumber <=31)
				{
					*NVIC_ICER0	|= (1 << IRQNumber);
				}else if(IRQNumber > 31 && IRQNumber <= 64)
				{
					*NVIC_ICER1	|= (1 << IRQNumber % 32);
				}else if(IRQNumber > 64 && IRQNumber <= 96)
				{
					*NVIC_ICER2	|= (1 << IRQNumber % 64);
				}
			}
}

void GPIO_IRQPriorityConfig(uint16_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t	shift_amount =	(8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	//IRQPriority = IRQPriority << 4;//only lower 4 bits
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount );
}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);//CLEARING PENDING REGISTER
	}
}
