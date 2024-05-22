/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: Mar 11, 2024
 *      Author: voine
 */

#include "stm32f401xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
		if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		{//16 bit
			pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;

			(uint16_t*)pSPIHandle->pTxBuffer++;
				//sent 2 bytes
		}else
		{//8 bit
			pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
		}
		if(! pSPIHandle->TxLen)
		{
			SPI_CloseTransmission(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
		}
}


static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
		if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		{//16 bit
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer--;
			pSPIHandle->pRxBuffer--;
					//sent 2 bytes
		}else
		{//8 bit
			*pSPIHandle->pRxBuffer = (uint8_t)pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer--;
		}

		if(! pSPIHandle->RxLen)
		{
			SPI_CloseReception(pSPIHandle);
				SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}
}
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//clear ovr flag:
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}

void SPI_PCLKControl(SPI_RegDef_t *pSPIx,uint8_t ENorDS)
{
	if(ENorDS == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DS();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DS();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DS();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DS();
		}
	}
}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	/////////////MASTER/SLAVE
	uint32_t temp=0;

	SPI_PCLKControl(pSPIHandle->pSPIx,ENABLE);

	temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	/////////////////////BUS

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		temp &=	~( 1 << SPI_CR1_BIDIMODE );
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		temp	|=	( 1 << SPI_CR1_BIDIMODE );
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		temp	&=	~( 1 << SPI_CR1_BIDIMODE );
		temp	|=	( 1 << SPI_CR1_RXONLY );
	}

	/////////////////SPEED

	temp	|=	( pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR );


	/////////////////DFF

	temp	|=	( pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF );

	///////////////SSM

	temp	|=	( pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM );


	////////////////CPOL CPHA
	temp	|=	( pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA );

	temp	|=	( pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL );


	pSPIHandle->pSPIx->CR1	=	temp;	//we can use assign cr1 is fresh


}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
			{
				SPI1_REG_RESET();
			}else if(pSPIx == SPI2)
			{
				SPI2_REG_RESET();
			}else if(pSPIx == SPI3)
			{
				SPI3_REG_RESET();
			}else if(pSPIx == SPI4)
			{
				SPI4_REG_RESET();
			}

}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len >0)
	{
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		{//16 bit
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
			//sent 2 bytes
		}else
		{//8 bit
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

	while(Len >0)
		{
			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);

			if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
			{//16 bit
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer--;
				(uint16_t*)pRxBuffer--;
				//sent 2 bytes
			}else
			{//8 bit
				*(pRxBuffer) = pSPIx->DR;
				Len--;
				pRxBuffer--;
			}
		}
}

void SPI_IRQITConfig(uint16_t IRQNumber, uint8_t ENorDS)
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

void SPI_IRQPriorityConfig(uint16_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section = IRQNumber % 4;
		uint8_t	shift_amount =	(8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
		//IRQPriority = IRQPriority << 4;//only lower 4 bits
		*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount );
}



void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDS)
{
	if(ENorDS == ENABLE)
	{
		pSPIx->CR1	|=	( 1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1	&=	~( 1 << SPI_CR1_SPE);
	}
}

void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDS)
{
	if(ENorDS == ENABLE)
		{
			pSPIx->CR1	|=	( 1 << SPI_CR1_SSI);
		}else
		{
			pSPIx->CR1	&=	~( 1 << SPI_CR1_SSI);
		}
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	//1
	if(state != SPI_BUSY_IN_TX)
	{
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3 TXEIE
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE);
	}

	return state;
}



uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
		//1
		if(state != SPI_BUSY_IN_RX)
		{
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = Len;
			//2
			pSPIHandle->RxState = SPI_BUSY_IN_RX;
			//3 TXEIE
			pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE);
		}

		return state;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1,temp2;

	temp1 = pSPIHandle->pSPIx->SR & SPI_TXE_FLAG;

	temp2 =	pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		//handle txe
		spi_txe_interrupt_handle(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SR & SPI_RXNE_FLAG;

	temp2 =	pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		//handle rxne
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SR & SPI_OVR_FLAG;

	temp2 =	pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
			//handle ovr
		spi_ovr_interrupt_handle(pSPIHandle);
	}

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}





void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen=0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen=0;
	pSPIHandle->RxState = SPI_READY;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t EVENT)
{

}




