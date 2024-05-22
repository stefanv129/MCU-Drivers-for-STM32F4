/*
 * stm32f401xx_i2c_driver.c
 *
 *  Created on: Mar 16, 2024
 *      Author: voine
 */

#include "stm32f401xx_i2c_driver.h"

static void I2C_GenStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_GenStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START );
}
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr <<1 ;
	SlaveAddr &= ~(1);//clear bit 0 for r/w
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr <<1 ;
	SlaveAddr |= (1);//clear bit 0 for r/w
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ClearAck(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 &= ~( 1 << 10 );
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead=0;
	I2C_ClearAck(pI2CHandle->pI2Cx);

	if(pI2CHandle->pI2Cx->SR2 & (1 << 0))
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				I2C_ClearAck(pI2CHandle->pI2Cx);
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
		}else
		{
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
		}

	}
	else{
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead;
		}
}







static void I2C_ENAck(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << 10 );
}


static void I2C_GenStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP );
}



void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR2 &= ~(I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(I2C_CR2_ITEVFEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2CConfig.I2C_ACKControl == I2C_ACK_EN)
	{
		I2C_ENAck(pI2CHandle->pI2Cx);
	}
}


void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR2 &= ~(I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(I2C_CR2_ITEVFEN);
}


void I2C_PCLKControl(I2C_RegDef_t *pI2Cx,uint8_t ENorDS){
	if(ENorDS == ENABLE)
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_EN();
			}else if(pI2Cx == I2C2)
			{
				I2C2_PCLK_EN();
			}else if(pI2Cx == I2C3)
			{
				I2C3_PCLK_EN();
			}
		}else
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_DS();
			}else if(pI2Cx == I2C2)
			{
				I2C2_PCLK_DS();
			}else if(pI2Cx == I2C3)
			{
				I2C3_PCLK_DS();
			}
		}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{

	uint32_t tempreg=0;
	tempreg |= (pI2CHandle->I2CConfig.I2C_ACKControl << 10);
	pI2CHandle->pI2Cx->CR1 = tempreg;

	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value();
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	tempreg = 0;
	tempreg |=(pI2CHandle->I2CConfig.I2C_DeviceAddress << 1);
	pI2CHandle->pI2Cx->OAR1 = tempreg;


	tempreg = 0;
	uint16_t CCR =0;

	if(pI2CHandle->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		CCR = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2CConfig.I2C_SCLSpeed);
		tempreg |= (CCR & 0xFFF);
	}else
	{
		tempreg |= ( 1 << 15 );
		tempreg |= (pI2CHandle->I2CConfig.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			CCR = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2CConfig.I2C_SCLSpeed);
		}else
		{
			CCR = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2CConfig.I2C_SCLSpeed);
		}
		tempreg |= (CCR & 0xFFF);
	}

	pI2CHandle->pI2Cx->CCR = tempreg;

	tempreg = 0;

	if(pI2CHandle->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{

			tempreg = RCC_GetPCLK1Value() /1000000U + 1;
		}else
		{
			tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;
		}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
			{
				I2C1_REG_RESET();
			}else if(pI2Cx == I2C2)
			{
				I2C2_REG_RESET();
			}else if(pI2Cx == I2C3)
			{
				I2C3_REG_RESET();
			}
}





void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	I2C_GenStartCondition(pI2CHandle->pI2Cx);

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG) );

	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,SlaveAddr);

	//SET ADDR
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG) );

	//CLEAR ADDR
	I2C_ClearADDRFlag(pI2CHandle);

	while(Len>0)
	{
		//WAIT FOR TXE SET
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_BTF_FLAG));

	I2C_GenStopCondition(pI2CHandle->pI2Cx);
}
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len,uint8_t SlaveAddr)
{
	//generate start
	I2C_GenStartCondition(pI2CHandle->pI2Cx);
	//confirm start
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG) );
	//send slave address with rw
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	 //check addr
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG) );

	//read only 1 byte
	if(Len == 1)
	{
		//disable ack
		I2C_ClearAck(pI2CHandle->pI2Cx);



		//clear addr
		I2C_ClearADDRFlag(pI2CHandle);
		//wait rxne=1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_RXNE_FLAG) );

		//gen stop position matters
		I2C_GenStopCondition(pI2CHandle->pI2Cx);

		//read data in buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
		return;
	}


	if(Len > 1)
	{
		//clear addr
		I2C_ClearADDRFlag(pI2CHandle);
		//read until len 0
		for(uint32_t i=Len;i>0;i--)
		{
			//wait rxne=1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_RXNE_FLAG) );


			if(i==2)
			{
				//clear ack
				I2C_ClearAck(pI2CHandle->pI2Cx);
				//gen stop
				I2C_GenStopCondition(pI2CHandle->pI2Cx);
			}
			//read dr
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			//inc buffer adr
			pRxBuffer++;
		}
	}
	//re enable ack
	if(pI2CHandle->I2CConfig.I2C_ACKControl == I2C_ACK_EN)
	{
		I2C_ENAck(pI2CHandle->pI2Cx);
	}
}


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuffer = pTxBuffer;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_GenStartCondition(pI2CHandle->pI2Cx);//needs modification for repeated start

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= I2C_CR2_ITBUFEN;

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= I2C_CR2_ITEVFEN;

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= I2C_CR2_ITERREN;

		}

		return busystate;

}




uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pRxBuffer = pRxBuffer;
			pI2CHandle->RxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_GenStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= I2C_CR2_ITBUFEN	;

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= I2C_CR2_ITEVFEN;

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= I2C_CR2_ITERREN;
		}

		return busystate;
}


void I2C_IRQITConfig(uint16_t IRQNumber, uint8_t ENorDS)
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
void I2C_IRQPriorityConfig(uint16_t IRQNumber, uint32_t IRQPriority)
{

}


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint8_t temp1,temp2,temp3;

		temp1 = pI2CHandle->pI2Cx->CR2 & I2C_CR2_ITEVFEN;

		temp2 =	pI2CHandle->pI2Cx->CR2 & I2C_CR2_ITBUFEN;

		temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB );

		if( temp1 && temp3)
		{
			//SB SET
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
			}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
			}
		}


		temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR );

		if( temp1 && temp3)
		{
			//ADR SET
			I2C_ClearADDRFlag(pI2CHandle);

		}


		temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF );

		if( temp1 && temp3)
		{
			//BTF SET
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
					//TXE SET?
					if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE))
					{
							if(pI2CHandle->TxLen == 0)
							{
							//BTF TCE SET CLOSE TRANSMISSION
								if(pI2CHandle->Sr == 0)
								{
									I2C_GenStopCondition(pI2CHandle->pI2Cx);
								}
									//reset all member elements
							I2C_CloseSendData(pI2CHandle);


							//notify app about tr complete
							I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT );

							}
					}
			}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
					//nothing to do
			}


		}


		temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF );//read SR1

		if( temp1 && temp3)
		{
			//STOP SET
			//CLEAR STOPF read sr1 write to cr1

			pI2CHandle->pI2Cx->CR1 |= 0x0000;

			//notify about stop detected
			I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP );

		}

		temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE );

		if( temp1 && temp2 && temp3)
		{
			//txe f SET =?>dr empty
			//we need to confirm device iS master
			if(pI2CHandle->pI2Cx->SR2 & (1 << 0)){

				if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
				{
					if(pI2CHandle->TxLen > 0)
					{
						pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

						pI2CHandle->TxLen--;

						pI2CHandle->pTxBuffer++;
					}
				}

			}else
			{
				if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
				{
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
				}
			}

		}

		temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE );

		if( temp1 && temp2 && temp3)
		{

			if(pI2CHandle->pI2Cx->SR2 & (1 << 0))
			{
			//rxne f SET
				if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
				{
					if(pI2CHandle->RxSize==1)
					{
						*(pI2CHandle->pRxBuffer)= pI2CHandle->pI2Cx->DR;
						pI2CHandle->RxLen--;
					}

					if(pI2CHandle->RxSize > 1)
					{
						if(pI2CHandle->RxLen==2)
						{
							I2C_ClearAck(pI2CHandle->pI2Cx);
						}

						*(pI2CHandle->pRxBuffer)= pI2CHandle->pI2Cx->DR;
						pI2CHandle->pRxBuffer++;
						pI2CHandle->RxLen--;
					}
					if(pI2CHandle->RxLen==0)
					{
						I2C_GenStopCondition(pI2CHandle->pI2Cx);
						I2C_CloseReceiveData(pI2CHandle);
						I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT );
					}

				}

			}
			else
			{
				if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
				{
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
				}
			}
		}

}



void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

	    //Know the status of  ITERREN control bit in the CR2
		temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
		if(temp1  && temp2 )
		{
			//This is Bus error

			//Implement the code to clear the buss error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

			//Implement the code to notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
		}

	/***********************Check for arbitration lost error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
		if(temp1  && temp2)
		{
			//This is arbitration lost error

			//Implement the code to clear the arbitration lost error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);
			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

		}

	/***********************Check for ACK failure  error************************************/

		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
		if(temp1  && temp2)
		{
			//This is ACK failure error

		    //Implement the code to clear the ACK failure error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
		}

	/***********************Check for Overrun/underrun error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
		if(temp1  && temp2)
		{
			//This is Overrun/underrun

		    //Implement the code to clear the Overrun/underrun error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);
			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
		}

	/***********************Check for Time out error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
		if(temp1  && temp2)
		{
			//This is Time out error

		    //Implement the code to clear the Time out error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
		}
}


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t value)
{
	pI2Cx->DR = value;
}




uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}
