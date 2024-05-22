/*
 * stm32f401xx_spi_driver.h
 *
 *  Created on: Mar 11, 2024
 *      Author: voine
 */

#ifndef INC_STM32F401XX_SPI_DRIVER_H_
#define INC_STM32F401XX_SPI_DRIVER_H_

#include "stm32f401xx.h"


typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;


}SPI_Config_t;



typedef struct{

	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t		*pTxBuffer;
	uint8_t		*pRxBuffer;
	uint32_t	TxLen;
	uint32_t	RxLen;
	uint8_t		TxState;
	uint8_t		RxState;

}SPI_Handle_t;

#define	SPI_DEVICE_MODE_MASTER	1
#define	SPI_DEVICE_MODE_SLAVE	0


#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2	//HALF_DUPLEX
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

#define	SPI_SCLK_SPEED_DIV2			0
#define	SPI_SCLK_SPEED_DIV4			1
#define	SPI_SCLK_SPEED_DIV8			2
#define	SPI_SCLK_SPEED_DIV16		3
#define	SPI_SCLK_SPEED_DIV32		4
#define	SPI_SCLK_SPEED_DIV64		5
#define	SPI_SCLK_SPEED_DIV128		6
#define	SPI_SCLK_SPEED_DIV256		7


#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0

#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0

#define SPI_SSM_HW			0
#define SPI_SSM_SW			1


#define SPI_SR_TXE			1
#define SPI_TXE_FLAG	( 1 << SPI_SR_TXE )

#define SPI_SR_RXNE			0
#define SPI_RXNE_FLAG	( 1 << SPI_SR_RXNE )

#define SPI_SR_OVR			6
#define SPI_OVR_FLAG	( 1 << SPI_SR_OVR )


#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

void SPI_PCLKControl(SPI_RegDef_t *pSPIx,uint8_t ENorDS);

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
//BLOCKING

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);
//NON BLOCKING

void SPI_IRQITConfig(uint16_t IRQNumber, uint8_t ENorDS);
void SPI_IRQPriorityConfig(uint16_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDS);
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDS);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t EVENT);
#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
