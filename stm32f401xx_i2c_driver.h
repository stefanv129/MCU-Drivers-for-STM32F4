/*
 * stm32f401xx_i2c_driver.h
 *
 *  Created on: Mar 16, 2024
 *      Author: voine
 */

#ifndef INC_STM32F401XX_I2C_DRIVER_H_
#define INC_STM32F401XX_I2C_DRIVER_H_

#include "stm32f401xx.h"

typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle;
}I2C_Config_t;


typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2CConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t DevAddr;
	uint32_t RxSize;
	uint8_t Sr;

}I2C_Handle_t;

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName);

uint32_t RCC_GetPCLK1Value(void);
void I2C_PCLKControl(I2C_RegDef_t *pI2Cx,uint8_t ENorDS);
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len,uint8_t SlaveAddr);
//BLOCKING
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
//NON BLOCKING
void I2C_IRQITConfig(uint16_t IRQNumber, uint8_t ENorDS);
void I2C_IRQPriorityConfig(uint16_t IRQNumber, uint32_t IRQPriority);

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDS);
//void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t EVENT);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t value);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

#define I2C_SR1_TXE		7
#define I2C_TXE_FLAG	( 1 << I2C_SR1_TXE )

#define I2C_SR1_RXNE	6
#define I2C_RXNE_FLAG	( 1 << I2C_SR1_RXNE )

#define I2C_SR1_SB		0
#define I2C_SB_FLAG		( 1 << I2C_SR1_RXNE )


#define I2C_SR1_BTF		2
#define I2C_BTF_FLAG	( 1 << I2C_SR1_BTF )

#define I2C_SR1_ADDR	1
#define I2C_ADDR_FLAG	( 1 << I2C_SR1_ADDR )

#define I2C_SR1_STOPF	4


#define I2C_READY		0
#define I2C_BUSY_IN_RX	1
#define I2C_BUSY_IN_TX	2


#define I2C_SCL_SPEED_SM	100000 //KHZ
#define I2C_SCL_SPEED_FM4K	400000

#define I2C_ACK_EN		1
#define I2C_ACK_DS		0

#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1


#define I2C_CR2_ITBUFEN		( 1 << 10 )
#define I2C_CR2_ITEVFEN		( 1 << 9 )
#define I2C_CR2_ITERREN		( 1 << 8 )

#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2

#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7

#define I2C_EV_DATA_REQ 8
#define I2C_EV_DATA_RCV 9


#endif /* INC_STM32F401XX_I2C_DRIVER_H_ */
