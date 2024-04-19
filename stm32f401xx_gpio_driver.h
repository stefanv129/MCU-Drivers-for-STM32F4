/*
 * stm32f401xx_gpio_driver.h
 *
 *  Created on: Mar 3, 2024
 *      Author: voine
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"



//DECLARED AS EXTERN IN HEADER


typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;		/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_Pin_Cfg_t;

/*
 * Handle structure for GPIO pin
 */
typedef struct{

	GPIO_RegDef_t *pGPIOx;		//GPIO BASE ADDRESS
	GPIO_Pin_Cfg_t GPIO_PinConfig;  //GPIO pin config settings

}GPIO_Handle_t;


/*
 * APIS
 */

/*
 * PCLK Setup
 */
void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDS);

/*
 * INIT DEINIT
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * R W
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx,  uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


void GPIO_IRQITConfig(uint16_t IRQNumber, uint8_t ENorDS);
void GPIO_IRQPriorityConfig(uint16_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15
#define GPIO_PIN_NO_16		16

/*
 * @GPIO_PIN_MODES
 */
#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_AF 		2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT 	4
#define GPIO_MODE_IT_RT 	5
#define GPIO_MODE_IT_RFT 	6


#define GPIO_OP_TYPE_PP 	0
#define GPIO_OP_TYPE_OD 	1


#define GPIO_SPEED_LOW 		0
#define GPIO_SPEED_MED 		1
#define GPIO_SPEED_FAST 	2
#define GPIO_SPEED_HIGH 	3


#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			 2







#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
