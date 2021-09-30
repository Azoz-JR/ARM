/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Sep 29, 2021
 *      Author: azoz
 */


#include <stdio.h>
#include "stm32f407xx.h"
#ifndef DRIVERS_INC_STM32F407XX_GPIO_DRIVER_H_
#define DRIVERS_INC_STM32F407XX_GPIO_DRIVER_H_
#endif /* DRIVERS_INC_STM32F407XX_H_ */









typedef struct
{
	uint8_t GPIO_PinNumber ;        /*!<possible values from @GPIO_PIN_NUMBERS>!*/
	uint8_t GPIO_PinMode ;          /*!<possible values from @GPIO_PIN_MODES>!*/
	uint8_t GPIO_PinSpeed ;         /*!<possible values from @GPIO_PIN_SPEEDS>!*/
	uint8_t GPIO_PinPupdControl ;   /*!<possible values from @GPIO_PUPD>!*/
	uint8_t GPIO_PinOPType ;        /*!<possible values from @GPIO_PIN_OP_TYPES>!*/
	uint8_t GPIO_PinAltFunMode ;

}GPIO_PinConfig_t;



//this is a Handle structure for a gpio pin


typedef struct
{

	//pointer to handle the base address of the GPIO peripheral

	GPIO_RegDef_t  *pGPIOx ;
	GPIO_PinConfig_t GPIO_PinConfig ;


}GPIO_Handle_t;


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi );

//Initialization and de-initialization functions
void  GPIO_Init(GPIO_Handle_t  *pGPIOHandle);
void  GPIO_DeInit(GPIO_RegDef_t  *pGPIOx);



//IO operation functions
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WritePin(GPIO_RegDef_t GPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritePort(GPIO_RegDef_t GPIOx, uint16_t Value);


void GPIO_TogglePin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
void GPIO_LockPin(GPIO_RegDef_t* pGPIOx, uint16_t PinNumber);
void GPIO_IRQConfig(uint8_t IRQNumber , uint8_t IRQPriority , uint8_t EnorDi);
void GPIO_IRQHandler(uint8_t PinNumber);
void GPIO_EXTI_Callback(uint16_t PinNumber);






/*
* @GPIO_PIN_MODES
* GPIO pin possible modes
*/
#define GPIO_MODE_IN      0
#define GPIO_MODE_OUT     1
#define GPIO_MODE_ALTFN   2
#define GPIO_MODE_ANALOG  3
#define GPIO_MODE_IT_FT   4
#define GPIO_MODE_IT_RT   5
#define GPIO_MODE_IT_RFT  6


/*@GPIO_PIN_OP_TYPES
 * GPIO output type modes
 */
#define GPIO_OP_TYPE_PP    0
#define GPIO_OP_TYPE_OD    1


/*GPIO output possible speed modes
 * @GPIO_PIN_SPEEDS
 */

#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_FAST     2
#define GPIO_SPEED_HIGH     3



// GPIO pull up and pull down config macros
// @GPIO_PUPD

#define GPIO_NO_PUPD     0
#define GPIO_PIN_PU      1
#define GPIO__PIN_PD     2

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */

#define GPIO_PIN_NO_0               0
#define GPIO_PIN_NO_1               1
#define GPIO_PIN_NO_2               2
#define GPIO_PIN_NO_3               3
#define GPIO_PIN_NO_4               4
#define GPIO_PIN_NO_5               5
#define GPIO_PIN_NO_6               6
#define GPIO_PIN_NO_7               7
#define GPIO_PIN_NO_8               8
#define GPIO_PIN_NO_9               9
#define GPIO_PIN_NO_10              10
#define GPIO_PIN_NO_11              11
#define GPIO_PIN_NO_12              12
#define GPIO_PIN_NO_13              13
#define GPIO_PIN_NO_14              14
#define GPIO_PIN_NO_15              15


