/*
 * stm32f407xx.h
 *
 *  Created on: Sep 26, 2021
 *      Author: azoz
 */
#define __IO  volatile
#include <stdio.h>


#define Enable 1
#define Disable 0
#define SET Enable
#define RESET Disable
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET




#ifndef DRIVERS_INC_STM32F407XX_H_
#define DRIVERS_INC_STM32F407XX_H_

#endif /* DRIVERS_INC_STM32F407XX_H_ */


#define Flash_BASEADDR         0x08000000U
#define SRAM1_BASEADDR         0x20000000U
#define SRAM 	               SRAM1_BASEADDR
#define SRAM2_BASEADDRE        0x2001C00U
#define ROM_BASEADDRE          0x1FFF0000U
#define RCC_BASEADDR           (AHB1PERIPH_BASE + 0x3800U)


/*
 * AHBx and APBx bus peripheral base addresses
 */




#define PERIPH_BASE            0x40000000U
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE        0x40010000U
#define AHB1PERIPH_BASE        0x40020000U
#define AHB2PERIPH_BASE        0x50000000U


/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR         AHB1PERIPH_BASE
#define GPIOB_BASEADDR         (AHB1PERIPH_BASE + 0x400U)
#define GPIOC_BASEADDR         (AHB1PERIPH_BASE + 0x800U)
#define GPIOD_BASEADDR         (AHB1PERIPH_BASE + 0xC00U)
#define GPIOE_BASEADDR         (AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASEADDR         (AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASEADDR         (AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASEADDR         (AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASEADDR         (AHB1PERIPH_BASE + 0x2000U)




/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR          (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR          (APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASEADDR          (APB1PERIPH_BASE + 0x5C00U)
#define SPI2_BASEADDR          (APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASEADDR          (APB1PERIPH_BASE + 0x3C00U)
#define USART2_BASEADDR          (APB1PERIPH_BASE + 0x4400U)
#define USART3_BASEADDR          (APB1PERIPH_BASE + 0x4800U)
#define UART4_BASEADDR          (APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASEADDR          (APB1PERIPH_BASE + 0x5000U)



/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */



#define SPI1_BASEADDR          (APB2PERIPH_BASE + 0x3000U)
#define SPI4_BASEADDR          (APB2PERIPH_BASE + 0x3400U)
#define USART1_BASEADDR          (APB2PERIPH_BASE + 0x1000U)
#define USART6_BASEADDR          (APB2PERIPH_BASE + 0x1400U)
#define EXTI_BASEADDR          (APB2PERIPH_BASE + 0x3C00U)
#define SYSCFG_BASEADDR          (APB2PERIPH_BASE + 0x3800U)






/*
 * peripheral register definition structure for GPIO
 */




typedef struct
  {
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRRL;    /*!< GPIO port bit set/reset low register,  Address offset: 0x18      */
  __IO uint32_t BSRRH;    /*!< GPIO port bit set/reset high register, Address offset: 0x1A      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
  }GPIO_RegDef_t;


#define GPIOA       ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB       ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC       ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD       ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE       ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF       ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG       ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH       ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI       ((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC       ((RCC_RegDef_t*)RCC_BASEADDR)


/*
 * to access this registers :   GPIO_RegDef_t *pGPIOA = GPIOA ;
 *
 */

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define  GPIOA_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 0 ) )
#define  GPIOB_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 1 ) )
#define  GPIOC_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 2 ) )
#define  GPIOD_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 3 ) )
#define  GPIOE_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 4 ) )
#define  GPIOF_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 5 ) )
#define  GPIOG_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 6 ) )
#define  GPIOH_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 7 ) )
#define  GPIOI_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 8 ) )






/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()    (RCC->APB1ENR |= ( 1 << 21))








/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define  GPIOA_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 0 ) )
#define  GPIOB_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 1 ) )
#define  GPIOC_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 2 ) )
#define  GPIOD_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 3 ) )
#define  GPIOE_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 4 ) )
#define  GPIOF_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 5 ) )
#define  GPIOG_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 6 ) )
#define  GPIOH_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 7 ) )
#define  GPIOI_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 8 ) )


/*
 * Macros to Reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()            do{ (RCC->AHB1RSTR |= ( 1 << 0 ) ) ;  (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()            do{ (RCC->AHB1RSTR |= ( 1 << 1 ) ) ;  (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()            do{ (RCC->AHB1RSTR |= ( 1 << 2 ) ) ;  (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()            do{ (RCC->AHB1RSTR |= ( 1 << 3 ) ) ;  (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()            do{ (RCC->AHB1RSTR |= ( 1 << 4 ) ) ;  (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOF_REG_RESET()            do{ (RCC->AHB1RSTR |= ( 1 << 5 ) ) ;  (RCC->AHB1RSTR &= ~(1<<5)); }while(0)
#define GPIOG_REG_RESET()            do{ (RCC->AHB1RSTR |= ( 1 << 6 ) ) ;  (RCC->AHB1RSTR &= ~(1<<6)); }while(0)
#define GPIOH_REG_RESET()            do{ (RCC->AHB1RSTR |= ( 1 << 7 ) ) ;  (RCC->AHB1RSTR &= ~(1<<7)); }while(0)
#define GPIOI_REG_RESET()            do{ (RCC->AHB1RSTR |= ( 1 << 8 ) ) ;  (RCC->AHB1RSTR &= ~(1<<8)); }while(0)









typedef struct
{
	__IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
	  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
	  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
	  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
	  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
	  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
	  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
	  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
	  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
	  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
	  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
	  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
	  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
	  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
	  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
	  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
	  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
	  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
	  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
	  __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
	  __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
	  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
	  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
	  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
	  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
	  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
	  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
	  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
	  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
	  __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
	  __IO uint32_t PLLSAICFGR;    /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
	  __IO uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
	  __IO uint32_t CKGATENR;      /*!< RCC Clocks Gated Enable Register,                            Address offset: 0x90 */ /* Only for STM32F412xG, STM32413_423xx and STM32F446xx devices */
	  __IO uint32_t DCKCFGR2;      /*!< RCC Dedicated Clocks configuration register 2,               Address offset: 0x94 */ /* Only for STM32F410xx, STM32F412xG, STM32413_423xx and STM32F446xx devices */

 }RCC_RegDef_t;



