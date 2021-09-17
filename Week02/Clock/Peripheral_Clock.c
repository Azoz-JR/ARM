
#include <stdint.h>
#include "stm32f4xx.h"



int main(void)
{
  ADC_TypeDef *pADC;
  RCC_TypeDef *pRCC;
  GPIO_TypeDef *pGPIO;


  pADC=ADC1;
  pRCC=RCC;
  pGPIO=GPIOA;

  // first enable clock then access the register
  pRCC->APB2ENR=pRCC->APB2ENR|(1<<8);
  pRCC->AHB1ENR=pRCC->AHB1ENR|(1<<0);
  //pRCC->APB2ENR|=(1<<8);

  pADC->CR1=0x55;
  pGPIO->PUPDR=0x11;

  return 0;
 }
