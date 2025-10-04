#include <stdint.h>
#include "stm32f4xx.h"

#define LED_PIN 5

void delay_ms(uint32_t milliseconds);


volatile uint32_t ticks;
void main(void){
    RCC->APB1ENR |= RCC_APB1ENR_PWREN_Msk;
    volatile uint32_t dummy;
    dummy = RCC->AHB1ENR;
    dummy = RCC->AHB1ENR;
    PWR->CR |= (0b11 << PWR_CR_VOS_Pos);
    FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
    RCC->CR |= RCC_CR_HSEBYP_Msk | RCC_CR_HSEON_Msk; 
    while (!(RCC->CR & RCC_CR_HSERDY_Msk));

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk |
                    RCC_PLLCFGR_PLLN_Msk |
                    RCC_PLLCFGR_PLLP_Msk);
  
    // Set PLLM, PLLN and PLLP, and select HSE as PLL source
    RCC->PLLCFGR |= ((4 << RCC_PLLCFGR_PLLM_Pos) | 
                    (200 << RCC_PLLCFGR_PLLN_Pos) |
                    (1 << RCC_PLLCFGR_PLLP_Pos) |
                    (1 << RCC_PLLCFGR_PLLSRC_Pos));

    RCC->CFGR |= (0b100 << RCC_CFGR_PPRE1_Pos);
    
    RCC->CR |= RCC_CR_PLLON_Msk;
    while (! (RCC->CR & RCC_CR_PLLRDY_Msk));
    
    RCC->CFGR |= (RCC_CFGR_SW_PLL << RCC_CFGR_SW_Pos);
    while (! (RCC->CFGR & RCC_CFGR_SWS_PLL));

    RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos);

    dummy = RCC->APB1ENR;
    dummy = RCC->APB1ENR;
    PWR->CR |= (0b11 << PWR_CR_VOS_Pos);
    FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
    RCC->CR |= RCC_CR_HSEBYP_Msk | RCC_CR_HSEON_Msk;
    while (!(RCC->CR & RCC_CR_HSERDY_Msk));


    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLP_Msk);
    
    // Set PLLM, PLLN and PLLP, and select HSE as PLL source
    RCC->PLLCFGR |= ((4 << RCC_PLLCFGR_PLLM_Pos) | (200 << RCC_PLLCFGR_PLLN_Pos) | (1 << RCC_PLLCFGR_PLLP_Pos) | (1 << RCC_PLLCFGR_PLLSRC_Pos));
    
    // Set APB1 prescaler to 2
    RCC->CFGR |= (0b100 << RCC_CFGR_PPRE1_Pos);
    
    // Enable PLL and wait for ready
    RCC->CR |= RCC_CR_PLLON_Msk;
    while (! (RCC->CR & RCC_CR_PLLRDY_Msk));

    // Select PLL output as system clock
    RCC->CFGR |= (RCC_CFGR_SW_PLL << RCC_CFGR_SW_Pos);
    while (! (RCC->CFGR & RCC_CFGR_SWS_PLL));



    // Select PLL output as system clock
    SysTick_Config(100000);
    __enable_irq();

    while (1)
    {
        GPIOA->ODR ^= (1 << LED_PIN);
        delay_ms(500);
    }
}
void delay_ms(uint32_t milliseconds)
{
  uint32_t start = ticks;
  uint32_t end = start + milliseconds;

  if (end < start) // handle overflow
  {
    while (ticks > start); // wait for ticks to wrap around to zero
  }

  while (ticks < end);
}


void systick_handler()
{
  ticks++;
}