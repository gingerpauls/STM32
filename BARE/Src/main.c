/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include "main.h"

#define SYSTICK_RELOAD_VALUE ((SystemCoreClock) / 1000 - 1)

#define BUTTON_MODE 0 										//0-HOLD 1-TOGGLE

#define LED_GREEN GPIO_ODR_OD12
#define LED_ORANGE GPIO_ODR_OD13
#define LED_RED GPIO_ODR_OD14
#define LED_BLUE GPIO_ODR_OD15

void HSI_PLL_CLK_EN(void);
void HSE_PLL_CLK_EN(void);

void TIM2_CONFIG(void);

void SYSTICK_CONFIG(void);
void GPIO_CONFIG(void);

void delay(uint32_t time);

int main(void)
{
	SystemCoreClockUpdate();

	SYSTICK_CONFIG();
	GPIO_CONFIG();
	HSI_PLL_CLK_EN();
	//TIM2_CONFIG();

	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY)){}


	RCC->CFGR |= RCC_CFGR_PPRE1_2 | RCC_CFGR_SW_PLL;
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)){}

	SystemCoreClockUpdate();

	SysTick->LOAD |= SysTick_LOAD_RELOAD_Msk;

	while(1){
//		switch (BUTTON_MODE) {
//		  case 0: //
//				while(GPIOA->IDR & GPIO_IDR_IDR_0){
//					GPIOD->ODR |= LED_GREEN;
//				}
//				GPIOD->ODR &= ~LED_GREEN;
//		    break;
//		  case 1:
//				if(GPIOA->IDR & GPIO_IDR_IDR_0){
//					GPIOD->ODR ^= LED_GREEN;
//					delay(200); //de-bounce
//				}
//		    break;
//		  default:
//		}

//		uint32_t timerCount = TIM2->CNT;
//		if(timerCount = (0xFFFFFFF0)){
//			GPIOD->ODR ^= LED_BLUE;
//		}
		GPIOD->ODR |= LED_GREEN;
	}
}

void SYSTICK_CONFIG(void){
	PWR->CR |= PWR_CR_VOS;
	FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_3WS;
	SysTick->LOAD &= SysTick_LOAD_RELOAD_Msk; // this is about 1 second
	//NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
	SysTick->VAL &= ~SysTick_VAL_CURRENT_Msk;
	SysTick->CTRL |= 	SysTick_CTRL_CLKSOURCE_Msk 	|
						SysTick_CTRL_TICKINT_Msk 	|
						SysTick_CTRL_ENABLE_Msk;
}
void GPIO_CONFIG(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN;

	GPIOD->MODER |= GPIO_MODER_MODE12_0 |
					GPIO_MODER_MODE13_0 |
					GPIO_MODER_MODE14_0 |
					GPIO_MODER_MODE15_0; // set GPIO to "output" mode for LEDs
}

void TIM2_CONFIG(void){
	// TIM2 clocked by pre-scaler output "CK_CNT"
	// enable CEN before CK_CNT
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_PWREN;
	//TIM2->ARR &= ~TIM_ARR_ARR;
	//TIM2->ARR = 0x1F4;
	//NVIC->ISER
	//NVIC_SetPriority (TIM2_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
	//TIM2->CNT &= ~TIM_CNT_CNT;
	TIM2->EGR |= TIM_EGR_UG;
	//TIM2->SMCR |= TIM_SMCR_ECE; // external clock enable?? does this mean HSE or a pin
	//TIM2->DIER |= TIM_DIER_TIE | TIM_DIER_UIE;
	//TIM2->PSC = 0xC350;
	__NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->CR1 |= TIM_CR1_CEN;
}

void HSI_PLL_CLK_EN(void){
	/* PLL 		M-8 	| N-192 	| P-4 	| Q-8
	 * PLLI2S	M-10	| N-200		| R-2
	 * AHB		1
	 * APB1		4
	 * APB2		1
	 * */
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_3;
	RCC->PLLCFGR &= ~(	RCC_PLLCFGR_PLLQ_0 	|
						RCC_PLLCFGR_PLLQ_1 	|
						RCC_PLLCFGR_PLLQ_2	);

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_0;
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP_1);

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_3;
	RCC->PLLCFGR &= ~(	RCC_PLLCFGR_PLLM_0	|
						RCC_PLLCFGR_PLLM_1 	|
						RCC_PLLCFGR_PLLM_2 	|
						RCC_PLLCFGR_PLLM_4 	|
						RCC_PLLCFGR_PLLM_5	);

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;

	RCC->CR |= RCC_CR_HSION;
	while(!(RCC->CR & RCC_CR_HSIRDY)){}

}
void HSE_PLL_CLK_EN(void){
	/* PLL 		M-4 	| N-192 	| P-4 	| Q-8
	 * PLLI2S	M-5	| N-200		| R-2
	 * AHB		1
	 * APB1		4
	 * APB2		1
	 * */
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_3;
	RCC->PLLCFGR &= ~(	RCC_PLLCFGR_PLLQ_0 	|
						RCC_PLLCFGR_PLLQ_1 	|
						RCC_PLLCFGR_PLLQ_2	);

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_0;
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP_1);

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_2;
	RCC->PLLCFGR &= ~(	RCC_PLLCFGR_PLLM_0	|
						RCC_PLLCFGR_PLLM_1 	|
						RCC_PLLCFGR_PLLM_3 	|
						RCC_PLLCFGR_PLLM_4 	|
						RCC_PLLCFGR_PLLM_5	);

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

	RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY)){}
}


void delay(uint32_t timeMS){
	for(uint32_t i = 0; i < timeMS * 7059; i++){} //7059 is from experimentation
}

void SysTick_Handler(void){
	GPIOD->ODR ^= LED_BLUE;
}

void TIM2_IRQHandler(void){
	GPIOD->ODR |= LED_ORANGE;
}
