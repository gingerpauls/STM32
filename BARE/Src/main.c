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
#include "string.h"
#include "lcd.h"

#define LED_GREEN GPIO_ODR_OD12
#define LED_ORANGE GPIO_ODR_OD13
#define LED_RED GPIO_ODR_OD14
#define LED_BLUE GPIO_ODR_OD15

void FLASH_AND_POWER_CONFIG(void);
void SYSTICK_CONFIG(void);
void GPIO_CONFIG(void);

void HSI_PLL_CLK_EN(void);
void HSE_PLL_CLK_EN(void);

void TIM2_CONFIG(uint32_t timeMilliSeconds);
void TIM10_CONFIG(uint32_t frequency);

void USART2_CONFIG(uint32_t, uint32_t, uint32_t);

void delay(uint32_t cycles);

int main(void) {
	char *word = "jesus tapdancing christ!";
	int cycles = 9600000;

	FLASH_AND_POWER_CONFIG(); // for HCLK = 96MHz
	GPIO_CONFIG();
	HSE_PLL_CLK_EN();
	//Reset_Baud_Rate(); // MUST REMOVE AFTER RESETTING
	USART2_CONFIG(0x27, 0x1, 0x0); // 38400 Baud Rate
	Change_Baud_Rate(0x10); // 38400 Baud Rate

	Blink_Cursor();
	delay(cycles);
	for (;;) {
		Clear_Display();
		delay(cycles);
		GPIOD->ODR |= LED_GREEN;
		Write_String(word);
		delay(cycles);
		GPIOD->ODR &= ~LED_GREEN;
		Clear_Display();
	}

	RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
	USART2->CR1 &= ~USART_CR1_TE;
}

void FLASH_AND_POWER_CONFIG(void) {
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;
	FLASH->ACR |= 	FLASH_ACR_ICEN 			|
					FLASH_ACR_DCEN			|
					FLASH_ACR_PRFTEN 		|
					FLASH_ACR_LATENCY_3WS	;
}
void GPIO_CONFIG(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN;

	GPIOD->MODER |= 	GPIO_MODER_MODE12_0 |
						GPIO_MODER_MODE13_0 |
						GPIO_MODER_MODE14_0 |
						GPIO_MODER_MODE15_0	; // set GPIO to "output" mode for LEDs
}

void HSI_PLL_CLK_EN(void) {
	/* PLL 		M-8 	| N-192 	| P-4 	| Q-8
	 * PLLI2S	M-10	| N-200		| R-2
	 * AHB		1
	 * APB1		4
	 * APB2		1
	 * */
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_3;
	RCC->PLLCFGR &= ~( 	RCC_PLLCFGR_PLLQ_0 	|
						RCC_PLLCFGR_PLLQ_1 	|
						RCC_PLLCFGR_PLLQ_2	);

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_0;
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP_1);

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_3;
	RCC->PLLCFGR &= ~( 	RCC_PLLCFGR_PLLM_0 	|
						RCC_PLLCFGR_PLLM_1 	|
						RCC_PLLCFGR_PLLM_2 	|
						RCC_PLLCFGR_PLLM_4 	|
						RCC_PLLCFGR_PLLM_5	);

	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;

	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY)) {}

	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY)) {}

	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) {}
}
void HSE_PLL_CLK_EN(void) {
	/* PLL 		M-4 	| N-192 	| P-4 	| Q-8
	 * PLLI2S	M-5	| N-200		| R-2
	 * AHB		1
	 * APB1		4
	 * APB2		1
	 * */
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_3;
	RCC->PLLCFGR &= ~( 	RCC_PLLCFGR_PLLQ_0 |
						RCC_PLLCFGR_PLLQ_1 |
						RCC_PLLCFGR_PLLQ_2);

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_0;
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP_1);

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_2;
	RCC->PLLCFGR &= ~( 	RCC_PLLCFGR_PLLM_0 |
						RCC_PLLCFGR_PLLM_1 |
						RCC_PLLCFGR_PLLM_3 |
						RCC_PLLCFGR_PLLM_4 |
						RCC_PLLCFGR_PLLM_5);

	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

	RCC->CR |= RCC_CR_HSEON;
	while (!(RCC->CR & RCC_CR_HSERDY)) {}

	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY)) {}

	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) {}
}

void SYSTICK_CONFIG(void) {
	SysTick->LOAD |= SysTick_LOAD_RELOAD_Msk; // this is about 1 second
	//NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
	SysTick->VAL &= ~SysTick_VAL_CURRENT_Msk;
	SysTick->CTRL |= 	SysTick_CTRL_CLKSOURCE_Msk 	|
						SysTick_CTRL_TICKINT_Msk 	|
						SysTick_CTRL_ENABLE_Msk		;
}
void SysTick_Handler(void) {
	GPIOD->ODR ^= LED_BLUE;
}

void TIM2_CONFIG(uint32_t timeMilliSeconds) {
	SystemCoreClockUpdate();
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_PWREN;
	TIM2->ARR = timeMilliSeconds; // 1000 * 1ms = 1s
	TIM2->PSC = ((SystemCoreClock / 2) / 1000) - 1; // 1ms
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR1 |= TIM_CR1_CEN;
	//__NVIC_EnableIRQ(TIM2_IRQn); //
	NVIC->ISER[0] |= 1 << TIM2_IRQn; // IRQn of TIM2 => 0x10000000 = '28' (bit 28)
}
void TIM2_IRQHandler(void) {
	TIM2->SR &= ~TIM_SR_UIF;
	GPIOD->ODR ^= LED_ORANGE;
}

void TIM10_CONFIG(uint32_t frequency) {
	SystemCoreClockUpdate();

	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
	TIM10->ARR = (1000000 / (2 * frequency)); // set in us
	TIM10->PSC = (SystemCoreClock / 1000000) - 1; // 1 us
	TIM10->DIER |= TIM_DIER_UIE;
	TIM10->CR1 |= TIM_CR1_CEN;
	//__NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	NVIC->ISER[0] |= 1 << TIM1_UP_TIM10_IRQn; // IRQn of TIM2 = 25
}
void TIM1_UP_TIM10_IRQHandler(void) {
	TIM10->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
	GPIOD->ODR ^= LED_GREEN; // Toggle the green LED
}

void USART2_CONFIG(uint32_t mantissa, uint32_t fraction, uint32_t stop) {
	/* 	USART2_TX -> PA2 (alternate function) uses AHB
	 *	USART on PA2 -> AF7
	 *	LCD:
	 *		5V TTL
	 *		9600 baud
	 *		8 bits
	 *		1 stop
	 *		no parity
	 *
	 */
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // APB1 PERIPH CLK = 24MHz

	GPIOA->MODER |= GPIO_MODER_MODE2_1; // Alternate function
	GPIOA->AFR[0] |= ( 	GPIO_AFRL_AFRL2_0 	|
						GPIO_AFRL_AFRL2_1 	|
						GPIO_AFRL_AFRL2_2	);
	//GPIOA->PUPDR |= GPIO_PUPDR_PUPD2_0; // pull-up

	USART2->CR2 |= (stop << USART_CR2_STOP_Pos);// Stop bit: 0b00 is 1 stop bit by default
	USART2->CR1 |= USART_CR1_UE; 		// Enables USART
	//USART2->CR1 |= USART_CR1_OVER8; 		// if 0-> 16x over-sampling; if 1-> 8x over-sampling

	USART2->BRR &= ~USART_BRR_DIV_Mantissa_Msk;		// 9600 Baud
	USART2->BRR &= ~USART_BRR_DIV_Fraction_Msk;

	USART2->BRR |= (mantissa << USART_BRR_DIV_Mantissa_Pos);		// 9600 Baud
	USART2->BRR |= (fraction << USART_BRR_DIV_Fraction_Pos);

	USART2->CR1 |= USART_CR1_TE; 		// Transmitter enabled
}

void delay(uint32_t cycles) {
	for (uint32_t i = 0; i < cycles; i++) {} //7059 is from experimentation
}
