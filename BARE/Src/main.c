#include "main.h"
#include "string.h"
#include "lcd.h"

#define BUTTON_MODE 0 										//0-HOLD 1-TOGGLE

#define LED_GREEN GPIO_ODR_OD12
#define LED_ORANGE GPIO_ODR_OD13
#define LED_RED GPIO_ODR_OD14
#define LED_BLUE GPIO_ODR_OD15

void FLASH_AND_POWER_CONFIG(void);

void SYSTICK_CONFIG(void);

void HSI_PLL_CLK_EN(void);
void HSE_PLL_CLK_EN(void);

void GPIO_CONFIG(void);

void TIM2_CONFIG(uint32_t cycles);
void TIM2_START(void);
void TIM2_STOP(void);
uint32_t TIM2_GETCURRENTVALUE(void);

void TIM10_CONFIG(uint32_t cycles);

void USART2_CONFIG(uint32_t, uint32_t, uint32_t);

void delay(uint32_t cycles);


int main(void) {
	char *word = "Hello Dad!     ";
	int cycles = 9600000;
	uint32_t timercount = 0;

	FLASH_AND_POWER_CONFIG(); // for HCLK = 96MHz
	GPIO_CONFIG();
	HSE_PLL_CLK_EN();
	USART2_CONFIG(0x9C, 0x4, 0x0); 	// 9600
	TIM2_CONFIG(TIM_ARR_ARR); // loads max counter value
	SystemCoreClockUpdate();

	while(1){
	GPIOD->ODR |= LED_ORANGE;
	TIM2_START();

	Clear_Display();
	Write_String(word);

	//delay(96);
	TIM2_STOP();
	GPIOD->ODR |= LED_GREEN;

	timercount = TIM2->CNT;
	//sscanf(timercount, "%")
	//Write_String(timercount);
	}

	// reset timer
}


void FLASH_AND_POWER_CONFIG(void) {
	PWR->CR |= PWR_CR_VOS;
	FLASH->ACR |= 	FLASH_ACR_ICEN 			|
					FLASH_ACR_DCEN			|
					FLASH_ACR_PRFTEN 		|
					FLASH_ACR_LATENCY_3WS	;
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

void GPIO_CONFIG(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN;

	GPIOD->MODER |= 	GPIO_MODER_MODE12_0 |
						GPIO_MODER_MODE13_0 |
						GPIO_MODER_MODE14_0 |
						GPIO_MODER_MODE15_0	; // set GPIO to "output" mode for LEDs

//	GPIOD->OSPEEDR |= 	GPIO_OSPEEDER_OSPEEDR12 |
//						GPIO_OSPEEDER_OSPEEDR13 |
//						GPIO_OSPEEDER_OSPEEDR14 |
//						GPIO_OSPEEDER_OSPEEDR15;
}

void TIM2_CONFIG(uint32_t cycles) {
	SystemCoreClockUpdate();
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_PWREN;
	TIM2->ARR = cycles; // 1000000 * 1us = 1s
	TIM2->PSC = ((SystemCoreClock / 2) / 1000000) - 1; // 1us; divide by two for APB1 Timer clock freq.
	TIM2->DIER |= TIM_DIER_UIE;
	__NVIC_EnableIRQ(TIM2_IRQn); //
	NVIC->ISER[0] |= 1 << TIM2_IRQn; // IRQn of TIM2 => 0x10000000 = '28' (bit 28)
}

void TIM2_START(void){
	TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM2_STOP(void){
	TIM2->CR1 &= ~TIM_CR1_CEN;
}

uint32_t TIM2_GETCURRENTVALUE(void){
	return TIM2->CNT;
}

void TIM2_IRQHandler(void) {
	TIM2->SR &= ~TIM_SR_UIF;
	GPIOD->ODR |= LED_RED;
}
void TIM10_CONFIG(uint32_t cycles) {
	SystemCoreClockUpdate();

	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
	TIM10->ARR = cycles; // set in us
	TIM10->PSC = (SystemCoreClock / 1000000) - 1; //1 us
	TIM10->DIER |= TIM_DIER_UIE;
	TIM10->CR1 |= TIM_CR1_CEN;
	//__NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	NVIC->ISER[0] |= 1 << TIM1_UP_TIM10_IRQn; // IRQn of TIM2 = 25
}
void TIM1_UP_TIM10_IRQHandler(void) {
	TIM10->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
	GPIOD->ODR ^= LED_GREEN; // Toggle the green LED
}

void USART2_CONFIG(uint32_t mantissa, uint32_t fraction, uint32_t stopbits) {
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

	USART2->CR2 |= (stopbits << USART_CR2_STOP_Pos);// Stop bit: 0b00 is 1 stop bit by default
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




