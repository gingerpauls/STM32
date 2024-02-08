#include "main.h"
#include "string.h"
#include "lcd.h"
#include "stdio.h"
#include "stdbool.h"
#include "math.h"

#define LED_GREEN GPIO_ODR_OD12
#define LED_ORANGE GPIO_ODR_OD13
#define LED_RED GPIO_ODR_OD14
#define LED_BLUE GPIO_ODR_OD15

typedef enum
{
	HSI,
	HSE,
} PPL_CLK_SRC;

void FLASH_AND_POWER_CONFIG(void);
void SYSTICK_CONFIG(void);
void LED_CONFIG(void);

void PPL_CLK_EN(int clocksource);

void TIM2_CONFIG(uint32_t cycles);
void TIM10_CONFIG(uint32_t frequency);

void USART2_CONFIG(uint32_t baudrate, bool over8, uint32_t stopbits);

void I2C1_CONFIG(void);
void I2C1_START(void);
void I2C1_STOP(void);
void I2C1_WRITE(uint8_t, uint8_t);

void delay(uint32_t cycles);

int main(void) {
	char *word = "ABCDEFGHIJKLMNOPQRSTUVWXYZ123456";
	int cycles = 9600000;
	uint32_t timercount;
	char timerword[32];


	FLASH_AND_POWER_CONFIG();
	LED_CONFIG();
	PPL_CLK_EN(HSE);
	TIM2_CONFIG(TIM_ARR_ARR);
	//Reset_Baud_Rate(); // MUST REMOVE AFTER RESETTING
	USART2_CONFIG(38400, 0, 0);
	//Change_Baud_Rate(38400); // remove after setting
	I2C1_CONFIG();

	while(1) {
		GPIOD->ODR |= LED_GREEN;
		TIM2->CR1 |= TIM_CR1_CEN;
		Clear_Display();
		Write_String(word);
		TIM2->CR1 &= ~TIM_CR1_CEN;
		GPIOD->ODR &= ~LED_GREEN;
		timercount = ((TIM2->CNT * 1e9) / 96e6);
		sprintf(timerword, "%dns", timercount);
		delay(cycles);
		Clear_Display();
		Write_String(timerword);
		delay(cycles);
		TIM2->CNT = 0;
		Clear_Display();
		delay(cycles);
	}
}

void FLASH_AND_POWER_CONFIG(void) {
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;
	FLASH->ACR |= 	FLASH_ACR_ICEN 			|
					FLASH_ACR_DCEN			|
					FLASH_ACR_PRFTEN 		|
					FLASH_ACR_LATENCY_3WS	;
}
void LED_CONFIG(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER |= 	GPIO_MODER_MODE12_0 |
						GPIO_MODER_MODE13_0 |
						GPIO_MODER_MODE14_0 |
						GPIO_MODER_MODE15_0	; // set GPIO to "output" mode for LEDs
}

void PPL_CLK_EN(int clocksource){
	if(clocksource == HSI){
			/* PLL 		M-8 	| N-192 	| P-4 	| Q-8
			 * PLLI2S	M-10	| N-200		| R-2
			 * AHB		1
			 * APB1		4
			 * APB2		1
			*/
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
			RCC->DCKCFGR |= RCC_DCKCFGR_TIMPRE;

			RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;

			RCC->CR |= RCC_CR_HSION;
			while (!(RCC->CR & RCC_CR_HSIRDY));

			RCC->CR |= RCC_CR_PLLON;
			while (!(RCC->CR & RCC_CR_PLLRDY));

			RCC->CFGR |= RCC_CFGR_SW_PLL;
			while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
			SystemCoreClockUpdate();
	}
	else if (clocksource == HSE)
	{
		/* PLL 		M-4 	| N-192 	| P-4 	| Q-8
		 * PLLI2S	M-5		| N-200		| R-2
		 * AHB		1
		 * APB1		4
		 * APB2		1
		*/
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
		RCC->DCKCFGR |= RCC_DCKCFGR_TIMPRE;

		RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

		RCC->CR |= RCC_CR_HSEON;
		while (!(RCC->CR & RCC_CR_HSERDY)) {}

		RCC->CR |= RCC_CR_PLLON;
		while (!(RCC->CR & RCC_CR_PLLRDY)) {}

		RCC->CFGR |= RCC_CFGR_SW_PLL;
		while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) {}
		SystemCoreClockUpdate();
	}
	else
	{
		fprintf(stderr, "Select HSI or HSE as clock source.\n");
	}

}

void SYSTICK_CONFIG(void) {
	SysTick->LOAD |= SysTick_LOAD_RELOAD_Msk;
	//NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
	SysTick->VAL &= ~SysTick_VAL_CURRENT_Msk;
	SysTick->CTRL |= 	SysTick_CTRL_CLKSOURCE_Msk 	|
						SysTick_CTRL_TICKINT_Msk 	|
						SysTick_CTRL_ENABLE_Msk		;
}
void SysTick_Handler(void) {
	GPIOD->ODR ^= LED_BLUE;
}

void TIM2_CONFIG(uint32_t cycles) {
	SystemCoreClockUpdate();
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_PWREN;
	TIM2->ARR = cycles;
	TIM2->PSC = 0;
	TIM2->DIER |= TIM_DIER_UIE;
	//TIM2->CR1 |= TIM_CR1_CEN;
	//__NVIC_EnableIRQ(TIM2_IRQn); //
	//NVIC->ISER[0] |= 1 << TIM2_IRQn; // IRQn of TIM2 => 0x10000000 = '28' (bit 28)
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
	//TIM10->CR1 |= TIM_CR1_CEN;
	//__NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	//NVIC->ISER[0] |= 1 << TIM1_UP_TIM10_IRQn; // IRQn of TIM2 = 25
}
void TIM1_UP_TIM10_IRQHandler(void) {
	TIM10->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
	GPIOD->ODR ^= LED_GREEN; // Toggle the green LED
}

void USART2_CONFIG(uint32_t baudrate, bool over8, uint32_t stopbits) {
	/* 	USART2_TX: PA2->alternate function uses AHB & "Alternate Function 7"
	 *	LCD:
	 *		5V TTL
	 *		9600 baud default
	 *		8 bits
	 *		1 stop
	 *		no parity
	 */
	float usart_brr;
	uint32_t mantissa, fraction;

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	GPIOA->MODER |= GPIO_MODER_MODE2_1; // Alternate function
	GPIOA->AFR[0] |= ( 	GPIO_AFRL_AFRL2_0 	|
						GPIO_AFRL_AFRL2_1 	|
						GPIO_AFRL_AFRL2_2	);

	USART2->CR2 |= (stopbits << USART_CR2_STOP_Pos); // 0b00 is 1 stop bit by default
	USART2->CR1 |= USART_CR1_UE;
	USART2->CR1 |= over8; 		// if 0 -> 16x over-sampling; if 1 -> 8x over-sampling

	USART2->BRR &= ~USART_BRR_DIV_Mantissa_Msk;
	USART2->BRR &= ~USART_BRR_DIV_Fraction_Msk;

	usart_brr = (float)(SystemCoreClock / 4) / ( 8 * (2 - over8) * baudrate );
	mantissa = (uint32_t)usart_brr;
	fraction = round((usart_brr - mantissa) * (8 * (2 - over8)) );

	USART2->BRR |= (mantissa << USART_BRR_DIV_Mantissa_Pos);
	USART2->BRR |= (fraction << USART_BRR_DIV_Fraction_Pos);

	USART2->CR1 |= USART_CR1_TE;
}

void I2C1_CONFIG(void){
	/* I2C CONFIG - DAC - CS43L22
	 *
	 * CS43		??		 		Pin			Alternate function
	 * -------------------------------------------------------
	 * SDA 		Audio_SDA 		PB9			I2C1_SDA		AF04
	 * SCL 		Audio_SCL 		PB6			I2C1_SCL		AF04
	 *
	 * AIN1A/B	Audio_DAC_OUT	PA4
	 * AIN4A/B	PDM_OUT			PC3(4 too?)	I2S2_SD
	 *
	 * APB1 => I2C1 & IS23 bus
	 * APB2 frequency = 24MHz
	 */

	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // RCC_APB1ENR_PWREN? "Power interface clock enable"
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	GPIOB->MODER |= GPIO_MODER_MODE6_1; // alt function
	GPIOB->AFR[0] |= (0x4<<24); // AFR[0] is lower; 0x4 is AF4; 24 is bit position of Pin 6
	GPIOB->MODER |= GPIO_MODER_MODE9_1; // alt function
	GPIOB->AFR[1] |= (0x4<<4); // AFR[1] is higher; 0x4 is AF4; 4 is bit position of Pin 9
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD9_0; // pull up
	GPIOB->OTYPER |= GPIO_OTYPER_OT6 | GPIO_OTYPER_OT9; // open drain - is this correct config for I2C?

	GPIOD->MODER |= GPIO_MODER_MODE4_0; // output to RESET
	//GPIOD->MODER |= GPIO_MODER_MODE4_1; // alt function to RESET
	//GPIOD->AFR[0] |= 0x4<<16; // AFR[0] is lower; 0x4 is AF4; 16 is bit position of Pin 4
	GPIOD->ODR |= GPIO_ODR_OD4; // Sets RESET high (device is powered on)

	I2C1->CR1 |= I2C_CR1_SWRST; // this will clear the BUSY flag in SR2
	I2C1->CR1 &= ~I2C_CR1_SWRST; // this will clear the BUSY flag in SR2

	I2C1->CR2 |= (24<<I2C_CR2_FREQ_Pos); // APB1 => 24 MHz
	I2C1->CCR |= (120<<I2C_CCR_CCR_Pos); // CCR = Thigh / Tpclk1 = 5us/41.666ns = 120
	//I2C1->TRISE &= ~(I2C_TRISE_TRISE_Msk);
	I2C1->TRISE |= (25<<I2C_TRISE_TRISE_Pos); // (1000 ns / (1 / 24 MHz))
	I2C1->CR1 |= I2C_CR1_ACK; // ACK on
	I2C1->CR1 |= I2C_CR1_PE; // Peripheral Enable I2C
}
void I2C1_START(void){
	I2C1->CR1 |= I2C_CR1_START; // starts master mode from default
	while(!(I2C1->SR1 & I2C_SR1_SB)){} // wait for start bit to go high
}
void I2C1_STOP(void){
	I2C1->CR1 |= I2C_CR1_STOP;
	while(!(I2C1->SR2 & I2C_SR2_BUSY));
}
void I2C1_WRITE(uint8_t regaddress, uint8_t data){
	I2C1_START();
	I2C1->DR = 0x94;
	while(I2C1->SR1 & I2C_SR1_AF){}
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}
	(void)I2C1->SR2; // reading SR2 after SR1 will clear the ADDR flag
	I2C1->DR = regaddress;
	while(I2C1->SR1 & I2C_SR1_AF){}
	while(!(I2C1->SR1 & I2C_SR1_BTF)){}
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}
	I2C1->DR = data;
	while(I2C1->SR1 & I2C_SR1_AF){}
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}
	while(!(I2C1->SR1 & I2C_SR1_BTF)){}
	I2C1_STOP();
}

void delay(uint32_t cycles) {
	for (uint32_t i = 0; i < cycles; i++);
}
