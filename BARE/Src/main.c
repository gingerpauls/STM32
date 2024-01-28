#include "main.h"
#include "string.h"
#include "lcd.h"
#include "stdio.h"
#include "time.h"
#include "stdlib.h"
#include "gmp.h"

#define BUTTON_MODE 0 	//0-HOLD 1-TOGGLE
#define LED_GREEN GPIO_ODR_OD12
#define LED_ORANGE GPIO_ODR_OD13
#define LED_RED GPIO_ODR_OD14
#define LED_BLUE GPIO_ODR_OD15
#define NUMTRIALS 100

void TIM2_START(void);
void TIM2_STOP(void);
void delay(const uint32_t cycles);

typedef struct {
	char letter;
} Struct8;

typedef struct {
	char* sentence;
} Struct128;

typedef struct {
	char* sentence;
} Struct1024;

int main(void) {
	const int cycles = 9600000;
	volatile uint32_t timercount = 0;
	char timerword[32];

	volatile uint32_t integer1, integer2;
	volatile uint64_t longlong1, longlong2;
	volatile float int_result, longlong_result;
	volatile uint64_t time_of_all_trials, average_run_time;


	srand((unsigned) time(0));

	volatile Struct8 letter1, letter2;
	letter1.letter = 'c';
	letter2.letter = ' ';
	volatile Struct128 sentence1a, sentence1b;

	sentence1a.sentence = 	"The quick brown fox jumps over the lazy dog, "
							"showcasing its agility and speed in a single "
							"graceful leap.";
	volatile Struct1024 sentence2a, sentence2b;
	sentence2a.sentence = 	"In the vast expanse of the digital realm, where "
							"information flows like a ceaseless river, technology "
							"intertwines with human existence, shaping the landscape "
							"of our interconnected lives, influencing communication, "
							"innovation, and the very fabric of society, as we "
							"navigate this complex network of ones and zeros, forging "
							"a path towards an uncertain future filled with both promise "
							"and challenges, where the boundaries between the physical and "
							"virtual blur, giving rise to new possibilities and profound "
							"transformations that echo across the ever-evolving tapestry "
							"of human experience.";
	sentence1b.sentence = " ";
	sentence2b.sentence = " ";
/*
	{
		integer1 = rand();
		integer2 = rand();
		int_result = integer1 + integer2;
	}
*/

/*
	{
		longlong1 = rand() + rand();
		longlong2 = rand() + rand();
		longlong_result = longlong1 + longlong2;
	}
*/

/*

	{
		integer1 = rand();
		integer2 = rand();
		int_result = integer1 * integer2;
	}

*/

/*
	{
		longlong1 = rand() + rand();
		longlong2 = rand() + rand();
		longlong_result = longlong1 * longlong2;
	}

*/

/*

	{
		integer1 = rand();
		integer2 = rand();
		if(integer2 > 0){
			int_result = (float)integer1 / (float)integer2;
		}
		else{
			fprintf(stderr, "Cannot divide by zero\n");
		}
	}

*/

/*

	{
		longlong1 = rand() + rand();
		longlong2 = rand() + rand();
		if(longlong2 > 0){
			longlong_result = (float)longlong1 / (float)longlong2;
		}
		else{
			fprintf(stderr, "Cannot divide by zero\n");
		}
	}
*/

/*

	{
		letter2.letter = letter1.letter;
		sentence1b.sentence = sentence1a.sentence;
		sentence2b.sentence = sentence2a.sentence;
	}
*/


	/* FLASH AND POWER */
	{
		PWR->CR |= PWR_CR_VOS;
		FLASH->ACR |= 	FLASH_ACR_ICEN 			|
						FLASH_ACR_DCEN			|
						FLASH_ACR_PRFTEN 		|
						FLASH_ACR_LATENCY_3WS	;
	}

	/* SYSTICK CONFIG */
/*
	{
		SysTick->LOAD |= SysTick_LOAD_RELOAD_Msk; // this is about 1 second
		//NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
		SysTick->VAL &= ~SysTick_VAL_CURRENT_Msk;
		SysTick->CTRL |= 	SysTick_CTRL_CLKSOURCE_Msk 	|
							SysTick_CTRL_TICKINT_Msk 	|
							SysTick_CTRL_ENABLE_Msk		;
	}
*/

	/* HSE PLL CONFIG AND ENABLE */
	/* PLL 		M-8 	| N-192 	| P-4 	| Q-8
	 * PLLI2S	M-10	| N-200		| R-2
	 * AHB		1
	 * APB1		4
	 * APB2		1
	 */
	{
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
	}

	/* LED CONFIG */
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
		GPIOD->MODER |= 	GPIO_MODER_MODE12_0 |
							GPIO_MODER_MODE13_0 |
							GPIO_MODER_MODE14_0 |
							GPIO_MODER_MODE15_0	; // set GPIO to "output" mode for LEDs
	}

	//Reset_Baud_Rate();

	/* 	USART2_TX */
	/*  PA2 (alternate function) uses AHB
	 *	USART on PA2 -> AF7
	 *	LCD:
	 *		5V TTL
	 *		9600 baud
	 *		8 bits
	 *		1 stop
	 *		no parity
	 */
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		GPIOA->MODER |= GPIO_MODER_MODE2_1; // Alternate function
		GPIOA->AFR[0] |= ( 	GPIO_AFRL_AFRL2_0 	|
							GPIO_AFRL_AFRL2_1 	|
							GPIO_AFRL_AFRL2_2	);
		USART2->CR2 |= (0x0 << USART_CR2_STOP_Pos);// Stop bit: 0b00 is 1 stop bit by default
		USART2->CR1 |= USART_CR1_UE; 		// Enables USART
		//USART2->CR1 |= USART_CR1_OVER8; 		// if 0-> 16x over-sampling; if 1-> 8x over-sampling
		USART2->BRR &= ~USART_BRR_DIV_Mantissa_Msk;		// 9600 Baud
		USART2->BRR &= ~USART_BRR_DIV_Fraction_Msk;
		USART2->BRR |= (0x9C << USART_BRR_DIV_Mantissa_Pos);
		USART2->BRR |= (0x4 << USART_BRR_DIV_Fraction_Pos);
		USART2->CR1 |= USART_CR1_TE; 		// Transmitter enabled
	}

	/* TIM2 CONFIG */
	{
		SystemCoreClockUpdate();
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_PWREN;
		TIM2->ARR = TIM_ARR_ARR;
		TIM2->PSC = 0;
		TIM2->EGR |= TIM_EGR_UG;
	//	TIM2->DIER |= TIM_DIER_UIE;
	//	__NVIC_EnableIRQ(TIM2_IRQn);
	//	NVIC->ISER[0] |= 1 << TIM2_IRQn; // IRQn of TIM2 => 0x10000000 = '28' (bit 28)
	}

	/* START PROMPT */
	Clear_Display();
	Write_String("BEGIN STOPWATCH");
	delay(cycles);
	Clear_Display();
	while(1)
	{
		// ADD 32 INT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for(volatile int i = 0; i < NUMTRIALS; i++)
			{
				integer1 = rand();
				integer2 = rand();
				TIM2_START();
				int_result = integer1 + integer2;
				TIM2_STOP();
				timercount = ((TIM2->CNT*1e9)/96e6); // converts count to nanoseconds 	(PSC = 0 96MHz)
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
			}
			GPIOD->ODR |= LED_GREEN;
			average_run_time = time_of_all_trials / NUMTRIALS;
			sprintf(timerword, "ADD32: %d ns", average_run_time);
			Clear_Display();
			Write_String(timerword);
			delay(cycles);
			Clear_Display();
			GPIOD->ODR &= ~LED_BLUE;
			GPIOD->ODR &= ~LED_GREEN;
			delay(cycles);
		}
		// ADD 64 INT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for(volatile int i = 0; i < NUMTRIALS; i++)
			{
				longlong1 = rand() + rand();
				longlong2 = rand() + rand();
				TIM2_START();
				longlong_result = longlong1 + longlong2;
				TIM2_STOP();
				timercount = ((TIM2->CNT*1e9)/96e6); // converts count to nanoseconds 	(PSC = 0 96MHz)
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
			}
			GPIOD->ODR |= LED_GREEN;
			average_run_time = time_of_all_trials / NUMTRIALS;
			sprintf(timerword, "ADD64: %d ns", average_run_time);
			Clear_Display();
			Write_String(timerword);
			delay(cycles);
			Clear_Display();
			GPIOD->ODR &= ~LED_BLUE;
			GPIOD->ODR &= ~LED_GREEN;
			delay(cycles);
		}
		// MULTIPLY 32 INT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for(volatile int i = 0; i < NUMTRIALS; i++)
			{
				longlong1 = rand() + rand();
				longlong2 = rand() + rand();
				TIM2_START();
				int_result = integer1 * integer2;
				TIM2_STOP();
				timercount = ((TIM2->CNT*1e9)/96e6); // converts count to nanoseconds 	(PSC = 0 96MHz)
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
			}
			GPIOD->ODR |= LED_GREEN;
			average_run_time = time_of_all_trials / NUMTRIALS;
			sprintf(timerword, "MULT32: %d ns", average_run_time);
			Clear_Display();
			Write_String(timerword);
			delay(cycles);
			Clear_Display();
			GPIOD->ODR &= ~LED_BLUE;
			GPIOD->ODR &= ~LED_GREEN;
			delay(cycles);
		}
		// MUTIPLY 64 INT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for(volatile int i = 0; i < NUMTRIALS; i++)
			{
				longlong1 = rand() + rand();
				longlong2 = rand() + rand();
				TIM2_START();
				longlong_result = longlong1 * longlong2;
				TIM2_STOP();
				timercount = ((TIM2->CNT*1e9)/96e6); // converts count to nanoseconds 	(PSC = 0 96MHz)
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
			}
			GPIOD->ODR |= LED_GREEN;
			average_run_time = time_of_all_trials / NUMTRIALS;
			sprintf(timerword, "MULT64: %d ns", average_run_time);
			Clear_Display();
			Write_String(timerword);
			delay(cycles);
			Clear_Display();
			GPIOD->ODR &= ~LED_BLUE;
			GPIOD->ODR &= ~LED_GREEN;
			delay(cycles);
		}

		// DIVIDE 32 INT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for(volatile int i = 0; i < NUMTRIALS; i++)
			{
				integer1 = rand();
				integer2 = rand();
				TIM2_START();
				if(integer2 > 0){
					int_result = (float)integer1 / (float)integer2;
				}
				else{
					fprintf(stderr, "Cannot divide by zero\n");
				}
				TIM2_STOP();
				timercount = ((TIM2->CNT*1e9)/96e6); // converts count to nanoseconds 	(PSC = 0 96MHz)
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
			}
			GPIOD->ODR |= LED_GREEN;
			average_run_time = time_of_all_trials / NUMTRIALS;
			sprintf(timerword, "DIV32: %d ns", average_run_time);
			Clear_Display();
			Write_String(timerword);
			delay(cycles);
			Clear_Display();
			GPIOD->ODR &= ~LED_BLUE;
			GPIOD->ODR &= ~LED_GREEN;
			delay(cycles);
		}
		// DIVIDE 64 INT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for(volatile int i = 0; i < NUMTRIALS; i++)
			{
				longlong1 = rand() + rand();
				longlong2 = rand() + rand();
				TIM2_START();
				if(longlong2 > 0){
					longlong_result = (float)longlong1 / (float)longlong2;
				}
				else{
					fprintf(stderr, "Cannot divide by zero\n");
				}
				TIM2_STOP();
				timercount = ((TIM2->CNT*1e9)/96e6); // converts count to nanoseconds 	(PSC = 0 96MHz)
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
			}
			GPIOD->ODR |= LED_GREEN;
			average_run_time = time_of_all_trials / NUMTRIALS;
			sprintf(timerword, "DIV64: %d ns", average_run_time);
			Clear_Display();
			Write_String(timerword);
			delay(cycles);
			Clear_Display();
			GPIOD->ODR &= ~LED_BLUE;
			GPIOD->ODR &= ~LED_GREEN;
			delay(cycles);
		}
		// COPY 8 BYTE STRUCT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for(volatile int i = 0; i < NUMTRIALS; i++)
			{
				TIM2_START();
				letter2.letter = letter1.letter;
				TIM2_STOP();
				timercount = ((TIM2->CNT*1e9)/96e6); // converts count to nanoseconds 	(PSC = 0 96MHz)
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
			}
			GPIOD->ODR |= LED_GREEN;
			average_run_time = time_of_all_trials / NUMTRIALS;
			sprintf(timerword, "CPY8: %d ns", average_run_time);
			Clear_Display();
			Write_String(timerword);
			delay(cycles);
			Clear_Display();
			GPIOD->ODR &= ~LED_BLUE;
			GPIOD->ODR &= ~LED_GREEN;
			delay(cycles);
		}
		// COPY 128 BYTE STRUCT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for(volatile int i = 0; i < NUMTRIALS; i++)
			{
				TIM2_START();
				sentence1b.sentence = sentence1a.sentence;
				TIM2_STOP();
				timercount = ((TIM2->CNT*1e9)/96e6); // converts count to nanoseconds 	(PSC = 0 96MHz)
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
			}
			GPIOD->ODR |= LED_GREEN;
			average_run_time = time_of_all_trials / NUMTRIALS;
			sprintf(timerword, "CPY128: %d ns", average_run_time);
			Clear_Display();
			Write_String(timerword);
			delay(cycles);
			Clear_Display();
			GPIOD->ODR &= ~LED_BLUE;
			GPIOD->ODR &= ~LED_GREEN;
			delay(cycles);
		}
		// COPY 1024 BYTE STRUCT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for(volatile int i = 0; i < NUMTRIALS; i++)
			{
				TIM2_START();
				sentence2b.sentence = sentence2a.sentence;
				TIM2_STOP();
				timercount = ((TIM2->CNT*1e9)/96e6); // converts count to nanoseconds 	(PSC = 0 96MHz)
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
			}
			GPIOD->ODR |= LED_GREEN;
			average_run_time = time_of_all_trials / NUMTRIALS;
			sprintf(timerword, "CPY1024: %d ns", average_run_time);
			Clear_Display();
			Write_String(timerword);
			delay(cycles);
			Clear_Display();
			GPIOD->ODR &= ~LED_BLUE;
			GPIOD->ODR &= ~LED_GREEN;
			delay(cycles);
		}
	}
	return 0;
}

void TIM2_IRQHandler(void) {
	TIM2->SR &= ~TIM_SR_UIF;
	GPIOD->ODR ^= LED_RED;
}
void TIM2_START(void){
	TIM2->CR1 |= TIM_CR1_CEN;
}
void TIM2_STOP(void){
	TIM2->CR1 &= ~TIM_CR1_CEN;
}
void delay(const uint32_t cycles) {
	for (volatile uint32_t i = 0; i < cycles; i++) {}
}
