#include "main.h"
#include "string.h"
#include "lcd.h"
#include "stdio.h"
#include "time.h"
#include "stdlib.h"
//#include "gmp.h"

#define BUTTON_MODE 0 	//0-HOLD 1-TOGGLE
#define LED_GREEN GPIO_ODR_OD12
#define LED_ORANGE GPIO_ODR_OD13
#define LED_RED GPIO_ODR_OD14
#define LED_BLUE GPIO_ODR_OD15
#define NUMTRIALS 100

typedef enum
{
	ZERO, RANDOM,
} Init_Struct_Value;

typedef struct
{
	uint64_t num;
} Struct8;

typedef struct
{
	uint64_t num[16];
} Struct128;

typedef struct
{
	uint64_t num[128];
} Struct1024;

void TIM2_START(void);
void TIM2_STOP(void);
void delay(const uint32_t cycles);

void init_struct(void *struct_to_init, int size, Init_Struct_Value value);

void init_hardware(void);

int main(void)
{
	srand((unsigned) time(0));

	const int cycles = 9600000;
	uint32_t timercount = 0;
	char timerword[32];

	uint32_t integer1, integer2;
	uint64_t longlong1, longlong2;
	float int_result, longlong_result;
	uint64_t time_of_all_trials, average_run_time;

	Struct8 num8a, num8b;
	Struct128 num128a, num128b;
	Struct1024 num1024a, num1024b;

	num8a.num = 0;
	num8b.num = rand();

	init_struct(&num128b, sizeof(num128b), ZERO);
	init_struct(&num128b, sizeof(num128b), RANDOM);
	init_struct(&num1024b, sizeof(num1024b), ZERO);
	init_struct(&num1024b, sizeof(num1024b), RANDOM);

	init_hardware();

	// MAIN LOOP
	while (1)
	{
		// ADD 32 INT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for (volatile int i = 0; i < NUMTRIALS; i++)
			{
				integer1 = rand();
				integer2 = rand();
				TIM2_START();
				int_result = integer1 + integer2;
				TIM2_STOP();
				timercount = ((TIM2->CNT * 1e9) / 96e6);
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
				printf("+32: %d", int_result);
			}
			{
				GPIOD->ODR |= LED_GREEN;
				average_run_time = time_of_all_trials / NUMTRIALS;
				sprintf(timerword, "ADD32: %7dns", average_run_time);
				Clear_Display();
				Write_String(timerword);
				delay(cycles);
				GPIOD->ODR &= ~LED_BLUE;
				GPIOD->ODR &= ~LED_GREEN;
				delay(cycles);
			}

		}
		// ADD 64 INT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for (volatile int i = 0; i < NUMTRIALS; i++)
			{
				longlong1 = rand() + rand();
				longlong2 = rand() + rand();
				TIM2_START();
				longlong_result = longlong1 + longlong2;
				TIM2_STOP();
				timercount = ((TIM2->CNT * 1e9) / 96e6);
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
				printf("+64: %d", longlong_result);
			}
			{
				GPIOD->ODR |= LED_GREEN;
				average_run_time = time_of_all_trials / NUMTRIALS;
				sprintf(timerword, "ADD64: %7dns", average_run_time);
				Clear_Display();
				Write_String(timerword);
				delay(cycles);
				GPIOD->ODR &= ~LED_BLUE;
				GPIOD->ODR &= ~LED_GREEN;
				delay(cycles);
			}

		}
		// MULTIPLY 32 INT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for (volatile int i = 0; i < NUMTRIALS; i++)
			{
				longlong1 = rand() + rand();
				longlong2 = rand() + rand();
				TIM2_START();
				int_result = integer1 * integer2;
				TIM2_STOP();
				timercount = ((TIM2->CNT * 1e9) / 96e6);
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
				printf("x32: %d", int_result);
			}
			{
				GPIOD->ODR |= LED_GREEN;
				average_run_time = time_of_all_trials / NUMTRIALS;
				sprintf(timerword, "MULT32: %6dns", average_run_time);
				Clear_Display();
				Write_String(timerword);
				delay(cycles);
				GPIOD->ODR &= ~LED_BLUE;
				GPIOD->ODR &= ~LED_GREEN;
				delay(cycles);
			}
		}
		// MUTIPLY 64 INT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for (volatile int i = 0; i < NUMTRIALS; i++)
			{
				longlong1 = rand() + rand();
				longlong2 = rand() + rand();
				TIM2_START();
				longlong_result = longlong1 * longlong2;
				TIM2_STOP();
				timercount = ((TIM2->CNT * 1e9) / 96e6);
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
				printf("x64: %d", longlong_result);
			}
			{
				GPIOD->ODR |= LED_GREEN;
				average_run_time = time_of_all_trials / NUMTRIALS;
				sprintf(timerword, "MULT64: %6dns", average_run_time);
				Clear_Display();
				Write_String(timerword);
				delay(cycles);
				GPIOD->ODR &= ~LED_BLUE;
				GPIOD->ODR &= ~LED_GREEN;
				delay(cycles);
			}

		}
		// DIVIDE 32 INT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for (volatile int i = 0; i < NUMTRIALS; i++)
			{
				integer1 = rand();
				integer2 = rand();
				TIM2_START();
				if (integer2 > 0)
				{
					int_result = (float) integer1 / (float) integer2;
				}
				else
				{
					fprintf(stderr, "Cannot divide by zero\n");
				}
				TIM2_STOP();
				timercount = ((TIM2->CNT * 1e9) / 96e6);
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
				printf("/32: %d", int_result);
			}
			{
				GPIOD->ODR |= LED_GREEN;
				average_run_time = time_of_all_trials / NUMTRIALS;
				sprintf(timerword, "DIV32: %7dns", average_run_time);
				Clear_Display();
				Write_String(timerword);
				delay(cycles);
				GPIOD->ODR &= ~LED_BLUE;
				GPIOD->ODR &= ~LED_GREEN;
				delay(cycles);
			}
		}
		// DIVIDE 64 INT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for (volatile int i = 0; i < NUMTRIALS; i++)
			{
				longlong1 = rand() + rand();
				longlong2 = rand() + rand();
				TIM2_START();
				if (longlong2 > 0)
				{
					longlong_result = (float) longlong1 / (float) longlong2;
				}
				else
				{
					fprintf(stderr, "Cannot divide by zero\n");
				}
				TIM2_STOP();
				timercount = ((TIM2->CNT * 1e9) / 96e6);
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
				printf("/64: %d", longlong_result);
			}
			{
				GPIOD->ODR |= LED_GREEN;
				average_run_time = time_of_all_trials / NUMTRIALS;
				sprintf(timerword, "DIV64: %7dns", average_run_time);
				Clear_Display();
				Write_String(timerword);
				delay(cycles);
				GPIOD->ODR &= ~LED_BLUE;
				GPIOD->ODR &= ~LED_GREEN;
				delay(cycles);
			}

		}
		// COPY 8 BYTE STRUCT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for (volatile int i = 0; i < NUMTRIALS; i++)
			{
				num8a.num = 0;
				num8b.num = rand();
				TIM2_START();
				num8a = num8b;
				TIM2_STOP();
				timercount = ((TIM2->CNT * 1e9) / 96e6);
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
				printf("+32: %d", num8a.num);
			}
			{
				GPIOD->ODR |= LED_GREEN;
				average_run_time = time_of_all_trials / NUMTRIALS;
				sprintf(timerword, "CPY8: %8dns", average_run_time);
				Clear_Display();
				Write_String(timerword);
				delay(cycles);
				GPIOD->ODR &= ~LED_BLUE;
				GPIOD->ODR &= ~LED_GREEN;
				delay(cycles);
			}

		}
		// COPY 128 BYTE STRUCT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for (volatile int i = 0; i < NUMTRIALS; i++)
			{
				init_struct(&num128a, sizeof(&num128a), ZERO);
				init_struct(&num128b, sizeof(&num128b), RANDOM);
				TIM2_START();
				num128a = num128b;
				TIM2_STOP();
				timercount = ((TIM2->CNT * 1e9) / 96e6);
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
				printf("CPY128: %d", num128a);
			}
			{
				GPIOD->ODR |= LED_GREEN;
				average_run_time = time_of_all_trials / NUMTRIALS;
				sprintf(timerword, "CPY128: %6dns", average_run_time);
				Clear_Display();
				Write_String(timerword);
				delay(cycles);
				GPIOD->ODR &= ~LED_BLUE;
				GPIOD->ODR &= ~LED_GREEN;
				delay(cycles);
			}

		}
		// COPY 1024 BYTE STRUCT
		{
			time_of_all_trials = 0;
			GPIOD->ODR |= LED_BLUE;
			for (volatile int i = 0; i < NUMTRIALS; i++)
			{
				init_struct(&num1024a, sizeof(&num1024a), ZERO);
				init_struct(&num1024b, sizeof(&num1024b), RANDOM);
				TIM2_START();
				num1024a = num1024b;
				TIM2_STOP();
				timercount = ((TIM2->CNT * 1e9) / 96e6);
				time_of_all_trials += timercount;
				TIM2->CNT = 0;
				printf("CPY1024: %d", num1024a);
			}
			{
				GPIOD->ODR |= LED_GREEN;
				average_run_time = time_of_all_trials / NUMTRIALS;
				sprintf(timerword, "CPY1024: %5dns", average_run_time);
				Clear_Display();
				Write_String(timerword);
				delay(cycles);
				GPIOD->ODR &= ~LED_BLUE;
				GPIOD->ODR &= ~LED_GREEN;
				delay(cycles);
			}

		}
	}
	return 0;
}

void TIM2_IRQHandler(void)
{
	TIM2->SR &= ~TIM_SR_UIF;
	GPIOD->ODR ^= LED_RED;
}
void TIM2_START(void)
{
	TIM2->CR1 |= TIM_CR1_CEN;
}
void TIM2_STOP(void)
{
	TIM2->CR1 &= ~TIM_CR1_CEN;
}
void delay(const uint32_t cycles)
{
	for (volatile uint32_t i = 0; i < cycles; i++)
	{
	}
}

void init_struct(void *struct_to_init, int size, Init_Struct_Value value)
{
	// for Struct128
	if (((size == (sizeof(*(Struct128*) struct_to_init)) && value == ZERO)))
	{
		for (volatile int i = 0;
				i < (size / (sizeof((*(Struct128*) struct_to_init).num[i])));
				i++)
		{
			(*(Struct128*) struct_to_init).num[i] = 0;
		}
	}
	else if (((size == (sizeof(*(Struct128*) struct_to_init)) && value == RANDOM)))
	{
		for (volatile int i = 0;
				i < (size / (sizeof((*(Struct128*) struct_to_init).num[i])));
				i++)
		{
			(*(Struct128*) struct_to_init).num[i] = rand();
		}
	}
	else
	{
		fprintf(stderr, "unknown struct type\n");
	}
	// for Struct1024
	if (((size == (sizeof(*(Struct1024*) struct_to_init)) && value == ZERO)))
	{
		for (volatile int i = 0;
				i < (size / (sizeof((*(Struct1024*) struct_to_init).num[i])));
				i++)
		{
			(*(Struct1024*) struct_to_init).num[i] = 0;
		}
	}
	else if (((size == (sizeof(*(Struct1024*) struct_to_init))
			&& value == RANDOM)))
	{
		for (volatile int i = 0;
				i < (size / (sizeof((*(Struct1024*) struct_to_init).num[i])));
				i++)
		{
			(*(Struct1024*) struct_to_init).num[i] = rand();
		}
	}
	else
	{
		fprintf(stderr, "unknown struct type\n");
	}
}

void init_hardware(void)
{
	// @formatter:off
	/* FLASH AND POWER */

		{
			PWR->CR |= PWR_CR_VOS;
			FLASH->ACR |= 	FLASH_ACR_ICEN 			|
							FLASH_ACR_DCEN			|
							FLASH_ACR_PRFTEN 		|
							FLASH_ACR_LATENCY_3WS	;
		}

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
		}
			// @formatter:on
	}
