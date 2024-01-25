#include "main.h"

#define BUTTON_MODE 0 										//0-HOLD 1-TOGGLE

#define LED_GREEN GPIO_ODR_OD12
#define LED_ORANGE GPIO_ODR_OD13
#define LED_RED GPIO_ODR_OD14
#define LED_BLUE GPIO_ODR_OD15

void FLASH_AND_POWER_CONFIG(void);

void GPIO_CONFIG(void);

void HSI_PLL_CLK_EN(void);
void HSE_PLL_CLK_EN(void);
void PPLI2S_CONFIG(void);
void I2S_CONFIG(void);

void SYSTICK_CONFIG(void);
void TIM2_CONFIG(uint32_t timeMilliSeconds);
void TIM10_CONFIG(uint32_t frequency);

void I2C_CONFIG(void);
void I2C_START(void);
void I2C_STOP(void);
void I2C_WRITE(uint8_t, uint8_t);
void DAC_POWER_UP(void);

void delay(uint32_t cycles);

int main(void)
{

	SystemCoreClockUpdate();

	FLASH_AND_POWER_CONFIG(); // for HCLK = 96MHz
	GPIO_CONFIG();
	HSE_PLL_CLK_EN();
	PPLI2S_CONFIG();
	SystemCoreClockUpdate();
	I2S_CONFIG();
	I2C_CONFIG();
	DAC_POWER_UP();


	I2C_WRITE(0x1C, 0x7F);	// Beep address; beep frequency (260Hz) & time (5.2s)
	I2C_WRITE(0x1E, 0xC0);	// beep & tone configuration address
	I2C_WRITE(0x04, 0x2);	// headphones always on
	I2C_WRITE(0x05, 0x1);	// auto detect clock when CS43L22 is slave

	GPIOD->ODR |= LED_GREEN;

}

void FLASH_AND_POWER_CONFIG(void){
	PWR->CR |= PWR_CR_VOS;
	FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_3WS;
}

void SYSTICK_CONFIG(void){
	SysTick->LOAD |= SysTick_LOAD_RELOAD_Msk; // this is about 1 second
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

//	GPIOD->OSPEEDR |= 	GPIO_OSPEEDER_OSPEEDR12 |
//						GPIO_OSPEEDER_OSPEEDR13 |
//						GPIO_OSPEEDER_OSPEEDR14 |
//						GPIO_OSPEEDER_OSPEEDR15;
}

void TIM2_CONFIG(uint32_t timeMilliSeconds){
	SystemCoreClockUpdate();
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_PWREN;
	TIM2->ARR = timeMilliSeconds; // 1000 * 1ms = 1s
	TIM2->PSC = ((SystemCoreClock / 2) / 1000) - 1; // 1ms
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR1 |= TIM_CR1_CEN;
	//__NVIC_EnableIRQ(TIM2_IRQn); //
	NVIC->ISER[0] |= 1 << TIM2_IRQn; // IRQn of TIM2 => 0x10000000 = '28' (bit 28)
}
void TIM10_CONFIG(uint32_t frequency){
	SystemCoreClockUpdate();

	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
	TIM10->ARR = (1000000/(2*frequency)); // set in us
	TIM10->PSC = (SystemCoreClock / 1000000) - 1; //1 us
	TIM10->DIER |= TIM_DIER_UIE;
	TIM10->CR1 |= TIM_CR1_CEN;
	//__NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	NVIC->ISER[0] |= 1 << TIM1_UP_TIM10_IRQn; // IRQn of TIM2 = 25
}
void delay(uint32_t cycles){
	for(uint32_t i = 0; i < cycles; i++){}
}
void SysTick_Handler(void){
	//GPIOD->ODR ^= LED_BLUE;
}
void TIM2_IRQHandler(void){
	TIM2->SR &= ~TIM_SR_UIF;
	//GPIOD->ODR ^= LED_ORANGE;
}
void TIM1_UP_TIM10_IRQHandler(void){
	TIM10->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
	//GPIOD->ODR ^= LED_GREEN; // Toggle the green LED
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

	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;

	RCC->CR |= RCC_CR_HSION;
	while(!(RCC->CR & RCC_CR_HSIRDY)){}

	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY)){}

	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)){}
}
void PPLI2S_CONFIG(void){
	// PLLI2S	M-5	| N-200	| R-2
	RCC->PLLI2SCFGR |= (	RCC_PLLI2SCFGR_PLLI2SN_7 |
							RCC_PLLI2SCFGR_PLLI2SN_6 |
							RCC_PLLI2SCFGR_PLLI2SN_3 );
	RCC->PLLI2SCFGR &= ~(	RCC_PLLI2SCFGR_PLLI2SN_8 |
							RCC_PLLI2SCFGR_PLLI2SN_5 |
							RCC_PLLI2SCFGR_PLLI2SN_4 |
							RCC_PLLI2SCFGR_PLLI2SN_2 |
							RCC_PLLI2SCFGR_PLLI2SN_1 |
							RCC_PLLI2SCFGR_PLLI2SN_0 );

	RCC->PLLI2SCFGR |= (	RCC_PLLI2SCFGR_PLLI2SM_2 |
							RCC_PLLI2SCFGR_PLLI2SM_0 );
	RCC->PLLI2SCFGR &= ~(	RCC_PLLI2SCFGR_PLLI2SM_5 |
							RCC_PLLI2SCFGR_PLLI2SM_4 |
							RCC_PLLI2SCFGR_PLLI2SM_3 |
							RCC_PLLI2SCFGR_PLLI2SM_1 );
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

	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

	RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY)){}

	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY)){}

	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)){}
}
void I2S_CONFIG(void){
	/* DAC - CS43L22
	 *
	 * CS43		??		 		Pin			Alternate function
	 * -------------------------------------------------------
	 * SDA 		Audio_SDA 		PB9			I2C1_SDA		AF04
	 * SCL 		Audio_SCL 		PB6			I2C1_SCL		AF04
	 *
	 * MCLK 	I2S3_MCK 		PC7			I2S3_MCK		AF06
	 * SCLK 	I2S3_SCK		PC10		I2S3_CK
	 * SDIN 	I2S3_SD			PC12		I2S3_SD
	 * LRCK		I2S3_WS			PA4			I2S3_WS
	 * RESET 	Audio_RST		PD4			n/a
	 *
	 * AIN1A/B	Audio_DAC_OUT	PA4
	 * AIN4A/B	PDM_OUT			PC3(4 too?)	I2S2_SD
	 *
	 * APB1 => I2C1 & IS23 bus
	 * APB2 frequency = 24MHz
	*/

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN; // unsure if necessary, SPI3 shares port with I2S3
	GPIOC->MODER |= GPIO_MODER_MODE7_1; // alt function
	GPIOC->AFR[0] |= (0x6<<28); // AF6 (0x6) in pin 7 position
	GPIOC->PUPDR |= GPIO_PUPDR_PUPD7_0; // pull up
	GPIOC->OTYPER |= GPIO_OTYPER_OT7; // open drain - is this correct config for I2C?

	SPI3->I2SCFGR |= SPI_I2SCFGR_I2SMOD;
	SPI3->I2SCFGR |= SPI_I2SCFGR_I2SCFG; // do before enabled I2S; what mode??
	SPI3->I2SPR |= SPI_I2SPR_MCKOE;
    //SPI3->I2SPR |= (1 << 8); // stole this... need to configure
    //SPI3->I2SPR |= (3 << 0); // stole this... need to configure
	SPI3->I2SCFGR |= SPI_I2SCFGR_I2SE; // Enable I2S
}

void I2C_CONFIG(void){

	/* DAC - CS43L22
	 *
	 * CS43		??		 		Pin			Alternate function
	 * -------------------------------------------------------
	 * SDA 		Audio_SDA 		PB9			I2C1_SDA		AF04
	 * SCL 		Audio_SCL 		PB6			I2C1_SCL		AF04
	 *
	 * MCLK 	I2S3_MCK 		PC7			I2S3_MCK
	 * SCLK 	I2S3_SCK		PC10		I2S3_CK
	 * SDIN 	I2S3_SD			PC12		I2S3_SD
	 * LRCK		I2S3_WS			PA4			I2S3_WS
	 * RESET 	Audio_RST		PD4			n/a
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

	// starts in slave, generate START => master mode (automatically?)
	// sequence start
	I2C1->CR2 |= 0x18; // 24 MHz
	I2C1->CCR |= 0x78; // CCR = Thigh / Tpclk1 = 5us/41.666ns = 120
	I2C1->TRISE &= ~(I2C_TRISE_TRISE_Msk);
	//I2C1->TRISE |= 0x19; // (1000 ns / (1 / 24 MHz))
	I2C1->TRISE |= (25<<0); // (1000 ns / (1 / 24 MHz))
	I2C1->CR1 |= I2C_CR1_ACK; // ACK on
	I2C1->CR1 |= I2C_CR1_PE; // Peripheral Enable I2C
}
void I2C_WRITE(uint8_t regaddress, uint8_t data){
	I2C_START();
	// chip address always starts with '0b1001010x' (0x94) AD0 is always 0 (connected to DGND) I think...
	I2C1->DR = 0x94; // address phase
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}
	//dummy = I2C1->SR1 | I2C1->SR2; // this clears ADDR
	I2C1->SR1 | I2C1->SR2;
	I2C1->DR = regaddress;
	//while(!(I2C1->SR1 & I2C_SR1_AF)){}
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}
	I2C1->DR = data;
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}
	while(!(I2C1->SR1 & I2C_SR1_BTF)){}
	I2C_STOP();
}
void I2C_START(void){
	I2C1->CR1 |= I2C_CR1_START; // starts master mode from default
	while(!(I2C1->SR1 & I2C_SR1_SB)){} // wait for start bit to go high
}
void I2C_STOP(void){
	I2C1->CR1 |= I2C_CR1_STOP;
	while(!(I2C1->SR2 & I2C_SR2_BUSY));
}
void DAC_POWER_UP(void){
	// Power Ctl.1 (is already set to this by default but idk?)
	I2C_WRITE(0x02, 0x01);
	// write 0x99 to register 0x00
	I2C_WRITE(0x00, 0x99);
	// write 0x80 to register 0x47
	I2C_WRITE(0x47, 0x80);
	// write '1'b to bit 7 in register 0x32
	I2C_WRITE(0x32, 0xBC);// default is 0b00111011 => 0xBC //(0x0<<7)
	// write '0'b to bit 7 in register 0x32
	I2C_WRITE(0x32, 0x3C); // default is 0b00111011 => 0x3C //(0x0<<7)
	// write 0x00 to register 0x00
	I2C_WRITE(0x00,0x00);
	// apply MCLK at appropriate freq
	I2C_WRITE(0x05,0x23); // 48KHz: 0bx0100011 => 0x23
	// Power Ctl.1 'Powered Up'
	I2C_WRITE(0x02,0x9E);
	delay(1000000);
}
