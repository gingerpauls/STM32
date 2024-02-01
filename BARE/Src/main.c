#include "main.h"

#define LED_GREEN GPIO_ODR_OD12
#define LED_ORANGE GPIO_ODR_OD13
#define LED_RED GPIO_ODR_OD14
#define LED_BLUE GPIO_ODR_OD15

void I2C_START(void);
void I2C_STOP(void);
void I2C_WRITE(uint8_t, uint8_t);
void delay(uint32_t cycles);

int main(void)
{
	/* Enable clock to set PWR register */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	/* POWER & FLASH CONFIG */
	{
		PWR->CR |= PWR_CR_VOS;
		FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_3WS;
	}

	/* LED GPIO CONFIG */
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

		GPIOD->MODER |= GPIO_MODER_MODE12_0 |
						GPIO_MODER_MODE13_0 |
						GPIO_MODER_MODE14_0 |
						GPIO_MODER_MODE15_0	;
	}


	/* HSE & PLL CONFIG
	 * PLL 		M-4 	| N-192 	| P-4 	| Q-8
	 * PLLI2S	M-5	| N-200		| R-2
	 * AHB		1
	 * APB1		4
	 * APB2		1
	 */
	{
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
	{
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

	/* I2SPLL CONFIG - DAC - CS43L22
	 * PLLI2S	M-5	| 	N-200	| 	R-2
	 */
	{
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

		RCC->CR |= RCC_CR_PLLI2SON;
		while(!(RCC->CR & RCC_CR_PLLI2SRDY)){}
	}

	/* I2S3 CONFIG - DAC - CS43L22
	 * PLLI2S	M-5	| 	N-200	| 	R-2
	 *
	 * CS43		??		 		Pin			Alternate function
	 * -------------------------------------------------------
	 * MCLK 	I2S3_MCK 		PC7			I2S3_MCK	AF06
	 * SCLK 	I2S3_SCK		PC10		I2S3_CK		AF06
	 * SDIN 	I2S3_SD			PC12		I2S3_SD		AF06
	 * LRCK		I2S3_WS			PA4			I2S3_WS		AF06
	 * RESET 	Audio_RST		PD4			n/a
	 *
	 * AIN1A/B	Audio_DAC_OUT	PA4
	 * AIN4A/B	PDM_OUT			PC3(4 too?)	I2S2_SD
	 *
	 * APB1 => I2C1 & IS23 bus
	 * APB2 frequency = 24MHz
	 */
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
		RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

		GPIOA->MODER |= GPIO_MODER_MODE4_1; // Port A4 => alternate function
		GPIOA->OSPEEDR |= (0x3<<8); // high speed
		GPIOA->AFR[0] |= (0x6<<16); // Port A4 => AF06

		GPIOC->MODER |= GPIO_MODER_MODE7_1 | GPIO_MODER_MODE10_1 | GPIO_MODER_MODE12_1;
		GPIOC->OSPEEDR |= (0x3<<14) | (0x3<<20) | (0x3<<24);
		GPIOC->AFR[0] |= GPIO_AFRL_AFRL7_1 | GPIO_AFRL_AFRL7_2;
		GPIOC->AFR[1] |= (0x6<<8) | (0x6<<16); //AF06 on Port C 10 & 12
	}

	/* Configure DAC - do before or after power up? */
	{

		I2C_WRITE(0x1C, 0x7F);	// Beep address; beep frequency (260Hz) & time (5.2s)
		I2C_WRITE(0x1E, 0xC0);	// beep & tone configuration address
		I2C_WRITE(0x04, 0x2);	// headphones always on
		I2C_WRITE(0x05, 0x1);	// auto detect clock when CS43L22 is slave
	}

	/* DAC POWER UP */
	{
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

	GPIOD->ODR |= LED_GREEN;

}

void I2C_WRITE(uint8_t regaddress, uint8_t data){
	I2C_START();
	// chip address always starts with '0b1001010x' (0x94) AD0 is always 0 (connected to DGND) I think...
	I2C1->DR = 0x94; // address phase
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}
	//dummy = I2C1->SR1 | I2C1->SR2; // this clears ADDR
	(void)I2C1->SR2;
	I2C1->DR = regaddress;
	//while(!(I2C1->SR1 & I2C_SR1_AF)){}
	while(!(I2C1->SR1 & I2C_SR1_BTF)){}
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

void delay(uint32_t cycles){
	for(uint32_t i = 0; i < cycles; i++){}
}
