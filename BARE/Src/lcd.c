/*
 * lcd.c
 *
 *  Created on: Feb 7, 2024
 *      Author: hutch
 */

#include "main.h"

void Clear_Display(void) {
	USART2->DR |= 0xFE; // 0xFE and 0x7C are special commands
	while (!(USART2->SR & USART_SR_TXE));
	USART2->DR |= 0x01; // Clear display
	while (!(USART2->SR & USART_SR_TXE));
}
void Write_Character(char letter) {
	USART2->DR |= letter;
	while (!(USART2->SR & USART_SR_TXE));

}
void Write_String(char *word) {
//	for(int i = 0; i < strlen(*(word); i++){
//		USART2->DR |= *word[i];
//		while(!(USART2->SR & USART_SR_TXE));
//	}
	while (*word != '\0') {
		USART2->DR |= *word;
		while (!(USART2->SR & USART_SR_TXE));
		word++;
	}
	while (!(USART2->SR & USART_SR_TC));
}
void Scroll_Right(void) {
	USART2->DR |= 0xFE;
	while (!(USART2->SR & USART_SR_TXE));
	USART2->DR |= 0x1C;
	while (!(USART2->SR & USART_SR_TXE));
	while (!(USART2->SR & USART_SR_TC));
}
void Scroll_Left(void) {
	USART2->DR |= 0xFE;
	while (!(USART2->SR & USART_SR_TXE));
	USART2->DR |= 0x18;
	while (!(USART2->SR & USART_SR_TXE));
	while (!(USART2->SR & USART_SR_TC));
}
void Blink_Cursor(void) {
	USART2->DR |= 0xFE;
	while (!(USART2->SR & USART_SR_TXE));
	USART2->DR |= 0x0D;
	while (!(USART2->SR & USART_SR_TXE));
	while (!(USART2->SR & USART_SR_TC));
}
void Change_Baud_Rate(uint32_t lcdbaudrate) {
	if (lcdbaudrate == 2400){
		lcdbaudrate = (uint8_t)0x0B;
	}
	else if (lcdbaudrate == 4800){
		lcdbaudrate = (uint8_t)0x0C;
	}
	else if (lcdbaudrate == 9600){
		lcdbaudrate = (uint8_t)0x0D;
	}
	else if (lcdbaudrate == 14400){
		lcdbaudrate = (uint8_t)0x0E;
	}
	else if (lcdbaudrate == 19200){
		lcdbaudrate = (uint8_t)0x0F;
	}
	else if (lcdbaudrate == 38400){
		lcdbaudrate = (uint8_t)0x10;
	}
	USART2->DR |= 0x7C;
	while (!(USART2->SR & USART_SR_TXE));
	USART2->DR |= lcdbaudrate;
	while (!(USART2->SR & USART_SR_TXE));
	while (!(USART2->SR & USART_SR_TC));
}
void Reset_Baud_Rate(void) {
	USART2->DR |= 0x12;
	while (!(USART2->SR & USART_SR_TXE));
	while (!(USART2->SR & USART_SR_TC));
}
void Backlight_OFF(void) {
	USART2->DR |= 0x7C;
	while (!(USART2->SR & USART_SR_TXE));
	USART2->DR |= 0x80;
	while (!(USART2->SR & USART_SR_TXE));
	while (!(USART2->SR & USART_SR_TC));
}
void Backlight_ON(void) {
	USART2->DR |= 0x7C;
	while (!(USART2->SR & USART_SR_TXE));
	USART2->DR |= 0x9D;
	while (!(USART2->SR & USART_SR_TXE));
	while (!(USART2->SR & USART_SR_TC));
}

void SetCursor(uint8_t position){
	USART2->DR |= 0XFE;
	while (!(USART2->SR & USART_SR_TXE));
	USART2->DR |= 0x80 + position;
	while (!(USART2->SR & USART_SR_TXE));
	while (!(USART2->SR & USART_SR_TC));

}
