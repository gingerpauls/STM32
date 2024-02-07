/*
 * lcd.h
 *
 *  Created on: Feb 7, 2024
 *      Author: hutch
 */

#ifndef LCD_H_
#define LCD_H_

void Clear_Display(void);
void Write_Character(char);
void Write_String(char*);
void Scroll_Right(void);
void Scroll_Left(void);
void Blink_Cursor(void);
void Change_Baud_Rate(uint8_t);
void Reset_Baud_Rate(void);
void Backlight_OFF(void);
void Backlight_ON(void);

#endif /* LCD_H_ */
