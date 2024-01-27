/* Configured for Sparkfun SerLCD v2.5
 *
 */
#include "stm32f4xx.h"

void Clear_Display(void);
void Write_Character(char);
void Write_String(volatile char*);
void Scroll_Right(void);
void Scroll_Left(void);
void Blink_Cursor(void);
void Change_Baud_Rate(uint8_t);
void Reset_Baud_Rate(void);
void Backlight_OFF(void);
void Backlight_ON(void);
