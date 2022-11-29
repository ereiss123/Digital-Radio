#include "keypad.h"
#include "lcd.h"
#include "stm32l476xx.h"


/*================================================================================================
*Author: Ash Carlsen
*Author: Eric Reiss
*Description: Digital FM receiver built using Nucleo L476rg board and  Sparkfun Evaluation Tuner 
*Pins: 
*	KEYPAD:
*		COLS: B8-11
*		ROWS: B4-7
*	LCD:
*		DATA: B0-4
*		EN:	C10
*		RW: C11
*		RS: C12
*	I2C:
*		SDL: C1 Alternate function 4- I2C3
*		SCL: C0 Alternate Function 4- I2C3
*	
*
===================================================================================================*/