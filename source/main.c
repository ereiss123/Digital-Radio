#include "stm32l476xx.h"
#include "keypad.h"
#include "lcd.h"
#include "si4703_constants.h"
#include "i2c.h"
#include "interrupts.h"
#include <string.h>
#include <stdio.h>
#define UP 1
#define DOWN 0
/*================================================================================================
*Author: Ashton Carlsen
*Author: Eric Reiss
*Description: Digital FM receiver built using Nucleo L476rg board and  Sparkfun Evaluation Tuner 
*Pins: 
*	KEYPAD:
*		COLS: B8-11
*		ROWS: B4-7
*	LCD:
*		DATA: B0-3
*		EN:	C10
*		RW: C11
*		RS: C12
*	I2C:
*		SDL: C1 Alternate function 4- I2C3
*		SCL: C0 Alternate Function 4- I2C3
*		SEN: C2
*		RST: C3
*		GPIO1: C4		Might not need	Maybe hookup LED's to indicate seek finish or tune finish
*		GPIO2: C5		Might not need
*
===================================================================================================*/

//================================================================================================
//Global Variables
unsigned static char si4703_read_registers[32];
unsigned static char si4703_write_registers[32];
unsigned static int Tdelay;
static char* freqString;
static int muteF = 0;
//=================================================================================================*/

//=================================================================================================
//Main Function Pre-defines
void si4703_init(void);
void GPIO_init(void);
void read_registers(void);
void write_registers(void);
void read_to_write(void);
void delay(unsigned int ms);
void view_arrays(unsigned char* read, unsigned char* write);
void tune(float station);
void seek(int direction);
void mute(void);
void volumeUP(void);
void volumeDOWN(void);
void channelUP(void);
void channelDOWN(void);
void update(void);
void tuneSelect(void);
//=================================================================================================
/*
si4703_write_registers				si4703_read_register
i register addr 					i 	register addr
0 	0x02							0	0X0A
1 	0x03							1	0x0B
2 	0x04							2	0x0C
3 	0x05							3	0x0D
4 	0x06 							4	0x0E
5 	0x07							5	0x0F
6 	0x08							6	0x00
7 	0x09							7	0x01
8 	0x0A							8	0x02
9 	0x0B							9	0x03
10 	0x0C							10	0x04
11 	0x0D							11	0x05
12 	0x0E							12	0x06
13 	0x0F							13	0x07
14 	0x00							14	0x08
15 	0x01							15	0x09
*/

int main(void){
	char data = 'z';
	int i;
	for(i = 0; i<32; i++){
		si4703_read_registers[i] = 0;
	}
	
	RCC->CR |= RCC_CR_HSION;	// turn on HSI
	while((RCC->CR & RCC_CR_HSIRDY) == 0);	//wait till HSI is ready
	SysTick_Init(16000); 	//initialize SysTick for every 1 ms
	GPIO_init();			//initialize GPIO for si4703
	LCD_Init();				//initialize LCD
	delay(5);
	LCD_Clear();			//Clear the LCD
	delay(5);
	keypad_init();			//initialize Keypad pins
	LCD_DisplayString(0, (unsigned char *)"Initializing\0");	//test LCD type display
	delay(20);
	si4703_init();
	delay(510);				//should delay for 500 ms
	LCD_Clear();			//for testing purpose
	while(1){
		read_registers();
		data = keypadPoll();
		switch(data){
			case '1': channelUP(); LCD_Clear(); break;
			case '4': channelDOWN(); LCD_Clear(); break;
			case 'A': tune(103.9f);LCD_Clear(); break;
			case 'B': seek(UP); LCD_Clear();break;
			case 'C': seek(DOWN); LCD_Clear();break;
			case 'D': mute(); break;
			case '*': volumeDOWN(); break;
			case '#': volumeUP(); break;
			default: break;
		}
		update();
	}
}

void SysTick_Init(uint32_t ticks) {
	SysTick->CTRL = 0;
	SysTick->LOAD = ticks -1;
	NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS)-1);
	SysTick->VAL = 0;
	SysTick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk) ;
}

void SysTick_Handler(void) {
	Tdelay--;
}



void view_arrays(unsigned char* read, unsigned char* write){
	volatile unsigned int zero = 0;
	*read |= zero;
	*write |= zero;
}


/*==========================================================================
*
*	Name: GPIO_Init
*
*	Function: initialize pins C0 and C1 for I2C communication
*===========================================================================
*
*BEGIN
*	Enable the clock for GPIOB and GPIOC
*	Set C0 and C1 output type to open-drain 
*	set the C0 and C1 mode to alternate function
*	set the alternate function low register 0 and 1 to AF4
*END
*
*===========================================================================*/

void GPIO_init(void) {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;	//enable clock on GPIOB and GPIOC
	GPIOC->OTYPER &= 0xFFFFFFFC;	//clear pins C0 and C1 then set to open-drain
	GPIOC->OTYPER |= 0x3;		//set to open-drain mode
	
	GPIOC->MODER &= 0xFFFFFF00; //clear C0-C3 mode register
	GPIOC->MODER |= 0x00000055;	//set C0-C3 to output.
	GPIOC->OTYPER &= 0xFFFFFFF3; //set C2 and C3 to push-pull output type
	
}


void si4703_init(void){
	//Select two-wire mode
	GPIOC->ODR &= ~GPIO_ODR_OD3; //Set ~rst low
	GPIOC->ODR |= GPIO_ODR_OD0; //Ensure SCLK is high until first start bit is sent
	GPIOC->ODR &= ~GPIO_ODR_OD1;//ensure SDIO is low
	//GPIOC->ODR |= GPIO_ODR_OD2; //Ensure ~SEN is high
	delay(10);
	GPIOC->ODR |= GPIO_ODR_OD3; //Activate rst
	//delay(110);
	//GPIOC->ODR &= ~GPIO_ODR_OD3; //set ~rst high
	delay(110);
	I2C_init();
	read_registers();
	//Turn on crystal oscillator
	si4703_write_registers[10] |= 1<<7;
	write_registers();
	delay(510);
	//Disable mute and enable chip
	si4703_write_registers[0] |= (1<<6);
	si4703_write_registers[1] |= 1;
	si4703_write_registers[1] &= ~(1<<6);
	write_registers();
	read_registers();
	si4703_write_registers[5] |= 0xC;
	write_registers();
	read_registers();
//	view_arrays(si4703_read_registers,si4703_write_registers);
	//Set the band and volume
	si4703_write_registers[6] = 1U; //Seek threshold of 28
	si4703_write_registers[7] &= 0x0000; //Band 00 is USA
	si4703_write_registers[7] |= 0xA;
	write_registers();
	read_registers();
	//Enable RDS
	si4703_write_registers[4] |= 1<<4;
	write_registers();
	read_registers();
	//Set seek to wrap around
	si4703_write_registers[0] &= ~(1<<2);
	write_registers();
	read_registers();
}

void read_registers(void){
	I2C_ReceiveData(I2C3,SI4703,si4703_read_registers,32);
	read_to_write();
}
void write_registers(void){
	I2C_SendData(I2C3,SI4703,si4703_write_registers,32);
	delay(10);
}

void read_to_write(void){
	int j;
	for(int i = 0; i<32; i++){
		if(i < 16)
			j = i+16;
		else
			j = i-16;
		si4703_write_registers[i] = si4703_read_registers[j];
	}
	
}

//write tuning to the si4703
void tune(float station){
	unsigned int channel = 0;
	channel = (unsigned int)(5*(station - 87.5f));	//conversion found in si4703
	channel &= 0x000003FF;					//10bit wide bit mask
	char channel_high = (channel & 0x300)>>8;	//isoate upper 2 bits
	char channel_low = channel & 0x0FF;		//isolate lower 8 bits
	read_registers();
	si4703_write_registers[2] |= (1<<7 | channel_high); //set tune bit and upper channel bits
	si4703_write_registers[3] = channel_low;		//set lower channel bits
	write_registers();
	read_registers();
	while(((si4703_write_registers[16] & 0x40)>>6) == 0){
			read_registers();
		}
	si4703_write_registers[2] &= ~(1<<7); //turn off tune bit
	write_registers();
}

void seek(int direction){
	if(direction == UP){
		si4703_write_registers[0] |= (1<<1); //Set seek up bit
	}else{
		si4703_write_registers[0] &= ~(1<<1); //Set seek down bit
	}
		si4703_write_registers[0] |= 1U; //Set seek bit
		write_registers();
		read_registers();
		while(((si4703_write_registers[16] & 0x40)>>6) == 0){
			read_registers();
			update();
		}
		si4703_write_registers[0] &= ~1U;	//clear seek bit
		write_registers();
}

void mute(void){
	read_registers();
	
	if(muteF){
		si4703_write_registers[0] |= 1<<6;
	}else{
		si4703_write_registers[0] &= ~(1<<6);
	}
	muteF = ~muteF;
	write_registers();
}

void volumeUP(void){
		unsigned char volume;
		read_registers();
		volume = si4703_write_registers[7] & 0x0F;
		if(volume < 15){
				volume = volume+1;
		}
		si4703_write_registers[7] &= 0xF0;
		si4703_write_registers[7] |= volume;
		write_registers();
}
	
void volumeDOWN(void){
		unsigned char volume;
		read_registers();
		volume = si4703_write_registers[7] & 0x0F;
		if(volume > 0){
				volume = volume-1;
		}
		si4703_write_registers[7] &= 0xF0;
		si4703_write_registers[7] |= volume;
		write_registers();
}

void tuneSelect(void){

}

void channelUP(void) {
	unsigned char channel_high = 0;
	unsigned char channel_low = 0;
	unsigned int channel = 0;
	read_registers();		//read registers from chip
	channel_high = si4703_write_registers[18];	//get upper 2 bits from the high register
	channel_low = si4703_write_registers[19];	//get lower byte from the low register
	channel = channel_high | channel_low;			// isolate the 10 bits of the channel [9:0]
	channel = channel & 0x3FF;
	if(channel < 102) {
		channel = channel + 1;
	} else {
		channel = 0;
	}
	channel_high =  (channel & 0x300)>>8;
	channel_low = (channel & 0xFF);
	si4703_write_registers[2] &= 0xC;	//clear lower 2 bits of 3h register
	si4703_write_registers[2] |= (1<<7) |  channel_high;	//set tune bit and upper channel bit
	si4703_write_registers[3] &= 0;		//clear byte in 3L register
	si4703_write_registers[3] |= channel_low;	//set lower channel bits
	write_registers();
	read_registers();
	while(((si4703_write_registers[16] & 0x40)>>6) == 0){
			read_registers();
		}
	si4703_write_registers[2] &= ~(1<<7); //turn off tune bit
	write_registers();
}

void channelDOWN(void) {
	unsigned char channel_high = 0;
	unsigned char channel_low = 0;
	unsigned int channel = 0;
	read_registers();		//read registers from chip
	channel_high = si4703_write_registers[18];	//get upper 2 bits from the high register
	channel_low = si4703_write_registers[19];	//get lower byte from the low register
	channel = channel_high | channel_low;			// isolate the 10 bits of the channel [9:0]
	channel = channel & 0x3FF;
	if(channel > 0) {
		channel = channel - 1;
	} else {
		channel = 102;
	}
	channel_high =  (channel & 0x300)>>8;
	channel_low = (channel & 0xFF);
	si4703_write_registers[2] &= 0xC;	//clear lower 2 bits of 3h register
	si4703_write_registers[2] |= (1<<7) |channel_high;	//set tune bit and upper channel bit
	si4703_write_registers[3] &= 0;		//clear byte in 3L register
	si4703_write_registers[3] |= channel_low;	//set lower channel bits
	write_registers();
	read_registers();
	while(((si4703_write_registers[16] & 0x40)>>6) == 0){
			read_registers();
		}
	si4703_write_registers[2] &= ~(1<<7); //turn off tune bit
	write_registers();
}


void update(void){
	char buffer[10];
	float frequency = 0;
	unsigned char channel_high = 0;
	unsigned char channel_low = 0;
	unsigned int channel = 0;
	read_registers();		//read registers from chip
	channel_high = si4703_write_registers[18];	//get upper 2 bits from the high register
	channel_low = si4703_write_registers[19];	//get lower byte from the low register
	channel = channel_high | channel_low;			// isolate the 10 bits of the channel [9:0]
	channel = channel & 0x3FF;
	frequency = (((float)channel)/5)+87.5f;
	sprintf(buffer, "%.1f", frequency);
	LCD_DisplayString(0, (unsigned char*)buffer);
}

//delay function using systick
void delay(unsigned int ms){
	volatile int count = 0;
	Tdelay = ms;
	while(Tdelay > 0){
		count++;
	}
}




