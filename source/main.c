#include "stm32l476xx.h"
#include "keypad.h"
#include "lcd.h"
#include "si4703_constants.h"
#include "i2c.h"
#include "interrupts.h"
/*================================================================================================
*Author: Ashton Carlsen
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
*		SEN: C2
*		RST: C3
*		GPIO1: C4		Might not need	Maybe hookup LED's to indicate seek finish or tune finish
*		GPIO2: C5		Might not need
*
===================================================================================================*/

//================================================================================================
//Global Variables
unsigned static short int si4703_read_registers[16];
unsigned static short int si4703_write_registers[16];
unsigned static int Tdelay;

//=================================================================================================*/

//=================================================================================================
//Main Function Pre-defines
void si4703_init(void);
void GPIO_init(void);
void read_registers(void);
void write_registers(void);
void read_to_write(void);
void delay(unsigned int ms);

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
	RCC->CR |= RCC_CR_HSION;	// turn on HSI
	while((RCC->CR & RCC_CR_HSIRDY) == 0);	//wait till HSI is ready
	SysTick_Init(16000); 	//initialize SysTick for every 1 ms
	GPIO_init();			//initialize GPIO for si4703
	LCD_Init();				//initialize LCD
	LCD_Clear();			//Clear the LCD
	keypad_init();			//initialize Keypad pins
	LCD_DisplayString(0, (unsigned char *)"Initializing Radio");	//test LCD type display
	si4703_init();
	delay(510);				//should delay for 500 ms
	LCD_Clear();			//for testing purpose
	while(1){
		read_registers();
		data = keypadPoll();
		switch(data){
			case 'A': /*tuning function*/ break;
			case 'B': /*seek up function*/ break;
			case 'C': /*seek down function*/ break;
			case 'D': /*Mute function*/ break;
			default: break;
		}
	}
	return 0;
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
	GPIOC->MODER &= 0xFFFFFFF0;
	GPIOC->MODER |= 0xA;	//set C0 and C1 to alternate function mode
	GPIOC->AFR[0] &= 0xFFFFFF00;	//clear AFSEL1 and AFSEL0
	GPIOC->AFR[0] |= 0x00000044;	//select AF4 mode for C0 and C1 for I2C communication
	GPIOC->MODER &= 0xFFFFFF0F; //clear C2 and C3 mode register
	GPIOC->MODER |= 0x00000050;	//set C2 and C3 to output.
	GPIOC->OTYPER &= 0xFFFFFFF3; //set C2 and C3 to push-pull output type
	//Need to set C4 C5 for the rest of the chip pins
}


void si4703_init(void){
	//Select two-wire mode
	GPIOC->ODR |= GPIO_ODR_OD3; //Set ~rst low
	GPIOC->ODR |= GPIO_ODR_OD0; //Ensure SCLK is high until first start bit is sent
	GPIOC->ODR &= ~GPIO_ODR_OD1;//ensure SDIO is low
	GPIOC->ODR &= ~GPIO_ODR_OD2; //Ensure ~SEN is high
	GPIOC->ODR &= ~GPIO_ODR_OD3; //Activate rst
	//Need 110 ms delay
	GPIOC->ODR |= GPIO_ODR_OD3; //set ~rst high
	//Delay 110ms
	I2C_init();
	read_registers();
	si4703_write_registers[5] = 0x8100; //Enable crystal oscillator in TEST register
	write_registers();
	read_registers();
	si4703_write_registers[0] = 0x4001; //Set enable bit and DMUTE bit in POWERCFG
	si4703_write_registers[2] = 1<<RDS; //Enable RDS data
	write_registers();
	
	//Set band data
	read_registers();
	si4703_write_registers[3] = 0x1C07; //Set Seek threshold to 0x1C, Set band select and channel spacing to 0x0 (USA), Set volume to 0x7
	write_registers();
}

void read_registers(void){
	I2C_ReceiveData(I2C3,SI4703,si4703_read_registers,16);
	read_to_write();
}
void write_registers(void){
	I2C_SendData(I2C3,SI4703,si4703_write_registers,16);
	delay(510);
}

void read_to_write(void){
	int j;
	for(int i = 0; i<16; i++){
		if(i < 8)
			j = i+8;
		else
			j = i-8;
		si4703_write_registers[i] = si4703_read_registers[j];
	}
	
}
/*====================================================================================================================
								ARDUINO POWERUP CODE


//To get the Si4703 inito 2-wire mode, SEN needs to be high and SDIO needs to be low after a reset
//The breakout board has SEN pulled high, but also has SDIO pulled high. Therefore, after a normal power up
//The Si4703 will be in an unknown state. RST must be controlled
void si4703_init(void) {
  Serial.println("Initializing I2C and Si4703");
  
  pinMode(resetPin, OUTPUT);
  pinMode(SDIO, OUTPUT); //SDIO is connected to A4 for I2C
  digitalWrite(SDIO, LOW); //A low SDIO indicates a 2-wire interface
  digitalWrite(resetPin, LOW); //Put Si4703 into reset
  delay(1); //Some delays while we allow pins to settle
  digitalWrite(resetPin, HIGH); //Bring Si4703 out of reset with SDIO set to low and SEN pulled high with on-board resistor
  delay(1); //Allow Si4703 to come out of reset

  Wire.begin(); //Now that the unit is reset and I2C inteface mode, we need to begin I2C

  si4703_readRegisters(); //Read the current register set
  //si4703_registers[0x07] = 0xBC04; //Enable the oscillator, from AN230 page 9, rev 0.5 (DOES NOT WORK, wtf Silicon Labs datasheet?)
  si4703_registers[0x07] = 0x8100; //Enable the oscillator, from AN230 page 9, rev 0.61 (works)
  si4703_updateRegisters(); //Update

  delay(500); //Wait for clock to settle - from AN230 page 9

  si4703_readRegisters(); //Read the current register set
  si4703_registers[POWERCFG] = 0x4001; //Enable the IC
  //  si4703_registers[POWERCFG] |= (1<<SMUTE) | (1<<DMUTE); //Disable Mute, disable softmute
  si4703_registers[SYSCONFIG1] |= (1<<RDS); //Enable RDS
*/

void delay(unsigned int ms){
	volatile int count = 0;
	Tdelay = ms;
	while(Tdelay > 0){
		count++;
	}
}




