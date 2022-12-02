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

void I2C_init(){
	uint32_t OwnAddress = 0x52;
	RCC->APB1ENR1 |= RCC_APB1ENR_I2C3EN;	//enable clock for I2C3
	
	RCC->CCIPR &= ~RCC_CCIPR_I2C3SEL;
	RCC->CCIPR |= RCC_CCIPR_I2C3SEL_0; 	//enable sysclock for I2C3
	RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C3RST;	//1 reset I2C3
	RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C3RST;	//Finish reset
	
	//When the I2C is disabled (PE=0), 
	I2C3->CR1 &= ~NI2C_CR1_PE;	//Disable I2C 	
	I2C3->CR1 &= ~NI2C_CR1_ANFOFF; 	//0: Analog noise filter enabled
	I2C3->CR1 &= ~NI2C_CR1_DNF; 	//0000:Digital Filter Disabled
	I2C3->CR1 |= I2C_CR1_ERRIE; 	//errors interrupt enable
	I2C3->CR1 &= ~NI2C_CR1_SMBUS; 	//SMBus Mode: 0 = I2C mode; 1 = SMBus Mode
	I2C3->CR1 &= ~NI2C_CR1_NOSTRETCH; 	//Enable Clock stretching
	
	//I2C Timing Register
	/*	Need to change these value for our sparkfun tuner*/
	
	I2C3->TIMINGR = 0; 			
	I2C3->TIMINGR &= ~I2C_TIMINGR_PRESC; // Clear the prescaler 
	I2C3->TIMINGR |= 7U << 28; // Set clock prescaler to 7 
	I2C3->TIMINGR |= 49U; // SCLL: SCL Low period (master mode) > 4.7 us 
	I2C3->TIMINGR |= 49U << 8; // SCLH: SCL high period (master mode) > 4.0 us 
	I2C3->TIMINGR |= 14U << 20; // SCLDEL: Data setup time > 1.e us 
	I2C3->TIMINGR |= 15U << 16; // SDADEL: Data hold time > 1.25 us

	//I2C Own Address Register
	I2C3->OAR1 &= ~I2C_OAR1_0A1EN; 
	I2C3->OAR1 = I2C_OAR1_0A1EN | OwnAddr; // 7-bit own address 
	I2C3->OAR1 &= ~I2C_OAR2_0A2EN; // Disable own address 2 
	
	//I2C CR2 configuration
	I2C3->CR2 &= ~I2C_CR2_ADD10;	//0 = 7-bit mode: 1 = 10-bit mode 
	I2C3->CR2 |= I2C_CR2_AUTOEND; 	//Enable the auto end
	I2C3->CR2 |= I2C_CR2_NACK; 		//For slave mode: set NACK
	I2C3->CR1 |= I2C_CR1_PE; 		//Enable I2C
}

void I2C_start(I2C_TypeDef * I2Cx, uint32_t DevAddress, uint8_t Size, uint8_t Direction){
	uint32_t tmpreg = I2Cx->CR2;
	tmpreg &= (uint32_t) ~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP));
	if(direction == READ_FROM_SLAVE){
		tmpreg |= I2C_CR2_RD_WRN;
	} else {
		tmpreg &= ~I2C_CR2_RD_WRN;		
	}
	tmpreg |= (uint32_t)(((uint32_t) DevAddress & I2C_CR2_SADD) | (((uint32_t) Size << 16) & I2C_CR2_NBYTES));
	tmpreg |= I2C_CR2_START;
	I2Cx->CR2 = tmpreg;
}

void I2C_Stop(I2C_TypeDef * I2Cx){ 
	//Master: Generate STOP bit after the current byte has been transferred 
	I2Cx->CR2 |= I2C_CR2_STOP; 
	//Wait until STOPF flag is reset 
	while( (I2Cx->ISR & I2C_ISR_STOPF) == 0 ); 
	I2Cx->ICR |= I2C_ICR_STOPCF; //Write 1 to clear STOPF flag 
}

void I2C_Waitlineidle(I2C_TypeDef * I2Cx){ 
	//Wait until I2C bus is ready 
	while( (I2Cx->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY ); //If busy, wait 
}

int8_t I2C_SendData(I2C_TypeDef *I2Cx, uint8_t SlaveAddress, uint16_t *pData, uint8_t Size) { 
	int i; 
	if (Size <= 0 I I pData == NULL) return -1; 
	// Wait until the Line is idle 
	I2C_WaitLineidle(I2Cx); 
	// The Last argument: e = Sending data to the slave 
	I2C_Start(I2Cx, SlaveAddress, Size, 0); 
	for (i = 0; i < Size; i++) {  
		// TXIS bit is set by hardware when the TXDR register is empty and the 
		// data to be transmitted must be written in the TXDR register. It is 
		// cleared when the next data to be sent is written in the TXDR register. 
		// The TXIS flag is not set when a NACK is received. 
		while( (I2Cx->ISR & I2C_ISR_TXIS) == 0 ); 
		// TXIS is cleared by writing to the TXDR register 
		I2Cx->TXDR = pData[i] & I2C_TXDR_TXDATA;
	} 
	//Wait until TC flag is set 
	while((I2Cx->ISR & I2C_ISR_TC) == 0 && (I2Cx->ISR & I2C_ISR_NACKF) 0); 
	if( (I2Cx->ISR & I2C_ISR_NACKF) != 0 ) 
		return -1; 
	I2C_Stop(I2Cx); 
	return 0; 
}

int8_t I2C_ReceiveData(I2C_TypeDef * I2Cx, uint8_t SlaveAddress, uint16_t *pData, uint8_t Size) { 
	int i; 
	if (Size <= 0 I I pData == NULL) return -1; 
	I2C_WaitLineidle(I2Cx); 
	I2C_Start(I2Cx, SlaveAddress, Size, 1); // 1 Receiving from the slave 
	for (i = 0; i < Size; i++) { 
	//Wait until RXNE flag is set 
	while( (I2Cx->ISR & I2C_ISR_RXNE) == 0 ); 
		pData[i] = I2Cx->RXDR & I2C_RXDR_RXDATA; 
	} 
	while((I2Cx->ISR & I2C_ISR_TC) == 0); // Wait until TCR flag is set 
	I2C_Stop(I2Cx); 
	return 0; 
}

void GPIO_Init() {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;	//enable clock on GPIOB and GPIOC
	GPIOC->MODER &= 0xFFFFFFF0;
	GPIOC->MODER |= 0xA;	//set C0 and C1 to alternate function mode
	GPIOC->OTYPER &= 0xFFFFFFFC;	//clear pins C0 and C1 then set to open-drain
	GPIOC->OTYPER |= 0x3;		//set to open-drain mode
	GPIOC->AFRL &= 0xFFFFFF00;	//clear AFSEL1 and AFSEL0
	GPIOC->AFRL |= 0x00000044;	//select AF4 mode for C0 and C1 for I2C communication
}










