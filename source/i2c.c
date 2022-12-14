#include "i2c.h"
#include "si4703_constants.h"
#define NULL (void*)0

void I2C_init(){
	GPIOC->MODER &= 0xFFFFFFF0;
	GPIOC->MODER |= 0xA;	//set C0 and C1 to alternate function mode
	GPIOC->AFR[0] &= 0xFFFFFF00;	//clear AFSEL1 and AFSEL0
	GPIOC->AFR[0] |= 0x00000044;	//select AF4 mode for C0 and C1 for I2C communication
	
	uint32_t OwnAddress;
	OwnAddress = 0x52;
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C3EN;	//enable clock for I2C3
	
	RCC->CCIPR &= ~RCC_CCIPR_I2C3SEL;
	RCC->CCIPR |= RCC_CCIPR_I2C3SEL_0; 	//enable sysclock for I2C3
	RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C3RST;	//1 reset I2C3
	RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C3RST;	//Finish reset
	
	//When the I2C is disabled (PE=0), 
	I2C3->CR1 &= ~I2C_CR1_PE;	//Disable I2C 	
	I2C3->CR1 &= ~I2C_CR1_ANFOFF; 	//0: Analog noise filter enabled
	I2C3->CR1 &= ~I2C_CR1_DNF; 	//0000:Digital Filter Disabled
	I2C3->CR1 |= I2C_CR1_ERRIE; 	//errors interrupt enable
	//I2C3->CR1 &= ~I2C_CR1_SMBHEN; 	//<------UNSURE ABOUT THIS; WAS I2C_CR1_SMBUS SMBus Mode: 0 = I2C mode; 1 = SMBus Mode
	I2C3->CR1 &= ~I2C_CR1_NOSTRETCH; 	//Enable Clock stretching
	
	//I2C Timing Register
	/*	Need to change these value for our sparkfun tuner*/
	
	//16MHz clock. Timing equations found in 22.2.5.5
	I2C3->TIMINGR = 0; 			
	I2C3->TIMINGR &= ~I2C_TIMINGR_PRESC; // Clear the prescaler 
	I2C3->TIMINGR |= 7U << 28; // Set clock prescaler to 7 
	I2C3->TIMINGR |= 49U; // SCLL: SCL Low period (master mode) > 4.7 us 
	I2C3->TIMINGR |= 49U << 8; // SCLH: SCL high period (master mode) > 4.0 us 
	I2C3->TIMINGR |= 14U << 20; // SCLDEL: Data setup time > 0.1 us 
	I2C3->TIMINGR |= 15U << 16; // SDADEL: Data hold time > 0.6 us

	//I2C Own Address Register
	I2C3->OAR1 &= ~I2C_OAR1_OA1EN; 
	I2C3->OAR1 = I2C_OAR1_OA1EN | OwnAddress; // 7-bit own address 
	I2C3->OAR1 &= ~I2C_OAR2_OA2EN; // Disable own address 2 
	
	//I2C CR2 configuration
	I2C3->CR2 &= ~I2C_CR2_ADD10;	//0 = 7-bit mode: 1 = 10-bit mode 
	I2C3->CR2 |= I2C_CR2_AUTOEND; 	//Enable the auto end
	I2C3->CR2 |= I2C_CR2_NACK; 		//For slave mode: set NACK
	I2C3->CR1 |= I2C_CR1_PE; 		//Enable I2C
}

void I2C_Start(I2C_TypeDef * I2Cx, uint32_t DevAddress, uint8_t Size, uint8_t direction){
	uint32_t tmpreg = I2Cx->CR2;
	tmpreg &= (uint32_t) ~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP));
	if(direction == 1){					//MIGHT BE A 0 
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

void I2C_WaitLineIdle(I2C_TypeDef * I2Cx){ 
	//Wait until I2C bus is ready 
	while( (I2Cx->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY ); //If busy, wait 
}

/*=========================================================================
*
*	Name: I2C_SendData
*
*	Function: send an array of bytes to a slave using I2C communication
*
*==========================================================================
*BEGIN
*	check that we are able to send data
*	wait until the lines are not in use
*	send a start bit
*	for each bite in the array
*		wait until the the ACK bit is recieved
*		write the byte to teh transmit register
*	wait until the ACK flag and sending is done
*	if not acknowledged
*		return with error
*	send a stop bit
*END
*
*=========================================================================*/

int8_t I2C_SendData(I2C_TypeDef *I2Cx, uint8_t SlaveAddress, uint8_t *pData, uint8_t Size) { 
	int i; 
	if (Size <= 0 || pData == NULL) return -1; 
	// Wait until the Line is idle 
	I2C_WaitLineIdle(I2Cx); 
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
	while((I2Cx->ISR & I2C_ISR_TC) == 0 && (I2Cx->ISR & I2C_ISR_NACKF) == 0); 
	if( (I2Cx->ISR & I2C_ISR_NACKF) != 0 ) 
		return -1; 
	I2C_Stop(I2Cx); 
	return 0; 
}

/*=======================================================================
*
*	Name: I2C_RecieveData
*
*	Function: recieve data from a slave through I2C communication
*========================================================================
*
*BEGIN
*	check that we are able to recieve data
*	wait until the line is idle
*	send a start bit
*	for each byte of data
*		wait until the recieved flag is found
*		set first index of recieved data to the data over the line
*	wait until all communication is done
*	send stop bit
*END 
*
*=======================================================================*/

int8_t I2C_ReceiveData(I2C_TypeDef * I2Cx, uint8_t SlaveAddress, uint8_t *pData, uint8_t Size) { 
	int i; 
	if (Size <= 0 || pData == NULL) return -1; 
	I2C_WaitLineIdle(I2Cx); 
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


