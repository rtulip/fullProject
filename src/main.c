/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via DEBUG).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int CF;
int FIRST_RISING_EDGE = 0;
int GLOBAL_FRQ = 0;
double GLOBAL_RES = 0;

#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)
#define myTIM3_PERIOD ((uint32_t)0x3840)

#define LCD_RS_0 0x0
#define LCD_RS_1 0x40

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myGPIOC_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myEXTI_Init(void);
void myADC_init();
void myDAC_init();
void mySPI_Init();
void myLCD_Init();
void write_LCD(uint8_t rs,uint8_t data);
void write_LCD_HZ(int value);
void write_LCD_OH(int value);
void clear_LCD();
void update_LCD();
void set_LCD_ADDR(uint8_t addr);
int SPI_BSY(SPI_TypeDef* SPIx);
int SPI_TXE(SPI_TypeDef* SPIx);

int main(int argc, char* argv[]){
	// At this stage the system clock should have already been configured
	// at high speed.
	myGPIOA_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */
	myGPIOC_Init();
	myADC_init();
	myDAC_init();
	myLCD_Init();

	// Infinite loop
	ADC1->CR |= ADC_CR_ADSTART;

	while (1){

		while((ADC1->ISR & ADC_ISR_EOSEQ) == 0);			// Wait for end of sequence
		ADC1->ISR &= ~ADC_ISR_EOC;							// Reset end of conversation flag
		int voltage = (ADC1->DR & 0x00FF);					// Read ADC data
		double res = 5000 + ((double) voltage/255.0)*5000;	// Calculate resistance
		GLOBAL_RES = (int) res;
		DAC->DHR8R1 = test;									// Write ADC value to DAC
		
		update_LCD();

	}

}

void myGPIOB_Init(){										// GPIOB Used for SPI communication

	/* Enable clock for GPIOC peripheral */
		// Relevant register: RCC->AHBENR
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

		GPIOB->AFR[0] &= 0;									// RESET AFR
		GPIOB->AFR[1] &= 0;									// RESET AFR

		// Relevant register: GPIOC->MODER
		GPIOB->MODER |= GPIO_MODER_MODER3_1;				// SCK  -> Alternate Function
		GPIOB->MODER |= GPIO_MODER_MODER4_0;				// LCK  -> Output
		GPIOB->MODER |= GPIO_MODER_MODER5_1;				// MOSI -> Alternate Function

		/* Ensure no pull-up/pull-down for PB3,4,5 */
		// Relevant register: GPIOC->PUPDR
		GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
		GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
		GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);

}

void myLCD_Init(){											// Setup LCD

	myGPIOB_Init();											// Init GPIOB
	mySPI_Init();											// Init SPI
	myTIM3_Init();											// Init TIM3	

	write_LCD(LCD_RS_0, 0x02);								// Set to 4-bit interface
	write_LCD(LCD_RS_0, 0x28);								// DL = 0, N = 1, F = 0
	write_LCD(LCD_RS_0, 0x0C);								// D = 1, C = 0, B = 0
	write_LCD(LCD_RS_0, 0x06);								// I/D = 1, S = 0
	clear_LCD();											// Clear Display

	
}

void mySPI_Init(){

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;						// Enable SPI1 clock

	SPI_InitTypeDef SPI_InitStructInfo;					
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;	// Create struct to initiate SPI

	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;

	SPI_Init(SPI1,SPI_InitStruct);							// Init SPI with SPI_InitStruct
	SPI_Cmd(SPI1,ENABLE);									// Enable SPI1

}

void write_LCD(uint8_t RS, uint8_t data){					// Function to write to LCD.

	uint8_t splits[6];										// Splits data into 6 different messages

	uint8_t H = ((data & 0xF0) >> 4);						// High end of data
	uint8_t L = (data & 0x0F);								// Low end of data

	uint8_t ENABLE = 0x80;									// Sequence Enable bit
	uint8_t DISABLE = 0x00;									// Sequence Disable bit

	splits[0] = DISABLE | RS | H;							// High Disable	
	splits[1] = ENABLE  | RS | H;							// High Enable
	splits[2] = DISABLE | RS | H;							// High Disable

	splits[3] = DISABLE | RS | L;							// High Disable	
	splits[4] = ENABLE  | RS | L;							// High Enable
	splits[5] = DISABLE | RS | L;							// High Disable

	for (int i = 0; i < 6; i++){
		GPIOB->BRR = 0x10;									// Force LCK to 0;
		while( SPI_BSY(SPI1) || !SPI_TXE(SPI1) );			// Wait while SPI is busy or no room on TXE
		SPI_SendData8(SPI1,splits[i]);						// Send split[i] over SPI1
		while( SPI_BSY(SPI1) );								// Wait while SPI is busy
		GPIOB->BSRR = 0x10;									// Force LCK to 1;
	}

}

void set_LCD_ADDR(uint8_t addr){							// Function to set the address of LCD

	// Should check for address in bounds

	write_LCD(LCD_RS_0, addr | 0x80);						// Sends addr | 0x80

}

void write_LCD_HZ(int value){								// Function to set Hz value
	// Should check in bounds 

	uint8_t addr = 0x00;									// Addresss of start of 1st row

	int thou = value / 1000;								// Calculates thousands
	int hund = (value / 100 ) % 10;							// Calculates hundreds
	int tens = (value / 10) % 10;							// Calculates tens
	int ones = (value % 10);								// Calculates ones

	int ascii_offset = 0x30;								// ASCII digit offset

	set_LCD_ADDR(addr);										// Set address
	write_LCD(LCD_RS_1, (uint8_t) 'F');						// Write letter
	addr++;													// Increment address
	set_LCD_ADDR(addr);										// Repeat ...
	write_LCD(LCD_RS_1, (uint8_t) ':');	
	addr++;
	set_LCD_ADDR(addr);
	write_LCD(LCD_RS_1, (uint8_t) (thou + ascii_offset));
	addr++;
	set_LCD_ADDR(addr);
	write_LCD(LCD_RS_1, (uint8_t) (hund+ascii_offset));
	addr++;
	set_LCD_ADDR(addr);
	write_LCD(LCD_RS_1, (uint8_t) (tens+ascii_offset));
	addr++;
	set_LCD_ADDR(addr);
	write_LCD(LCD_RS_1, (uint8_t) (ones+ascii_offset));
	addr++;
	set_LCD_ADDR(addr);
	write_LCD(LCD_RS_1, (uint8_t) 'H');
	addr++;
	set_LCD_ADDR(addr);
	write_LCD(LCD_RS_1, (uint8_t) 'z');
	addr++;

}

void write_LCD_OH(int value){								// Function to set the Oh value

	// Should check to see if in bounds

	uint8_t addr = 0x40;									// Address of start of 2nd row

	int thou = value / 1000;								// Same as in write_LCD_HZ
	int hund = (value / 100 ) % 10;
	int tens = (value / 10) % 10;
	int ones = (value % 10);

	int ascii_offset = 0x30;								// ASCII digit offset 

	set_LCD_ADDR(addr);										// Set address
	write_LCD(LCD_RS_1, (uint8_t) 'R');						// Write to address
	addr++;													// Increment address
	set_LCD_ADDR(addr);										// Repeat ...
	write_LCD(LCD_RS_1, (uint8_t) ':');
	addr++;
	set_LCD_ADDR(addr);
	write_LCD(LCD_RS_1, (uint8_t) (thou + ascii_offset));
	addr++;
	set_LCD_ADDR(addr);
	write_LCD(LCD_RS_1, (uint8_t) (hund+ascii_offset));
	addr++;
	set_LCD_ADDR(addr);
	write_LCD(LCD_RS_1, (uint8_t) (tens+ascii_offset));
	addr++;
	set_LCD_ADDR(addr);
	write_LCD(LCD_RS_1, (uint8_t) (ones+ascii_offset));
	addr++;
	set_LCD_ADDR(addr);
	write_LCD(LCD_RS_1, (uint8_t) 'O');
	addr++;
	set_LCD_ADDR(addr);
	write_LCD(LCD_RS_1, (uint8_t) 'h');
	addr++;

}

void clear_LCD(){											// Function to clear display

	write_LCD(LCD_RS_0, 0x01);								// Writes 0x01 to LCD with RS = 0
	trace_printf("Clearing... \n");							// Print statement for delay
	trace_printf("Clearing... \n");							// Print statement for delay

}

void update_LCD(){											// Function to update the LCD 
	
	write_LCD_HZ(GLOBAL_FRQ);								// Write global frequency
	write_LCD_OH(GLOBAL_RES);								// Write global resistance

}

int SPI_BSY(SPI_TypeDef* SPIx){								// Function to check if SPI_BSY flag set 

	return (SPI_I2S_GetFlagStatus(SPIx,SPI_SR_BSY) == SET);

}

int SPI_TXE(SPI_TypeDef* SPIx){								// Function to check if SPI_TXE flag set 

	return (SPI_I2S_GetFlagStatus(SPIx, SPI_SR_TXE) == SET);

}

void myADC_init(){											

	// USE PC0 (ADC_IN10)

	ADC1->ISR = 0x0000;										// Ensure ADC_ISR is reset
	ADC1->IER = 0x0000;										// Disable all interrupts
	ADC1->CFGR1 = 0x0000;									// Ensure configuration is reset
	ADC1->CFGR2 = 0x0000;									// Ensure configuration is reset

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;						// Activate ADC clock1
	ADC1->CFGR2 |= ADC_CFGR2_CKMODE_1;						// Set clock mode

	ADC1->CFGR1 |= ADC_CFGR1_CONT;							// Set to continuous mode
	ADC1->CFGR1 |= ADC_CFGR1_RES_1;							// Set resolution of conversation to 8bit

	ADC1->SMPR |= ADC_SMPR1_SMPR;							// Set lowest sampling rate

	ADC1->CHSELR = 0x0400;									// Select Channel 10
	ADC1->CR = 0x0000;										// Ensure ADC_CR_ADEN == 0
	ADC1->CR |= ADC_CR_ADCAL;								// Set ADC_CR_ADCAL to 1

	while(((ADC1->CR & ADC_CR_ADCAL) << 1) == 1);			// Wait for ADCAL to == 0

	CF = (ADC1->DR & 0x007F);								// Read calibration factor
	trace_printf("Calibrated: %d\n",CF);
															// ENABLE ADC
	ADC1->CR |= ADC_CR_ADEN;								// Raise ADC_CR_ADEN to 1
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);				// Wait for ADRDY to == 1
	trace_printf("ADC Ready\n");

}

void myDAC_init(){
	// PA4
	GPIOA->MODER &= ~(GPIO_MODER_MODER4);					// Configure PA4 as Analog

	RCC->APB1ENR |= RCC_APB1ENR_DACEN;						// Activate DAC clock

	DAC->CR |= DAC_CR_EN1;									// Enable DAC

}


void myGPIOA_Init()
{
	//Used to detect signal from generator

	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;


	/* Configure PA0 as output */
	// Relevant register: GPIOA->MODER

	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	/* Ensure no pull-up/pull-down for PA0 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

}

void myGPIOC_Init()
{

	/* Enable clock for GPIOC peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	/* Configure PC0 as analog mode */
	// Relevant register: GPIOC->MODER
	GPIOC->MODER |= GPIO_MODER_MODER0;

	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOC->PUPDR
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

}

myTIM2_Init(){
	//Timer setup.

	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = ((uint16_t)0x0000);

	// Is setting up NVIC needed?

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority

	NVIC_SetPriority(TIM2_IRQn,0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ

	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= 0x1;

}

void myTIM3_Init()
{
	//Timer setup.

	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM3->CR1 = ((uint16_t)0x009C);

	/* Set clock prescaler value */
	TIM3->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM3->ARR = myTIM3_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM3->EGR = ((uint16_t)0x0000);

	// Is setting up NVIC needed?

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority

	NVIC_SetPriority(TIM3_IRQn,0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ

	NVIC_EnableIRQ(TIM3_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM3->DIER |= 0x1;

	//TIM3->CR1 |= 0x1;

}

void myEXTI_Init()
{
	//Interrupt setup

	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] = ((uint16_t)0x80);

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR = 0xFFFFFFFF;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= 0x2;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn,0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void TIM2_IRQHandler()
{
	// Used to track time

	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= 0xFFBE; 			//set bit 0 & 6 to 0 and keep everything else the same
		TIM2->CNT = 0x0;
		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
	}
}

void TIM3_IRQHandler()
{
	// Used to update display

	/* Check if update interrupt flag is indeed set */
	if ((TIM3->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR

		//write_LCD_HZ(GLOBAL_FRQ);

		TIM3->SR &= 0xFFBE; 			//set bit 0 & 6 to 0 and keep everything else the same
		TIM3->CNT = 0x0;
		TIM3->CR1 |= 0x1;
		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
	}
}


void EXTI0_1_IRQHandler()
{
	// Interrupt Handler

	// Maximum time is when TIM2 overflows:
		// 2^32/48MHz ~= 90s
		// 48MHz / 2^32 ~= 0.01118 Hz = F min
		// In practice: Overflow at 0.01 Hz

	// Minimum Time is when the period is smaller than the time it takes to turn off TIM2. May vary slightly.
		// In practice: 700 kHz

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		if (FIRST_RISING_EDGE){
			TIM2->CR1 &= 0;							//	- Stop timer (TIM2->CR1).
			unsigned int time = TIM2->CNT;			//	- Read out count register (TIM2->CNT).
			//	- Calculate signal period and frequency.
			//	- Print calculated values to the console.
			//	  NOTE: Function trace_printf does not work
			//	  with floating-point numbers: you must use
			//	  "unsigned int" type to print your signal
			//	  period and frequency.
			//
			double period = (double) time / 48000000;
			double frq = 1 / period;

			GLOBAL_FRQ = (int) frq;

			//trace_printf("Period: %lf s Frq: %lf Hz\n",period,frq);

			// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
			//

			EXTI->PR |= EXTI_PR_PR1;				
			FIRST_RISING_EDGE = 0;


		} else {


			//
			// 1. If this is the first edge:
			TIM2->CNT = 0x0;						//	- Clear count register (TIM2->CNT).
			TIM2->CR1 |= 0x1;						//	- Start timer (TIM2->CR1).

			// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
			//
			EXTI->PR |= EXTI_PR_PR1;				// Do we need to do this?
			FIRST_RISING_EDGE = 1;

		}





	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
