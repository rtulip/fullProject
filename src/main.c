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
void myADC_init();
void myDAC_init();
void myGPIOC_init();
void myGPIOB_Init();
void mySPI_Init();
void write_SPI(uint8_t data);

int main(int argc, char* argv[]){
	// At this stage the system clock should have already been configured
	// at high speed.
	myGPIOC_init();
	mySPI_Init();
	myADC_init();

	// Infinite loop
	ADC1->CR |= ADC_CR_ADSTART;

	while (1){

		//trace_printf("Waiting for EOC...\n");
		while((ADC1->ISR & ADC_ISR_EOSEQ) == 0);
		//trace_printf("Reset EOC\n");
		ADC1->ISR &= ~ADC_ISR_EOC;
		int test = (ADC1->DR & 0x00FF);
		trace_printf("Value: %d\n",test);
		ADC1->ISR &= ~ADC_ISR_EOSEQ;

	}

}

void myADC_init(){

	// USE PC0 (ADC_IN10)
	ADC1->ISR = 0x0000;								// Ensure ADC_ISR is reset
	ADC1->IER = 0x0000;								// Disable all interrupts
	ADC1->CFGR1 = 0x0000;							// Ensure configuration is reset
	ADC1->CFGR2 = 0x0000;							// Ensure configuration is reset

	ADC1->IER |= ADC_IER_EOCIE;						// Enable EOC interrupts
	ADC1->IER |= ADC_IER_ADRDYIE;					// Enable ADRDY interrupts

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;				// Activate ADC clock1
	ADC1->CFGR2 |= ADC_CFGR2_CKMODE_1;				// Set clock mode

	ADC1->CFGR1 |= ADC_CFGR1_CONT;					// Set to continuous mode
	ADC1->CFGR1 |= ADC_CFGR1_RES_1;					// Set resolution of conversation to 8bit

	ADC1->SMPR |= ADC_SMPR1_SMPR;					// Set lowest sampling rate

	ADC1->CHSELR = 0x0400;							// Select Channel 10
	ADC1->CR = 0x0000;								// Ensure ADC_CR_ADEN == 0
	ADC1->CR |= ADC_CR_ADCAL;						// Set ADC_CR_ADCAL to 1

	while(((ADC1->CR & ADC_CR_ADCAL) << 1) == 1);	// Wait for ADCAL to == 0

	CF = (ADC1->DR & 0x007F);						// Read calibration factor
	trace_printf("Calibrated: %d\n",CF);
													// ENABLE ADC
	ADC1->CR |= ADC_CR_ADEN;						// Raise ADC_CR_ADEN to 1
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);		// Wait for ADRDY to == 1
	trace_printf("ADC Ready\n");
}

void myDAC_init(){
	// PA4
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;				//Activate DAC clock

	DAC->CR |= DAC_CR_EN1;							//Enable DAC

}

void myGPIOC_init()
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

void myGPIOB_Init(){

	/* Enable clock for GPIOC peripheral */
		// Relevant register: RCC->AHBENR
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

		GPIOB->AFR[0] &= 0;							// RESET AFR
		GPIOB->AFR[1] &= 0;							// RESET AFR

		// Relevant register: GPIOC->MODER
		GPIOB->MODER |= GPIO_MODER_MODER3_1;			// SCK
		GPIOB->MODER |= GPIO_MODER_MODER4_0;			// LCK
		GPIOB->MODER |= GPIO_MODER_MODER5_1;			// MOSI

		/* Ensure no pull-up/pull-down for PB3,4,5 */
		// Relevant register: GPIOC->PUPDR
		GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
		GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
		GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);



}

void mySPI_Init(){

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	myGPIOB_Init();

	SPI1->CR1 |= SPI_CR1_SSM;

	SPI_InitTypeDef SPI_InitStructInfo;
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;

	SPI_Init(SPI1,SPI_InitStruct);

	SPI_Cmd(SPI1,ENABLE);

	uint8_t data = 0b00100000;				// Set to 4-bit interface
	GPIOB->ODR &= ~(0x0010);				// FORCE LCK to 0;
	while( ((SPI1->SR & SPI_SR_BSY) >> 7 ) != 0 || ((SPI1->SR & SPI_SR_TXE) >> 1) != 1);
	SPI_SendData8(SPI1,data);
	trace_printf("Finished writing\n");
	while ((SPI1->SR & SPI_SR_BSY >> 7) == 1);
	GPIOB->ODR |= 0x0010;					// FORCE LCK to 1;

	uint8_t i2 = 0x28;
	uint8_t i3 = 0x0C;
	uint8_t i4 = 0x06;
	uint8_t i5 = 0x01;

	write_SPI(i2);
	write_SPI(i3);
	write_SPI(i4);
	write_SPI(i5);

	uint8_t addr = 0x80;

	write_SPI(addr);
	write_SPI((uint8_t) 'F');

}

void write_SPI(uint8_t data){


	uint8_t splits[6];

	splits[0] = 0x00 & ((data & 0xF0) >> 4);
	splits[1] = 0x80 & ((data & 0xF0) >> 4);
	splits[2] = 0x00 & ((data & 0xF0) >> 4);

	splits[3] = 0x00 & (data & 0x0F);
	splits[4] = 0x80 & (data & 0x0F);
	splits[5] = 0x00 & (data & 0x0F);

	for (int i = 0; i < 6; i++){
		GPIOB->ODR = 0x00000000;	// FORCE LCK to 0;
		while( ((SPI1->SR & SPI_SR_BSY) >> 7 ) != 0 || ((SPI1->SR & SPI_SR_TXE) >> 1) != 1);
		SPI_SendData8(SPI1,splits[i]);
		trace_printf("wait 1 ");
		trace_printf("wait 2 ");
		trace_printf("wait 3\n");
		while ((SPI1->SR & SPI_SR_BSY >> 7) == 1);
		GPIOB->ODR = 0x00000010;	// FORCE LCK to 1;
	}


}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
