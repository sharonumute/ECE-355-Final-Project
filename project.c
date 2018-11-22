//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "assert.h"
#include "lcd.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
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


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
#define ONE_MS_PER_TICK_PRESCALER ((uint16_t)((SystemCoreClock - 1) / 1000))

/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)
#define LCD_UPDATE_PERIOD_MS ((uint32_t)250)

/* Circuit tuning parameterss */
#define RESISTANCE_MAX_VALUE (5000.0) // PBMCUSLK uses a 5k pot
#define ADC_MAX_VALUE ((float)(0xFFF)) // ADC bit resolution (12 by default)
#define DAC_MAX_VALUE ((float)(0xFFF)) // DAC bit resolution (12 by default)
#define DAC_MAX_VOLTAGE (2.95) // measured output from PA4 when DAC->DHR12R1 = DAC_MAX_VALUE
#define OPTO_DEADBAND_END_VOLTAGE (1.0) // voltage needed to overcome diode

void myGPIOA_Init(void);
void myTIM2_Init(void);
void myTIM16_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
uint32_t getPotADCValue(void);

// Your global variables...
float signalFrequency = 0.0;
float resistance = 0.0;

int
main(int argc, char* argv[])
{

	trace_printf("This is Part 2 of Introductory Lab...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */
	myADC_Init();
	myDAC_Init();
	LCD_Config();
	myTIM16_Init();

	while (1)
	{
		/*Get ADC value*/
		uint32_t potADCValue = getPotADCValue();
		
        /* Update the DAC value */
        DAC->DHR12R1 = potADCValue;

        /* Convert to resistance range */
        float normalizedPotADC = (((float)potADCValue) / ADC_MAX_VALUE);
        resistance = normalizedPotADC * RESISTANCE_MAX_VALUE;

        trace_printf("ADC Value: %d\n", potADCValue);
        trace_printf("Resistance: %f\n", resistance);
	}

	return 0;

}


void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	/* Configure PA0 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);

	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	/* Configure PA1 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
	
	/* Configure PA4 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER4);

	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
}


void myTIM2_Init()
{
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
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;
	/* Start counting timer pulses */
	TIM2->CR1 |= TIM_CR1_CEN;
}



void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] &= 0x0000FF0F;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn,0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}

void myTIM16_Init()
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; /*Enable clock for TIM16*/
	
    TIM16->CR1 = ((uint16_t) 0x008C); /* Set-up TIM16 */    
	
    TIM16->PSC = ONE_MS_PER_TICK_PRESCALER; /* Set-up clock prescaler */
	
    TIM16->ARR = LCD_UPDATE_PERIOD_MS;  /* Set-up auto-reloaded delay */
	
    TIM16->EGR = ((uint16_t) 0x0001); /* Timer Update */
	
    NVIC_SetPriority(TIM16_IRQn, 1); /* Set-up TIM16 interrupt priority = 1 in NVIC (gets triggered by edge measurments, from P0) */
    
    NVIC_EnableIRQ(TIM16_IRQn); /* Enable TIM16 interrupts in NVIC */

    TIM16->DIER |= TIM_DIER_UIE; /* Enable update interrupt generation */

    /* Start counting timer pulses */
    TIM16->CR1 |= TIM_CR1_CEN;
}


void myADC_Init(){
    /* Enable clock for ADC */
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    /* Begin ADC Callibration*/
    ADC1->CR = ADC_CR_ADCAL;
    while (ADC1->CR == ADC_CR_ADCAL) {};

    /* Configure ADC */
    ADC1->CFGR1 |= (ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD);

    /* Select Channel 0 for PA0*/
    ADC1->CHSELR = ADC_CHSELR_CHSEL0;

    /* Enable ADC */
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {};
}

void myDAC_Init() {
    /* Enable DAC clock */
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    /* Enable DAC */
    DAC->CR |= DAC_CR_EN1;
}

void TIM16_IRQHandler()
{
    /* Check if update interrupt flag is set */
    if ((TIM16->SR & TIM_SR_UIF) != 0) {
		LCD_SetValues(resistance, signalFrequency);

        /* Clear update interrupt flag */
        TIM16->SR &= ~(TIM_SR_UIF);

        /* Restart timer */
        TIM16->CR1 |= TIM_CR1_CEN;
    }
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	// Your local variables...
	uint32_t CountRegister;
	uint16_t TimerTriggered;
	float signalPeriod;

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		TimerTriggered = (TIM2->CR1 & TIM_CR1_CEN);
		
		if (TimerTriggered) {
			/* Stop timer */
			/* get timer value */
			TIM2->CR1 &= ~(TIM_CR1_CEN);
			CountRegister = TIM2->CNT;
			
			if(CountRegister < SystemCoreClock){

				signalFrequency = ((double)SystemCoreClock)/((double)CountRegister);
				signalPeriod = 1/signalFrequency;
			
				trace_printf("Signal Period: %f \n", (float)(signalPeriod));
				trace_printf("Signal Frequency: %f \n", (float)signalFrequency);
				
			}else{
				
				signalPeriod = ((double)CountRegister)/((double)SystemCoreClock);
				signalFrequency = 1/signalPeriod;

				trace_printf("Signal Period: %f \n", (float)signalPeriod );
				trace_printf("Signal Frequency: %f \n", (float)(signalFrequency));
			}

		} else {
			/* Restart timer */
			TIM2->CNT = (uint32_t)0x0;
			TIM2->CR1 |= TIM_CR1_CEN;
		}

		/* clear EXT1 */
		EXTI->PR |= EXTI_PR_PR1;
	}
}

uint32_t getPotADCValue(){
    /* start ADC */
    ADC1->CR |= ADC_CR_ADSTART;

    /* loop till end */
    while (!(ADC1->ISR & ADC_ISR_EOC)) {};

    /* reset EOC flag */
    ADC1->ISR &= ~(ADC_ISR_EOC);

    /* Apply data mask to data register */
    return ((ADC1->DR) & ADC_DR_DATA);
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
