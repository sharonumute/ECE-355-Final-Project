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
#define myTIM3_PRESCALER ((uint16_t)47999)
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

void myGPIOA_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);

// Your global variables...
/* Elapsed timer pulses */
uint32_t timerPulses = 0;
/* TIM2 flag */
uint32_t timerActive = 0;

uint16_t DAC_converted = 0;
unsigned int signalFrequency = 0;
unsigned int voltage = 0;
uint16_t resistance = 0;

int main(int argc, char *argv[])

{

    trace_printf("This is Part 2 of Introductory Lab...\n");
    trace_printf("System clock: %u Hz\n", SystemCoreClock);

    myGPIOA_Init(); /* Initialize I/O port PA */
    myTIM2_Init();  /* Initialize timer TIM2 */
    myTIM3_Init();  /* Initialize timer TIM3 */
    myEXTI_Init();  /* Initialize EXTI */
    myADC_Init();   /* Initialize ADC */
    myDAC_Init();   /* Initialize DAC */
    LCD_Config();   /* Configure and initialize LCD */

    while (1)
    {
        /*Trigger ADC conversion in Software */
        ADC1->CR |= 0x00000004;

        /* Test EOC flag */
        while ((ADC1->ISR & 0x00000004) == 0)
        {
        };

        /* calculate the voltage */
        voltage = (unsigned int)(ADC1->DR);
        DAC->DHR12R1 = (voltage);
        /* Get ADC1 converted data */
        DAC_converted = DAC->DOR1;

        /* Resistance calculations */
        float rescalc = 5000 * ((4095 - ((float)DAC_converted)) / 4095);
        resistance = ((uint16_t)rescalc);

        /*Computer Console prints */
        trace_printf("Frequency:%d\n", (int)signalFrequency);
        trace_printf("voltage:%d\n", (int)voltage);
        trace_printf("Resistance:%d\n\n", (int)resistance);

        LCD_SetValues(resistance, signalFrequency);
    }

    return 0;
}

void myGPIOA_Init()
{
    /* Enable clock for GPIOA peripheral */
    // Relevant register: RCC->AHBENR

    RCC->AHBENR |= 0x00020000;

    /* Configure PA1 as input */
    // Relevant register: GPIOA->MODER

    GPIOA->MODER |= 0x28000000;

    /* Ensure no pull-up/pull-down for PA1 */
    // Relevant register: GPIOA->PUPDR

    GPIOA->PUPDR |= 0x24000000;
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
    TIM2->EGR |= 0x0001;

    /* Assign TIM2 interrupt priority = 0 in NVIC */
    // Relevant register: NVIC->IP[3], or use NVIC_SetPriority
    NVIC_SetPriority(TIM2_IRQn, 0);

    /* Enable TIM2 interrupts in NVIC */
    // Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
    NVIC_EnableIRQ(TIM2_IRQn);

    /* Enable update interrupt generation */
    // Relevant register: TIM2->DIER
    TIM2->DIER |= TIM_DIER_UIE;
    //TIM2->CR1 |= TIM_CR1_CEN;
}

void myEXTI_Init()
{
    /* Enable GPIOA clock */
    RCC->AHBENR |= 0x00020000;
    // Clear PA1 field = IN mode
    GPIOA->MODER |= 0x28000000;
    //Clear PA1 field = NO pull­up/down
    GPIOA->PUPDR |= 0x24000000;

    /* Map EXTI1 line to PA1 */
    // Relevant register: SYSCFG->EXTICR[0]
    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI1);

    /* EXTI1 l
     * define interrupts: set rising-edge trigger */
    // Relevant register: EXTI->RTSR
    EXTI->RTSR |= EXTI_RTSR_TR1;

    /* Unmask interrupts from EXTI1 line */
    // Relevant register: EXTI->IMR
    EXTI->IMR |= EXTI_IMR_MR1;

    /* Assign EXTI1 interrupt priority = 0 in NVIC */
    // Relevant register: NVIC->IP[1], or use NVIC_SetPriority
    NVIC_SetPriority(EXTI0_1_IRQn, 0);

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
        /* Clear update interrupt flag */
        // Relevant register: TIM2->SR
        TIM2->SR &= ~(TIM_SR_UIF);

        /* Restart stopped timer */
        // Relevant register: TIM2->CR1
        TIM2->CR1 |= TIM_CR1_CEN;
    }
}

void myTIM3_Init(void)
{
    /* Enable clock for TIM3 peripheral */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* Configure TIM3: buffer auto­reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
    TIM2->CR1 = ((uint16_t)0x008C);

    /* Set clock prescaler value: 48MHz/(47999+1) = 1 KHz */
    TIM3->PSC = myTIM3_PRESCALER;

    /* Default auto­reloaded delay: 100 ms */
    TIM3->ARR = 100;

    /* Update timer registers */
    TIM3->EGR |= 0x0001;
}

void myADC_Init()
{
    //make pc1 input and analog
    //check ADC -> SR for the EOC bit, if 1 then read the data, otherwise do nothing
    RCC->AHBENR |= 0x00080000; //enable clock on port C

    GPIOC->MODER |= 0x000000c0; //set mode to analog input
    GPIOC->PUPDR |= 0x24000000;

    RCC->APB2ENR |= 0x00000200; //enable ADC clock

    ADC1->CFGR1 &= ~(0x00000020); //continuous
    ADC1->CFGR1 |= 0x00002000;

    ADC1->SMPR |= 0x00000007; //sample length

    ADC1->CHSELR |= 0x00000800; //mapped to port c1

    ADC1->CR |= 0x80000000; //12 bit resolution
    while ((ADC1->CR & 0x80000000) != 0)
    {
    };
    ADC1->CR |= 0x00000001;
    while ((ADC1->ISR & 0x00000001) == 0)
    {
    };
}

void myDAC_Init()
{
    RCC->APB1ENR |= 0x20000000; // Enable DAC clock

    GPIOC->MODER |= 0x00000300;

    DAC->CR &= 0xfffffcff;

    DAC->CR |= 0x00000000; // set bit

    DAC->CR |= 0x00000001; //enable bit to channel 1
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

        if (TimerTriggered)
        {
            /* Stop timer */
            /* get timer value */
            TIM2->CR1 &= ~(TIM_CR1_CEN);
            CountRegister = TIM2->CNT;

            if (CountRegister < SystemCoreClock)
            {

                signalFrequency = ((double)SystemCoreClock) / ((double)CountRegister);
                signalPeriod = 1 / signalFrequency;
            }
            else
            {

                signalPeriod = ((double)CountRegister) / ((double)SystemCoreClock);
                signalFrequency = 1 / signalPeriod;
            }
        }
        else
        {
            /* Restart timer */
            TIM2->CNT = (uint32_t)0x0;
            TIM2->CR1 |= TIM_CR1_CEN;
        }

        /* clear EXT1 */
        EXTI->PR |= EXTI_PR_PR1;
    }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
