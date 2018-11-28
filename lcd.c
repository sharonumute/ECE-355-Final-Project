#include <stdio.h>
#include <string.h>
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_spi.h"
#include "cmsis/cmsis_device.h"
#include "diag/Trace.h"
#include "assert.h"
#include "lcd.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

void LCD_SetValues(uint16_t resistance, unsigned int frequency)
{

    LCD_SetPosition(2, 1);

    Write_Text_to_lcd("R:");

    Write_numb_to_lcd(resistance);

    Write_Text_to_lcd("Oh");

    LCD_SetPosition(1, 1);

    Write_Text_to_lcd("F:");

    Write_numb_to_lcd(frequency);

    Write_Text_to_lcd("Hz");

    time_Delay(250);
}

void LCD_Config()
{
    myGPIOB_Init(); /* Initialize GPIOB */
    mySPI_Init();   /* Initialize SPI */
    LCD_Init();     /* Initialize LCD */
    trace_printf("\nLCD initialization complete\n");
}

void LCD_Init()
{

    Write_HC595(0x02); // Let EN = 0, RS = 0, DB[7:4] = 0010
    Write_HC595(0x82); // Let EN = 1, RS = 0, DB[7:4] = 0010
    Write_HC595(0x02); // Let EN = 0, RS = 0, DB[7:4] = 0010

    time_Delay(20);

    Write_cmd_to_lcd(0x28); // 4­bits, 2 lines, 5x7 font
    Write_cmd_to_lcd(0x0E); // Display ON, No cursors
    Write_cmd_to_lcd(0x06); // Entry mode­ Auto­increment, No Display shifting
    Write_cmd_to_lcd(0x01); // Clear display

    time_Delay(20);
}

void time_Delay(uint32_t d)
{

    TIM3->CNT |= 0x00000000; /* Clear timer */

    TIM3->ARR = d; /* Time­out value */

    TIM3->EGR |= 0x0001; /* Update registers */

    TIM3->CR1 |= TIM_CR1_CEN; /* Start timer */

    while ((TIM3->SR & TIM_SR_UIF) == 0)
    {
    } /* Wait for time­out delay */

    TIM3->SR &= ~(TIM_SR_UIF);   /* Clear update interrupt flag */
    TIM3->CR1 &= ~(TIM_CR1_CEN); /* Stop timer */
}

void Write_cmd_to_lcd(char cmd)
{
    char Low_Nibble = (cmd & 0x0f);
    char High_Nibble = ((cmd & 0xf0) >> 4);
    //High
    Write_HC595(0x00 + High_Nibble); // Let RS = 0 and EN = 0
    Write_HC595(0x80 + High_Nibble); // Let RS = 0 and EN = 1
    Write_HC595(0x00 + High_Nibble); // Let RS = 0 and EN = 0

    //Low
    Write_HC595(0x00 + Low_Nibble); // Let RS = 0 and EN = 0
    Write_HC595(0x80 + Low_Nibble); // Let RS = 0 and EN = 1
    Write_HC595(0x00 + Low_Nibble); // Let RS = 0 and EN = 0
}

void Write_Data_to_lcd(char data)
{
    char Low_Nibble = (data & 0x0f);
    char High_Nibble = ((data & 0xf0) >> 4);
    //High
    Write_HC595(0x40 + High_Nibble); // Let RS = 1 and EN = 0
    Write_HC595(0xc0 + High_Nibble); // Let RS = 1 and EN = 1
    Write_HC595(0x40 + High_Nibble); // Let RS = 1 and EN = 0

    //Low
    Write_HC595(0x40 + Low_Nibble); // Let RS = 1 and EN = 0
    Write_HC595(0xc0 + Low_Nibble); // Let RS = 1 and EN = 1
    Write_HC595(0x40 + Low_Nibble); // Let RS = 1 and EN = 0
}

void Write_HC595(char data)
{

    // LCK = 0
    GPIOB->BRR = GPIO_Pin_4;

    // Wait until not busy
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET)
    {
        ;
    }

    SPI_SendData8(SPI1, data);

    // Wait until not busy
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET)
    {
        ;
    }

    // LCK = 1 (rising edge)
    GPIOB->BSRR = GPIO_Pin_4;
}

void LCD_SetPosition(unsigned short x, unsigned short y)

{
    unsigned short temp = 127 + y;
    if (x == 2)
        temp = temp + 64;
    Write_cmd_to_lcd(temp); //moves cursor
}

void Write_Text_to_lcd(char *StrData)
{
    unsigned short p = 0;
    unsigned short q = strlen(StrData); //Calculates the string length
    while (p < q)
    {
        Write_Data_to_lcd(StrData[p]); //Writes the string data to the LCD
        p++;
    }
}

void Write_numb_to_lcd(uint16_t num)
{
    char Pot_st[5] = {'0', '0', '0', '0', 0};
    Convert_to_ascii(num, Pot_st); //Converters the num in to ASCII and puts it in the array
    Write_Text_to_lcd(Pot_st);
}

//Converts integer to character
void Convert_to_ascii(uint16_t n, char s[])
{
    //uint16_t count = 3;
    //off set by 0x30 or '0' for dec to ascii
    s[0] = n / 1000 + '0';
    s[1] = ((n % 1000) / 100) + '0';
    s[2] = ((n % 100) / 10) + '0';
    s[3] = n % 10 + '0';
}

void myGPIOB_Init(void)
{
    //Clock enable
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    GPIO_InitTypeDef GPIO_InitStruct;

    // RCC config
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    // GPIO Alternate Function config
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_0);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_0);

    // Configure PB3 and PB5 alternate function
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure PB4 in output mode to be used as storage clock input in 74HC595
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configure pins required for SPI peripheral */
    /* Enable clock for GPIOB peripheral */
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    /* Configure PB3 as alternate function [SCK]*/
    GPIOB->MODER |= GPIO_MODER_MODER3_1;

    /* Select alternate function for PB3  */
    /* Set AFRy[3:0] to 0 to select AF0 for PB3 */
    GPIOB->AFR[0] |= GPIO_AFRL_AFR0;

    /* Configure PB4 as output [LCK] */
    GPIOB->MODER |= GPIO_MODER_MODER4_0;

    /* Ensure no pull-up/pull-down for PB4 */
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

    /* Configure PB5 as alternate function [MOSI]*/
    GPIOB->MODER |= GPIO_MODER_MODER5_1;

    /* Select alternate function for PB5 */
    GPIOB->AFR[0] |= GPIO_AFRL_AFR0;
}

void mySPI_Init()
{
    /*Create SPI Struct*/
    SPI_InitTypeDef SPI_InitStructInfo;

    /*Initialize our SPI Struct with the struct Info*/
    SPI_InitTypeDef *SPI_InitStruct = &SPI_InitStructInfo;

    //FROM DALERS SLIDES 21
    /*Specifies the SPI unidirectional or bidirectional data mode*/
    SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;

    /*Turn Master mode on*/
    SPI_InitStruct->SPI_Mode = SPI_Mode_Master;

    /*Set Data size*/
    SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;

    /*Initialize serial clock steady state*/
    SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;

    /*Initialize the clock active edge for the bit capture*/
    SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;

    /*Initialize the NSS signal to be measured by hardware*/
    SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;

    /*Specifies the Baud Rate prescaler value which will be
    used to configure the transmit and receive SCK clock.*/
    SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;

    /*Specifies the data transfer to start from the MSB*/
    SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;

    /*Specify the polynomial used for calculation*/
    SPI_InitStruct->SPI_CRCPolynomial = 7;

    //Initialize SPI1 with the SPI_InitStruct
    SPI_Init(SPI1, SPI_InitStruct);

    //Enable SPI1
    SPI_Cmd(SPI1, ENABLE);
}
