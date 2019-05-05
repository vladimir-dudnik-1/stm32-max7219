//*****************************************************************************
//
//! \file main.c
//! \brief main application
//! \version 1.0.0.0
//! \date $Creat_time$
//! \author $Creat_author$
//! \copy
//!
//! Copyright (c) 2014 CooCox.  All rights reserved.
//
//! \addtogroup project
//! @{
//! \addtogroup main
//! @{
//*****************************************************************************

#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"

#define SPI_MAX7219 SPI1
#define NSS         GPIO_Pin_4
#define SCK         GPIO_Pin_5
#define MOSI        GPIO_Pin_7

/* timing is not guaranteed :) */
void simple_delay(uint32_t us)
{
	/* simple delay loop */
	while (us--) {
		__asm volatile ("nop");
	}
}


// max7219
typedef enum
{
    REG_NO_OP         = 0x00 << 8,
    REG_DIGIT_0       = 0x01 << 8,
    REG_DIGIT_1       = 0x02 << 8,
    REG_DIGIT_2       = 0x03 << 8,
    REG_DIGIT_3       = 0x04 << 8,
    REG_DIGIT_4       = 0x05 << 8,
    REG_DIGIT_5       = 0x06 << 8,
    REG_DIGIT_6       = 0x07 << 8,
    REG_DIGIT_7       = 0x08 << 8,
    REG_DECODE_MODE   = 0x09 << 8,
    REG_INTENSITY     = 0x0A << 8,
    REG_SCAN_LIMIT    = 0x0B << 8,
    REG_SHUTDOWN      = 0x0C << 8,
    REG_DISPLAY_TEST  = 0x0F << 8,

} MAX7219_REGISTERS;


typedef enum
{
    DIGIT_1 = 1,
    DIGIT_2 = 2,
    DIGIT_3 = 2,
    DIGIT_4 = 2

} MAX7219_Digits;


/*  DP A B C  D E F G  */
// A 0 1 1 1  0 1 1 1  0x77
// b 0 0 0 1  1 1 1 1  0x1f
// C 0 1 0 0  1 1 1 0  0x4e
// d 0 0 1 1  1 1 0 1  0x3d
// E 0 1 0 0  1 1 1 1  0x4f
// F 0 1 0 0  0 1 1 1  0x47

static
uint16_t SYMBOLS[] =
{
    0x7E, // 0
	0x30, // 1
	0x6D, // 2
	0x79, // 3
	0x33, // 4
    0x5B, // 5
	0x5F, // 6
	0x70, // 7
	0x7F, // 8
	0x7B, // 9
	0x77, // A
	0x1f,
	0x4e,
	0x3d,
	0x4f,
	0x47,
	0x00
};


uint16_t getSymbol(uint8_t number)
{
    return SYMBOLS[number];
}


void sendData(uint16_t data)
{
    SPI_I2S_SendData(SPI_MAX7219, data);

    while (SPI_I2S_GetFlagStatus(SPI_MAX7219, SPI_I2S_FLAG_BSY) == SET)
        GPIO_ResetBits(GPIOA, NSS);

    GPIO_SetBits(GPIOA, NSS);
}


void max7219_init(uint8_t intensivity)
{
    if (intensivity > 0x0F)
        return;

    sendData(REG_SHUTDOWN | 0x01);
    sendData(REG_DECODE_MODE | 0x00);
    sendData(REG_SCAN_LIMIT | 0x07);
    sendData(REG_INTENSITY | intensivity);
}


void max7219_clean()
{
    sendData(REG_DIGIT_0 | 0x00);
    sendData(REG_DIGIT_1 | 0x00);
    sendData(REG_DIGIT_2 | 0x00);
    sendData(REG_DIGIT_3 | 0x00);
    sendData(REG_DIGIT_4 | 0x00);
    sendData(REG_DIGIT_5 | 0x00);
    sendData(REG_DIGIT_6 | 0x00);
    sendData(REG_DIGIT_7 | 0x00);
}


void displayTime(uint8_t* array)
{
    sendData(REG_DIGIT_0 | getSymbol(array[0]));
    sendData(REG_DIGIT_1 | getSymbol(array[1]) | (1 << 7)); // +dot
    sendData(REG_DIGIT_2 | getSymbol(array[2]));
    sendData(REG_DIGIT_3 | getSymbol(array[3]));
    sendData(REG_DIGIT_4 | getSymbol(array[4]));
    sendData(REG_DIGIT_5 | getSymbol(array[5]));
    sendData(REG_DIGIT_6 | getSymbol(array[6]));
    sendData(REG_DIGIT_7 | getSymbol(array[7]));
}


void display32(uint32_t* array)
{
	uint8_t* p = (uint8_t*)array;

	uint8_t b0 = p[0];
	uint8_t b1 = p[1];
	uint8_t b2 = p[2];
	uint8_t b3 = p[3];

	sendData(REG_DIGIT_0 | getSymbol(b0 & 0x0f));
    sendData(REG_DIGIT_1 | getSymbol(b0 >> 4));
    sendData(REG_DIGIT_2 | getSymbol(b1 & 0x0f));
    sendData(REG_DIGIT_3 | getSymbol(b1 >> 4));
    sendData(REG_DIGIT_4 | getSymbol(b2 & 0x0f));
    sendData(REG_DIGIT_5 | getSymbol(b2 >> 4));
    sendData(REG_DIGIT_6 | getSymbol(b3 & 0x0f));
    sendData(REG_DIGIT_7 | getSymbol(b3 >> 4));
}


int led_display(void)
{
    /* reset rcc */
    RCC_DeInit();
    SPI_I2S_DeInit(SPI_MAX7219);

    /* enable clock to GPIOC */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    /* port A CS */
    GPIO_InitTypeDef port_a;
    port_a.GPIO_Pin = NSS;
    port_a.GPIO_Speed = GPIO_Speed_2MHz;
    port_a.GPIO_Mode = GPIO_Mode_Out_PP;
    /* apply configuration */
    GPIO_Init(GPIOA, &port_a);

    /* port A SPI1 AF */
//    GPIO_InitTypeDef port_b;
    port_a.GPIO_Pin = SCK | MOSI; // SCK, MOSI
    port_a.GPIO_Speed = GPIO_Speed_50MHz;
    port_a.GPIO_Mode = GPIO_Mode_AF_PP;
    /* apply configuration */
    GPIO_Init(GPIOA, &port_a);

    /* port C for LED */
    GPIO_InitTypeDef port_c;
    port_c.GPIO_Pin   = GPIO_Pin_9; // LED
    port_c.GPIO_Speed = GPIO_Speed_2MHz;
    port_c.GPIO_Mode  = GPIO_Mode_Out_PP;
    /* apply configuration */
    GPIO_Init(GPIOC, &port_c);

    /* SPI initialization */
    SPI_InitTypeDef spi;

    spi.SPI_Direction         = SPI_Direction_1Line_Tx;
    spi.SPI_DataSize          = SPI_DataSize_16b;
    spi.SPI_CPOL              = SPI_CPOL_Low;
    spi.SPI_CPHA              = SPI_CPHA_1Edge;
    spi.SPI_NSS               = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    spi.SPI_FirstBit          = SPI_FirstBit_MSB;
    spi.SPI_Mode              = SPI_Mode_Master;

    SPI_Init(SPI_MAX7219, &spi);
    SPI_Cmd(SPI_MAX7219, ENABLE);

    SPI_NSSInternalSoftwareConfig(SPI_MAX7219, SPI_NSSInternalSoft_Set);

    max7219_init(15);
    max7219_clean();

    uint8_t vals[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    uint32_t cnt = 0;

    /* main program loop */
    for (;;)
    {
        /* set led on */
        GPIO_SetBits(GPIOC, GPIO_Pin_9);
        /* delay */
        simple_delay(100000);
        /* clear led */
        GPIO_ResetBits(GPIOC, GPIO_Pin_9);
        /* delay */
        simple_delay(100000);

//    	vals[0]++;
//    	if(vals[0] > 0xf)
//    		vals[0] = 0;

//    	displayTime(vals);
    	display32(&cnt);

    	cnt++;
    }

    return 0;
}


#define LED_PORT GPIOC

//void TIM4_IRQHandler(void)
//{
//    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
//    {
//        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
//        GPIOC->ODR ^= GPIO_Pin_13;
//    }
//}

#if 0
int led_flash(void)
{
	// GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef gpio_c;
	gpio_c.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_c.GPIO_Mode  = GPIO_Mode_Out_PP;
	gpio_c.GPIO_Pin   = GPIO_Pin_13 ;
	GPIO_Init(GPIOC, &gpio_c);

    // TIMER4
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseInitTypeDef tim4;
    TIM_TimeBaseStructInit(&tim4);
    tim4.TIM_CounterMode = TIM_CounterMode_Up;
    tim4.TIM_Prescaler = 720000;
    tim4.TIM_Period = 1000;
    TIM_TimeBaseInit(TIM4, &tim4);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);

    /* NVIC Configuration */
    NVIC_InitTypeDef nvic;
    /* Enable the TIM4_IRQn Interrupt */
    nvic.NVIC_IRQChannel = TIM4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    while(1)
    {
//      GPIO_WriteBit(LED_PORT,GPIO_Pin_13,Bit_SET);
//      for (unsigned int i=0;i<1000000;i++);

//        GPIO_WriteBit(LED_PORT,GPIO_Pin_13,Bit_RESET);
//        for (unsigned int i=0;i<1000000;i++);
    }

    return 0;
}
#endif

//void TIM1_IRQHandler(void)
//{
//    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
//    {
//        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
//        GPIOC->ODR ^= GPIO_Pin_13;
//    }
//}

#if 0
int timer_pwm(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitTypeDef gpio;

	//	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = GPIO_Pin_8; // T1C1
    GPIO_Init(GPIOA, &gpio);

    //    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = GPIO_Pin_13; // T1C1N
    GPIO_Init(GPIOB, &gpio);

    TIM_TimeBaseInitTypeDef tim1;
    TIM_TimeBaseStructInit(&tim1);

    tim1.TIM_Prescaler = 72000;
    tim1.TIM_Period = 200;
    TIM_TimeBaseInit(TIM1, &tim1);

    TIM_OCInitTypeDef tim1_oc;
    TIM_OCStructInit(&tim1_oc);
    tim1_oc.TIM_Pulse        = 200;
    tim1_oc.TIM_OCMode       = TIM_OCMode_Toggle;
    tim1_oc.TIM_OutputState  = TIM_OutputState_Enable;
    tim1_oc.TIM_OutputNState = TIM_OutputNState_Enable;

    TIM_OC1Init(TIM1, &tim1_oc);

    TIM_BDTRInitTypeDef tim1_bdtr;
    TIM_BDTRStructInit(&tim1_bdtr);
    tim1_bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &tim1_bdtr);

    TIM_Cmd(TIM1, ENABLE);

    while(1)
    {
//    	__NOP();
        GPIOC->ODR ^= GPIO_Pin_13;
        simple_delay(10000);
    }

	return 0;
}
#endif

int main(void)
{
//	timer_pwm();
	led_display();
	return 0;
}
//! @}
//! @}
