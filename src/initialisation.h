#pragma once

#include "stm32f7xx.h"


// Coverage profiler macros using timer 4 to count clock cycles / 10
#define CP_ON		TIM9->EGR |= TIM_EGR_UG; TIM9->CR1 |= TIM_CR1_CEN; coverageTimer=0;
#define CP_OFF		TIM9->CR1 &= ~TIM_CR1_CEN;
#define CP_CAP		TIM9->CR1 &= ~TIM_CR1_CEN; coverageTotal = (coverageTimer * 65536) + TIM9->CNT;

// Button debounce timer
#define DB_ON		TIM5->EGR |= TIM_EGR_UG; TIM5->CR1 |= TIM_CR1_CEN;
#define DB_OFF		TIM5->CR1 &= ~TIM_CR1_CEN;

//	Define encoder pins and timers for easier reconfiguring
#define L_ENC_CNT	TIM8->CNT
#define R_ENC_CNT	TIM4->CNT
#define R_BTN_NO(a) a ## 7
#define R_BTN_GPIO	GPIOA
#define L_BTN_NO(n) n ## 2
#define L_BTN_GPIO	GPIOC

// Define LCD DMA and SPI registers
#define LCD_DMA_STREAM			DMA1_Stream5
#define LCD_SPI 				SPI3
#define LCD_CLEAR_DMA_FLAGS		DMA1->HIFCR = DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5 | DMA_HIFCR_CTEIF5;

// Define macros for setting and clearing GPIO SPI pins
#define LCD_RST_RESET	GPIOB->BSRR |= GPIO_BSRR_BR_0
#define LCD_RST_SET 	GPIOB->BSRR |= GPIO_BSRR_BS_0
#define LCD_DCX_RESET	GPIOC->BSRR |= GPIO_BSRR_BR_0
#define LCD_DCX_SET		GPIOC->BSRR |= GPIO_BSRR_BS_0

#define ADC_BUFFER_LENGTH 12

extern volatile uint16_t ADC_array[];
enum encoderType { HorizScaleCoarse, HorizScaleFine, CalibVertScale, CalibVertOffset, VoltScale, TriggerChannel, TriggerY, FFTAutoTune, FFTChannel, ChannelSelect };
enum mode { Oscilloscope, Fourier, Waterfall, Circular, MIDI };
enum oscChannel {channelA, channelB, channelC, channelNone};

void SystemClock_Config(void);
void InitSysTick();
void InitLCDHardware(void);
void InitADC(void);
void InitSampleAcquisition();
void InitCoverageTimer();
void InitDebounceTimer();
void InitEncoders();
void InitUART();
void InitDAC();
