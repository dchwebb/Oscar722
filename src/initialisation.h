#pragma once

#include "stm32f7xx.h"


// Coverage profiler macros using timer 4 to count clock cycles / 10
#define CP_ON		TIM9->EGR |= TIM_EGR_UG; TIM9->CR1 |= TIM_CR1_CEN; coverageTimer=0;
#define CP_OFF		TIM9->CR1 &= ~TIM_CR1_CEN;
#define CP_CAP		TIM9->CR1 &= ~TIM_CR1_CEN; coverageTotal = (coverageTimer * 65536) + TIM9->CNT;

#define DB_ON		TIM5->EGR |= TIM_EGR_UG; TIM5->CR1 |= TIM_CR1_CEN;
#define DB_OFF		TIM5->CR1 &= ~TIM_CR1_CEN;

//	Define encoder pins and timers for easier reconfiguring
#define L_ENC_CNT	TIM8->CNT
#define R_ENC_CNT	TIM4->CNT
#define R_BTN_NO(a) a ## 7
#define R_BTN_GPIO	GPIOA
#define L_BTN_NO(n) n ## 2
#define L_BTN_GPIO	GPIOC

#define ADC_BUFFER_LENGTH 12

extern volatile uint16_t ADC_array[];
enum encoderType { HorizScaleCoarse, HorizScaleFine, CalibVertScale, CalibVertOffset, VoltScale, TriggerChannel, TriggerY, FFTAutoTune, FFTChannel, ChannelSelect };
enum mode { Oscilloscope, Fourier, Waterfall, Circular, MIDI };
enum oscChannel {channelA, channelB, channelC, channelNone};

//	Encoder state table
#define DIR_CW 0x10
#define DIR_CCW 0x20
#define R_START 0x0
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6

const uint8_t encTable[7][4] = {
 {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
 {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
 {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
 {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
 {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
 {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
 {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};

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
