// Main sample capture
void TIM3_IRQHandler(void) {

	TIM3->SR &= ~TIM_SR_UIF;					// clear UIF flag

	if (displayMode == Fourier || displayMode == Waterfall) {
		if (capturePos == fft.samples && capturing) {
			fft.dataAvailable[captureBufferNumber] = true;
			capturing = false;
		}

		if (capturing) {
			// For FFT Mode we want a value between +- 2047
			if (fft.channel == channelA)
				fft.FFTBuffer[captureBufferNumber][capturePos] = 2047 - ((float)(ADC_array[0] + ADC_array[3] + ADC_array[6] + ADC_array[9]) / 4);
			else if (fft.channel == channelB)
				fft.FFTBuffer[captureBufferNumber][capturePos] = 2047 - ((float)(ADC_array[1] + ADC_array[4] + ADC_array[7] + ADC_array[10]) / 4);
			else if (fft.channel == channelC)
				fft.FFTBuffer[captureBufferNumber][capturePos] = 2047 - ((float)(ADC_array[2] + ADC_array[5] + ADC_array[8] + ADC_array[11]) / 4);
			capturePos ++;
		}

	} else if (displayMode == Circular) {
		// Average the last four ADC readings to smooth noise
		if (fft.channel == channelA)
			adcA = ADC_array[0] + ADC_array[3] + ADC_array[6] + ADC_array[9];
		else if (fft.channel == channelB)
			adcA = ADC_array[1] + ADC_array[4] + ADC_array[7] + ADC_array[10];
		else
			adcA = ADC_array[2] + ADC_array[5] + ADC_array[8] + ADC_array[11];


		// check if we should start capturing - ie there is a buffer spare and a zero crossing has occured
		if (!capturing && oldAdc < CalibZeroPos && adcA >= CalibZeroPos && (!circDataAvailable[0] || !circDataAvailable[1])) {
			capturing = true;
			captureBufferNumber = circDataAvailable[0] ? 1 : 0;		// select correct capture buffer based on whether buffer 0 or 1 contains data
			capturePos = 0;				// used to check if a sample is ready to be drawn
			zeroCrossings[captureBufferNumber] = 0;
		}

		// If capturing store current readings in buffer and increment counters
		if (capturing) {
			OscBufferA[captureBufferNumber][capturePos] = adcA;

			// store array of zero crossing points
			if (capturePos > 10 && oldAdc < CalibZeroPos && adcA >= CalibZeroPos) {
				zeroCrossings[captureBufferNumber] = capturePos;
				circDataAvailable[captureBufferNumber] = true;

				captureFreq[captureBufferNumber] = FreqFromPos(capturePos);		// get frequency here before potentially altering sampling speed
				capturing = false;

				// auto adjust sample time to try and get the longest sample for the display (280 is number of pixels wide we ideally want the captured wave to be)
				if (capturePos < 280) {
					int16_t newARR = capturePos * (TIM3->ARR + 1) / 280;
					if (newARR > 0)
						TIM3->ARR = newARR;
				}

			// reached end  of buffer and zero crossing not found - increase timer size to get longer sample
			} else if (capturePos == DRAWWIDTH - 1) {
				capturing = false;
				TIM3->ARR += 30;

			} else {
				capturePos++;
			}
		}
		oldAdc = adcA;

	} else if (displayMode == Oscilloscope) {
		// Average the last four ADC readings to smooth noise
		adcA = ADC_array[0] + ADC_array[3] + ADC_array[6] + ADC_array[9];
		adcB = ADC_array[1] + ADC_array[4] + ADC_array[7] + ADC_array[10];
		adcC = ADC_array[2] + ADC_array[5] + ADC_array[8] + ADC_array[11];

		// check if we should start capturing - ie not drawing from the capture buffer and crossed over the trigger threshold (or in free mode)
		if (!capturing && (!drawing || captureBufferNumber != drawBufferNumber) && (osc.TriggerTest == nullptr || (bufferSamples > osc.TriggerX && oldAdc < osc.TriggerY && *osc.TriggerTest >= osc.TriggerY))) {
			capturing = true;

			if (osc.TriggerTest == nullptr) {								// free running mode
				capturePos = 0;
				drawOffset[captureBufferNumber] = 0;
				capturedSamples[captureBufferNumber] = -1;
			} else {
				// calculate the drawing offset based on the current capture position minus the horizontal trigger position
				drawOffset[captureBufferNumber] = capturePos - osc.TriggerX;
				if (drawOffset[captureBufferNumber] < 0)	drawOffset[captureBufferNumber] += DRAWWIDTH;

				capturedSamples[captureBufferNumber] = osc.TriggerX - 1;	// used to check if a sample is ready to be drawn
			}
		}

		// if capturing check if write buffer is full and switch to next buffer if so; if not full store current reading
		if (capturing && capturedSamples[captureBufferNumber] == DRAWWIDTH - 1) {
			captureBufferNumber = captureBufferNumber == 1 ? 0 : 1;		// switch the capture buffer
			bufferSamples = 0;			// stores number of samples captured since switching buffers to ensure triggered mode works correctly
			capturing = false;
		}

		// If capturing or buffering samples waiting for trigger store current readings in buffer and increment counters
		if (capturing || !drawing || captureBufferNumber != drawBufferNumber) {
			DAC->DHR12R1 = ADC_array[0];
			OscBufferA[captureBufferNumber][capturePos] = adcA;
			OscBufferB[captureBufferNumber][capturePos] = adcB;
			OscBufferC[captureBufferNumber][capturePos] = adcC;
			oldAdc = *osc.TriggerTest;

			if (capturePos == DRAWWIDTH - 1)	capturePos = 0;
			else								capturePos++;

			if (capturing)	capturedSamples[captureBufferNumber]++;
			else 			bufferSamples++;

		}
	}
}

// Left Encoder Button
void EXTI2_IRQHandler(void) {

	if (!(GPIOC->IDR & L_BTN_NO(GPIO_IDR_IDR_))) 				// Encoder button pressed - L_BTN_NO() adds number of encoder button eg GPIO_IDR_IDR_2
		DB_ON													// Enable debounce timer
	if (GPIOC->IDR & L_BTN_NO(GPIO_IDR_IDR_) && TIM5->CNT > 100) {	// Encoder button released - check enough time has elapsed to ensure not a bounce. A quick press if around 300, a long one around 8000+
		encoderBtnL = true;
		DB_OFF													// Disable debounce timer
	}
	EXTI->PR |= L_BTN_NO(EXTI_PR_PR);							// Clear interrupt pending
}

// Right Encoder Button
void EXTI9_5_IRQHandler(void) {

	if (!(R_BTN_GPIO->IDR & R_BTN_NO(GPIO_IDR_IDR_))) 			// Encoder button pressed - R_BTN_NO() adds number of encoder button eg GPIO_IDR_IDR_7
		DB_ON													// Enable debounce timer
	if (R_BTN_GPIO->IDR & R_BTN_NO(GPIO_IDR_IDR_) && TIM5->CNT > 100) {	// Encoder button released - check enough time has elapsed to ensure not a bounce. A quick press if around 300, a long one around 8000+
		encoderBtnR = true;
		DB_OFF													// Disable debounce timer
	}
	EXTI->PR |= R_BTN_NO(EXTI_PR_PR);							// Clear interrupt pending
}

//	Coverage timer
void TIM1_BRK_TIM9_IRQHandler(void) {
	TIM9->SR &= ~TIM_SR_UIF;									// clear UIF flag
	coverageTimer ++;
}

// MIDI Decoder
void UART4_IRQHandler(void) {
#ifndef STM32F722xx
	if (UART4->SR | USART_SR_RXNE) {
		midi.MIDIQueue.push(UART4->DR);							// accessing DR automatically resets the receive flag
	}
#else
	if (UART4->ISR | USART_ISR_RXNE) {
		midi.MIDIQueue.push(UART4->RDR);						// accessing DR automatically resets the receive flag
	}
#endif
}
