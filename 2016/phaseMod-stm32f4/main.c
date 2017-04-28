//
// phase modulation for the stm32f4-discovery microcontroller
//
// Based on the the work by Tom Erbe and Liviu Ionescu
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <stm32f4_discovery_audio.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "sinetable.h"
#include "freqtable.h"

// ----- main() ---------------------------------------------------------------

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


// pins connected on discovery board
// A0 4 5 6 7 9 10 11 12
// B6 9 10
// C0 3 4 7 10 12 14 15
// D4 5 12 13 14 15
// E0 1 3
// DAC B0, B1,
// SWITCH D0 1 2 3
// Timer, LED and Switch globals
TIM_HandleTypeDef TimHandle;
float millisecs;
int32_t ledselected;
int32_t time, button_time[16];

// ADC globals
ADC_HandleTypeDef AdcHandle;
__IO uint16_t adcBuffer[8];
uint32_t smoothadc[8], avgtotal[8];
uint32_t adc0average[64];
uint32_t adc1average[64];
int16_t avgindex;

// DAC globals
DAC_HandleTypeDef    DacHandle;
uint16_t dacBuffer[32]; // 32 samples X 1 channels

// I2S globals
extern I2S_HandleTypeDef hAudioOutI2s;
extern I2S_HandleTypeDef hAudioInI2s;
int16_t codecBuffer[64]; // 32 samples X 2 channels
int bspAudioState;

// Sound globals
void audioBlock(float *input, float *output, int32_t samples);
float inBuffer[1024], outBuffer[1024]; // interleaved - LRLRLRLRLRLRLRLRLRLRLR - inBuffer[frame << 1] inBuffer[(frame << 1) + 1]

// Synthesis types
struct rando
{
	uint32_t seed;
};

struct osco
{
	float pi;
	float phs;
	float *tab;
	uint32_t ts;
	uint32_t tm;
};

struct lino
{
	float val;		// current value
	float dst;		// value to move to
	float inc;		// increment per sample to get from val to dst
	int up;         // upwards movement flag
};

// Synthesis globals
struct rando r;
struct osco o[8];

int32_t f[8]; // random frequencies
struct lino amp[4]; // envelopes
struct lino pmi[4]; // phase modulation indices
int32_t env_len = 1000000; // 1m samples


void
led_init()
{
  // Enable GPIO Peripheral clock
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure pin in output push/pull mode
  GPIO_InitStructure.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void
switch_init()
{
  // Enable GPIO Peripheral clock
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure for interrupt
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
   HAL_NVIC_SetPriority(EXTI0_IRQn, 8, 3);
   HAL_NVIC_EnableIRQ(EXTI0_IRQn);

   HAL_NVIC_SetPriority(EXTI1_IRQn, 8, 2);
   HAL_NVIC_EnableIRQ(EXTI1_IRQn);

   HAL_NVIC_SetPriority(EXTI2_IRQn, 8, 1);
   HAL_NVIC_EnableIRQ(EXTI2_IRQn);

   HAL_NVIC_SetPriority(EXTI3_IRQn, 8, 0);
   HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void
timer3_init()
{
	  __HAL_RCC_TIM3_CLK_ENABLE();

	  TimHandle.Instance = TIM3;
	  TimHandle.Init.Period = 8399; // Clock = 168000000/2 = 84000000/(8399+1) = 10000
	  TimHandle.Init.Prescaler = 0;
	  TimHandle.Init.ClockDivision = 0;
	  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	  HAL_TIM_Base_Init(&TimHandle) ;
	  HAL_TIM_Base_Start_IT(&TimHandle);

	  /*##-2- Configure the NVIC for TIMx ########################################*/
	  /* Set Interrupt Group Priority */
	  HAL_NVIC_SetPriority(TIM3_IRQn, 4, 0);
	  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void adc1_init(void)
{
	int i;

	for(i = 0; i < 64; i++)
		adc0average[i] = adc1average[i] = 0.0f;
    avgindex = 0;
    avgtotal[0] = avgtotal[1] = 0;

    ADC_ChannelConfTypeDef sConfig;
	  AdcHandle.Instance = ADC1;

	  AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
	  AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
	  AdcHandle.Init.ScanConvMode = ENABLE;	// to scan multiple channels
	  AdcHandle.Init.ContinuousConvMode = ENABLE;
	  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
	  AdcHandle.Init.NbrOfDiscConversion = 0;
	  AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	  AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  AdcHandle.Init.NbrOfConversion = 2;
	  AdcHandle.Init.DMAContinuousRequests = ENABLE;
	  AdcHandle.Init.EOCSelection = DISABLE;

	  if(HAL_ADC_Init(&AdcHandle) != HAL_OK)
	  {
	    /* Initialization Error */
	    ;
	  }

	  /*##-2- Configure ADC regular channel ######################################*/
	  /* Note: Considering IT occurring after each number of size of              */
	  /*       "uhADCxConvertedValue"  ADC conversions (IT by DMA end             */
	  /*       of transfer), select sampling time and ADC clock with sufficient   */
	  /*       duration to not create an overhead situation in IRQHandler.        */
	  sConfig.Channel = ADC_CHANNEL_9;	// Port B, Pin 1
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	  sConfig.Offset = 0;

	  HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);


	  /*##-3- Start the conversion process and enable interrupt ##################*/
	  /* Note: Considering IT occurring after each number of ADC conversions      */
	  /*       (IT by DMA end of transfer), select sampling time and ADC clock    */
	  /*       with sufficient duration to not create an overhead situation in    */
	  /*        IRQHandler. */
	  HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)&adcBuffer[0], 1);
}

// random number [0, 15]
int32_t noise(void)
{
    r.seed = (r.seed * 196314165) + 907633515;
    return (r.seed >> 28);
}

int
main(int argc, char* argv[])
{
	int i;
	timer_start();

	// synth init
	for (i = 0; i<8; i++)
	{
		o[i].phs = 0.0f;
		o[i].pi = 0.01f;
		o[i].tab = sinetable;
		o[i].ts = 8192;
		o[i].tm = 8191;
	}

	// initialize amps and PMIs
	for (i = 0; i<4; i++)
	{
		amp[i].dst = 1.0f;
		amp[i].val = 0.0f;
		amp[i].val = ((float) i) / (amp[i].dst * 4.0f);
		amp[i].inc = amp[i].dst / env_len;
		amp[i].up = 1;

		pmi[i].dst = 10.0f;
		pmi[i].val = 0.0f;
		pmi[i].val = ((float) i) / (pmi[i].dst * 4.0f);
		pmi[i].inc = pmi[i].dst / env_len;
	}

	// initialize init frequencies
	for(int32_t i = 0; i < 8; i++) { f[i] = noise(); }

	led_init();
	switch_init();
	timer3_init();
	adc1_init();

	// output device, volume, sample rate
	BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 80, 48000);

	// initialize globals
	time = 240;
	button_time[0] = button_time[1] = button_time[2] = button_time[3] = button_time[4] = 0;
	millisecs = 0.0f;
	ledselected = 0;
	bspAudioState = 0;

	// set up first buffer (in bytes?!?!)
	BSP_AUDIO_OUT_Play((uint16_t *)codecBuffer, 128);

	// Infinite loop
	while (1)
	{
		if(bspAudioState == 1)
		{
			bspAudioState = 0;
			audioBlock(inBuffer, outBuffer, 16);
			for(i = 0; i < 32; i+=2)
			{
				codecBuffer[i+0] = (int16_t)((outBuffer[i]) * 32767.0f);
				codecBuffer[i+1] = (int16_t)((outBuffer[i+1]) * 32767.0f);
			}

		}
		else if(bspAudioState == 2)
		{
			bspAudioState = 0;

			audioBlock(inBuffer, outBuffer, 16);
			for(i = 0; i < 32; i+=2)
			{
				codecBuffer[i+32] = (int16_t)((outBuffer[i]) * 32767.0f);
				codecBuffer[i+33] = (int16_t)((outBuffer[i+1]) * 32767.0f);
			}
			// set up next buffer (in samples)
		}
	}
  // Infinite loop, never return.
}

// callback from DMA interrupt function HAL_DMA_IRQHandler()
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  time = (adcBuffer[0] * adcBuffer[0]) >> 12;

  avgtotal[0] -= adc0average[avgindex];
  adc0average[avgindex] = adcBuffer[0];
  avgtotal[0] += adc0average[avgindex];
  smoothadc[0] = avgtotal[0] >> 6;

  avgtotal[1] -= adc1average[avgindex];
  adc1average[avgindex] = adcBuffer[1];
  avgtotal[1] += adc1average[avgindex];
  smoothadc[1] = avgtotal[1] >> 6;
  avgindex++;
  if(avgindex > 63) avgindex = 0;
}

float phase_mod(int32_t car, int32_t mod, int32_t osc0, int32_t osc1, float interp)
{
	float phaseMod, output;
	int32_t iphase;
	int32_t cpi = (car * 8191) / 72;
	int32_t mpi = (mod * 8191) / 72;
	o[osc0].pi = sineinc[cpi];
	o[osc1].pi = sineinc[mpi];
	o[osc0].phs += o[osc0].pi;
	o[osc1].phs += o[osc1].pi;
	iphase = (int32_t)(o[osc1].phs * 8192.0f);
	phaseMod = o[osc0].phs + o[osc1].tab[iphase & o[osc1].tm];
	phaseMod *= pmi[osc1 / 2].val;
	iphase = (int32_t)(phaseMod * 8192.0f);
	output = o[osc0].tab[iphase & o[osc0].tm];
	return output;
}

float sat3(float sample)
{
	if(sample > 1.0f)
		sample = 1.0f;
	if(sample < -1.0f)
		sample = -1.0f;

	sample = sample * 1.5f - sample * sample * sample * 0.5f;
	return(sample);
}

void lineInc(void)
{
	for(int32_t i = 0; i < 4; i++)
	{
		amp[i].val += amp[i].inc * 2;
		if(amp[i].val > amp[i].dst) amp[i].val -= amp[i].dst;


		pmi[i].val += pmi[i].inc;
		if(pmi[i].val >= pmi[i].dst) pmi[i].val -= pmi[i].dst;
	}
}

void audioBlock(float *input, float *output, int32_t samples)
{
	float knob0 = (float)(smoothadc[0]) / 3000.0f;
	// phase modulation
	for(int32_t i = 0; i < samples; i++)
	{
		float out0, out1, out2, out3, l, r;
		out0 = phase_mod(12 + f[0], 12 + f[1], 0, 1, 1.0) * amp[0].val;
		out1 = phase_mod(12 + f[2], 12 + f[3], 2, 3, 1.0) * amp[1].val;
		out2 = phase_mod(12 + f[4], 12 + f[5], 4, 5, 1.0) * amp[2].val;
		out3 = phase_mod(12 + f[6], 12 + f[7], 6, 7, 1.0) * amp[3].val;

		l = (out0*0.1f) + (out1*0.1f) + (out2*0.1f);
		r = (out1*0.1f) + (out2*0.1f) + (out3*0.1f);

		output[i<<1]     = sat3(l) * knob0;
		output[(i<<1)+1] = sat3(r) * knob0;
		lineInc();
	}

	for(int32_t v = 0; v < 8; v++)
	{
		while(o[v].phs >= 1.0f) o[v].phs -= 1.0f;
		while(o[v].phs < 0.0f)  o[v].phs += 1.0f;
	}
}

// interrupt from switch - pin A0, D0
void EXTI0_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }

// interrupt from switch - pin D1
void EXTI1_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1); }

// interrupt from switch - pin D2
void EXTI2_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2); }

// interrupt from switch - pin D3
void EXTI3_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3); }

// callback from switch interrupt function HAL_GPIO_EXTI_IRQHandler()
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
// debouncing
	int32_t time_elapsed;
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	{
		time_elapsed = HAL_GetTick() - button_time[0];
		if(time_elapsed > 1)
		{
			// register new button down
			button_time[0] = HAL_GetTick();
		}
	}
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0))
	{
		time_elapsed = HAL_GetTick() - button_time[1];
		if(time_elapsed > 5)
		{
			// register new button down
			button_time[1] = HAL_GetTick();
			f[0] = noise();
			f[1] = noise();
		}
	}
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1))
	{
		time_elapsed = HAL_GetTick() - button_time[2];
		if(time_elapsed > 5)
		{
			// register new button down
			button_time[2] = HAL_GetTick();
			f[2] = noise();
			f[3] = noise();
		}
	}
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2))
	{
		time_elapsed = HAL_GetTick() - button_time[3];
		if(time_elapsed > 5)
		{
			// register new button down
			button_time[3] = HAL_GetTick();
			f[4] = noise();
			f[5] = noise();
		}
	}
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3))
	{
		time_elapsed = HAL_GetTick() - button_time[4];
		if(time_elapsed > 5)
		{
			// register new button down
			button_time[4] = HAL_GetTick();
			f[6] = noise();
			f[7] = noise();
		}
	}
}

// interrupt from timer 3
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}

// callback from timer interrupt function HAL_TIM_IRQHandler()
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	millisecs += 0.1f;
	if(millisecs > time)
	{
		millisecs = 0.0f;
		switch(ledselected)
		{
		case 0:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		}
		ledselected += 1;
		ledselected &= 3;
	}
}

// interrupt from DMA 2 - Stream 0 (ADC1)
void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(AdcHandle.DMA_Handle);
}

// interrupt from DMA 1 - Stream 5 (DAC Ch 1)
void DMA1_Stream5_IRQHandler(void) { HAL_DMA_IRQHandler(DacHandle.DMA_Handle1); }

// callback from DMA interrupt function HAL_DMA_IRQHandler()
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* DacHandle)
{
	int i,j;

	audioBlock(inBuffer, outBuffer, 16);
	for(i = 0, j = 0; i < 16; i++, j+=2)
	{
		dacBuffer[i] = (int16_t)((outBuffer[j] + outBuffer[j+1] + 2.0f) * 1023.0f);
	}
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* DacHandle)
{
	int i;

	audioBlock(inBuffer, outBuffer, 16);
	for(i = 0; i < 16; i++)
	{
		dacBuffer[i+16] = (int16_t)((outBuffer[(i * 2)] + outBuffer[(i * 2)+1] + 2.0f) * 1023.0f);
	}
}

/**
  * @brief  This function handles main I2S interrupt.
  * @param  None
  * @retval 0 if correct communication, else wrong communication
  */
void I2S3_IRQHandler(void) { HAL_DMA_IRQHandler(hAudioOutI2s.hdmatx); }

/**
  * @brief  This function handles DMA Stream interrupt request.
  * @param  None
  * @retval None
  */
void I2S2_IRQHandler(void) { HAL_DMA_IRQHandler(hAudioInI2s.hdmarx); }

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) { bspAudioState = 1; }

/**
* @brief  Calculates the remaining file size and new position of the pointer.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
	int i;
	bspAudioState = 2;
	BSP_AUDIO_OUT_ChangeBuffer((uint16_t *)codecBuffer, 64);
}

void BSP_AUDIO_OUT_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  while (1) {}
}

/**
* @brief  This function handles DMA interrupt request.
* @param  None
* @retval None
*/
void DACx_DMA_IRQHandler2(void) { HAL_DMA_IRQHandler(DacHandle.DMA_Handle2); }

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
