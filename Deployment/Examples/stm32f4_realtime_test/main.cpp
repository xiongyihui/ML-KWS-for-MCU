/*
 * Copyright (C) 2018 Arm Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description: Example code for running keyword spotting on Cortex-M boards
 */

// How to use STM32F4xx ADC - https://visualgdb.com/tutorials/arm/stm32/adc/
// Timer 3 (16000 fs) -> ADC -> DMA

#include "kws.h"
#include "wav_data.h"

#define AUDIO_BLOCK_SIZE    (1 * FRAME_LEN)
#define ADC_BUFFER_LENGTH   (AUDIO_BLOCK_SIZE * 2)

uint32_t audio_input_buffer[ADC_BUFFER_LENGTH]; // 2 for ping-pong buffer
int16_t audio_buffer[16000] = WAVE_DATA;
// int16_t audio_buffer[AUDIO_BLOCK_SIZE];

volatile uint32_t sample_count = 0;
q7_t scratch_buffer[SCRATCH_BUFFER_SIZE];
char output_class[12][8] = {"Silence", "Unknown","yes","no","up","down","left","right","on","off","stop","go"};

Serial pc(USBTX, USBRX);
Timer T;

ADC_HandleTypeDef g_AdcHandle;
DMA_HandleTypeDef g_DmaHandle;


void ConfigureTIM(void)
{
    __TIM3_CLK_ENABLE();

    TIM_HandleTypeDef s_TimerInstance = { 
        .Instance = TIM3
    };

    s_TimerInstance.Init.Prescaler = 0;
    s_TimerInstance.Init.CounterMode = TIM_COUNTERMODE_UP;

    // 16 KHz, from 84 MHz TIM2CLK (TIM2CLK = HCLK/2)
    s_TimerInstance.Init.Period = (84000000 / 16000) - 1;
    s_TimerInstance.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_TimerInstance.Init.RepetitionCounter = 0;

    TIM_MasterConfigTypeDef sMasterConfig;
    // sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&s_TimerInstance, &sMasterConfig);

    HAL_TIM_Base_Init(&s_TimerInstance);
    HAL_TIM_Base_Start(&s_TimerInstance);
}

void ConfigureADC()
{
    GPIO_InitTypeDef gpioInit;

    __GPIOC_CLK_ENABLE();
    __ADC1_CLK_ENABLE();

    gpioInit.Pin = GPIO_PIN_0;
    gpioInit.Mode = GPIO_MODE_ANALOG;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpioInit);

    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);

    ADC_ChannelConfTypeDef adcChannel;

    g_AdcHandle.Instance = ADC1;

    g_AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
    g_AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
    g_AdcHandle.Init.ScanConvMode = DISABLE;
    g_AdcHandle.Init.ContinuousConvMode = DISABLE;
    g_AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    g_AdcHandle.Init.NbrOfDiscConversion = 0;
    g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    g_AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
    g_AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_AdcHandle.Init.NbrOfConversion = 1;
    g_AdcHandle.Init.DMAContinuousRequests = ENABLE;
    g_AdcHandle.Init.EOCSelection = DISABLE;

    HAL_ADC_Init(&g_AdcHandle);

    adcChannel.Channel = ADC_CHANNEL_10;
    adcChannel.Rank = 1;
    adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcChannel.Offset = 0;

    if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel) != HAL_OK)
    {
        asm("bkpt 255");
    }
}

void ConfigureDMA()
{
    __DMA2_CLK_ENABLE();
    g_DmaHandle.Instance = DMA2_Stream4;

    g_DmaHandle.Init.Channel = DMA_CHANNEL_0;
    g_DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    g_DmaHandle.Init.MemInc = DMA_MINC_ENABLE;
    g_DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    g_DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    g_DmaHandle.Init.Mode = DMA_CIRCULAR;
    g_DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;
    g_DmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    g_DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    g_DmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
    g_DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;

    HAL_DMA_Init(&g_DmaHandle);

    __HAL_LINKDMA(&g_AdcHandle, DMA_Handle, g_DmaHandle);

    HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
}

extern "C" {

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
    // copy the new recording data
    for (int i = 0; i < AUDIO_BLOCK_SIZE; i++)
    {
        audio_buffer[i] = audio_input_buffer[AUDIO_BLOCK_SIZE + i] - 1024 - 335;
    }
    sample_count++;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
    // copy the new recording data
    for (int i = 0; i < AUDIO_BLOCK_SIZE; i++)
    {
        audio_buffer[i] = audio_input_buffer[i] - 1024 - 335;
    }
    sample_count++;
}

void DMA2_Stream4_IRQHandler()
{
    HAL_DMA_IRQHandler(&g_DmaHandle);
}

void ADC_IRQHandler()
{
    HAL_ADC_IRQHandler(&g_AdcHandle);
}
}

int main()
{
    KWS kws(audio_buffer,scratch_buffer);

    pc.baud(576000);
    printf("---- KWS ----\r\n");

    kws.extract_features();

    ConfigureADC();
    ConfigureDMA();
    HAL_ADC_Start_DMA(&g_AdcHandle, audio_input_buffer, ADC_BUFFER_LENGTH);

    ConfigureTIM();

    uint32_t detect_count = 0;
    uint32_t start;
    uint32_t end;
    uint32_t last = 0;

    T.start();
    while (1) {
        if (detect_count < sample_count) {
            detect_count++;

            // int32_t sum = 0;
            // int16_t max = 0;
            // int16_t min = 0;
            // for (int i=0; i<AUDIO_BLOCK_SIZE; i++) {
            //     sum += audio_buffer[i];

            //     if (audio_buffer[i] > max) {
            //         max = audio_buffer[i];
            //     } else if (audio_buffer[i] < min) {
            //         min = audio_buffer[i];
            //     }
            // }


            start = T.read_us();

            //Averaging window for smoothing out the output predictions
            int averaging_window_len = 3;  //i.e. average over 6 inferences or 240ms
            int detection_threshold = 80;  //in percent

            kws.extract_features(1); //extract mfcc features

            kws.classify();	  //classify using dnn

            kws.average_predictions(averaging_window_len);
            
            int max_ind = kws.get_top_detection(kws.averaged_output);


            end = T.read_us();

            // if ((max_ind) != 0 && (kws.averaged_output[max_ind] >= detection_threshold*128/100)) 
            {
                printf("Detected %s (%d%%)\r\n",output_class[max_ind],((int)kws.averaged_output[max_ind]*100/128));
            }

            
            // if ((detect_count & 0x1F) == 0) 
            // {
            //     printf("sum: %d, avg: %d, max: %d, min: %d\r\n", sum, sum / AUDIO_BLOCK_SIZE, max, min);
            //     printf("processing time: %d us, period: %d @ %d, %d\r\n", end - start, end - last, sample_count, detect_count);
            // }
    

            last = end;
        } else {
            // TODO: sleep
        }

    }

    T.stop();


    return 0;
}
