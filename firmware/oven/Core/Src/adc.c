/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <math.h>
#include "Utils/utils.h"
#include "tim.h"
static int flag = 0;
static uint32_t adc_buffer_0[50] = {0};
static uint32_t adc_buffer_1[50] = {0};
static int samples_ready = 0;
static int idx = 0;
#define RUNNING_MEAN 64
static float exts[RUNNING_MEAN];
static float voltages[RUNNING_MEAN];
static float currents[RUNNING_MEAN];
static float temps[RUNNING_MEAN];

static float v_ref = 0;
static float raw_ext = 0;
static float raw_t = 0;
static float raw_v = 0;
static float raw_i = 0;

static uint32_t change_tick = 0;
static int changed = 0;
static float current_ext = 20;
static float current_v = 240;
static float current_i = 0;
static float current_t = 20;
//float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
//float c1 = 1.753159443e-03, c2 = 0.5683506275e-04, c3 = 18.16478028e-07;
float c1 = 0.8269846691e-03;
float c2 = 2.088175627e-04;
float c3 = 0.8055701799e-07;

void request_report(); //Request upper system to report change in voltages

void adc_start_dma()
{
    if(flag)
    {
        flag = 0;
        HAL_ADC_Start_DMA(&hadc1, adc_buffer_1, 100);  // Wait for 20 * 5 channels, 100 ADC conversions, 200 B
    }
    else
    {
        flag = 1;
        HAL_ADC_Start_DMA(&hadc1, adc_buffer_0, 100);
    }
}

void adc_get_raw(float *v, float *i, float *t)
{
    *v = raw_v;
    *i = raw_i;
    *t = raw_t;
}

void adc_get_values(float *ext, float *v, float *i, float *t)
{
//    *v = current_v;
//    *i = current_i;
//    *t = current_t;

    *ext = (raw_ext * 100) - 273;
    *v = raw_v * 640.5;
    *i = raw_i * 46;

    float ntc = (100e3 * raw_t) / (v_ref - raw_t);
    float logR2 = log(ntc);
    *t = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2)) - 273.15;

}

void adc_show()
{
    printf("Vref: %0.3f\n", v_ref);
    printf("E: %-.3f V ", raw_ext);
    printf("V: %-.3f V ", raw_v);
    printf("A: %-.3f V ", raw_i);
    printf("T: %-.3f V\n", raw_t);

    float ext, v, i, t;
    adc_get_values(&ext, &v, &i, &t);

    printf("E: %-.3f C ", ext);
    printf("V: %-.3f V ", v);
    printf("A: %-.3f A ", i);
    printf("T: %-.3f C\n", t);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(samples_ready)
        samples_ready = -1;
    else
        samples_ready = HAL_GetTick();


//    HAL_ADC_Stop_DMA(&hadc1);
//    HAL_TIM_Base_Stop(&htim3);
    adc_start_dma();
}

void adc_run()
{
    if(samples_ready < 0)
    {
        printf("ADC Overflow\n");
        samples_ready = 0;
        return;
    }

    if(samples_ready)
    {
//         HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
        // printf("DMA done %d\n", samples_ready);
        uint8_t *ptr = (uint8_t*)adc_buffer_1;
        if(flag == 0)
        {
            ptr = (uint8_t*)adc_buffer_0;
        }

//        //Print all voltages for debugging
//        for(int k = 0; k < 200; )
//        {
//            uint16_t v = ptr[k] | (ptr[k + 1] << 8);
//            printf("%0.3f ", (float)v * 805.664e-6);
//            k +=2;
//
//            if(!(k % 10))
//                    printf("\n");
//        }

//         diag_dump_buf(ptr, 16);

        uint16_t v_max = 0x00;
        uint16_t v_min = 0xFFFF;

        uint16_t i_max = 0x00;
        uint16_t i_min = 0xFFFF;
        // Calcultate Step using internal Vref
        uint16_t mean_v_int = 0;
        for(int k = 0; k < 200; k += 10)
        {
            uint16_t v = ptr[k + 8] | (ptr[k + 9] << 8);
            if(v < v_min)
                v_min = v;
            if(v > v_max)
                v_max = v;

            v = ptr[k + 4] | (ptr[k + 5] << 8);
            if(v < i_min)
                i_min = v;
            if(v > i_max)
                i_max = v;

            mean_v_int += ptr[k + 2] | (ptr[k + 3] << 8);
        }
        mean_v_int /= 20;
        //printf("VINT: 0x%04X\n", mean_v_ref);

        uint16_t dc = (v_min + v_max) >> 1;
        uint16_t i_dc = (i_min + i_max) >> 1;


        float adc_step = 1.2 / (float)mean_v_int;
        v_ref = 4096 * adc_step;
        float v_offset = (float)dc * adc_step;
        float i_offset = (float)i_dc * adc_step;
        float mean_ext = 0;
        float mean_v = 0;
        float mean_t = 0;
        float mean_i = 0;

         for(int k = 0; k < 200; k += 10)
         {
             uint16_t t = ptr[k + 8] | (ptr[k + 9] << 8);
             // printf("0x%04X ", t);
             float volt = ((float)t * adc_step) - v_offset;
             // mean_vin += volt;
             mean_v += (volt * volt);

             t = ptr[k] | (ptr[k + 1] << 8);
             // printf("0x%04X ", t);
             mean_ext += ((float)t * adc_step);

             // printf("%0.3f - ", volt);
             t = ptr[k + 4] | (ptr[k + 5] << 8);
             // printf("0x%04X ", t);
             volt = ((float)t * adc_step) - i_offset;
             // mean_current += volt;
             mean_i += (volt * volt);

             // printf("%0.3f - ", volt);
             t = ptr[k + 6] | (ptr[k + 7] << 8);
             // printf("0x%04X ", t);
             volt = ((float)t * adc_step);
             // printf("%0.3f\n", volt);
             mean_t += volt;
         }

         mean_ext /= 20.0;
         mean_t /= 20.0;
         mean_v /= 20.0;
         mean_i /= 20.0;

         mean_i = sqrtf(mean_i);
         mean_v = sqrtf(mean_v);

         //Calculate a running mean of the values
         exts[idx] = mean_ext;
         voltages[idx] = mean_v;
         currents[idx] = mean_i;
         temps[idx] = mean_t;
         idx = (idx + 1) % RUNNING_MEAN;

         float ext = 0;
         float v = 0;
         float i = 0;
         float t = 0;

         for(int k = 0; k < RUNNING_MEAN; k++)
         {
             ext += exts[k];
             v += voltages[k];
             i += currents[k];
             t += temps[k];
         }

         ext /= RUNNING_MEAN;
         v /= RUNNING_MEAN;
         i /= RUNNING_MEAN;
         t /= RUNNING_MEAN;

         //filter noise from current measurement
         if(i < 0.008)
             i = 0;

         //Offset temperature
         ext += 0.085;
         t -= 0.012;

         raw_ext = ext;
         raw_v = v;
         raw_i = i;
         raw_t = t;

         //Get scaled values
         adc_get_values(&ext, &v, &i, &t);

//          HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);

         if(((current_v - 5) > v) || (v > (current_v + 5)))
         {
             changed = 1;
         }
         if(((current_i - 0.2) > i) || (i > (current_i + 0.2)))
         {
             changed = 1;
         }
         if(((current_t - 3) > t) || (t > (current_t + 3)))
         {
             changed = 1;
         }
         if(((current_ext - 3) > ext) || (ext > (current_ext + 3)))
         {
             changed = 1;
         }

         if(changed)
         {
             changed = 0;
             change_tick = HAL_GetTick() + 1000;

             current_ext = ext;
             current_v = v;
             current_i = i;
             current_t = t;

             printf("E: %-.3f ", ext);
             printf("V: %-.3f ", v);
             printf("A: %-.3f ", i);
             printf("T: %-.3f\n", t);
         }

        samples_ready = 0;
    }

    // Wait until change has stabilized before reporting
    if(change_tick && (change_tick < HAL_GetTick()))
    {
        change_tick = 0;
        request_report();
    }
}
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
    for(int k = 0; k < RUNNING_MEAN; k++)
    {
        voltages[k] = 0.3747;
        currents[k] = 0;
        temps[k] = 2.9;
    }
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 5;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
