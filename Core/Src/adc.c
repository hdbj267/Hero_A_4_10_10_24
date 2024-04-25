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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

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
    PA6     ------> ADC1_IN6
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
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
    PA6     ------> ADC1_IN6
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

    /* ADC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
  * @brief          
  * @author         
  * @param[in]      
  * @retval			返回�?
  * @note   		adc1 通道0
  */
uint16_t get_adc1_value(void)
{
	ADC_ChannelConfTypeDef adc_channelconf;
	adc_channelconf.Channel = ADC_CHANNEL_6;//通道8
	adc_channelconf.Rank = 1;//�?1个序列，序列1
	adc_channelconf.SamplingTime = ADC_SAMPLETIME_56CYCLES;//56个周�? 22.4mhz�?
	adc_channelconf.Offset = 0;//无偏移量
	HAL_ADC_ConfigChannel(&hadc1, &adc_channelconf);
	HAL_ADC_Start(&hadc1);                   //�?启ADC转换
	HAL_ADC_PollForConversion(&hadc1,10);    //等待转换结束
	return (uint16_t)HAL_ADC_GetValue(&hadc1);	//返回�?近一次ADC1规则组的转换结果
}

/*******************************************************************************
* �? �? �?         : Get_ADC_Value
* 函数功能		   : 获取按键
* �?    �?         : 	 
* �?    �?         : 
*******************************************************************************/
uint16_t range = 400;
uint16_t key_release_val = 4096;
uint16_t key_up_val = 2450;
uint16_t key_down_val = 3250;
uint16_t key_left_val = 860;
uint16_t key_right_val = 1650;
uint16_t key_center_val = 0;
uint8_t key_scanf(void)
{
	uint16_t temp_val=get_adc1_value();
	//中间0-5
	if (temp_val <= key_center_val+range)
	{
		return OLED_key_center;
	}
	//按上�?2400-2500
	else if (temp_val >= key_up_val-range && temp_val<= key_up_val+range)
	{
		return OLED_key_up;
	}
	//按下�?3200-3300
	else if (temp_val >= key_down_val-100 && temp_val <= key_down_val+100)
	{
		return OLED_key_down;
	}
	//按左�?840-870
	else if (temp_val >= key_left_val-range && temp_val <= key_left_val+range)
	{
		return OLED_key_left;
	}
	//按右�?1600-1700
	else if (temp_val >=key_right_val-range && temp_val <= key_right_val+range)
	{
		return OLED_key_right;
	}
	else
	{
		return OLED_key_release;
	}
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
