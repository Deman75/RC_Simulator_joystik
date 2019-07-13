/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "LCD_baomei_bm-8001b.h"
#include "tim.h"
#include "adc.h"
#include "dma.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SWITCH_2 HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port, SWITCH_2_Pin) == GPIO_PIN_RESET

#define SW1_UP   HAL_GPIO_ReadPin(SWITCH_1_UP_GPIO_Port, SWITCH_1_UP_Pin)
#define SW1_DOWN HAL_GPIO_ReadPin(SWITCH_1_DWN_GPIO_Port, SWITCH_1_DWN_Pin)

#define PWM TIM1->CCR1
#define ARR TIM1->ARR

#define BTN(x,y) (x>(y-100))&&(x<(y+100))

#define RUD_R 1390
#define RUD_L 2048

#define THR_UP 0
#define THR_DOWN 700

#define BTN_TOP 2740

#define ELV_UP 0
#define ELV_DOWN 700

#define ELR_L 2090
#define ELR_R 1390


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t TrainerActive = 0;

int8_t EncoderTmp = 0, Encoder = 0;

//----------------ADC--------------------//
uint16_t ADC_Data[7];
float alpha = 0.2;
uint16_t i = 0;
extern uint16_t channel[9];

struct channel_t {
	float inMin;
	float inCent;
	float inMax;
	float outMin;
	float outCent;
	float outMax;
	float koefUp;
	float koefDown;
	float koefUpOut;
	float koefDownOut;
	float exponent;
	float rate;
	float output;
	uint8_t inverse;
};

struct channel_t ch1, ch2, ch3, ch4;

struct button_t {
	uint8_t down;
	uint8_t up;
	uint16_t downTime;
	uint16_t upTime;
	uint8_t read;
};

struct button_t
	trimCh3 = {0,0,0,1,0}, 
	trimCh4L = {0,0,0,1,0},
	trimCh4R = {0,0,0,1,0},
	trimCh1L = {0,0,0,1,0},
	trimCh1R = {0,0,0,1,0},
	trimCh2U = {0,0,0,1,0},
	trimCh2D = {0,0,0,1,0},
	btnL    = {0,0,0,1,0},
	btnR    = {0,0,0,1,0};

uint16_t BtnLeft, BtnRight;
uint8_t switch1 = 0;
uint16_t ADC_Calibrate_0 = 0, ADC_Calibrate_1 = 0, ADC_Calibrate_2 = 0, ADC_Calibrate_3 = 0;
//----------------ADC--------------------//
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId MenyTaskHandle;
osThreadId BtnTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void adcInit(void);
void adcWork(void);
void encoderWork(void);
void init(struct channel_t *ch);
float calc(float input, struct channel_t *ch);
void button(void);



/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartMenuTask(void const * argument);
void StartBtnTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of MenyTask */
  osThreadDef(MenyTask, StartMenuTask, osPriorityNormal, 0, 128);
  MenyTaskHandle = osThreadCreate(osThread(MenyTask), NULL);

  /* definition and creation of BtnTask */
  osThreadDef(BtnTask, StartBtnTask, osPriorityNormal, 0, 128);
  BtnTaskHandle = osThreadCreate(osThread(BtnTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		if (TrainerActive) {
			LCD_Update();
			
			if ((SW1_UP == GPIO_PIN_SET) && (SW1_DOWN == GPIO_PIN_SET)) {
				switch1 = 1;
				ch1.rate = 0.8;
				ch2.rate = 0.8;
				ch4.rate = 0.8;
			} else if ((SW1_UP == GPIO_PIN_RESET) && (SW1_DOWN == GPIO_PIN_SET)) {
				switch1 = 0;
				ch1.rate = 1;
				ch2.rate = 1;
				ch4.rate = 1;
			} else if ((SW1_UP == GPIO_PIN_SET) && (SW1_DOWN == GPIO_PIN_RESET)) {
				switch1 = 2;
				ch1.rate = 0.6;
				ch2.rate = 0.6;
				ch4.rate = 0.6;
			}
		}
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMenuTask */
/**
* @brief Function implementing the MenyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMenuTask */
void StartMenuTask(void const * argument)
{
  /* USER CODE BEGIN StartMenuTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMenuTask */
}

/* USER CODE BEGIN Header_StartBtnTask */
/**
* @brief Function implementing the BtnTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBtnTask */
void StartBtnTask(void const * argument)
{
  /* USER CODE BEGIN StartBtnTask */
  /* Infinite loop */
  for(;;)
  {
		
		if (SWITCH_2) {
			LCD_On();
			if (!TrainerActive) {
				TrainerActive = 1;
				HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
				TIM2->CNT = 20;
				
				while(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK); // калибровка АЦП
				
				HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &ADC_Data, 7);
				
				adcInit();
				
				ch3.exponent = (50.0/100.0)*1.0;
				
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
				HAL_TIM_Base_Start_IT(&htim1);
				PWM = 400;
				ARR = 1500; // ARR 110 - 1.11mS, 150 - 1.51mS, 192 - 1.931mS
			}
					
			encoderWork();
			
			adcWork();
	
			
			BtnLeft = ADC_Data[4];
			BtnRight = ADC_Data[5];
			
			button();
			
			LcdNumber((uint8_t) Encoder);
			//LCD_Update();
		} else {
			LCD_Off();
			HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
			HAL_ADC_Stop_DMA(&hadc1);
			TrainerActive = 0;
			
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_Base_Stop_IT(&htim1);
		}
		
		
		
    osDelay(1);
  }
  /* USER CODE END StartBtnTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void init(struct channel_t *ch) {
	ch->inMin = 0.0;
	ch->inCent = 2048.0;
	ch->inMax = 4085.0;
	
	ch->outMin = 1108.0;
	ch->outCent = 1520.0;
	ch->outMax = 1933.0;
	
	ch->inverse = 0;
	ch->rate = 1;
	
	
	ch->koefUp = ch->inMax-ch->inCent;
  ch->koefDown = ch->inCent - ch->inMin;
  ch->koefUpOut = ch->outMax - ch->outCent;
  ch->koefDownOut = ch->outCent - ch->outMin;
	ch->exponent = (50.0/100.0)*40.0;
}

void trim(struct channel_t *ch, int8_t value) {
	ch->outCent += value;
	
	ch->koefUp = ch->inMax-ch->inCent;
  ch->koefDown = ch->inCent - ch->inMin;
  ch->koefUpOut = ch->outMax - ch->outCent;
  ch->koefDownOut = ch->outCent - ch->outMin;
}

float calc(float input, struct channel_t *ch) {
	
	if (ch->inverse) {
		input = ch->inMax - input;
	}
	
	float tmp;

  if (input >= ch->inCent) {
    tmp = ((float)input - ch->inCent) / ch->koefUp;
    tmp = (powf(ch->exponent,tmp)-1.0)/(ch->exponent-1.0);
    tmp = (tmp * (ch->koefUpOut * ch->rate)) + ch->outCent;
  } else {
    tmp = (ch->inCent - (float)input) / ch->koefDown;
    tmp = (powf(ch->exponent,tmp)-1.0)/(ch->exponent-1.0);
    tmp = ch->outCent - (tmp * (ch->koefDownOut * ch-> rate));
  }

  return roundf(tmp);
}

void adcInit() {
	i = 0;
	while (i < 100) {
		i++;
		ADC_Calibrate_0 = (alpha * ADC_Data[0]) + ((1 - alpha) * ADC_Calibrate_0);
		ADC_Calibrate_1 = (alpha * ADC_Data[1]) + ((1 - alpha) * ADC_Calibrate_1);
		ADC_Calibrate_2 = (alpha * ADC_Data[2]) + ((1 - alpha) * ADC_Calibrate_2);
		ADC_Calibrate_3 = (alpha * ADC_Data[3]) + ((1 - alpha) * ADC_Calibrate_3);
		
		HAL_Delay(10);
	}
	ch1.inCent = ADC_Calibrate_0;
	ch2.inCent = ADC_Calibrate_1;
	ch3.inCent = ADC_Calibrate_2;
	ch4.inCent = ADC_Calibrate_3;
	
	init(&ch1);
	init(&ch2);
	init(&ch3);
	init(&ch4);
}

void adcWork() {
	ADC_Calibrate_0 = (alpha * ADC_Data[0]) + ((1 - alpha) * ADC_Calibrate_0);
	ADC_Calibrate_1 = (alpha * ADC_Data[1]) + ((1 - alpha) * ADC_Calibrate_1);
	ADC_Calibrate_2 = (alpha * ADC_Data[2]) + ((1 - alpha) * ADC_Calibrate_2);
	ADC_Calibrate_3 = (alpha * ADC_Data[3]) + ((1 - alpha) * ADC_Calibrate_3);
	
	ch1.output = calc((float)ADC_Calibrate_0, &ch1);
	ch2.output = calc((float)ADC_Calibrate_1, &ch2);
	ch3.output = calc((float)ADC_Calibrate_2, &ch3);
	ch4.output = calc((float)ADC_Calibrate_3, &ch4);
	
	channel[1] = (uint16_t)ch1.output;
	channel[0] = (uint16_t)ch2.output;
	channel[2] = (uint16_t)ch3.output;
	channel[3] = (uint16_t)ch4.output;
}

void encoderWork () {
	if (TIM2->CNT % 4 == 0) {
		if (TIM2->CNT > 20) {
			TIM2->CNT = 20;
			if (Encoder > 0)
				Encoder--;
		} else if (TIM2->CNT < 20){
			TIM2->CNT = 20;
			if (Encoder < 119)
				Encoder++;
		}
	}
}

void buttonMachine(struct button_t *trimCh, struct channel_t *ch, uint16_t btn, uint16_t btnConst, int8_t value) {
	if (BTN(btn,btnConst)) {
		if (trimCh->down == 0) {
			trimCh->down = 1;
			trimCh->downTime = 0;
			trimCh->read = 0;
		}
		if (trimCh->downTime < 200)
			trimCh->downTime++;
		else {
			trimCh->downTime = 0;
			trimCh->read = 0;
		}
	} else {
		if (trimCh->down == 1 && trimCh->upTime > 10) {
			trimCh->down = 0;
			trimCh->downTime = 0;
			trimCh->read = 0;
		}
		if (trimCh->upTime <100) trimCh->upTime++;
	}
	
	if (trimCh->down && trimCh->read == 0) {
		trim(ch, value);
		trimCh->read = 1;
	}
}

void button() {
	
	buttonMachine(&trimCh4L, &ch4, BtnLeft, RUD_L, -4);
	buttonMachine(&trimCh4R, &ch4, BtnLeft, RUD_R, 4);
	
	buttonMachine(&trimCh1L, &ch1, BtnRight, ELR_L, -4);
	buttonMachine(&trimCh1R, &ch1, BtnRight, ELR_R, 4);
	
	buttonMachine(&trimCh2U, &ch2, BtnRight, ELV_UP, -4);
	buttonMachine(&trimCh2D, &ch2, BtnRight, ELV_DOWN, 4);
	

}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
