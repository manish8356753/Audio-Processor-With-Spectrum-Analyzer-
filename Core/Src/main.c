/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#define ARM_MATH_CM4
#include "main.h"
#include "cmsis_os.h"
#include "ILI9341.h"
#include "arm_math.h"

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2S2_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);

typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;

ADC_HandleTypeDef PotMeterADC1;
DMA_HandleTypeDef DMA_PotMeterADC1;

I2S_HandleTypeDef audioCom_i2s2;
DMA_HandleTypeDef DMA_audioCom_i2s2_rx;
DMA_HandleTypeDef DMA_audioCom_i2s2_tx;

SPI_HandleTypeDef displayCom_spi1;

TIM_HandleTypeDef PotMeter_tim1;
TIM_HandleTypeDef buttonDebounce_tim2;
TIM_HandleTypeDef filter_tim7;

// Definitions for ApplyFilterTxTa
osThreadId_t ApplyFilterTxTaHandle;
uint32_t ApplyFilterTxBuffer[ 128 ];
osStaticThreadDef_t ApplyFilterTxTaskControlBlock;
const osThreadAttr_t ApplyFilterTxTa_attributes = {
  .name = "ApplyFilterTxTa",
  .stack_mem = &ApplyFilterTxBuffer[0],
  .stack_size = sizeof(ApplyFilterTxBuffer),
  .cb_mem = &ApplyFilterTxTaskControlBlock,
  .cb_size = sizeof(ApplyFilterTxTaskControlBlock),
  .priority = (osPriority_t) osPriorityHigh,
};
// Definitions for LCDdisplayTask
osThreadId_t LCDdisplayTaskHandle;
uint32_t LCDdisplayTaskBuffer[ 128 ];
osStaticThreadDef_t LCDdisplayTaskControlBlock;
const osThreadAttr_t LCDdisplayTask_attributes = {
  .name = "LCDdisplayTask",
  .stack_mem = &LCDdisplayTaskBuffer[0],
  .stack_size = sizeof(LCDdisplayTaskBuffer),
  .cb_mem = &LCDdisplayTaskControlBlock,
  .cb_size = sizeof(LCDdisplayTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
// Definitions for DoFFTtask
osThreadId_t DoFFTtaskHandle;
uint32_t DoFFTtaskBuffer[ 128 ];
osStaticThreadDef_t DoFFTtaskControlBlock;
const osThreadAttr_t DoFFTtask_attributes = {
  .name = "DoFFTtask",
  .stack_mem = &DoFFTtaskBuffer[0],
  .stack_size = sizeof(DoFFTtaskBuffer),
  .cb_mem = &DoFFTtaskControlBlock,
  .cb_size = sizeof(DoFFTtaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
// Definitions for updateCutOffTas
osThreadId_t updateCutOffTasHandle;
uint32_t updateCutOffTasBuffer[ 128 ];
osStaticThreadDef_t updateCutOffTasControlBlock;
const osThreadAttr_t updateCutOffTas_attributes = {
  .name = "updateCutOffTas",
  .stack_mem = &updateCutOffTasBuffer[0],
  .stack_size = sizeof(updateCutOffTasBuffer),
  .cb_mem = &updateCutOffTasControlBlock,
  .cb_size = sizeof(updateCutOffTasControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
// Definitions for CalcBiquadTask
osThreadId_t CalcBiquadTaskHandle;
uint32_t CalcBiquadTaskBuffer[ 128 ];
osStaticThreadDef_t CalcBiquadTaskControlBlock;
const osThreadAttr_t CalcBiquadTask_attributes = {
  .name = "CalcBiquadTask",
  .stack_mem = &CalcBiquadTaskBuffer[0],
  .stack_size = sizeof(CalcBiquadTaskBuffer),
  .cb_mem = &CalcBiquadTaskControlBlock,
  .cb_size = sizeof(CalcBiquadTaskControlBlock),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
// Definitions for Qdisplay
osMessageQueueId_t QdisplayHandle;
uint8_t QdisplayBuffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t QdisplayControlBlock;
const osMessageQueueAttr_t Qdisplay_attributes = {
  .name = "Qdisplay",
  .cb_mem = &QdisplayControlBlock,
  .cb_size = sizeof(QdisplayControlBlock),
  .mq_mem = &QdisplayBuffer,
  .mq_size = sizeof(QdisplayBuffer)
};
// Definitions for SemFilterControl
osSemaphoreId_t SemFilterControlHandle;
osStaticSemaphoreDef_t SemFilterControlControlBlock;
const osSemaphoreAttr_t SemFilterControl_attributes = {
  .name = "SemFilterControl",
  .cb_mem = &SemFilterControlControlBlock,
  .cb_size = sizeof(SemFilterControlControlBlock),
};
// Definitions for SemUpdateBiquad
osSemaphoreId_t SemUpdateBiquadHandle;
osStaticSemaphoreDef_t SemUpdateBiquadControlBlock;
const osSemaphoreAttr_t SemUpdateBiquad_attributes = {
  .name = "SemUpdateBiquad",
  .cb_mem = &SemUpdateBiquadControlBlock,
  .cb_size = sizeof(SemUpdateBiquadControlBlock),
};
// Definitions for SemFcUpdate
osSemaphoreId_t SemFcUpdateHandle;
osStaticSemaphoreDef_t SemFcUpdateControlBlock;
const osSemaphoreAttr_t SemFcUpdate_attributes = {
  .name = "SemFcUpdate",
  .cb_mem = &SemFcUpdateControlBlock,
  .cb_size = sizeof(SemFcUpdateControlBlock),
};

arm_biquad_casd_df1_inst_f32 iirsettings_l, iirsettings_r;
arm_rfft_fast_instance_f32 fft_handler;

//4 delayed samples per biquad
float iir_l_state [4];
float iir_r_state [4];

uint16_t Fcutoff = 0;

float iir_coeffs [5];

uint16_t rxBuf[BLOCK_SIZE_U16];
uint16_t txBuf[BLOCK_SIZE_U16];
float l_buf_in [BLOCK_SIZE_FLOAT];
float r_buf_in [BLOCK_SIZE_FLOAT];
float l_buf_out [BLOCK_SIZE_FLOAT];
float r_buf_out [BLOCK_SIZE_FLOAT];

float fft_in_buf[BLOCK_SIZE_FLOAT];
float fft_out_buf[BLOCK_SIZE_FLOAT];

uint16_t potMeterRxBuf[50];
uint16_t soundLevel_in_db;
uint8_t filter_type = NO_FILTER;
uint16_t freqPoint[FREQ_SLOTS] = {2,4,11,22,44,87,131,175,218};
uint8_t button_busy = FALSE;

float fcAvgInput;
float Prev_fcAvgInput = 0;

int main(void)
{
	// Reset of all peripherals, Initializes the Flash interface and the Systick.
	HAL_Init();

	// Configure the system clock
	SystemClock_Config();

	// Initialize all configured peripherals
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_I2S2_Init();
	MX_TIM7_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM1_Init();

	//Initialize LCD hardware and UI)
	LCDdisplay_Init();

	//Initialize IIR structure
	arm_biquad_cascade_df1_init_f32 ( &iirsettings_l, 1, &iir_coeffs[0], &iir_l_state[0]);
	arm_biquad_cascade_df1_init_f32 ( &iirsettings_r, 1, &iir_coeffs[0], &iir_r_state[0]);

	HAL_I2SEx_TransmitReceive_DMA (&audioCom_i2s2, txBuf, rxBuf, BLOCK_SIZE_U16 / 2);
	HAL_TIM_Base_Start_IT(&PotMeter_tim1);

	arm_rfft_fast_init_f32(&fft_handler, BLOCK_SIZE_FLOAT / 2);

	osKernelInitialize();

	// Create the semaphores(s)
	SemFilterControlHandle = osSemaphoreNew(1, 1, &SemFilterControl_attributes);
	SemUpdateBiquadHandle = osSemaphoreNew(1, 1, &SemUpdateBiquad_attributes);
	SemFcUpdateHandle = osSemaphoreNew(1, 1, &SemFcUpdate_attributes);

	// Create the queue(s)
	QdisplayHandle = osMessageQueueNew (9, sizeof(uint16_t), &Qdisplay_attributes);

	// Create the thread(s)
	ApplyFilterTxTaHandle = osThreadNew(StartApplyFilterTxTask, NULL, &ApplyFilterTxTa_attributes);
	LCDdisplayTaskHandle = osThreadNew(StartLCDdisplayTask, NULL, &LCDdisplayTask_attributes);
	DoFFTtaskHandle = osThreadNew(StartDoFFTtask, NULL, &DoFFTtask_attributes);
	updateCutOffTasHandle = osThreadNew(StartupdateCutOffTask, NULL, &updateCutOffTas_attributes);
	CalcBiquadTaskHandle = osThreadNew(StartCalcBiquadTask, NULL, &CalcBiquadTask_attributes);

	osKernelStart();

	while (1)
	{

	}

}

void StartApplyFilterTxTask(void *argument)
{
	uint32_t w_ptr = 0;
	for(;;)
	{
		//This semaphore will be released from tim7 time elapsed callback
		osSemaphoreAcquire (SemFilterControlHandle, HAL_MAX_DELAY);
		taskENTER_CRITICAL();
		w_ptr = 0;
		//restore input sample buffer to float array
		for (uint32_t i=0; i<BLOCK_SIZE_U16; i=i+4) {
			l_buf_in[w_ptr] = (float) ((int) (rxBuf[i]<<16)|rxBuf[i+1]);
			r_buf_in[w_ptr] = (float) ((int) (rxBuf[i+2]<<16)|rxBuf[i+3]);
			w_ptr++;
		}

		w_ptr = 0;

		if(filter_type != NO_FILTER){
			//process IIR
			arm_biquad_cascade_df1_f32 (&iirsettings_l, &l_buf_in[0], &l_buf_out[0],BLOCK_SIZE_FLOAT);
			arm_biquad_cascade_df1_f32 (&iirsettings_r, &r_buf_in[0], &r_buf_out[0],BLOCK_SIZE_FLOAT);

			//restore processed float-array to output sample-buffer
			for (uint32_t i=0; i<BLOCK_SIZE_U16; i=i+4) {
				txBuf[i] =  (((int)l_buf_out[w_ptr])>>16)&0xFFFF;
				txBuf[i+1] = ((int)l_buf_out[w_ptr])&0xFFFF;
				txBuf[i+2] = (((int)r_buf_out[w_ptr])>>16)&0xFFFF;
				txBuf[i+3] = ((int)r_buf_out[w_ptr])&0xFFFF;
				w_ptr++;
			}
		}
		//no filtering applied (loop back audio)
		else{
			for (uint32_t i=0; i<BLOCK_SIZE_U16; i=i+4) {
				txBuf[i] =  (((int)l_buf_in[w_ptr])>>16)&0xFFFF;
				txBuf[i+1] = ((int)l_buf_in[w_ptr])&0xFFFF;
				txBuf[i+2] = (((int)r_buf_in[w_ptr])>>16)&0xFFFF;
				txBuf[i+3] = ((int)r_buf_in[w_ptr])&0xFFFF;
				w_ptr++;
			}
		}

		//This task will be put on blocked until tim7 time elapsed callback
		HAL_TIM_Base_Start_IT(&filter_tim7);
		taskEXIT_CRITICAL();
	}
}

void StartLCDdisplayTask(void *argument)
{
	for(;;)
	{
		// x y values of each frequency slots
		const uint16_t x_initial[FREQ_SLOTS] = {5,40,75,110,145,180,215,250,285};
		const uint16_t x_final[FREQ_SLOTS] = {37,72,107,142,177,212,247,282,317};
		static uint8_t y_initial[FREQ_SLOTS] = {INIT_Y_VAL};
		const uint8_t y_final = 212;

		uint16_t display_cmd;
		uint8_t new_y_initial;
		char fc_arr[5];

		for(int32_t i=0; i<FREQ_SLOTS; i++){
			osMessageQueueGet (QdisplayHandle, &display_cmd, NULL, HAL_MAX_DELAY);

			//Display level for a frequency slots
			if((display_cmd > MIN_LEVEL) && (display_cmd < MAX_LEVEL) || (display_cmd == 0)){
				if(y_initial[i] != INIT_Y_VAL){
					new_y_initial = y_final - display_cmd;
					if( y_initial[i] > new_y_initial){
						ILI9341_Fill_Rect_BottomUP(x_initial[i], new_y_initial, x_final[i], y_initial[i], COLOR_GREEN );
						y_initial[i] = new_y_initial;
					}
					else{
						ILI9341_Fill_Rect(x_initial[i], y_initial[i], x_final[i] - 1, new_y_initial, COLOR_BLACK );
						y_initial[i] = new_y_initial;
					}
				}
				else{
					y_initial[i] = y_final;
					ILI9341_Fill_Rect_BottomUP(x_initial[i], y_initial[i], x_final[i], y_final, COLOR_GREEN );
				}
				continue;
			}

			//Display frequency
			else if(display_cmd > MAX_LEVEL){
				if(filter_type != NO_FILTER){
					display_cmd /= 10;
					sprintf(fc_arr,"%d ",display_cmd);
					ILI9341_printText(fc_arr, 233, 13, COLOR_WHITE, COLOR_BLACK, 2);
				}
				i--;
				continue;
			}

			//Display current filter type
			else{
				if(display_cmd == NO_FILTER + 1){
					ILI9341_printText("NO FILTER", 7, 13, COLOR_WHITE, COLOR_BLACK, 2);
					ILI9341_printText("xxxx", 233, 13, COLOR_WHITE, COLOR_BLACK, 2);
				}
				else if(display_cmd == LPF_FILTER + 1){
					ILI9341_printText("LOWPASS  ", 7, 13, COLOR_WHITE, COLOR_BLACK, 2);
					sprintf(fc_arr,"%d ",Fcutoff);
					ILI9341_printText(fc_arr, 233, 13, COLOR_WHITE, COLOR_BLACK, 2);
				}
				else if(display_cmd == HPF_FILTER + 1){
					ILI9341_printText("HIGHPASS ", 7, 13, COLOR_WHITE, COLOR_BLACK, 2);
					sprintf(fc_arr,"%d ",Fcutoff);
					ILI9341_printText(fc_arr, 233, 13, COLOR_WHITE, COLOR_BLACK, 2);
				}

				i--;
				continue;

			}
		}
	}
}

void StartDoFFTtask(void *argument)
{
  for(;;)
  {
	  uint32_t fft_in_ptr = 0;
	  int32_t temp_soundLevel_in_db;

	  //make mono signal from tx buffer and store into fft_in_buf
	  for (uint32_t i=0; i<BLOCK_SIZE_U16; i=i+4) {
		  fft_in_buf[fft_in_ptr] = (float) ((int32_t) (txBuf[i]<<16)|txBuf[i+1]);
		  fft_in_buf[fft_in_ptr] += (float) ((int32_t) (txBuf[i+2]<<16)|txBuf[i+3]);

		  fft_in_ptr++;
	  }

	  //process fft
	  arm_rfft_fast_f32(&fft_handler, &fft_in_buf,&fft_out_buf,0);

	  //from fft_out_buf, pick frequency levels required then convert to db then send to display queue
	  for(uint8_t i=0; i<FREQ_SLOTS; i++){
		  temp_soundLevel_in_db = (int32_t)(20*log10f(complexABS(fft_out_buf[freqPoint[i]], fft_out_buf[freqPoint[i]+1]))) - OFFSET;
		  soundLevel_in_db = ((temp_soundLevel_in_db <= MIN_LEVEL) || (temp_soundLevel_in_db > MAX_LEVEL)) ? 0 : (uint16_t)temp_soundLevel_in_db;
		  osMessageQueuePut (QdisplayHandle, &soundLevel_in_db, 1, HAL_MAX_DELAY);
	  }
  }
}

void StartupdateCutOffTask(void *argument)
{
  for(;;)
  {
	  uint32_t sum = 0;
	  uint16_t freqSlot;
	  uint16_t freq;

	  osSemaphoreAcquire (SemFcUpdateHandle, HAL_MAX_DELAY);

	  //take average of potentiometer input (to smooth out input fluctuation)
	  for(uint8_t i=0; i<50; i++){
	  		sum += potMeterRxBuf[i];
	  }
	  fcAvgInput = sum/50;

	  //detect if the potentiometer knob has turned
	  if(abs(Prev_fcAvgInput - fcAvgInput) >= 50){
		  freqSlot = (((fcAvgInput-10) / (MAX_POT- 10)) * POT_TURN_LEVELS);
		  freqSlot = fcAvgInput == MAX_POT ? freqSlot : freqSlot + 1;
		  freq = freqSlot * POT_LEVELS_FREQ;
		  vTaskSuspendAll();
		  Fcutoff = freq;
		  xTaskResumeAll();
		  freq *= 10;
		  Prev_fcAvgInput = fcAvgInput;
		  osSemaphoreRelease (SemUpdateBiquadHandle);
		  osMessageQueuePut (QdisplayHandle, &freq, 1, HAL_MAX_DELAY);
	  }


  }
}

void StartCalcBiquadTask(void *argument)
{
  for(;;)
  {
	  const uint16_t Fs = 48000;
	  float Q = 0.7071;
	  const uint16_t sendFilterType[3] = {NO_FILTER + 1,LPF_FILTER + 1,HPF_FILTER + 1};

	  osSemaphoreAcquire (SemUpdateBiquadHandle, HAL_MAX_DELAY);

	  float K = tan(M_PI * Fcutoff / Fs);

	  if(filter_type == LPF_FILTER){
		  float norm = 1 / (1 + K / Q + K * K);
		  iir_coeffs[0] = K * K * norm;
		  iir_coeffs[1] = 2 * iir_coeffs[0];
		  iir_coeffs[2] = iir_coeffs[0];
		  iir_coeffs[3] = (2 * (K * K - 1) * norm) * -1;
		  iir_coeffs[4] = ((1 - K / Q + K * K) * norm) * -1;
		  osMessageQueuePut (QdisplayHandle, &sendFilterType[1], 1, HAL_MAX_DELAY);
		  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
		  button_busy = FALSE;
	  }
	  else if(filter_type == HPF_FILTER){
		  float norm = 1 / (1 + K / Q + K * K);
		  iir_coeffs[0] = 1 * norm;
		  iir_coeffs[1] = -2 * iir_coeffs[0];
		  iir_coeffs[2] = iir_coeffs[0];
		  iir_coeffs[3] = (2 * (K * K - 1) * norm) * -1;
		  iir_coeffs[4] = ((1 - K / Q + K * K) * norm) * -1;
		  osMessageQueuePut (QdisplayHandle, &sendFilterType[2], 1, HAL_MAX_DELAY);
		  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
		  button_busy = FALSE;
	  }

	  else if(filter_type == NO_FILTER){
		  osMessageQueuePut (QdisplayHandle, &sendFilterType[0], 1, HAL_MAX_DELAY);
		  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
		  button_busy = FALSE;
	  }
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == GPIO_PIN_12 && button_busy == FALSE){
		HAL_TIM_Base_Start_IT(&buttonDebounce_tim2);
		button_busy = TRUE;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_ADC_Stop_DMA(&PotMeterADC1);
	osSemaphoreRelease(SemFcUpdateHandle);
	HAL_TIM_Base_Start_IT(&PotMeter_tim1);

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  else if (htim->Instance == TIM7) {
	HAL_TIM_Base_Stop_IT(&filter_tim7);
	osSemaphoreRelease (SemFilterControlHandle);
  }

  else if (htim->Instance == TIM2){

	//cycle through filter type
	filter_type = ((filter_type + 1) % 3);

	HAL_TIM_Base_Stop_IT(&buttonDebounce_tim2);
	osSemaphoreRelease (SemUpdateBiquadHandle);

  }
  else if (htim->Instance == TIM1){
	HAL_TIM_Base_Stop_IT(&PotMeter_tim1);
	HAL_ADC_Start_DMA(&PotMeterADC1, (uint32_t*)&potMeterRxBuf, 50);
  }
}


float complexABS(float real, float compl) {
	return sqrtf((real*real)+(compl*compl));
}

void LCDdisplay_Init(void){

	uint16_t verticalLine[10] = {1,36,71,106,141,176,211,246,281,316};

	/*Display hardware initialization*/
	ILI9341_Init(&displayCom_spi1, GPIOD, CSX_Pin, GPIOD, DCX_Pin, GPIOD, RST_Pin);

	/*Display user interface initialization*/
	ILI9341_setRotation(2);
	ILI9341_Fill(COLOR_BLACK);
	ILI9341_printText("Fc =      Hz", 178, 13, COLOR_WHITE, COLOR_BLACK, 2);

	/*horizontal lines*/
	ILI9341_drawLine(3, 40, 319, 40, COLOR_WHITE);
	ILI9341_drawLine(3, 41, 319, 41, COLOR_WHITE);
	ILI9341_drawLine(3, 42, 319, 42, COLOR_WHITE);
	ILI9341_drawLine(3, 215, 319, 215, COLOR_WHITE);
	ILI9341_drawLine(3, 214, 319, 214, COLOR_WHITE);
	ILI9341_drawLine(3, 213, 319, 213, COLOR_WHITE);

	/*vertical lines*/
	for(int i=0; i<10; i++){
		ILI9341_drawLine(verticalLine[i]+1, 40, verticalLine[i]+1, 215, COLOR_WHITE);
		ILI9341_drawLine(verticalLine[i]+2, 40, verticalLine[i]+2, 215, COLOR_WHITE);
		ILI9341_drawLine(verticalLine[i]+3, 40, verticalLine[i]+3, 215, COLOR_WHITE);
	}

	/*frequency slots*/
	ILI9341_printText("46", 15, 220, COLOR_WHITE, COLOR_BLACK, 1);
	ILI9341_printText("92", 50, 220, COLOR_WHITE, COLOR_BLACK, 1);
	ILI9341_printText("252", 82, 220, COLOR_WHITE, COLOR_BLACK, 1);
	ILI9341_printText("504", 117, 220, COLOR_WHITE, COLOR_BLACK, 1);
	ILI9341_printText("1k", 155, 220, COLOR_WHITE, COLOR_BLACK, 1);
	ILI9341_printText("2k", 190, 220, COLOR_WHITE, COLOR_BLACK, 1);
	ILI9341_printText("3k", 225, 220, COLOR_WHITE, COLOR_BLACK, 1);
	ILI9341_printText("4k", 260, 220, COLOR_WHITE, COLOR_BLACK, 1);
	ILI9341_printText("5k", 295, 220, COLOR_WHITE, COLOR_BLACK, 1);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 150;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 96;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  PotMeterADC1.Instance = ADC1;
  PotMeterADC1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  PotMeterADC1.Init.Resolution = ADC_RESOLUTION_12B;
  PotMeterADC1.Init.ScanConvMode = DISABLE;
  PotMeterADC1.Init.ContinuousConvMode = ENABLE;
  PotMeterADC1.Init.DiscontinuousConvMode = DISABLE;
  PotMeterADC1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  PotMeterADC1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  PotMeterADC1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  PotMeterADC1.Init.NbrOfConversion = 1;
  PotMeterADC1.Init.DMAContinuousRequests = ENABLE;
  PotMeterADC1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&PotMeterADC1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&PotMeterADC1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2S2_Init(void)
{
  audioCom_i2s2.Instance = SPI2;
  audioCom_i2s2.Init.Mode = I2S_MODE_MASTER_TX;
  audioCom_i2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  audioCom_i2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  audioCom_i2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  audioCom_i2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  audioCom_i2s2.Init.CPOL = I2S_CPOL_LOW;
  audioCom_i2s2.Init.ClockSource = I2S_CLOCK_PLL;
  audioCom_i2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&audioCom_i2s2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  displayCom_spi1.Instance = SPI1;
  displayCom_spi1.Init.Mode = SPI_MODE_MASTER;
  displayCom_spi1.Init.Direction = SPI_DIRECTION_2LINES;
  displayCom_spi1.Init.DataSize = SPI_DATASIZE_8BIT;
  displayCom_spi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  displayCom_spi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  displayCom_spi1.Init.NSS = SPI_NSS_SOFT;
  displayCom_spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  displayCom_spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  displayCom_spi1.Init.TIMode = SPI_TIMODE_DISABLE;
  displayCom_spi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  displayCom_spi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&displayCom_spi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  PotMeter_tim1.Instance = TIM1;
  PotMeter_tim1.Init.Prescaler = 10000;
  PotMeter_tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  PotMeter_tim1.Init.Period = 7500;
  PotMeter_tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  PotMeter_tim1.Init.RepetitionCounter = 0;
  PotMeter_tim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&PotMeter_tim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&PotMeter_tim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&PotMeter_tim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  buttonDebounce_tim2.Instance = TIM2;
  buttonDebounce_tim2.Init.Prescaler = 999;
  buttonDebounce_tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  buttonDebounce_tim2.Init.Period = 50000;
  buttonDebounce_tim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  buttonDebounce_tim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&buttonDebounce_tim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&buttonDebounce_tim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&buttonDebounce_tim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  filter_tim7.Instance = TIM7;
  filter_tim7.Init.Prescaler = 999;
  filter_tim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  filter_tim7.Init.Period = 1400;
  filter_tim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&filter_tim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&filter_tim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RST_Pin|DCX_Pin|CSX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : filterUpdate_button_Pin */
  GPIO_InitStruct.Pin = filterUpdate_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(filterUpdate_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_Pin DCX_Pin CSX_Pin */
  GPIO_InitStruct.Pin = RST_Pin|DCX_Pin|CSX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void Error_Handler(void)
{

}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
