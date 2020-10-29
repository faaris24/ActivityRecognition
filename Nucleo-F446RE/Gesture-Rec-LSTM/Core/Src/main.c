/* USER CODE BEGIN Header */
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iks01a2_motion_sensors.h"
#include "math.h"
#include <string.h>
#include <stdio.h>
#include "ai_datatypes_defines.h"
#include "ai_platform.h"
#include "gesture_model.h"
#include "gesture_model_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct displayFloatToInt_s {
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t  out_int;
  uint32_t  out_dec;
} displayFloatToInt_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUF_SIZE 256
#define NUM_CAPTURED_SAMPLES_PER_GESTURE 120
#define NUM_FEATURES_PER_SAMPLE 6
#define TOTAL_SAMPLES NUM_CAPTURED_SAMPLES_PER_GESTURE * NUM_FEATURES_PER_SAMPLE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static char dataOut[MAX_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec);
static void Accelero_Sensor_Handler(uint32_t Instance,int *x,int *y,int *z);
static void Gyro_Sensor_Handler(uint32_t Instance,int *x,int *y,int *z);
void predictGesture(int samples,float data[]);
void captureData(int samples);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	ai_error ai_err;
	ai_i32 nbatch;
	float y_val,y_val2,y_val3;

	//holds intermedate values for neural network?
	AI_ALIGNED(4) ai_u8 activations[AI_GESTURE_MODEL_DATA_ACTIVATIONS_SIZE];
	//buffers used to store input and output tensors
	AI_ALIGNED(4) ai_i8 in_data[AI_GESTURE_MODEL_IN_1_SIZE_BYTES];
	AI_ALIGNED(4) ai_i8 out_data[AI_GESTURE_MODEL_OUT_1_SIZE_BYTES];

	// Pointer to our model
	ai_handle gesture_model = AI_HANDLE_NULL;

	// Initialize wrapper structs that hold pointers to data and info about the
	// data (tensor height, width, channels)
	ai_buffer ai_input[AI_GESTURE_MODEL_IN_NUM] = AI_GESTURE_MODEL_IN;
	ai_buffer ai_output[AI_GESTURE_MODEL_OUT_NUM] = AI_GESTURE_MODEL_OUT;

	// Set working memory and get weights/biases from model
	ai_network_params ai_params = {
	      AI_GESTURE_MODEL_DATA_WEIGHTS(ai_gesture_model_data_weights_get()),
	      AI_GESTURE_MODEL_DATA_ACTIVATIONS(activations)
	    };

	 // Set pointers wrapper structs to our data buffers
	ai_input[0].n_batches = 1;
	ai_input[0].data = AI_HANDLE_PTR(in_data);
	ai_output[0].n_batches = 1;
	ai_output[0].data = AI_HANDLE_PTR(out_data);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  snprintf(dataOut, MAX_BUF_SIZE,"\r\n\r\nSTM32 Gesture Recognition\r\n");
  HAL_UART_Transmit(&huart2,dataOut,strlen((char*)dataOut),HAL_MAX_DELAY);
  ai_err = ai_gesture_model_create(&gesture_model, AI_GESTURE_MODEL_DATA_CONFIG);
   if (ai_err.type != AI_ERROR_NONE)
   {
     snprintf(dataOut,MAX_BUF_SIZE, "Error: could not create NN instance\r\n");
     HAL_UART_Transmit(&huart2,dataOut,strlen((char*)dataOut),HAL_MAX_DELAY);
     while(1);
   }
   // Initialize neural network
   if (!ai_gesture_model_init(gesture_model, &ai_params))
   {
     snprintf(dataOut,MAX_BUF_SIZE, "Error: could not initialize NN\r\n");
     HAL_UART_Transmit(&huart2,dataOut,strlen((char*)dataOut),HAL_MAX_DELAY);
     while(1);
   }
  while(HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin)){} //while button not pressed stay here
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  float data[TOTAL_SAMPLES];

	  	  predictGesture(TOTAL_SAMPLES,data);
	  	  // Fill input buffer (use test value)
	  	  for (uint32_t i = 0; i < AI_GESTURE_MODEL_IN_1_SIZE; i++)
	  	  {
	  	   ((ai_float *)in_data)[i] = (ai_float)data[i];
	  	  }
	  	  // Perform inference
	  	  nbatch = ai_gesture_model_run(gesture_model, &ai_input[0], &ai_output[0]);
	  	  if (nbatch != 1) {
	  	    snprintf(dataOut,MAX_BUF_SIZE, "Error: could not run inference\r\n");
	  	    HAL_UART_Transmit(&huart2,dataOut,strlen((char*)dataOut),HAL_MAX_DELAY);
	  	  }
	  	  y_val = ((float *)out_data)[0];
	  	  y_val2 = ((float *)out_data)[1];
	  	  y_val3 = ((float *)out_data)[2];
	  	  snprintf(dataOut,MAX_BUF_SIZE,"Prediction:\r\n %f,\r\n%f\r\n%f,\r\n\n",y_val,y_val2,y_val3);
	  	  HAL_UART_Transmit(&huart2,dataOut,strlen((char*)dataOut),HAL_MAX_DELAY);
	  	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void motion_softmax(float *x,float *y, float *z) {
	float norm;
	/*snprintf(dataOut, MAX_BUF_SIZE, "Data before norm is: %f, %f, %f\n\r",
		             *x, *y,*z);
	HAL_UART_Transmit(&huart2,dataOut,strlen((char*)dataOut),HAL_MAX_DELAY); */

	norm = sqrt((  (*x) * (*x)  ) + ((*y) * (*y)) + ((*z) * (*z)));
	*x = (*x / norm);
	*y = (*y / norm);
	*z = (*z / norm);
	/*snprintf(dataOut, MAX_BUF_SIZE, "Data after norm is: %f, %f, %f\n\r",
		             *x, *y,*z);
	HAL_UART_Transmit(&huart2,dataOut,strlen((char*)dataOut),HAL_MAX_DELAY); */

}
/* captures data, saves to array, prints to termianl
 * param: #of samples to capture pass in global variable TOTAL_SAMPLES
 * return: none
 */
void captureData(int samples){
	int aX,aY,aZ;
	int gX,gY,gZ;
	int capturedSamples = 0;
	float dataArray[samples];
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
	while(capturedSamples < samples){
		Accelero_Sensor_Handler(0, &aX, &aY, &aZ);
		Gyro_Sensor_Handler(0, &gX, &gY, &gZ);

		dataArray[capturedSamples + 0] = aX;
		dataArray[capturedSamples + 1] = aY;
		dataArray[capturedSamples + 2] = aZ;
		dataArray[capturedSamples + 3] = gX;
		dataArray[capturedSamples + 4] = gY;
		dataArray[capturedSamples + 5] = gZ;

		capturedSamples += NUM_FEATURES_PER_SAMPLE;
		HAL_Delay(16.7);
	}
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);

	//print data array to terminal
	for(int i = 0; i < samples; i+= NUM_FEATURES_PER_SAMPLE){
		//normalize data
		motion_softmax(&dataArray[i+0],&dataArray[i+1],&dataArray[i+2]);
		motion_softmax(&dataArray[i+3],&dataArray[i+4],&dataArray[i+5]);

	    snprintf(dataOut, MAX_BUF_SIZE, "%f, %f, %f, %f, %f, %f\n\r",
	             dataArray[i + 0], dataArray[i + 1],dataArray[i + 2], dataArray[i + 3],
				 dataArray[i + 4], dataArray[i + 5]);
	    HAL_UART_Transmit(&huart2,dataOut,strlen((char*)dataOut),HAL_MAX_DELAY);
	}

	//skip a line
	snprintf(dataOut, MAX_BUF_SIZE, "\n\n\r");
	HAL_UART_Transmit(&huart2,dataOut,strlen((char*)dataOut),HAL_MAX_DELAY);

}

void predictGesture(int samples,float data[]){
	int aX,aY,aZ;
	int gX,gY,gZ;
	int capturedSamples = 0;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
	while(capturedSamples < samples){
		Accelero_Sensor_Handler(0, &aX, &aY, &aZ);
		Gyro_Sensor_Handler(0, &gX, &gY, &gZ);

		data[capturedSamples + 0] = aX;
		data[capturedSamples + 1] = aY;
		data[capturedSamples + 2] = aZ;
		data[capturedSamples + 3] = gX;
		data[capturedSamples + 4] = gY;
		data[capturedSamples + 5] = gZ;

		capturedSamples += NUM_FEATURES_PER_SAMPLE;
		HAL_Delay(16.7);
	}
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);

	for(int i = 0; i < samples; i+= NUM_FEATURES_PER_SAMPLE){
		//normalize data
		motion_softmax(&data[i+0],&data[i+1],&data[i+2]);
		motion_softmax(&data[i+3],&data[i+4],&data[i+5]);
	}

}


static void Accelero_Sensor_Handler(uint32_t Instance,int *x, int *y, int *z)
{

  IKS01A2_MOTION_SENSOR_Axes_t acceleration;

  if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nACC[%d]: Error\r\n", (int)Instance);
  }
/*  else
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nACC_X[%d]: %d, ACC_Y[%d]: %d, ACC_Z[%d]: %d\r\n", (int)Instance,
             (int)acceleration.x, (int)Instance, (int)acceleration.y, (int)Instance, (int)acceleration.z);
  } */
  	*x = (int)acceleration.x;
  	*y = (int)acceleration.y;
  	*z = (int)acceleration.z;
    //HAL_UART_Transmit(&huart2,dataOut,strlen((char*)dataOut),HAL_MAX_DELAY);

}
static void Gyro_Sensor_Handler(uint32_t Instance,int *x,int *y,int *z)
{
  IKS01A2_MOTION_SENSOR_Axes_t angular_velocity;
  if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nGYR[%d]: Error\r\n", (int)Instance);
  }
 /* else
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nGYR_X[%d]: %d, GYR_Y[%d]: %d, GYR_Z[%d]: %d\r\n", (int)Instance,
             (int)angular_velocity.x, (int)Instance, (int)angular_velocity.y, (int)Instance, (int)angular_velocity.z);
  } */
  *x = (int)angular_velocity.x;
  *y = (int)angular_velocity.y;
  *z = (int)angular_velocity.z;

  //HAL_UART_Transmit(&huart2,dataOut,strlen((char*)dataOut),HAL_MAX_DELAY);

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
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
