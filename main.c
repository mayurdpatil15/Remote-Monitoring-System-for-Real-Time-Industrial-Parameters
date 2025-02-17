/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "DHT11.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define A 116.6020682  // Calibration constant for CO2
#define B 2.211        // Calibration constant for CO2
#define R_L 10.0       // Load resistor value in kOhms
#define R0 76.63       // Pre-calibrated value of R0 in clean air

#define fr 0.0023841859331

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


//DHT11 = PC.11
//YF-S201 = PD.0
//MQ-135 = PA.0 (ADC)

//HX710B Pressure sensors pins
//SCLK = PC.6 (OUT)
//DOUT = PC.7 (Input)



//**********************************DHT11 Variables **********************************************************>
DHT_DataTypedef DHT11_Data;
float Temperature, Humidity;

//***********************************Flow Sensor Variable ******************************************************>
volatile int flow_frequency;
volatile unsigned long totalImpulses = 0;


////*************************************Pressure Sensor variable **********************
//char buffer[50];  // Buffer to store transmitted data
//long pressureValue = 0; // Variable to store pressure data from HX710B


//**********************************************USART Variable ******************************************************>



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//function to calculate Temperature and Humidity using dht11 sensor
void DHT11_function(){
	DHT_GetData(&DHT11_Data);
	Temperature = DHT11_Data.Temperature;
	Humidity = DHT11_Data.Humidity;
}

//function to calculate pwm from flow sensor
void flow () // Interrupt function to increment flow
{
   flow_frequency++;
   totalImpulses++;
}

//External interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	HAL_UART_Transmit(&huart2, (uint8_t *)"Interrupt\r\n", sizeof("Interrupt\r\n"), HAL_MAX_DELAY);
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0)) {
			flow();
		}
}

float calculate_flowrate(){

	//required variables;
	float l_minute;
	//float vol;
//	float xy = 0.0;
//	unsigned long currentTime;
//	unsigned long cloopTime = 0;
//	float imp = 0;
//	float total_ml;
//	float ML;


//	currentTime = HAL_GetTick();			//get current time
//	if (currentTime >= (cloopTime + 1000)) {
//
//			cloopTime = currentTime; // Updates cloopTime
//			if (flow_frequency != 0) {
				HAL_Delay(1000);
				l_minute = (flow_frequency / 7.5); // (Pulse frequency x 60 min) / 7.5 Q = flowrate in L/hour
//
//				  float ml_per_impulse = 0.95;
//			      total_ml = totalImpulses * ml_per_impulse;
//				 l_minute = l_minute / 60;
//			        l_minute = l_minute * 1000;
//			        vol = vol + (l_minute / 7.58);
//			    vol = vol + l_minute;
//			    xy = vol * 1000;
//			    xy /= 200;

//			    ML = totalImpulses * 0.48 ;
			    //sprintf((char *)disp.s_line, "p: %2.2f, %2.2f ml", vol, total_ml);
//			    sprintf(buffer, "Total impulse = %lu, flow rate = %.3f\n\r", totalImpulses, l_minute);
//				HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

			    flow_frequency = 0; // Reset Counter
//			}
//		}

	return l_minute;
}

//mq 135 gas sensor functions ----------->
uint32_t get_sensor_value(){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return adcValue;
}

float adc_to_ppm() {
	uint32_t adcValue = get_sensor_value();
	float Rs = 0, ratio = 0, concentration = 0;
	if (adcValue){
		Rs = R_L * ((4095.0 - adcValue) / adcValue);
		ratio = Rs / R0;
		concentration = A*pow(ratio, -B);
	}

	return concentration;
}


//HX710B Pressure sensor functions ----------->
long readHX710B(void)
{
  long result = 0;

  // Wait until DOUT is low, indicating data is ready
  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET);  // DOUT is high when not ready

  // Read 24 bits of data
  for (int i = 0; i < 26; i++)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);  // SCK high
    result = (result << 1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);  // Shift in bit from DOUT
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);  // SCK low
  }

  // Apply one more clock pulse to complete the data transfer
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_Delay(1);  // Short delay for pulse timing
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_Delay(1);

  if (result & 0x800000)
      {
          result |= 0xFF000000;  // Correct sign extension
      }

  return result;  // Return the 24-bit pressure value
}

//function to convert raw pressure sensor value into kpa
long convertToPa(long adcValue) {

        //return (adcValue * 40000) / 16777215;
		return  adcValue * fr;
}


////uart interrupt function
//void HAL_UART_IRQHandler(UART_HandleTypeDef *huart){
//	rcv_ack = 1;
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	char buffer[128];
	uint8_t rcv = '2';
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  currentTime = HAL_GetTick();

	  DHT11_function();              //get the value of dht11 sensor (Temperature and Humidity);
	  float flow_rate = calculate_flowrate();			// get the value of ys-s201 sensor (flow rate)
	  float concentration = adc_to_ppm();
	  long pressureRawValue = readHX710B();  // Read raw pressure data from HX710B
	  long pressureInPa = convertToPa(pressureRawValue);  // Convert raw ADC value to pressure in pascals


	  //field1 = humidty, field2 = temperature, field3 = flow rate, field4 = mq135 value,  field5 = pressure
	  sprintf(buffer, "&field1=%.2f&field2=%.2f&field3=%.2f&field4=%.2f&field5=%ld\r\n", Humidity,Temperature,flow_rate,concentration, pressureInPa);
	  HAL_UART_Transmit(&huart2, (uint8_t *)buffer,  strlen(buffer), HAL_MAX_DELAY);

	  //wait for acknowledgment
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
//	  while(1){
////		  HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
//		  HAL_UART_Receive(&huart2, &rcv, sizeof(rcv), HAL_MAX_DELAY);
//		  if(rcv == '1')
//			  break;
//	  }
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
//
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
//	  rcv = '2';


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
