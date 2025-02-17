


#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#define TYPE_DHT11



#define DHT_PORT GPIOC
#define DHT_PIN GPIO_PIN_11





uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM; uint8_t Presence = 0;

#include "DHT11.h"

uint32_t DWT_Delay_Init(void)
{

  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;

  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;


  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;

  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;


  DWT->CYCCNT = 0;


     __ASM volatile ("NOP");
     __ASM volatile ("NOP");
  __ASM volatile ("NOP");


     if(DWT->CYCCNT)
     {
       return 0;
     }
     else
  {
    return 1;
  }
}

__STATIC_INLINE void delay(volatile uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;


  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);


  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


void DHT_Start (void)
{
	DWT_Delay_Init();
	Set_Pin_Output (DHT_PORT, DHT_PIN);
	HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, 0);

#if defined(TYPE_DHT11)
	delay (18000);
#endif

#if defined(TYPE_DHT22)
	delay (1200);
#endif

    HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, 1);   // pull the pin high
    delay (20);   // wait for 30us
	Set_Pin_Input(DHT_PORT, DHT_PIN);    // set as input
}

uint8_t DHT_Check_Response (void)
{
	uint8_t Response = 0;
	delay (40);
	if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)))
	{
		delay (80);
		if ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN))) Response = 1;
		else Response = -1;
	}
	while ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));   // wait for the pin to go low

	return Response;
}

uint8_t DHT_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));   // wait for the pin to go high
		delay (40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));  // wait for the pin to go low
	}
	return i;
}



void DHT_GetData (DHT_DataTypedef *DHT_Data)
{
    DHT_Start ();
	Presence = DHT_Check_Response ();
	Rh_byte1 = DHT_Read ();
	Rh_byte2 = DHT_Read ();
	Temp_byte1 = DHT_Read ();
	Temp_byte2 = DHT_Read ();
	SUM = DHT_Read();

	if (SUM == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))
	{
		#if defined(TYPE_DHT11)
			DHT_Data->Temperature = Temp_byte1;
			DHT_Data->Humidity = Rh_byte1;
		#endif

		#if defined(TYPE_DHT22)
			DHT_Data->Temperature = ((Temp_byte1<<8)|Temp_byte2);
			DHT_Data->Humidity = ((Rh_byte1<<8)|Rh_byte2);
		#endif
	}
}

