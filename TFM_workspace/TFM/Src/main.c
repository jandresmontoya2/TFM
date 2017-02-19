/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "sx1276.h"
#include "radio.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define MAX_TX 3
RadioState_t rStatus;
const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

extern uint16_t BufferSize;					// XXX 4	BufferSize = BUFFER_SIZE = 4;
extern uint8_t Buffer[BUFFER_SIZE];
uint8_t BufferTx[BUFFER_SIZE];
uint16_t RcvMsg;
States_t State;

//#define IS_MASTER 1
#ifdef IS_MASTER
uint8_t isMaster = true;
#else
uint8_t isMaster = false;
#endif

#define PERIODO_TX 1000 // En milisegundos
#define VENTANA_COMPROBACION_RX 1000 // En milisegundos
#define PERIODO_ESCUCHA 5000000 // En microsegundas (us) alcanza este valor?

extern uint8_t txDoneFlag;
extern uint8_t rxDoneFlag;
RadioState_t rStatus;
uint8_t nt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void adc_read(void);
void temp_read(void);
void leds(void);
void d_out_conversion (uint8_t* dato);
void evaluar(void);
void reposo(void);
void transmitir(void);
void pedir_mediciones(void);
void recibir(void);

static int ambient_temp = 0;
uint8_t temp_tx[2];
static int conversion = 0;
uint8_t conversion_tx[2];
static int temp_max = 250;
static int volt_max = 35;
static uint8_t time_out_flag = 0;
static int enviar_umbral = 0;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM10_Init();

  /* USER CODE BEGIN 2 */
//  HAL_TIM_Base_Start_IT(&htim10);
  sx1276_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

#ifdef IS_MASTER
//	  HAL_Delay(PERIODO_TX);
	  pedir_mediciones();
	  evaluar();
//	  transmitir();
#else

	  recibir();
#endif

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void adc_read (void)  // ADC //
{
	uint8_t d_out[2] = {0};
	// BIT REGISTER OPTIONS:
	uint8_t in1 = 0b00000000; // the bits 3,4 and 5 are add3, add4 and add5
	uint8_t in2 = 0b00101000; // the bits 3,4 and 5 are add3, add4 and add5
	HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_SET); 	// to make sure that there is a low edge in CS
	HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_RESET); //PE3 // #1 enable CS pin to 0
	HAL_SPI_TransmitReceive(&hspi1, &in1, d_out, 2, 100);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);			// puts CS to 1 to end the conversion process
	HAL_Delay(50);
	d_out_conversion(d_out);
	//conversion = ((d_out[0] & 0x0F) <<4 ) + ((d_out[1] & 0xF0) >>4);
}

void d_out_conversion (uint8_t* dato)
{

	int data_out = ((dato[0] & 0x0F) <<4 ) + ((dato[1] & 0xF0) >>4) ;
	conversion = (data_out * 0.0117)*10;
	conversion_tx[1] = conversion & 0x00FF;
	conversion_tx[0] = (conversion >> 8) & 0x00FF;
}

void temp_read (void)
{
	uint8_t read_address = 0b10010001; 	// A0-A1-A2 -> "1"
	uint8_t write_address = 0b10010000; // A0-A1-A2 -> "1"
	uint8_t register_pointer[2], temp[2];
	register_pointer[0] = 0b00000000;	// register pointer itself to select the register to write on
	register_pointer[1] = 0b00000001;	// to write in the selected register // FOR T_ambient it is not needed
	temp[0] = 0b00000000;
	temp[1] = 0b11111111;
//activate A0-A1-A2 for slave address ---> X1001111
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
//	HAL_I2C_Master_Transmit(&hi2c1,write_address, register_pointer, 1, 50 );
	HAL_I2C_Master_Transmit(&hi2c1, write_address, &register_pointer[0], 1, 50 );
	HAL_I2C_Master_Receive(&hi2c1, read_address, temp, 2,150);

// Convert temp
	int tempInt = (temp[0] * 256 + (temp[1] & 0xF0)) / 16;  //  *256 is the same as  shifting <<8
	if(tempInt > 2047)										//  2^11=2048
	{
		tempInt -= 4096;									// if tempInt is >2047 it means that the 12 bit is negative
	}
	float celsius_temp = tempInt * 0.0625;
	ambient_temp = celsius_temp*10;
	temp_tx[1] = ambient_temp & 0x00FF;
	temp_tx[0] = (ambient_temp >> 8) & 0x00FF;

	HAL_Delay(100);
}
void reposo (void)
{
	HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);
}
void evaluar (void)
{
	if (ambient_temp > temp_max || conversion > volt_max)
	{
		transmitir();
	}
}
void leds(void)
{
//	HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
	HAL_Delay(250);
//	HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);
  	HAL_Delay(250);
}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	HAL_GPIO_TogglePin(GPIOD,LD4_Pin);
//	time_out_flag = 0;
//}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//  Interrupcion del SX1276 RxDone/TxDone
	 if (GPIO_Pin == GPIO_PIN_5)
	 {
		 SX1276OnDio0Irq();
	 }
	 // Interrupcion del boton
	 if (GPIO_Pin == GPIO_PIN_0)
	 {
#ifdef IS_MASTER
		 temp_max = 250;
		 volt_max = 35;
#else
		 if (enviar_umbral == 1)
		 {
			 enviar_umbral = 0;
		 }
		 else if (enviar_umbral == 0)
		 {
			enviar_umbral = 1;
		 }
#endif
	 }
}

void recibir(void)
{
	rStatus = SX1276GetStatus();

	// Nuevos umbrales (Temp 30 C, Volt 3 V)
	BufferTx[0] = 't';
	BufferTx[1] = 0x01;  // MSB de la temperatura (multiplicada por 10)
	BufferTx[2] = 0x5E;	 // LSB de la temperatura (multiplicada por 10)
	BufferTx[3] = 'v';
	BufferTx[4] = 0x00;	// MSB del voltaje medido por el ADC (*10)
	BufferTx[5] = 0x1E;	// LSB del voltaje medido por el ADC (*10)
	if ( rStatus == RF_IDLE )
	{
		if (rxDoneFlag)
		{
			rxDoneFlag = 0;
			// Comprobar que cumple el formato de mensaje "T--V--"
			if ( Buffer[0] == 'T' && Buffer[3] == 'V')
			{
				HAL_GPIO_WritePin(GPIOD, LD6_Pin, SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOD, LD6_Pin, RESET);
				// Printf valores
			}
			HAL_Delay(100);
//			SX1276SetRx( 0 );
			if (enviar_umbral == 1)
			{
				Radio.Send( BufferTx, BufferSize );
			}
		}
		else if (txDoneFlag)
		{
			txDoneFlag = 0;
			nt = 0;
			HAL_Delay(100);
//			SX1276SetRx( 0 );
//			SX1276SetRx( RX_TIMEOUT_VALUE );
		}
		else		// inicio del programa
		{
			SX1276SetRx( 0 );
//			Radio.Send( BufferTx, BufferSize );
		}
	}
	else if ( rStatus == RF_TX_RUNNING )
	{
		HAL_GPIO_WritePin(GPIOD, LD5_Pin, SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOD, LD5_Pin, RESET);
		nt++;
		if ( nt > MAX_TX )
		{
			nt = 0;
			txDoneFlag=1;
			SX1276SetStby();
		}
	}
	else if ( rStatus == RF_RX_RUNNING )
	{
		HAL_GPIO_WritePin(GPIOD, LD3_Pin, SET);
		HAL_Delay(VENTANA_COMPROBACION_RX);
		SX1276SetStby();
		HAL_GPIO_WritePin(GPIOD, LD3_Pin, RESET);
	}
}

void transmitir(void)
{
	rStatus = SX1276GetStatus();

	BufferTx[0] = 'T';
	BufferTx[1] = temp_tx[0];  // MSB de la temperatura (multiplicada por 10)
	BufferTx[2] = temp_tx[1];	 // LSB de la temperatura (multiplicada por 10)
	BufferTx[3] = 'V';
	BufferTx[4] = conversion_tx[0];	// MSB del voltaje medido por el ADC (*10)
	BufferTx[5] = conversion_tx[1];	// LSB del voltaje medido por el ADC (*10)
	if ( rStatus == RF_IDLE )
	{
		if (rxDoneFlag)
		{
			rxDoneFlag = 0;
//			HAL_GPIO_WritePin(GPIOD, LD6_Pin, SET);
//			HAL_Delay(100);
//			HAL_GPIO_WritePin(GPIOD, LD6_Pin, RESET);
//			HAL_Delay(100);
//			Radio.Send( BufferTx, BufferSize );

			// Comprobar que cumple el formato de mensaje "T--V--"
			if ( Buffer[0] == 't' && Buffer[3] == 'v')
			{
				HAL_GPIO_WritePin(GPIOD, LD6_Pin, SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOD, LD6_Pin, RESET);

				int temp_aux = Buffer[1] << 8;
				temp_max = (temp_aux & 0xFF00) + Buffer[2];

				int volt_aux = Buffer[4] << 8;
				volt_max = (volt_aux & 0xFF00) + Buffer[5];
				// Printf valores
			}
		}
		else if (txDoneFlag)
		{
			txDoneFlag = 0;
			nt = 0;
//	   		HAL_Delay(100);
//	   		HAL_GPIO_WritePin(GPIOD, LD4_Pin, SET);
//	   		HAL_Delay(100);
//	   		HAL_GPIO_WritePin(GPIOD, LD4_Pin, RESET);
	   		SX1276SetRx(0);
//	   		SX1276SetRx( RX_TIMEOUT_VALUE );
	   		HAL_Delay(PERIODO_TX);	// XXX-andres Mandar un mensaje cada X segundos
		}
		else	// ni recibo ni envio -> inicio del programa
		{
//	   	HAL_GPIO_WritePin(GPIOD, LD4_Pin, SET);
//		HAL_Delay(100);
//		HAL_GPIO_WritePin(GPIOD, LD4_Pin, RESET);
	   	Radio.Send( BufferTx, BufferSize );
		}
	}
	else if ( rStatus == RF_TX_RUNNING )
	{
		HAL_GPIO_WritePin(GPIOD, LD5_Pin, SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOD, LD5_Pin, RESET);
		nt++;
		if ( nt > MAX_TX )
		{
			nt = 0;
			txDoneFlag=1;
			SX1276SetStby();
		}
	}
	else if ( rStatus == RF_RX_RUNNING )
	{
		HAL_GPIO_WritePin(GPIOD, LD3_Pin, SET);
		HAL_Delay(3000);
		SX1276SetStby();
		HAL_GPIO_WritePin(GPIOD, LD3_Pin, RESET);
	}
  }
void pedir_mediciones(void)
{
	adc_read();
	temp_read();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
