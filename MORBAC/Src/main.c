/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************/
	
/*******************************************************************************/
/************************First som C stuff***************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>

/* Defines -----------------------------------------------------------------*/
#define ledPin 									GPIO_PIN_12
#define sensorVoltageSupplyPin 	GPIO_PIN_13
#define HIGH										GPIO_PIN_SET
#define LOW 										GPIO_PIN_RESET
#define timeDelay								5000
#define sensorDataPin						ADC_CHANNEL_4

/* Private variables ---------------------------------------------------------*/

uint8_t serial_input;
uint16_t sensorValue;
float voltage;

/*The "Float" (with capital F) data format is used when a float is sent via
the serial bus (UART)
-----------------------------------------------------------------------------*/
typedef union _data {
  float f;
  uint8_t  i[4];
} Float;


/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc;
extern ADC_ChannelConfTypeDef sConfig;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void makeMeasurements(void);
int Serial_available(void);
void digitalWrite(uint16_t, GPIO_PinState);
uint16_t analogRead(uint32_t);
void delay(volatile uint32_t);
void Serial_printString(char[]);
void Serial_PrintValue(float);



/***********************************************************************************
*************************The program starts here***********************************/

int main(void)
{

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
	
	
/******Blink the LED 5 times to se that the program started*****/
	for(int i = 0; i<5; i++){
		digitalWrite(ledPin,HIGH);
		delay(100);
		digitalWrite(ledPin,LOW);
		delay(100);		
	}
	
	
	/****************************************************************************/
	/***Main loop of the program. Equal to "loop()" in the Arduino code**********/
	
	
  while (1)
  {
		
		while(Serial_available() == 0){} //wait for input from user
		
			makeMeasurements();	//make sensor measurements
			
			Serial_PrintValue(voltage);	//send the voltage red from the sensor
			
			delay(1000); //wait a while

  }


}
/********************End of main loop************************/



/***********************************************************
makeMeasurements() reads the incomming serial command.
If the command is 0, a led is turned on and the sensor
value is read from the ADC, converted to a voltage level
and stored in the "voltage" variable.
If the comand is 1, the same sensor is read but with the LED turned off
*****************************************************/
void makeMeasurements(){
	
int command = serial_input-'0';
	
	
	if(command == 0){
		digitalWrite(ledPin, HIGH);
		digitalWrite(sensorVoltageSupplyPin, HIGH);
		delay(timeDelay);
		sensorValue = analogRead(sensorDataPin);
		voltage = sensorValue*(3.3/4096);
		delay(50);
		digitalWrite(ledPin, LOW);
		digitalWrite(sensorVoltageSupplyPin, LOW);
		delay(1000);
	}
	
		else if(command == 1){
		digitalWrite(ledPin, LOW);
		digitalWrite(sensorVoltageSupplyPin, HIGH);
		delay(timeDelay);
		sensorValue = analogRead(sensorDataPin);
		voltage = sensorValue*(3.3/4096);
		delay(50);
		digitalWrite(ledPin, LOW);
		digitalWrite(sensorVoltageSupplyPin, LOW);
		delay(1000);
	}
	else Serial_printString("Invalid");
	
}


/********************************************************************
								ARDUINO FUNCTIONS

These function ar writen to mimic the functions available in the 
Arduino IDE. Some code may be redundand or unnecessarry and it is 
recomended to rewrite thes into "real" C code later in the project.
**********************************************************************/

/*--------------------------------------------------------------------
Serial_available() reads incomming serial data.
while checking if data is available, it also store the first available byte in the
"serial_input" variable. If n input was available, the function returns 0. 
When a byte has been recieved, the function returns 1
-----------------------------------------------------------------------*/
int Serial_available(){
	
	if(HAL_UART_Receive(&huart1,&serial_input,1,2) == HAL_OK) return 1;
		else return 0;

}

/*----------------------------------------------------------
digitalWrite() sets an output of the GPIOB cluster to either 
high or low
-----------------------------------------------------------*/
void digitalWrite(uint16_t pinName, GPIO_PinState state ){

	HAL_GPIO_WritePin(GPIOB, pinName, state);

}

/*-------------------------------------------------------------
analogRead() reads one of the ADC channels and returns dhe 12bit value as 
a uint16_t value.
--------------------------------------------------------------*/
uint16_t analogRead(uint32_t channel){
	
uint16_t adcTemp=0;
 
	//first, configure what channel to read
	sConfig.Channel = channel;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);	

	//Calibrate the ADC
	HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
	
	HAL_Delay(1);
	
	//Start the ADC conversion
	HAL_ADC_Start(&hadc);

	//Wait for the conversion to end and save the value to be returned
	while(!(hadc.Instance->ISR & 0x4)){}
	adcTemp = hadc.Instance->DR;

	return adcTemp;
}

/*--------------------------------------------------------------------
Serial_PrintString() is used to send strings over the UART serial buss
----------------------------------------------------------------------*/
void Serial_printString(char input[]){
	
	char ln[1] = {'\n'};
	strcat(input,ln);

		HAL_UART_Transmit(&huart1,(uint8_t*)&input, strlen(input), 10);
		huart1.State=HAL_UART_STATE_READY;	

}

/*--------------------------------------------------------------------
Serial_PrintValue() is used to send floats over the UART serial buss
----------------------------------------------------------------------*/

void Serial_PrintValue(float input){
		
		Float tmp;
		tmp.f = input;

		HAL_UART_Transmit(&huart1,(uint8_t*)&tmp, 4, 10);
		huart1.State=HAL_UART_STATE_READY;	

}

/*---- delay function---------------*/
void delay(volatile uint32_t delay){
HAL_Delay(delay);	
}


/*******************End of ARDUINO FUNCTIONS**********************************/
/*****************************************************************************/




/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_3;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
