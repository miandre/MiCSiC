/**
  ******************************************************************************
  * File Name          : main.c
	
	Main program for SiC in Space experiment. 
	
	
	Note that in the current configuration, the software communicates via
	USART, and not via I2C (As the final software will have to do). 
	Some I2C functionality is written, but at the moment this will most 
	likely not work without further configuration. 
	
	To use the software as is, download the processing software from github 
	(https://github.com/Happsson/SiC-Analyzing), 
	connect the USART to the computer using for example this cable:
	(http://www.lawicel-shop.se/prod/TTL-232R-USB-5Vcable-33V-IO_794885/Sparkfun_64668/SWE/SEK) 
	and run the Processing code. 
	On the current most recent board, the TX and RX pinouts are located next to the SWD port like so:

	[TX]
	[RX]
	[NC]
	[GND]
	
	where TX is closest to the SWD port, and GND is closest to the reset button. 
	
	The I2C connections are located on the pin header with three pins like this:
	[SCL]
	[SDA]
	[GND]
	where GND is closest to the 3.3 power supply.
	
	The pin header for the SWD is configured in the following way:
	
	[NC] 
	[SWCLK]
	[GND]
	[SWDIO]
	[NRST]
	[NC]
	Where NRST and NC are closest to the USART port. 
	
	Refer to the Diptrace files for schematics and PCB design for 
	further pin connections. 
	
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "i2c.h"
#include "iwdg.h"
#include "gpio.h"
#include "usart.h"


/* Structs -------------------------------------------------------------------*/
struct experiment_package {
	/*
	temperature
	Vrb (voltage over resistor on base)
	Vrc (voltage over resistor on collector)
	Ube (voltage frop from base to emitter)
	*/
	uint16_t temperature;
	uint16_t vrb;
	uint16_t vrc;
	uint16_t ube;
	
};

/* Private variables ---------------------------------------------------------*/
uint16_t 														 number_of_tests = 16;
uint32_t 													   timerDelay = 1;
	

/* External variables*/
extern ADC_ChannelConfTypeDef        sConfigAdc;
extern DAC_ChannelConfTypeDef 			 sConfigDac;
extern ADC_HandleTypeDef             hadc;
extern DAC_HandleTypeDef    				 hdac;
extern UART_HandleTypeDef 					 huart1;
extern I2C_HandleTypeDef 							hi2c1;
static struct experiment_package  	 experiments[8];



/*
    PA0     ------> ADC_IN0
    PA1     ------> ADC_IN1
    PA2     ------> ADC_IN2
    PA3     ------> ADC_IN3
    PA5     ------> ADC_IN5
    PA6     ------> ADC_IN6
    PA7     ------> ADC_IN7
    PB0     ------> ADC_IN8
    PB1     ------> ADC_IN9 
		*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint8_t* create_i2c_package(uint8_t[]);
void send_message(uint8_t *);
void setDAC(uint32_t);
void readRollingADC(int);
void shiftAverages(void);
void normalRun(void);
void graphTestSweep(int);
void testProgram(void);
void SystemPower_Config(void);
uint8_t* create_graph_package(uint8_t[]);
uint16_t receive_OBC_message(void);

uint32_t tickCounterStart;
uint32_t tickCounterStop;
uint32_t value;
uint32_t timeTaken;


int main(void)
{
	tickCounterStart = 0;
	tickCounterStop  = 0;
	value=0;
	timeTaken=0;
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
	MX_USART1_UART_Init();
	

	//Calibrate ADCs in the beginning of every run
	if(HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK){
		while(1) {}
	}

	//Run experiment
	normalRun();
		
}

void normalRun(){

	uint16_t obc_message = receive_OBC_message(); //Receive message from OBC. Not used, read text in function.
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET); // Turn on power supply
	HAL_Delay(80); //Wait for power supply to settle
  
	//tickCounterStart = HAL_GetTick(); //Only used for timing purposes
	
	setDAC(0);
	HAL_Delay(2);
		
	/* Set DAC at voltage level 1 (3.1v 0xF07)*/
	setDAC(0xF07);
	HAL_Delay(2);
	for(int i = 0; i < 16; i++){ 
		readRollingADC(0); //All inputs 16 times.
	}
	setDAC(0);
	HAL_Delay(2);
		/* Set DAC at voltage level 1 (2.1v 0xA2E)*/
	setDAC(0xA2E);
	HAL_Delay(2);
	for(int i = 0; i < 16; i++){
		readRollingADC(2);
	}

		setDAC(0);
	HAL_Delay(2);
		/* Set DAC at voltage level 1 (1.1v 0x555)*/
	setDAC(0x555);
	HAL_Delay(2);
	for(int i = 0; i < 16; i++){
		
		readRollingADC(4);
	}
	setDAC(0);
	HAL_Delay(2);
		/* Set DAC at voltage level 1 (0.5v 0x260)*/
	setDAC(0x260);
	HAL_Delay(2);
	for(int i = 0; i < 16; i++){
		
		readRollingADC(6);
	}
	
	
	//tickCounterStop = HAL_GetTick(); //Not used, only for testing purposes.
	//timeTaken = (tickCounterStop  -  tickCounterStart); 
		shiftAverages();
	
	/* Create I2C message */
	uint8_t message[34] = {0};
	uint8_t* message_pointer = create_i2c_package(message);
	
	
	/* Send message */
	send_message(message_pointer);
	
	
	/*
	This is used for testing purposes. Connect USART to computer,
	run the test program written in Processing. 
	*/
	//Send UART Message
	HAL_UART_Transmit(&huart1,(uint8_t*)&message, 34, 10);
	huart1.State=HAL_UART_STATE_READY;
	
	
	SystemPower_Config();
			
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	
 }
	
 
uint16_t receive_OBC_message(){
	
	/*
	
	Since the I2C communication with the OBC is not yet final,
	this function is left empty as of now. 
	There is a high probablility that there will not be a message
	from the OBC in this stage, since there would be nothing 
	to gain from it.
	
	*/
	
	return 0;
}	
 

/*

Calculate the average of the readings.
Note that this could be removed, to send the complete
uncalculate data instead. This would actually give
a higher precision, since shifting the data
deletes information.

*/
void shiftAverages(){
	for(int i = 0; i < 8; i++){
		experiments[i].temperature = (experiments[i].temperature >> 4);
		experiments[i].ube = (experiments[i].ube >> 4);
		experiments[i].vrb = (experiments[i].vrb >> 4);
		experiments[i].vrc = (experiments[i].vrc >> 4);
	}
}
void setDAC(uint32_t voltage){
		HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, voltage);
		HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);
}

void readRollingADC(int index){
	HAL_Delay(1);

	//Start ADC reading
	if(HAL_ADC_Start(&hadc) != HAL_OK){
		while(1) {}
	}
	
	//Wait for EOC (end of conversion)
	while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	//Read ADC value
	experiments[0+index].temperature += hadc.Instance->DR;

		
	//Repeat for all channels.	
	while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	experiments[0+index].ube += hadc.Instance->DR;
	
  while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	experiments[0+index].vrb += hadc.Instance->DR;
		
	while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	experiments[0+index].vrc += hadc.Instance->DR;
	
	while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	experiments[1+index].temperature += hadc.Instance->DR;

	while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	experiments[1+index].ube += hadc.Instance->DR;
	
  while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	experiments[1+index].vrb += hadc.Instance->DR;
		
	while(!(hadc.Instance->ISR & ADC_ISR_EOC)){}
	experiments[1+index].vrc += hadc.Instance->DR;
		
}

void send_message(uint8_t * message){
	
	/*************************DISCLAIMER****************
	This code for the I2C communication is not valid for communication and only
	used for test purposes.
	This code should be rewritten before use.
	********************************************************/
	
	/*
	
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
   {
		 
   }
	
	while(HAL_I2C_Slave_Transmit(&hi2c1, (uint8_t*) &message, 33, 10)!= HAL_OK)
	{
	 
	}
	
	*/
	
}

uint8_t* create_i2c_package(uint8_t message[]){
	
	
	
	/*
	Note that this is only the measurements for the Si transistor.
	To send all data, add experiments[1], experiments[3]... to the message,
	and change to the correct size.
	*/
	
	uint16_t raw_data[16] = {
		//First experiment
		experiments[0].temperature,
		experiments[0].ube,
		experiments[0].vrb,
		experiments[0].vrc,
		//Second experiment
		experiments[2].temperature,
		experiments[2].ube,
		experiments[2].vrb,
		experiments[2].vrc,
		//First experiment
		experiments[4].temperature,
		experiments[4].ube,
		experiments[4].vrb,
		experiments[4].vrc,
	 //First experiment
		experiments[6].temperature,
		experiments[6].ube,
		experiments[6].vrb,
		experiments[6].vrc
	};
	
	
	uint8_t* ptr = (uint8_t*) &raw_data; 
	
	uint8_t checksum = 0;
	
	message[0] = 16*8 + 8;
	for(int i = 1; i < 33; i++){
		message[i] = *ptr;
		checksum += *ptr;
		ptr++;
	}
	message[33] = '\n'; //Only used when sending to Processing.
	//message[33] = checksum; //Uncomment this when communicating via I2C.
	
	return (uint8_t *) message;
}


/** System Clock Configuration
Code auto generated by CubeMX
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


void SystemPower_Config(void)
{
	//Used before entering power STOP mode.
	//Should be revised, could be optimized. 

  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();
  
  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();

  /* Select HSI as system clock source after Wake Up from Stop mode */
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
  
  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();

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
