/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
	
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "i2c.h"
#include "iwdg.h"
#include "gpio.h"
#include "stdio.h"
#include <stdlib.h>

struct experiment_package {
	
	uint16_t temperature_sic;
	uint16_t temperature_s;
	
	/*
	Vrb (voltage over resistor on base)
	Vrc (voltage over resistor on collector)
	Ube (voltage frop from base to emitter)
	*/
	uint16_t vrb_sic;
	uint16_t vrc_sic;
	uint16_t ube_sic;
	
	uint16_t vrb_s;
	uint16_t vrc_s;
	uint16_t ube_s;
	
};
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void read_sic(struct experiment_package*);
void read_s(struct experiment_package*);
uint8_t* create_i2c_package(struct experiment_package*);
static void SystemPower_Config(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

int main(void)
{ 

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



		
		for(int i = 0; i<3; i++){
		HAL_Delay(50);
		HAL_GPIO_TogglePin(LD_R_GPIO_Port,LD_R_Pin);
		HAL_Delay(50);
		HAL_GPIO_TogglePin(LD_R_GPIO_Port,LD_R_Pin);
		HAL_Delay(50);
		HAL_GPIO_TogglePin(LD_G_GPIO_Port,LD_G_Pin);
		HAL_Delay(50);
		HAL_GPIO_TogglePin(LD_G_GPIO_Port,LD_G_Pin);
		}

		struct experiment_package package;
		
		read_sic(&package);
		read_s(&package);
		
		uint8_t* message = create_i2c_package(&package);
		
		//Set all pins to analog and turn of gpio clocks
		SystemPower_Config();
		//Enter stop mode
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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

/* USER CODE BEGIN 4 */

static void SystemPower_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();
  
  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();

  /* Select HSI as system clock source after Wake Up from Stop mode */
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);
  
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
	
		GPIO_InitStructure.Pin = B1_Pin;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStructure);
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	

  /* Disable GPIOs clock*/ 
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Clear Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	//HAL_GPIO_WritePin(LD_R_GPIO_Port,LD_R_Pin,1);

}


/**
*
*
*/
uint8_t* create_i2c_package(struct experiment_package* pointer){
	
	uint16_t raw_data[8] = {
		//First experiment
		pointer->temperature_s,
		pointer->ube_s,
		pointer->vrb_s,
		pointer->vrc_s,
		//Seconf experiment
		pointer->temperature_sic,
		pointer->ube_sic,
		pointer->vrb_sic,
		pointer->vrc_sic
	};
	
	uint8_t * message = (uint8_t * ) malloc(17);
	
	message = (uint8_t*) raw_data; 
	
	return message;
}

