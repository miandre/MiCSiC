
/*
TODO: 

Timea ADC.

DONE:

ADC lirar utan delay. Löst genom att kolla EOC.


*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "i2c.h"
#include "iwdg.h"
#include "gpio.h"


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
/* ADC channel configuration structure declaration */
extern ADC_ChannelConfTypeDef        sConfigAdc;
extern DAC_ChannelConfTypeDef 			 sConfigDac;

uint32_t                      			 aResultDMA;

extern ADC_HandleTypeDef             hadc;
extern DAC_HandleTypeDef    				 hdac;

uint8_t 														 number_of_tests = 16;
static struct experiment_package  					 experiments[2];

uint32_t 														timerDelay = 1;
	
/* Defines -------------------------------------------------------------------*/
#define SiC_TEMPERATURE							 ADC_CHANNEL_0
#define SiC_VRB											 ADC_CHANNEL_9
#define SiC_VRC											 ADC_CHANNEL_2
#define SiC_UBE											 ADC_CHANNEL_3
#define S_TEMPERATURE 							 ADC_CHANNEL_5
#define S_VRB 											 ADC_CHANNEL_6
#define S_VRC 											 ADC_CHANNEL_7
#define S_UBE 											 ADC_CHANNEL_8
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

//#define TEST /* Uncomment this before launch! */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void ADC_ReadChannel(uint32_t);
void DAC_SetVoltage(uint32_t);
void ReadSiC(void);
void ReadSilicon(void);
void TestDACRolling(void);
uint8_t* create_i2c_package(uint8_t[]);
void send_message(uint8_t *);


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
	
	if(HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK){
		while(1) {}
	}
	#ifdef TEST /* Enable test to do the tests */
	TestDACRolling();
	ADC_ReadChannel(ADC_CHANNEL_0);
	#else
	
	HAL_Delay(100);
	
	ReadSiC();
	ReadSilicon();
	
	
	/* Create I2C message */
	uint8_t message[18] = {0};
	uint8_t* message_pointer = create_i2c_package(message);
	
	
	
	/* Send message */
	send_message(message_pointer);
	
	#endif
  /* Infinite loop */
  while (1)
  {

  }
}

void TestDACRolling(){
		
	HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);
	HAL_Delay(1000);
	
	int vals[] = {
		0x100,0x280,0x400,0x580,0x700,0x880,0xA00,0xB80,0xD00,0xE80,0xFFF
	};
	
	for(int i = 0; i < 9; i++){
		HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, vals[i]);
		HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);
		HAL_Delay(25);
	}
}

void send_message(uint8_t * message){
	
	//TODO Send data.
	
}

uint8_t* create_i2c_package(uint8_t message[]){
	
	uint16_t raw_data[8] = {
		//First experiment
		experiments[0].temperature,
		experiments[0].ube,
		experiments[0].vrb,
		experiments[0].vrc,
		//Second experiment
		experiments[1].temperature,
		experiments[1].ube,
		experiments[1].vrb,
		experiments[1].vrc
	};
	
	
	uint8_t* ptr = (uint8_t*) &raw_data; 
	
	uint8_t checksum = 0;
	
	message[0] = 16*8 + 8;
	for(int i = 1; i < 17; i++){
		message[i] = *ptr;
		checksum += *ptr;
		ptr++;
	}
	message[17] = checksum;
	
	return (uint8_t *) message;
}

void ReadSilicon(){
	/*Read Temperature and store in struct*/	
	ADC_ReadChannel(S_TEMPERATURE);
	experiments[1].temperature = (aResultDMA & 0xFFF);
	
	/*Read 16 samples of ADC, shift result 5 bits and store in struct */
	/*Read vrb*/
  uint32_t average = 0;
	for(int i = 0; i < number_of_tests; i++){
		ADC_ReadChannel(S_VRB);
		average += aResultDMA;
	}
	experiments[1].vrb = ((average >> 4) & 0xFFF);
	
	/*read vrc*/
  average = 0;
	for(int i = 0; i < number_of_tests; i++){
		ADC_ReadChannel(S_VRC);
		average += aResultDMA;
	}
	
	experiments[1].vrc = ((average >> 4) & 0xFFF);
	
	/*read ube*/
  average = 0;
	for(int i = 0; i < number_of_tests; i++){
		ADC_ReadChannel(S_UBE);
		average += aResultDMA;
	}
	experiments[1].ube = ((average >> 4) & 0xFFF);
	
}

void ReadSiC(){
	/*Read Temperature and store in struct*/	
	ADC_ReadChannel(SiC_TEMPERATURE);
	experiments[0].temperature = (aResultDMA & 0xFFF);
	
	/*Read 16 samples of ADC, shift result 5 bits and store in struct */
	/*Read vrb*/
  uint32_t average = 0;
	for(int i = 0; i < number_of_tests; i++){
		ADC_ReadChannel(SiC_VRB);
		average += aResultDMA;
	}
	experiments[0].vrb = ((average >> 4) & 0xFFF);
	
	/*read vrc*/
  average = 0;
	for(int i = 0; i < number_of_tests; i++){
		ADC_ReadChannel(SiC_VRC);
		average += aResultDMA;
	}
	experiments[0].vrc = ((average >> 4) & 0xFFF);
	
	/*read ube*/
  average = 0;
	for(int i = 0; i < number_of_tests; i++){
		ADC_ReadChannel(SiC_UBE);
		average += aResultDMA;
	}
	experiments[0].ube = ((average >> 4) & 0xFFF);
	
}

void ADC_ReadChannel(uint32_t channel){
	
	aResultDMA = 0;
	sConfigAdc.Channel = channel;
  
	if(HAL_ADC_ConfigChannel(&hadc, &sConfigAdc) != HAL_OK){
		while(1){}
	}
		
		
	if(HAL_ADC_Start(&hadc) != HAL_OK){ //Vad händer här inne nu?
		while(1) {}
	}
	
	//Kan vi titta på någon bit? 
/*	
	if(HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK){
		while(1) {} //Error handling?
	}
	*/
	//
	
	//Det här funkar eftersom att vi nu kollar på EOC (End of Conversion) innan vi gör läsningen.
	while(!(hadc.Instance->ISR & 0x4)){}
	
	//HAL_Delay(timerDelay);	//Korrekt nu med delay.
	aResultDMA = hadc.Instance->DR; //När vi läser från registret clearas EOC igen. 
	//0x40012440
	//aResultDMA = *((volatile uint32_t *) 0x40012440);
	

	*((volatile uint32_t*)0x40012428)=0; //Här nollar vi channel select-registret. 
	
}

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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
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
