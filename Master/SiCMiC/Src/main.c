
/*
TODO: 

Fixa en error handling på att vi fastnar i check på eoc

Fortsätt timea ADC. I dagsläget ligger vi på 0,8 ms (med 230cyclesBlaBla). 0,2 med 1cycle5
Vi fastnar i EOC när vi mäter väldigt många gånger, lista ut varför. 


Timea ADC.

Fixa USART-kommunikation till Processing

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
/* ADC channel configuration structure declaration */
extern ADC_ChannelConfTypeDef        sConfigAdc;
extern DAC_ChannelConfTypeDef 			 sConfigDac;

uint32_t                      			 aResultDMA;

extern ADC_HandleTypeDef             hadc;
extern DAC_HandleTypeDef    				 hdac;
extern UART_HandleTypeDef 						huart1;

uint16_t 														 number_of_tests = 16;

static struct experiment_package  	 experiments[8];

uint32_t 													  timerDelay = 1;
	
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
#define TEST_TIME 
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint8_t* create_i2c_package(uint8_t[]);
void send_message(uint8_t *);
void setDAC(uint32_t);
void readRollingADC(int);
void shiftAverages(void);

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
	

	
	if(HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK){
		while(1) {}
	}
	
	HAL_Delay(100);
	  while (1)
  {
		
		//HAL_GPIO_TogglePin(LD_G_GPIO_Port, LD_G_Pin);
		
	tickCounterStart = HAL_GetTick();
	
	HAL_Delay(5);	
	/* Set DAC at voltage level 1 (3.1v 0xF07)*/
	setDAC(0xF07);
	for(int i = 0; i < 16; i++){
		readRollingADC(0);
	}
	
	HAL_Delay(5);
		/* Set DAC at voltage level 1 (2.1v 0xA2E)*/
	setDAC(0xA2E);
	for(int i = 0; i < 16; i++){
		readRollingADC(2);
	}
	
	HAL_Delay(5);
		/* Set DAC at voltage level 1 (1.1v 0x555)*/
	setDAC(0x555);
	for(int i = 0; i < 16; i++){
		readRollingADC(4);
	}
	
	HAL_Delay(5);
		/* Set DAC at voltage level 1 (0.2v 0xF8)*/
	setDAC(0xF8);
	for(int i = 0; i < 16; i++){
		readRollingADC(6);
	}
	

	
	tickCounterStop = HAL_GetTick();
	timeTaken = (tickCounterStop  -  tickCounterStart);
		shiftAverages();
	
	/* Create I2C message */
	uint8_t message[18] = {0};
	uint8_t* message_pointer = create_i2c_package(message);
	
	//UART TEST MESSAGE
	uint8_t hej[3] = {'h','e','j'};
	
	
	/* Send message */
	send_message(message_pointer);
	
	//Send UART Message
	HAL_UART_Transmit(&huart1,(uint8_t*)&message, 18, 10);
	huart1.State=HAL_UART_STATE_READY;
	
  /* Infinite loop */
		while(1){
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_15);
		HAL_Delay(100);
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
		HAL_Delay(100);
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
		HAL_Delay(100);
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
		HAL_Delay(100);
		
	}

  }
}
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
	
	if(HAL_ADC_Start(&hadc) != HAL_OK){
		while(1) {}
	}
	
	while(!(hadc.Instance->ISR & 0x4)){}
	experiments[0+index].temperature += hadc.Instance->DR;

	while(!(hadc.Instance->ISR & 0x4)){}
	experiments[0+index].ube += hadc.Instance->DR;
	
  while(!(hadc.Instance->ISR & 0x4)){}
	experiments[0+index].vrb += hadc.Instance->DR;
		
	while(!(hadc.Instance->ISR & 0x4)){}
	experiments[0+index].vrc += hadc.Instance->DR;
		
	while(!(hadc.Instance->ISR & 0x4)){}
	experiments[1+index].temperature += hadc.Instance->DR;

	while(!(hadc.Instance->ISR & 0x4)){}
	experiments[1+index].ube += hadc.Instance->DR;
	
  while(!(hadc.Instance->ISR & 0x4)){}
	experiments[1+index].vrb += hadc.Instance->DR;
		
	while(!(hadc.Instance->ISR & 0x4)){}
	experiments[1+index].vrc += hadc.Instance->DR;
		
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
	message[17] = '\n';
	
	return (uint8_t *) message;
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
