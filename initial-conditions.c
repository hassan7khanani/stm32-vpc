/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "time.h"
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
RTC_TimeTypeDef sTime ;
RTC_DateTypeDef sDate ;
RTC_AlarmTypeDef sAlarm ;
char i2cdata[2];
char OEM_i2cdata[2];
float temperature;
uint8_t temperature1[2]="11";
uint16_t readvalue;
float voltage,current,raw_voltage;
float sensitivity=0.1;

time_t unixTime;
struct tm timeinfo;

uint32_t drain_solenoidTime = 0;
uint8_t  drain_solenoidState = 0; // 0: OFF, 1: ON
uint32_t drain_solenoidTargetTime = 0;
uint8_t  drain_solenoidflag=0;
uint8_t  cellcurrent_starttime=0;
uint32_t read_unixtime,write_unixtime;

uint32_t counter,counter_value,counter1;

uint32_t unixTime2=100,unixTime1;

uint32_t read_rtc_DR0,write_rtc_value=100;
bool read_rtc =false;


//for flash memory
uint32_t flash_memory_address=0x08060000;
uint8_t data[4]={0x23,0x24,0x25,0x26};
uint8_t data1[4]={0x5,0x6,0x7,0x8};
uint8_t rdata[4];
char uartBuffer[50];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
time_t getUnixTime()
{
		char Time[50];
		char Date[50];
	  char unixtime[50];
	  
	
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	
    
    timeinfo.tm_year = sDate.Year + 100; // Adjust for the year offset
    timeinfo.tm_mon =  sDate.Month - 1;   // Adjust for the month offset
    timeinfo.tm_mday = sDate.Date;
    timeinfo.tm_hour = sTime.Hours;
    timeinfo.tm_min =  sTime.Minutes;
    timeinfo.tm_sec =  sTime.Seconds;
    unixTime = mktime(&timeinfo);

	  sprintf(Time,"Time is %02d:%02d:%02d \n\r ",sTime.Hours, sTime.Minutes, sTime.Seconds);
		sprintf(Date,"Date is %02d-%02d-%02d \n\r ",sDate.Year,sDate.Month,sDate.Date);
    HAL_UART_Transmit(&huart2, (uint8_t*)Time, sizeof(Time), 100);
		HAL_UART_Transmit(&huart2, (uint8_t*)Date, sizeof(Date), 100);	
	  sprintf(unixtime, "unix time is %d \n\r ", unixTime);
    HAL_UART_Transmit(&huart2, (uint8_t *)unixtime, sizeof(unixtime), 300);
    return unixTime;
}

float measureCurrent(void) {
    uint16_t readvalue;
    float raw_voltage;
    float current;

    // Measure voltage using ADC
    HAL_ADC_PollForConversion(&hadc1, 1000);
    //readvalue = HAL_ADC_GetValue(&hadc1);
    readvalue=3000;
    // Convert ADC value to voltage
    raw_voltage = (float)readvalue * 3.3  / 4096;
	
    //(float(val)/4096) * 3.3;
    // Adjust for tolerance in voltage divider resistor & ADC accuracy
    if (raw_voltage != 2.5) {
        raw_voltage *= 1.035;
    }

    // Calculate current
    current = (raw_voltage - 2.5) / sensitivity;
		current=10;
		
		char voltage_value[30];
		sprintf(voltage_value,"voltage is %f \t\n\r " ,raw_voltage);
    HAL_UART_Transmit(&huart2, (uint8_t*)voltage_value, sizeof(voltage_value), 300);		
		
		char current_value[30];
		sprintf(current_value,"current is %f \t\n\r " ,current);
    HAL_UART_Transmit(&huart2, (uint8_t*)current_value, sizeof(current_value), 300);
    // Add delay if needed
    //HAL_Delay(100);
		return raw_voltage;
}

void cell_current()
{
  //float current_value=measureCurrent();
	float current_value=5.74;
	if (current_value > 0)
	{
	 char log[100]="in current value greater th";
	 cellcurrent_starttime=HAL_GetTick();
	 if (HAL_GetTick()-cellcurrent_starttime < 5000)	
	{
	 time_t Cell_Control_Fault_RTC = getUnixTime();

   char timeStr[100];
   sprintf(timeStr, "Condition met at Unix time: %d\n\r", unixTime);
   HAL_UART_Transmit(&huart2, (uint8_t*)timeStr, sizeof(timeStr), 300);	
		
	 uint8_t tx_buffer[100]="cell current is greater than 0 for less than 5 seconds";
	 HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
		
	 char CCF_RTC[100];
   sprintf(CCF_RTC, "unixtime when condition meet: %d\n\r", Cell_Control_Fault_RTC);
   HAL_UART_Transmit(&huart2, (uint8_t*)CCF_RTC, sizeof(CCF_RTC), 300);
	}
	else 
	{
	 uint8_t tx_buffer[100]="cell current is greater than 0";
	 HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	}

}
}
void NAOH_tank_level()
{
  
	GPIO_PinState status =HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);

	if (status ==GPIO_PIN_SET)
	{
		  uint8_t tx_buffer[100]=" NAOH_tank_level is high \n\r"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	else
	{
		  uint8_t tx_buffer[100]=" NAOH_tank_level is low \n\r"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
}

void HOCL_tank_level()
{
  
	GPIO_PinState status =HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);

	if (status ==GPIO_PIN_SET)
	{
		  uint8_t tx_buffer[100]=" HOCL_tank_level is high \n\r"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	else
	{
		  uint8_t tx_buffer[100]=" HOCL_tank_level is low \n\r"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
}
void drain_solenoid()
{
  // Check if 24 hours have passed
  if ((HAL_GetTick() - drain_solenoidTime) >= (10000))
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // Turn on solenoid
    uint8_t tx_buffer[100]="in if conditon HAL_GetTick \n\r"; 
    HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);

    if (drain_solenoidState == 0)
    {
      uint8_t tx_buffer[100]=" in drain_solenoidState == 0 \n\r"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // Turn on solenoid control pin
      // HAL_GPIO_WritePin(LED_GPIO_Port, LED_GPIO_PIN, GPIO_PIN_SET); // Turn on LED

      drain_solenoidState = 1;
      // Set the target time for 5 seconds
      drain_solenoidTargetTime = HAL_GetTick() + 5000; // 5 seconds in milliseconds
    }
  }

  // Check if 5 seconds have passed
  if (drain_solenoidState == 1 && HAL_GetTick() >= drain_solenoidTargetTime)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // Turn off solenoid
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // Turn off solenoid control pin
    // HAL_GPIO_WritePin(LED_GPIO_Port, LED_GPIO_PIN, GPIO_PIN_RESET); // Turn off LED
    uint8_t tx_buffer[100]=" in drain_solenoidState == 1 \n\r"; 
    HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);

    drain_solenoidState = 0;
    drain_solenoidTime = HAL_GetTick();
  }
}
   


void OEM_conductivity()
{
	
  HAL_I2C_Mem_Read(&hi2c2,0x64,0x01,1,(uint8_t*)i2cdata,2,100);
  
}

void salt_pump(void)
{
	//run salt pump for 1 minute every 60 minutes	
	if (sTime.Minutes==1)
		{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
		}
	 else
	 {
	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	 }
	
}

void write_data_to_flash_memory(uint32_t type, uint32_t Address, uint8_t* Data, uint32_t Datasize)
{
    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < Datasize; i++)
    {
        // Cast the data to uint64_t since HAL_FLASH_Program expects data of that type
        // Assuming HAL_FLASH_Program can handle the casting and write byte by byte
        HAL_FLASH_Program(type, Address + i,(uint64_t) Data[i]);
    }
    HAL_FLASH_Lock();
}



void erase_flash_memory(uint32_t Sector,uint8_t VoltageRange)
{
  HAL_FLASH_Unlock();
  FLASH_Erase_Sector(Sector,VoltageRange);
  HAL_FLASH_Lock();


}


void read_data_from_flash_memory(uint32_t flashAddress, uint32_t dataSize)
{
    uint8_t rdata[dataSize]; // Array to store the read data
    char uartBuffer[100];    // Buffer for UART transmission
    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < dataSize; i++)
    {
        rdata[i] = *(__IO uint8_t*)(flashAddress + i);
    }
    HAL_FLASH_Lock();

    sprintf(uartBuffer, "Data from flash:");
    for (uint32_t i = 0; i < dataSize; i++)
    {
        char tempBuffer[10];
        sprintf(tempBuffer, " %02X", rdata[i]);
        strcat(uartBuffer, tempBuffer);
    }
    strcat(uartBuffer, "\r\n");

    // Transmit the data over UART
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer), 100);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
	erase_flash_memory(7,FLASH_VOLTAGE_RANGE_3);
  write_data_to_flash_memory(FLASH_TYPEPROGRAM_BYTE,flash_memory_address,data,4);
	//write_data_to_flash_memory(FLASH_TYPEPROGRAM_BYTE,0x08060001,0x20);
  //write_data_to_flash_memory(FLASH_TYPEPROGRAM_BYTE,0x08060002,0x30);
	//write_data_to_flash_memory(FLASH_TYPEPROGRAM_BYTE,0x08060018,0x50);
	//HAL_FLASH_Unlock();
	//FLASH_Erase_Sector(FLASH_SECTOR_7,FLASH_VOLTAGE_RANGE_3);
	//HAL_FLASH_Lock();
	//read_data_from_flash_memory();
	read_data_from_flash_memory(flash_memory_address,6);

	//}
	
	

	
	
	
  /* USER CODE BEGIN 2 */
  char tickStr[20];
	char counter_b[50];
  char unix_buffer[100];	
	char counter_b1[50];
	int  a=0;
	char tempStr[32];
	char rtc_DR0_buffer[100];
	int raw_temp;
	HAL_ADC_Start(&hadc1);
  
	
	

 // HAL_FLASH_Lock();



  if(HAL_I2C_IsDeviceReady(&hi2c1,0XB5,1,100)==HAL_OK)
	{
			
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
	uint8_t tx_buffer[100]="Communication started on I2C with GY-906  sensor\n\r"; 
	HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	} 
	
	///*
	else
		{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
	  uint8_t tx_buffer[100]="Communication error on I2C with GY-906  sensor\n\r"; 
		HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
		}

  //I2C communication with  OEM Conductivity sensor
  if(HAL_I2C_IsDeviceReady(&hi2c1,0x65,1,100)==HAL_OK)
	{
			
	uint8_t tx_buffer[100]="Communication started on I2C with OEM Conductivity sensor \n\r"; 
	HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	} 
	
	///*
	else
		{
	  uint8_t tx_buffer[100]="Communication error on I2C with OEM Conductivity sensor\n\r"; 
		HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
		}		
		

    read_unixtime=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR0);
    
	//	unixTime1 = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR2);
   // read_rtc_DR0=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR0);

		//HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR0);

		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		

		//HAL_UART_Transmit(&huart2,tx_buffer,30,10);
		HAL_I2C_Mem_Read(&hi2c1,0xB4,0x07,1,(uint8_t*)i2cdata,2,100);
		raw_temp=((i2cdata[1]<<8) | (i2cdata[0]));
		temperature=raw_temp*0.02-273.15l;
		
	  sprintf(tempStr, "%f", temperature);
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		
		//HAL_Delay(1000);
		
	//  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	//	HAL_UART_Transmit(&huart2,(uint8_t*)&temperature,sizeof(temperature),100);
	//	HAL_UART_Transmit(&huart2,tempStr,sizeof(tempStr),100);
		
		 // if (HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1) != 0x32F2)
		//	{
		//	getDateTime();
		//	}
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // Turn off solenoid
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		//time_t unixTime = getUnixTime();
		
		
    //getDateTime();
		//getUnixTime();
		
		measureCurrent();
		HAL_GetTick();
		salt_pump();
    drain_solenoid();
		NAOH_tank_level();
		HOCL_tank_level();
    sprintf(tickStr, "HAL_Tick: %d\n\r", HAL_GetTick());
    HAL_UART_Transmit(&huart2, (uint8_t *)tickStr, sizeof(tickStr), 300);
    counter++;
		//unixTime++;
    
    write_unixtime=getUnixTime();
		HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,write_unixtime);


		
	//	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,counter);
	//	counter1 = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR0);
	//	sprintf(counter_b1, "Counter1 value is : %d\n\r", counter1);
  //  HAL_UART_Transmit(&huart2, (uint8_t *)counter_b1, sizeof(counter_b1),300);
		
		
		
		//unixTime = getUnixTime();
		
		// unixTime2=unixTime2 +2;
		// write_rtc_value=write_rtc_value+2;
		

		//HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR2,unixTime2);
     
		// HAL_Delay(100);
	  // sprintf(unix_buffer, "----Unixtime  value is : %d\n\r", unixTime1);
    // HAL_UART_Transmit(&huart2, (uint8_t *)unix_buffer, sizeof(unix_buffer), 300);	


    // read_rtc_DR0=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR0);
	  // sprintf(rtc_DR0_buffer, "----rtc-DR0 value is : %d\n\r", read_rtc_DR0);
    // HAL_UART_Transmit(&huart2, (uint8_t *)rtc_DR0_buffer, sizeof(rtc_DR0_buffer), 300);
    //counter_value = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR0);
		//sprintf(counter_b, "Counter value is : %d\n\r", counter_value);
   // HAL_UART_Transmit(&huart2, (uint8_t *)counter_b, sizeof(counter_b), 300);
		
		//read_unixtime = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1);
		//sprintf(unix_buffer, "unixtime value is : %d\n\r", read_unixtime);
   // HAL_UART_Transmit(&huart2, (uint8_t *)unix_buffer, sizeof(unix_buffer), 300);
		

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_8;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
  
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 23;
  sTime.Minutes = 59;
  sTime.Seconds = 59;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 14;
  sDate.Year = 23;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 3;
  sAlarm.AlarmTime.Minutes = 10;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 14;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led1_Pin|led2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : led1_Pin led2_Pin PA6 */
  GPIO_InitStruct.Pin = led1_Pin|led2_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
