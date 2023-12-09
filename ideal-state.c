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
//#include "main.h"
#include "NAOH_tank.h"

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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
RTC_TimeTypeDef sTime ;
RTC_DateTypeDef sDate ;
RTC_AlarmTypeDef sAlarm ;




uint16_t readvalue;
float voltage,current,raw_voltage;
float sensitivity=0.1;

time_t unixTime;
struct tm timeinfo;

uint32_t drain_solenoidTime = 0;
uint8_t  drain_solenoidState = 0; 
uint32_t drain_solenoidTargetTime = 0;
uint8_t  drain_solenoidflag=0;
uint8_t  cellcurrent_starttime=0;
uint32_t read_unixtime;
uint32_t write_unixtime;
uint32_t counter;

//for flash memory
static uint32_t lastSaveTime = 0;
uint32_t system_hours = 0;
uint32_t total_cycle_hours = 0;
uint32_t hours = 0, minutes = 0;

/*
typedef struct {
    uint32_t system_hours;
    uint32_t system_minutes;
	  uint32_t total_cycle_hours;
	  uint32_t total_cycle_mins;
} TimeComponents;


//current_time structure

typedef struct{
    uint64_t currenttime_ms;
    uint64_t currenttime_s;
	  uint64_t currenttime_m;
	  uint64_t currenttime_h;

} CurrentTime;
*/



//system hours structure
typedef struct{
    uint32_t systemtime_s;
	  uint32_t systemtime_m;
	  uint32_t systemtime_h;
	  uint32_t ticks;
} SystemTime;


SystemTime system_time,total_system_time;

//total cycle hours structure

typedef struct{
    uint32_t totalcycletime_s;
	  uint32_t totalcycletime_m;
	  uint32_t totalcycletime_h;
} TotalCycleTime;

TotalCycleTime total_cycle_time,cycle_time;


//for faults
	struct Fault_logs
{
 uint32_t counter;
 uint32_t	total_cycle_hours;
 bool fault_status;
 uint32_t system_hours;
 uint32_t real_time_clock; 	
} f1;

typedef enum {
    FAULT_TYPE_CKT_B,
    FAULT_TYPE_UNDR_CNT,
    FAULT_TYPE_CKT_A,
    FAULT_TYPE_UNDER_CURRENT,
    FAULT_TYPE_OVER_CURRENT,
    FAULT_TYPE_SALTTUBE_LEVEL,
    FAULT_TYPE_PRESSURE_LOSS,
    FAULT_TYPE_SALTPUMP_HOCL_FL,
    FAULT_TYPE_SALTPUMP_NAOH_FL,
    FAULT_TYPE_PH_HOCL,
    FAULT_TYPE_PH_NAOH,
    FAULT_TYPE_BRINE_CONDUCTIVITY,
    FAULT_TYPE_CELL_CONTROL,
    NUM_FAULT_TYPES
} fault_type_e;

typedef struct {
 uint32_t counter;
 uint32_t	cycle_hours;
 bool fault_status;
 uint32_t system_hours;
 uint32_t real_time_clock; 
	  
}__attribute__((packed)) fault_t;

typedef struct {
    fault_t faults[NUM_FAULT_TYPES][20];
} __attribute__((packed)) fault_array_t; 

// Initialize my_faults
fault_array_t my_faults;

fault_t fault_circuit_board,fault_under_current,fault_over_current;
fault_t fault_salt_tube_level,fault_pressure_loss;
fault_t fault_salt_pump_hocl_flowrate,fault_salt_pump_naoh_flowrate;
fault_t fault_ph_hocl,fault_ph_naoh,fault_brine_conductivity;

//counter for each fault type
int fault_circuit_board_count=0;
int fault_under_current_count=0;
int fault_over_current_count=0;
int fault_salt_tube_level_count=0;
int fault_pressure_loss_count=0;
int fault_salt_pump_hocl_flowrate_count=0;
int fault_salt_pump_naoh_flowrate_count=0;
int fault_ph_hocl_count=0;
int fault_ph_naoh_count=0;
int fault_brine_conductivity_count=0;
int fault_cellcontrol_count=0;



//tank fill time 
uint32_t TankFill_time=20;


//receive data on uart
uint8_t rx_data[100];

// for production
bool Masterflag=false;
bool Stop_signal=false;
bool Start_signal=false;
bool Resume_signal=false;
bool Production=true;

//flags for each state in idel
bool Idel_systemready_state=false;
bool Idel_tanksfull_state=false;
bool Idel_tankfill_timeexceeded_state=false;
bool Idel_lowpressure_state=false;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
//static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
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
    //calculate unixtime
    unixTime = mktime(&timeinfo);
    
    //print the date and time on uart
	  sprintf(Time,"Time is %02d:%02d:%02d \n\r ",sTime.Hours, sTime.Minutes, sTime.Seconds);
		sprintf(Date,"Date is %02d-%02d-%02d \n\r ",sDate.Year,sDate.Month,sDate.Date);
    HAL_UART_Transmit(&huart2, (uint8_t*)Time, sizeof(Time), 100);
		HAL_UART_Transmit(&huart2, (uint8_t*)Date, sizeof(Date), 100);	
	  sprintf(unixtime, "unix time is %d \n\r ", unixTime);
    HAL_UART_Transmit(&huart2, (uint8_t *)unixtime, sizeof(unixtime), 300);
    return unixTime;
}


//calculate current and voltage
float measure_current(void) {

    uint16_t readvalue;
    float raw_voltage;
    float current;

    // Measure voltage using ADC
    HAL_ADC_PollForConversion(&hadc1, 1000);
    readvalue = HAL_ADC_GetValue(&hadc1);
   // readvalue=3000; hardcoded value for testing

    // Convert ADC value to voltage
    raw_voltage = (float)readvalue * 3.3  / 4096;
	

    // Adjust for tolerance in voltage divider resistor & ADC accuracy
    if (raw_voltage != 2.5) {
        raw_voltage *= 1.035;
    }

    // Calculate current
    current = (raw_voltage - 2.5) / sensitivity;
		
		//print voltage on uart
		char voltage_value[30];
		sprintf(voltage_value,"voltage is %f \t\n\r " ,raw_voltage);
    HAL_UART_Transmit(&huart2, (uint8_t*)voltage_value, sizeof(voltage_value), 300);		
		
    //print current on uart
		char current_value[30];
		sprintf(current_value,"current is %f \t\n\r " ,current);
    HAL_UART_Transmit(&huart2, (uint8_t*)current_value, sizeof(current_value), 300);

		return current;
}

//function to check cell current is greater than 0 for less than 5 seconds
float cell_current()
{
	static uint8_t  cell_control_fault_5sec_instance=0;
	TotalCycleTime  cell_control_cycle_hours;
	SystemTime cell_control_system_hours;
	char cell_control_cycle_hours_buffer[100];
	char cell_control_system_hours_buffer[100];
	//SystemTime 

  float current_value=measure_current();
	//float current_value=5.74;  hardcoded value for testing

	if (current_value > 0)
	{
	 cellcurrent_starttime=HAL_GetTick();
	 if (HAL_GetTick()-cellcurrent_starttime < 5000)	
	{
	  cell_control_fault_5sec_instance++;	
	  cell_control_system_hours = system_time;
		cell_control_cycle_hours=total_cycle_time;

	  // Prepare a buffer to send data on uart
    char tx_buffer[300];
    int n = sprintf(tx_buffer, "cell_control_fault_5sec_instance: %d \r\n", cell_control_fault_5sec_instance);
		n+=sprintf(tx_buffer, "cell_control_system_hours %d, cell_control_system_mins%d,cell_control_system_sec %d  \r\n", cell_control_system_hours.systemtime_h,cell_control_system_hours.systemtime_m,cell_control_system_hours.systemtime_s);
		
		           // sprintf(hours_buffer, "Cycle Hours: %s \r\n", formatTotalCycleTime(cell_control_cycle_hours));

    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,sizeof(tx_buffer),100);

	}
	
	else 
	{
	 uint8_t tx_buffer[100]="cell current is greater than 0";
	 HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	}

}
return current_value;

}

/*
//function to monitor NAOH tank level
bool NAOH_tank_level()
{
  //read the tank level on pin PA1
	GPIO_PinState status =HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);

  //print the tank level on uart
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
	
	return status;
}
*/

// monitor salt tube bottom sensor
bool salt_tube_bottom()
{
  //read the salt tube bottom sensor on pin PA0
	GPIO_PinState status =HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11);

  //print the the salt tube bottom sensor on uart
	if (status ==GPIO_PIN_SET)
	{
		  uint8_t tx_buffer[100]=" salt tube bottom sensor is high \n\r"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	else
	{
		  uint8_t tx_buffer[100]=" salt tube bottom sensor is low \n\r"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	return status;
}
// monitor salt tube Top sensor
bool salt_tube_top()
{
  //read the salt tube top sensor on pin PA0
	GPIO_PinState status =HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12);

  //print the the salt tube top sensor on uart
	if (status ==GPIO_PIN_SET)
	{
		  uint8_t tx_buffer[100]=" salt tube top sensor is high \n\r"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	else
	{
		  uint8_t tx_buffer[100]=" salt tube top sensor is low \n\r"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	return status;
}

//function to monitor HOCL tank level
bool HOCL_tank_level()
{
  //read the HOCL tank level on pin PA0
	GPIO_PinState status =HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);

  //print the tank level on uart
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
	return status;
}

//function to monitor pressure
bool monitor_pressure()
{
  //read the pressure on pin PB0
	GPIO_PinState status =HAL_GPIO_ReadPin(GPIOB,pressure_Pin);

  //print the pressure status on uart
		if (status ==GPIO_PIN_SET)
	{
		  uint8_t tx_buffer[100]=" pressure is high \n\r"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	else
	{
		  uint8_t tx_buffer[100]=" pressure is low \n\r"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	return status;
}

//function to turn on or off cellfill solenoid
void cellfill_solenoid(bool status)
{
  if (status==true)
	{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
	monitor_pressure();
	}
	else
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
}

//function to turn on or off drain solenoid for 1 
void drain_solenoid()
{
  // Check if 24 hours have passed
  if ((HAL_GetTick() - drain_solenoidTime) >= (24*60*60*1000))
  {

    if (drain_solenoidState == 0)
    {

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // Turn on solenoid control pin
      drain_solenoidState = 1;

      // Set the target time for 1 minute
      drain_solenoidTargetTime = HAL_GetTick() + 60000; // 1 minute in milliseconds
    }
  }

  // Check if 1 minute have passed
  if (drain_solenoidState == 1 && HAL_GetTick() >= drain_solenoidTargetTime)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // Turn off solenoid control pin
    drain_solenoidState = 0;
    drain_solenoidTime = HAL_GetTick();
  }
}
   
//function to start I2C communication with ph sensor for NAOH on I2C
void pH_NAOH_initialization()
{ 
	//for NAOH ph sensor
  if(HAL_I2C_IsDeviceReady(&hi2c1,0X65,1,100)==HAL_OK)
	{
  uint8_t tx_buffer[100]="Communication started on I2C with pH NAOH sensor\n\r"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);

}
	else
	{
	  uint8_t tx_buffer[100]="Error in Communicating with pH NAOH sensor on I2C \n\r"; 
	  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	}
	
}

//function to start I2C communication with ph sensor for HOCL on I2C
void pH_HOCL_initialization()

{
  if(HAL_I2C_IsDeviceReady(&hi2c1,0X65,1,100)==HAL_OK)
	{
  uint8_t tx_buffer[100]="Communication started on I2C with pH HOCL sensor\n\r"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	
}
	else
	{
	uint8_t tx_buffer[100]="Error in Communicating with pH HOCL sensor on I2C \n\r"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);

	}
}

//function to start I2C communication with flowmeter NAOH on I2C
void measure_flow_NAOH()
{

  if(HAL_I2C_IsDeviceReady(&hi2c1,0X01,1,100)==HAL_OK)
	{
  uint8_t tx_buffer[100]="Communication started on I2C with flowmeter NAOH sensor\n\r"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	
}
	else
	{
	uint8_t tx_buffer[100]="Error in Communicating with flowmeter NAOH sensor on I2C \n\r"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);

	}

}
//function to start I2C communication with flowmeter HOCL on I2C
void measure_flow_HOCL()
{
  if(HAL_I2C_IsDeviceReady(&hi2c1,0X02,1,100)==HAL_OK)
	{
  uint8_t tx_buffer[100]="Communication started on I2C with flowmeter HOCL sensor\n\r"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	
}
	else
	{
	uint8_t tx_buffer[100]="Error in Communicating with flowmeter HOCL sensor on I2C \n\r"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);

	}

}
//function to start I2C communication with flowmeter saltpump on I2C
void measure_flow_saltpump()
{
  if(HAL_I2C_IsDeviceReady(&hi2c1,0X03,1,100)==HAL_OK)
	{
  uint8_t tx_buffer[100]="Communication started on I2C with flowmeter saltpump sensor\n\r"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	
}
	else
	{
	uint8_t tx_buffer[100]="Error in Communicating with flowmeter saltpump sensor on I2C \n\r"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);

	}

}
//function to turn on the salt pump every for 1 minute every 60 minutes
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

//function to erase complete sector from flash memory
void erase_flash_memory(uint32_t Sector,uint8_t VoltageRange)
{
  HAL_FLASH_Unlock();
  FLASH_Erase_Sector(Sector,VoltageRange);
  HAL_FLASH_Lock();
}

//function to write data to flash memory
void write_to_flash(uint32_t address, uint8_t *data, uint32_t size,uint32_t TypeProgram) {
    HAL_FLASH_Unlock();

    for (uint32_t i = 0; i < size; i++) {
        HAL_FLASH_Program(TypeProgram, address + i, data[i]);
    }
    
    HAL_FLASH_Lock();
}




//function to print temp fault array on uart
void print_temp_fault_array(fault_array_t *temp_fault_array) {
    char uartStr[1024]; // Buffer for UART string
    int n = 0; // Index for string formatting

    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 2; j++) 
        { 
            fault_t fault = temp_fault_array->faults[i][j];
            
            // Format each fault into the uartStr buffer
            n += snprintf(uartStr + n, sizeof(uartStr) - n, "Fault Type: %d, Index: %d, Counter: %d, ...", i, j, fault.counter); 
            // Add other fault fields in the format string above as needed

            // Check for buffer overflow
            if (n >= sizeof(uartStr)) {
                // Handle overflow, for example by sending the current buffer and resetting n
                HAL_UART_Transmit(&huart2, (uint8_t*)uartStr, strlen(uartStr), 100);
                n = 0; // Reset buffer index
            }
        }
    }

    // Transmit any remaining data in the buffer
    if (n > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t*)uartStr, strlen(uartStr), 100);
    }
}

//function to update fault
void update_fault(fault_array_t *fault_array, fault_type_e fault_type, int *index, fault_t fault_param) {

    //address to save the entire fault structure on sector t address
	  static uint32_t fault_log_address = 0x08060000;

    static uint32_t f1_address = 0x08040000;
    //static int f1_address_counter = 0;
    
    static bool isFirstWrite = true;
    static uint32_t read_back_f1_address = 0;
	
	  
    // Prepare a string for UART transmission
    char uartStr[1024]; // Adjust the size as needed
   // printf("a",fault_param.count);
    int n = snprintf(uartStr, sizeof(uartStr), "----------Received Arguments:");
    n += snprintf(uartStr + n, sizeof(uartStr) - n, "fault_type: %d, index: %d,fault_param.counter: %d,fault_param.cycle_hours: %d,fault_param.system_hours: %d,fault_param.fault_status: %d,fault_param.real_time_clock: %d,falult type %d,fault_array address %p \r\n", fault_type, *index,fault_param.counter,fault_param.cycle_hours,fault_param.system_hours,fault_param.fault_status,fault_param.real_time_clock,fault_type,fault_array);
    
		//read from flash memory
		fault_array_t temp_fault_array;
    memcpy(&temp_fault_array, (fault_array_t*)fault_log_address, sizeof(fault_array_t));
		
		    
    if (fault_array && *index >= 0 && *index < 20) {
        // Update the temp_fault_array with the new fault
			  temp_fault_array.faults[fault_type][*index] = fault_param;
        *index = *index + 1;
				n += snprintf(uartStr + n, sizeof(uartStr) - n,"----------index values is %d \r\n",*index);
				uint8_t data[sizeof(my_faults)];

        //convert fault array structure to byte array
        memcpy(data, &my_faults, sizeof(my_faults));
			  erase_flash_memory(7,VOLTAGE_RANGE_3);

        //write the complete fault array to flash memory
        write_to_flash(fault_log_address, data, sizeof(my_faults),FLASH_TYPEPROGRAM_BYTE);

        erase_flash_memory(6, FLASH_VOLTAGE_RANGE_3);
        HAL_FLASH_Unlock();
        HAL_StatusTypeDef flash_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, f1_address, fault_log_address);
        HAL_FLASH_Lock();
        //f1_address_counter++;
        
        //check the status of flash write
        if (flash_status == HAL_OK) {
            snprintf(uartStr + n, sizeof(uartStr) - n, "Flash Write: SUCCESS\n");
        } else {
            snprintf(uartStr + n, sizeof(uartStr) - n, "Flash Write: ERROR\n");
        }
			  
			}
    }
		
//function to scan connected devices on I2C 		
void I2C_Scan()
{
char Buffer[25] = {0};
uint8_t Space[] = " - ";
uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
uint8_t EndMSG[] = "Done! \r\n\r\n";
uint8_t i = 0, ret;

/*-[ I2C Bus Scanning ]-*/

HAL_UART_Transmit(&huart2, StartMSG, sizeof(StartMSG), 10000);
for(i=1; i<128; i++)
{
   ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
   if (ret != HAL_OK) /* No ACK Received At That Address */
   {
    HAL_UART_Transmit(&huart2, Space, sizeof(Space), 10000);
    }
   else if(ret == HAL_OK)
   {
   sprintf(Buffer, "0x%X", i);
   HAL_UART_Transmit(&huart2, (uint8_t *)Buffer, sizeof(Buffer), 10000);
   }
    }
    HAL_UART_Transmit(&huart2, EndMSG, sizeof(EndMSG), 10000);
    /*--[ Scanning Done ]--*/

}

SystemTime time_conversion()
{
    // SystemTime time_converted
    SystemTime time_converted;
    static uint32_t prev_ticks = 0;
    uint32_t current_ticks = HAL_GetTick();

    // Calculate elapsed time in milliseconds
    uint32_t elapsed_ms = current_ticks - prev_ticks;

    // Handle rollover/reset of milliseconds
    if (elapsed_ms >= 1000) {
        elapsed_ms %= 1000;
    }

    // Update previous tick value
    prev_ticks = current_ticks;

    // Convert elapsed_ms into ms, s, m, h
    time_converted.ticks = current_ticks;
    uint32_t elapsed_seconds = current_ticks / 1000;
    time_converted.systemtime_s = elapsed_seconds % 60;
    uint32_t elapsed_minutes = elapsed_seconds / 60;
    time_converted.systemtime_m = elapsed_minutes % 60;
    uint32_t elapsed_hours = elapsed_minutes / 60;
    time_converted.systemtime_h = elapsed_hours % 24; // Assuming you want a 24-hour format

    char time_converted_buffer[150];
    snprintf(time_converted_buffer, sizeof(time_converted_buffer), "time converted is : %02d:%02d:%02d:%03d\r\n",
             time_converted.systemtime_h, time_converted.systemtime_m,
             time_converted.systemtime_s, time_converted.ticks);
    HAL_UART_Transmit(&huart2, (uint8_t*)time_converted_buffer, strlen(time_converted_buffer), 100);

    return time_converted;
}

// Function to check if the system time is valid
bool isValidSystemTime(const SystemTime *systemTime) {
    // Check for valid hours, minutes, seconds, and ticks
    if (systemTime->systemtime_h >= 0 && systemTime->systemtime_h < 24 &&
        systemTime->systemtime_m >= 0 && systemTime->systemtime_m < 60 &&
        systemTime->systemtime_s >= 0 && systemTime->systemtime_s < 60 &&
        systemTime->ticks >= 0) {
        return true; // Valid time
    } else {
        return false; // Invalid time
    }
}
SystemTime read_systemtime_in_flashmemory()
{
    const uint32_t system_time_address = 0x08020000;
    SystemTime read_system_time;

    if (isValidSystemTime((SystemTime*)system_time_address)) {
        read_system_time = *(SystemTime*)system_time_address; // Read SystemTime from flash memory

        // Check for invalid time values
        if (read_system_time.systemtime_h < 0 || read_system_time.systemtime_h > 23 ||
            read_system_time.systemtime_m < 0 || read_system_time.systemtime_m > 59 ||
            read_system_time.systemtime_s < 0 || read_system_time.systemtime_s > 59) {
            // Initialize to 00:00:00.000 if any value is invalid
            read_system_time.systemtime_h = 0;
            read_system_time.systemtime_m = 0;
            read_system_time.systemtime_s = 0;
            read_system_time.ticks = 0;
        }
    } else {
        // Initialize to 00:00:00.000 if the system time in flash is not valid
        read_system_time.systemtime_h = 0;
        read_system_time.systemtime_m = 0;
        read_system_time.systemtime_s = 0;
        read_system_time.ticks = 0;
    }

    char buffer[150];
    // Format and transmit system time
    snprintf(buffer, sizeof(buffer), "System Time read: %02d:%02d:%02d:%03d\r\n",
             read_system_time.systemtime_h, read_system_time.systemtime_m,
             read_system_time.systemtime_s, read_system_time.ticks);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);

    return read_system_time;
}


//function to read the system time from flash memory
void write_systemtime_in_flashmemory()
{
    const uint32_t system_hours_address = 0x08020000;
    uint32_t size_of_system_structure = sizeof(system_time);
    uint8_t sytemtime_byte_array[size_of_system_structure];
	  static bool read_time=false;
	  static uint32_t total_seconds_flash=0;
	  static uint32_t total_seconds=0;
    static uint32_t total_seconds_current;
    // Read system time from flash memory
	  SystemTime system_time_fromflash ;

	  if (!read_time)
		{
		system_time_fromflash = read_systemtime_in_flashmemory();	
    char read_system_time_fromflash_buffer[150];
    snprintf(read_system_time_fromflash_buffer, sizeof(read_system_time_fromflash_buffer), 
             "!!!!! System Time received from flash: %02d:%02d:%02d:%03d\r\n",
             system_time_fromflash.systemtime_h, system_time_fromflash.systemtime_m,
             system_time_fromflash.systemtime_s, system_time_fromflash.ticks);
    HAL_UART_Transmit(&huart2, (uint8_t*)read_system_time_fromflash_buffer, strlen(read_system_time_fromflash_buffer), 100);			
	  
		// Convert system times to seconds
		total_seconds_flash = system_time_fromflash.systemtime_h * 3600 + system_time_fromflash.systemtime_m * 60 + system_time_fromflash.systemtime_s;
		char total_seconds_flash_buffer[150];
		snprintf(total_seconds_flash_buffer, sizeof(total_seconds_flash_buffer), "!!!!! Total seconds flash system time: %u\r\n", total_seconds_flash);
		HAL_UART_Transmit(&huart2, (uint8_t*)total_seconds_flash_buffer, strlen(total_seconds_flash_buffer), 100) ; 		
			
		read_time=true;
		}	
			
	  //get the current system time
	  SystemTime current_time=time_conversion();
    char system_time_buffer[150];
    snprintf(system_time_buffer, sizeof(system_time_buffer), " --- current system time: %02d:%02d:%02d:%03d\r\n",
             system_time.systemtime_h, system_time.systemtime_m,
             system_time.systemtime_s, system_time.ticks);
    HAL_UART_Transmit(&huart2, (uint8_t*)system_time_buffer, strlen(system_time_buffer), 100);

    // Calculate total seconds for current system time
	  total_seconds_current = system_time.systemtime_h * 3600 + system_time.systemtime_m * 60 + system_time.systemtime_s;
		char total_seconds_current_buffer[150];
		snprintf(total_seconds_current_buffer, sizeof(total_seconds_current_buffer), "--- Total seconds current system time: %u\r\n", total_seconds_current);
		HAL_UART_Transmit(&huart2, (uint8_t*)total_seconds_current_buffer, strlen(total_seconds_current_buffer), 100);

    // Add the seconds
    total_seconds = total_seconds_flash + total_seconds_current;
		char total_seconds_buffer[150];
		snprintf(total_seconds_buffer, sizeof(total_seconds_buffer), "--- Total seconds total system time: %u\r\n", total_seconds);
		HAL_UART_Transmit(&huart2, (uint8_t*)total_seconds_buffer, strlen(total_seconds_buffer), 100);    
		
    // Convert back to hours, minutes, and seconds
     total_system_time.systemtime_h = total_seconds / 3600;
     total_seconds %= 3600;
     total_system_time.systemtime_m = total_seconds / 60;
     total_system_time.systemtime_s = total_seconds % 60;

    char total_system_time_buffer[150];
    snprintf(total_system_time_buffer, sizeof(total_system_time_buffer), 
             "!!!!! System Time received from flash: %02d:%02d:%02d:%03d\r\n",
             total_system_time.systemtime_h, total_system_time.systemtime_m,
             total_system_time.systemtime_s,total_system_time.ticks);
    HAL_UART_Transmit(&huart2, (uint8_t*)total_system_time_buffer, strlen(total_system_time_buffer), 100);		


    // Convert systemtime structure into the byte array
    memcpy(sytemtime_byte_array, &total_system_time, size_of_system_structure);

    // Erase the flash memory before writing
    erase_flash_memory(5, FLASH_VOLTAGE_RANGE_3); // Adjust the erase function parameters as necessary

    // Write the byte array to flash memory
    write_to_flash(system_hours_address, sytemtime_byte_array, size_of_system_structure, FLASH_TYPEPROGRAM_BYTE);

    char buffer[150];
    snprintf(buffer, sizeof(buffer), "System Time written on flash: %02d:%02d:%02d:%03d\r\n",
             total_system_time.systemtime_h, total_system_time.systemtime_m,
             total_system_time.systemtime_s, total_system_time.ticks);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
}


//function to read total cycle time from flash memory
TotalCycleTime read_total_cycle_time_in_flashmemory()
{
    
    const uint32_t total_cycle_time_address = 0x08010000;
	  TotalCycleTime read_totalcycle_time= *(TotalCycleTime*)total_cycle_time_address; // Read SystemTime from flash memory
	  char buffer[100];
	 // Format and transmit system time
    snprintf(buffer, sizeof(buffer), "Totalcycle Time read: %02d:%02d:%02d\r\n",
             read_totalcycle_time.totalcycletime_h, read_totalcycle_time.totalcycletime_m,
             read_totalcycle_time.totalcycletime_s);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
    return read_totalcycle_time;
	}	
	
void write_total_cycle_time_in_flashmemory()
{
    const uint32_t total_cycle_hours_address = 0x08010000;
	  static SystemTime start_time, end_time; 
	  
	  //read total cycle hours from flash memory
	  TotalCycleTime totalcycle_time_fromflash=read_total_cycle_time_in_flashmemory();

	 //update the total cycle hours
	 //start counting the total cycle hours when production flag is high and stop updating time when production flag is low
	   
    if (Production && (start_time.systemtime_h == 0 && start_time.systemtime_m == 0 && start_time.systemtime_s == 0)) {
	   start_time = system_time;
	  }
	
    if (!Production && (start_time.systemtime_h != 0 || start_time.systemtime_m != 0 || start_time.systemtime_s != 0)) {
        end_time = system_time;		
	 
			
		total_cycle_time.totalcycletime_h=(end_time.systemtime_h-start_time.systemtime_h)     + totalcycle_time_fromflash.totalcycletime_h ;
		total_cycle_time.totalcycletime_m=(end_time.systemtime_m-start_time.systemtime_m)     + totalcycle_time_fromflash.totalcycletime_m;
	  total_cycle_time.totalcycletime_s=(end_time.systemtime_s-start_time.systemtime_s)     + totalcycle_time_fromflash.totalcycletime_s;
   // total_cycle_time.totalcycletime_ms=(end_time.systemtime_ms-start_time.systemtime_ms)  + totalcycle_time_fromflash.totalcycletime_ms;
    
			
    uint32_t size_of_totalcycle_structure = sizeof(total_cycle_time);
    uint8_t totalcycle_byte_array[size_of_totalcycle_structure];    
	  memcpy(totalcycle_byte_array, &total_cycle_time, size_of_totalcycle_structure);


    // Erase the flash memory before writing
    erase_flash_memory(4, FLASH_VOLTAGE_RANGE_3); // Adjust the erase function parameters as necessary

    // Write the byte array to flash memory
   //	Program a word (32-bit) at a specified address
    write_to_flash(total_cycle_hours_address,totalcycle_byte_array,size_of_totalcycle_structure,FLASH_TYPEPROGRAM_BYTE);
	 //write_to_flash(fault_log_address, data, sizeof(my_faults),FLASH_TYPEPROGRAM_BYTE);
	 
	 
	  char buffer[100];
    snprintf(buffer, sizeof(buffer), "Total Cycle Time written: %02d:%02d:%02d\r\n",
                 total_cycle_time.totalcycletime_h, total_cycle_time.totalcycletime_m, 
                 total_cycle_time.totalcycletime_s);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
	  
		// Reset start_time for next cycle
    memset(&start_time, 0, sizeof(start_time));
} 

}

// continously check if the current value is greater than 0 for 5 seconds then raise fault
bool cell_current_greater_than_zero() {
	  
    uint32_t startTime = 0; //variable to hold the start time
    uint32_t threshold = 5; // 5 seconds 
    if (cell_current > 0) {
        if (startTime == 0) {
            
            startTime = system_time.systemtime_s ;
        } 
				
				else if (system_time.systemtime_s - startTime > threshold) {
					
            // cell_current has been greater than zero for more than 5 seconds
            char buffer[120]=("Current greater than zero for more than 5 seconds\n");
					  HAL_UART_Transmit(&huart2,(uint8_t *)buffer,sizeof(buffer),100);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
            // Reset startTime
            startTime = 0;
					  return true;
        }
    } 
		
		
		else {
        // Reset startTime if cell_current is not greater than zero
			  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
        startTime = 0;
			  return false;
    }
		return 0;
}



//salt tube brine conductivity
bool salt_tube_filled_time()
{

}










//fucntion to check low pressure for 2 hours
bool low_pressure_check()
{
  // check if the pressure status is low for more than 2 hours then
	uint32_t startTime=0;
	uint32_t threshold = 2; // 2 hours
	if(!monitor_pressure())
	{
		
	if (startTime == 0) { 
  startTime = system_time.systemtime_h ;
  } 
	
	else if (system_time.systemtime_h - startTime > threshold) {
            char buffer[120]=("Pressure is low for more than 2 hours \n");
					  HAL_UART_Transmit(&huart2,(uint8_t *)buffer,sizeof(buffer),100);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
            // Reset startTime
            startTime = 0;
					  return true;
        }	
	}

	else {
        // Reset startTime if pressure is not low for more than 2 hours
			  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
        startTime = 0;
			  return false;
    }
   return false;
}



void idle_state()
{
	//In idle state function.IT will run continously when the device starts
	
   //Turn off all output signals off
	// Turn of cell power signal off
	// monitor cell current.Equal to 0 Amps
 	static bool cell_power_status =false;

	//For tank full
	static bool Autocycle=false;
	
	//for Low pressure condition
	static uint32_t low_pressure_counter=0;
	
	static SystemTime low_pressure_system_hours;
	static TotalCycleTime low_pressure_cycle_hours;
	
	uint32_t cellfill_solenoid_time = 0;
	static bool Iscellfillsolenoid = false;
	
	
	//for tank fill timeexceed
	static bool Tankfill_time=false;
	static bool Tank_fill_timer_expired=false;
	static bool Tankfill_timeexceed=false;
	static uint32_t Tankfill_timeexceed_counter=0;
	static bool Tankfill_time_trend_correctpasscode=false;


  static SystemTime Tankfill_time_exceed_system_hours;
	static TotalCycleTime Tankfill_time_exceed_cycle_hours;
	
	
	static uint32_t Tankfill_time_exceed_hours=0,Tankfill_time_exceed_mins=0;
	static uint32_t Tankfill_timeexceed_timestamps[3] = {0};
	static uint32_t current_time_in_mins=0;
	static uint32_t Tankfill_timetrend_counter=0;
	static uint32_t Tankfill_timetrend_code=0;
	static uint32_t Tankfill_timetrend_passcode=0;
	uint32_t user_Tankfill_timetrend_passcode=0;


	static bool prev_Tankfill_time=false,prev_Tankfill_timeexceed=false;
	
	uint8_t current_value=0;
  current_value=cell_current();
	
	if (current_value==0)
	{
	
	//system ready 
  if(Stop_signal && !Production && !Masterflag)
	{
	Idel_systemready_state=true;	
	Idel_tanksfull_state=false;
	Idel_tankfill_timeexceeded_state=false;
	Idel_lowpressure_state=false;
		
	Autocycle=false;
	
	}
	
	//Tank full state
	if (HOCL_tank_level() && NAOH_tank_level())
	{
	Idel_tanksfull_state=true;
		
	Idel_systemready_state=false;	
	Idel_tankfill_timeexceeded_state=false;
	Idel_lowpressure_state=false;
		
	Autocycle=true;
	}
	
	//LOW Pressure state
	
	if (!monitor_pressure())
	{
	Idel_lowpressure_state=true;	
	Idel_systemready_state=false;	
	Idel_tankfill_timeexceeded_state=false;
	Idel_lowpressure_state=false;		
		
		
	low_pressure_counter++;
	low_pressure_system_hours=system_time;
	low_pressure_cycle_hours=total_cycle_time;
	//Turn off cell fill solenoid off for 30 seconds and on for 5 seconds

if (!Iscellfillsolenoid && HAL_GetTick() - cellfill_solenoid_time >= 30000) {
    // Turn on the solenoid after it has been off for 30 seconds
    cellfill_solenoid(true);
    Iscellfillsolenoid = true;
    cellfill_solenoid_time = system_time.ticks; // Update the last change time
} else if (Iscellfillsolenoid && system_time.ticks - cellfill_solenoid_time >= 5000) {
    // Turn off the solenoid after it has been on for 5 seconds
    cellfill_solenoid(false);
    Iscellfillsolenoid = false;
    cellfill_solenoid_time = system_time.ticks; // Update the last change time
}
	
	

	}	
	
	//Tank fill time exceed
	if (Tankfill_time && Tankfill_timeexceed && !Resume_signal && !Tankfill_time_trend_correctpasscode )
 	{
		Idel_tankfill_timeexceeded_state=true;
		Idel_lowpressure_state=false;		
		Idel_systemready_state=false;	
		Idel_lowpressure_state=false;		
		
		Tankfill_timeexceed=true;
		Tankfill_timeexceed_counter++;
		Tankfill_time_exceed_system_hours=system_time;
		Tankfill_time_exceed_cycle_hours=total_cycle_time;
		 
      //read_system time from flash memory
      //SystemTime read_sys_time=read_systemtime_in_flashmemory();
	  	Tankfill_time_exceed_hours=system_time.systemtime_h;
		  current_time_in_mins = system_time.systemtime_m;// system mins
		
    // Shift timestamps and add the new one
    for (int i = 3 - 1; i > 0; i--) {
        Tankfill_timeexceed_timestamps[i] = Tankfill_timeexceed_timestamps[i - 1];
    }
    Tankfill_timeexceed_timestamps[0] = current_time_in_mins;
		
		if (Tankfill_timeexceed_counter >= 3) {
			
    // Check if three occurrences happened within the last 48 hours (2880 minutes)
    if (Tankfill_timeexceed_timestamps[0] - Tankfill_timeexceed_timestamps[3 - 1] <= 2880) {
       //printf("Tank fill time exceeded 3 times in 48 hours\n");
			
			Tankfill_timetrend_code=system_time.systemtime_h-total_cycle_time.totalcycletime_h;
			Tankfill_timetrend_passcode=Tankfill_timetrend_code+177;
			Tankfill_timeexceed_counter=0;

        }
    }
		
		Tankfill_timeexceed=false;
		if (Tankfill_timetrend_passcode==user_Tankfill_timetrend_passcode)
		{
		Tankfill_time_trend_correctpasscode=true;
		}
	}
	
}
}
		
		
		







/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char tickStr[20];
	char tempStr[32];
	uint32_t lastSaveTime = 0;
	uint32_t currentTime =0;
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
	I2C_Scan();
  
	
	SystemTime start_time;
	start_time=time_conversion();
  //erase_flash_memory(5, FLASH_VOLTAGE_RANGE_3);
  //write_systemtime_in_flashmemory();

  //testing for fault
  fault_circuit_board.counter = 1;
  fault_circuit_board.cycle_hours = total_cycle_hours;
	fault_circuit_board.fault_status=true;
	fault_circuit_board.system_hours = system_hours;
	fault_circuit_board.real_time_clock=4;
	
  update_fault(&my_faults,FAULT_TYPE_CKT_B , &fault_circuit_board_count, fault_circuit_board);

  //calculate the cycle hours
	
		
		
    

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 

	 Production=true;
   currentTime=system_time.systemtime_s;	 
		//write_systemtime_in_flashmemory();
		
	 //update the system_time structure instance
	 system_time=time_conversion();
	 
	 if (Production)
	 {
	  cycle_time.totalcycletime_h=system_time.systemtime_h - start_time.systemtime_h;
	  cycle_time.totalcycletime_m=system_time.systemtime_m - start_time.systemtime_m;
    cycle_time.totalcycletime_s=system_time.systemtime_s - start_time.systemtime_s;
	  

	 }
	 
   //write the system hours in flashmemory after every 1 minute
	if ((currentTime - lastSaveTime) >= 6) //write after 1 minute
    { 
			  uint8_t buffer[150]=" !!!! writing time into flash memory";
			  HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
        write_systemtime_in_flashmemory();
        lastSaveTime = currentTime; // Update the last save time
    }
	 
		
		


		//read_sys_cycle_hours_in_flashmemory();
		measure_current();
		HAL_GetTick();
		salt_pump();
    drain_solenoid();
		NAOH_tank_level();
		HOCL_tank_level();
		salt_tube_bottom();
		salt_tube_top();
    sprintf(tickStr, "HAL_Tick: %d\n\r", HAL_GetTick());
    HAL_UART_Transmit(&huart2, (uint8_t *)tickStr, sizeof(tickStr), 300);
    counter++;
		
		if (!Start_signal)	
		{
		idle_state();
		}
		
		
		if (Start_signal && !Idel_systemready_state && !Idel_tanksfull_state && !Idel_tankfill_timeexceeded_state && !Idel_lowpressure_state)
		{

		}
		
		
		


	

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  sConfig.Channel = ADC_CHANNEL_9;
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
  sTime.Hours = 0x11;
  sTime.Minutes = 0x52;
  sTime.Seconds = 0x10;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 0x27;
  sDate.Year = 0x23;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x15;
  sAlarm.AlarmTime.Minutes = 0x10;
  sAlarm.AlarmTime.Seconds = 0x10;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
  sAlarm.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
void MX_USART2_UART_Init(void);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */



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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|led1_Pin|led2_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, pressure_Pin|cellfill_solenoid_Pin|GPIO_PIN_4, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PA4 led1_Pin led2_Pin PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|led1_Pin|led2_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : pressure_Pin cellfill_solenoid_Pin PB4 */
  GPIO_InitStruct.Pin = pressure_Pin|cellfill_solenoid_Pin|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
	HAL_UART_Transmit(&huart2,rx_data,sizeof(rx_data),10);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */



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
