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
#include "main.h"
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


//global variables for functions
float voltage=0,current=0;
float flow=0;	
float pH_NAOH_value=0,pH_HOCL_value=0;
float flow_NAOH_value=0,flow_HOCL_value=0,flow_saltpump_value=0;
float brine_conc_value=0;



//flags for each function to monitor faults
bool cell_voltage_fault=false;
bool flow_fault=false;
bool low_pressure_fault=false;
bool ph_HOCL_fault=false;
bool ph_NAOH_fault=false;
bool brine_conductivity=false;
bool no_board_power_fault=false;

time_t unixTime;
struct tm timeinfo;
//
uint32_t drain_solenoidTime = 0;
uint8_t  drain_solenoidState = 0; 
uint32_t drain_solenoidTargetTime = 0;

/// variable for drain solenoid global cycle
uint8_t  drain_solenoidflag=0;  // 
uint8_t  cellcurrent_starttime=0;
uint32_t read_unixtime;
uint32_t write_unixtime;
uint32_t counter;

//for flash memory
static uint32_t lastSaveTime = 0;
uint32_t system_hours = 0;
uint32_t total_cycle_hours = 0;
uint32_t hours = 0, minutes = 0;

//structure for cell A,B and C

typedef struct
{
 float cell_voltage;
 float cell_current;
} Measurements ;
Measurements main_cell;

//structure for cell A,B and C
typedef struct
{
 float cellA_voltage;
 float cellA_current;
 float cellB_voltage;
 float cellB_current;
 float cellC_voltage;
 float cellC_current;
} CellsMeasurement;

//system hours structure
typedef struct{
    uint32_t systemtime_s;
	  uint32_t systemtime_m;
	  uint32_t systemtime_h;
	  uint32_t ticks;
} SystemTime;


SystemTime system_time,total_system_time, start_time;

//total cycle hours structure

typedef struct{
    uint32_t cycletime_s;
	  uint32_t cycletime_m;
	  uint32_t cycletime_h;
} TotalCycleTime;

TotalCycleTime total_cycle_time,current_cycle_time,HOCL_cycle_time,NAOH_cycle_time;

typedef enum {
    FAULT_TYPE_CKT_BOARD,
    FAULT_TYPE_UNDER_CURRENT,
    FAULT_TYPE_OVER_CURRENT,
    FAULT_TYPE_SALTTUBE_LEVEL,
    FAULT_TYPE_PRESSURE_LOSS,
    FAULT_TYPE_FLOW_HOCL,
    FAULT_TYPE_FLOW_NAOH,
    FAULT_TYPE_FLOW_SALTPUMP,
    FAULT_TYPE_PH_HOCL,
    FAULT_TYPE_PH_NAOH,
    FAULT_TYPE_BRINE_CONDUCTIVITY,
    FAULT_TYPE_CELL_CONTROL,
    NUM_FAULT_TYPES
} fault_type_e;

typedef struct {
 uint32_t counter;
 TotalCycleTime	cycle_hours;
 bool fault_status;
 SystemTime system_hours;
 RTC_DateTypeDef real_time_clock; 
	  
}__attribute__((packed)) fault_t;

typedef struct {
	
    fault_t faults[NUM_FAULT_TYPES][20];
} __attribute__((packed)) fault_array_t; 



// Initialize my_faults
fault_array_t my_faults;

// fault_t fault_circuit_board,fault_under_current,fault_over_current;
// fault_t fault_salt_tube_level,fault_pressure_loss,fault_cell_control;
// fault_t fault_salt_pump_hocl_flowrate,fault_salt_pump_naoh_flowrate;
// fault_t fault_ph_hocl,fault_ph_naoh,fault_brine_conductivity;

//counter for each fault type
int fault_circuit_board_count=0;
int fault_under_current_count=0;
int fault_over_current_count=0;
int fault_salt_tube_level_count=0;
int fault_pressure_loss_count=0;
int fault_hocl_flowrate_count=0;
int fault_naoh_flowrate_count=0;
int fault_salt_pump_flowrate_count=0;
int fault_ph_hocl_count=0;
int fault_ph_naoh_count=0;
int fault_brine_conductivity_count=0;
int fault_cellcontrol_count=0;

//flags for each fault type
bool fault_circuit_board_status=false;
bool fault_under_current_status=false;
bool fault_over_current_status=false;
bool fault_salt_tube_level_status=false;
bool fault_pressure_loss_status=false;
bool fault_hocl_flowrate_status=false;
bool fault_naoh_flowrate_status=false;
bool fault_saltpump_flowrate_status=false;
bool fault_ph_hocl_status=false;
bool fault_ph_naoh_status=false;
bool fault_brine_conductivity_status=false;
bool fault_cellcontrol_status=false;



//tank fill time 
uint32_t TankFill_time=20; //in seconds


//receive data on uart
uint8_t rx_data[100];

// for production
bool MasterFault=false;
bool Stop_signal=false;
bool Start_signal=false;
bool Resume_signal=false;
bool Production=false;


//flags for each state in Idle
bool Idle =false;
bool Idle_systemready_state=false;
bool Idle_tanksfull_state=false;
bool Idle_tankfill_timeexceeded_state=false;
bool Idle_lowpressure_state=false;
bool Tankfill_time_trend_correctpasscode=false;

//flags for production state
bool Priming_sequence=false;
bool productionON_sequence=false;
bool Flushing_sequence=false;
bool Tank_fill_timer_expired=false;
bool overcurrent=false,undercurrent=false,lowcurrent_trend=false;		



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

time_t getUnixTime(void)
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
Measurements measure_current_voltage(void) {
    uint16_t readvalue;
    const int num_samples = 10;
    float sum_readvalue = 0;
    float average_readvalue;
    const float sensitivity = 0.1;

    // Take 10 samples of analog reading
    for (int i = 0; i < num_samples; i++) {
        HAL_ADC_PollForConversion(&hadc1, 1000);
        readvalue = HAL_ADC_GetValue(&hadc1);
        sum_readvalue += readvalue;
    }

    // Calculate average of the readings
    average_readvalue = sum_readvalue / num_samples;

    // Convert average ADC value to voltage
     main_cell.cell_voltage = average_readvalue * 3.3 / 4096;

    // Adjust for tolerance in voltage divider resistor & ADC accuracy
    if (main_cell.cell_voltage != 2.5) {
        main_cell.cell_voltage *= 1.035;
    }

    // Calculate current based on average voltage
    main_cell.cell_current = (main_cell.cell_voltage - 2.5) / sensitivity;

    // Print voltage on uart
    char voltage_value[50];
    sprintf(voltage_value, "Average voltage is %f \t\n\r ", main_cell.cell_voltage);
    HAL_UART_Transmit(&huart2, (uint8_t*)voltage_value, sizeof(voltage_value), 300);

    // Print current on uart
    char current_value[50];
    sprintf(current_value, "Average current is %f \t\n\r ", main_cell.cell_current);
    HAL_UART_Transmit(&huart2, (uint8_t*)current_value, sizeof(current_value), 300);

    return main_cell;
}


bool compare_current_with_reference(float current_reference) {
    float average_current = measure_current_voltage().cell_current;

    if (average_current < current_reference * 0.2)

		{ // Check if 20% below reference
			  uint8_t buffer[100]="Current value is 20 percent below the reference value.\r\n" ;
			  HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
			  return true;
    }
		else 
		{
        uint8_t buffer[100]="Current value is within acceptable range.\r\n" ;
			  HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
   			printf("Current value is within acceptable range.\n");
			  return false;
    }
		return false;
}


void current_below_threshold_for_duration()
{
  const uint32_t threshold_current_value=7;
	const uint32_t time_threshold=10;
	const int duration= 10;
	float average_current = measure_current_voltage().cell_current;
	static uint32_t start_time_m=0,start_time_s=0;
	static uint32_t undercurrent_standby_counter=0;
	static uint32_t elapsed_time=0;
	const  uint32_t undercurrent_time_threshold=3600;
	char buffer[100];
	SystemTime under_current_standby_system_time;
	TotalCycleTime under_current_standby_cycle_time;
	
	if (average_current<threshold_current_value)
	{
  
	//turn off  the  cell fill solenoid 
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);		
		
	if (start_time_m==0 && start_time_s==0)
		{
		start_time_m = system_time.systemtime_m;
		start_time_s = system_time.systemtime_s;
		sprintf(buffer, " ----start time of cell current <7 is %d------ \r\n", start_time_s);					  
		HAL_UART_Transmit(&huart2,(uint8_t*)buffer,sizeof(buffer),160);
    
    //increment the under current counter
		undercurrent_standby_counter++;
		under_current_standby_system_time=system_time;
		under_current_standby_cycle_time=current_cycle_time;
		}
		
		//convert into seconds 
	  elapsed_time = (system_time.systemtime_m * 60 + system_time.systemtime_s) - (start_time_m * 60 + start_time_s);
       //check if the occur 5 times in an hour
	 if (undercurrent_standby_counter==5 && elapsed_time < undercurrent_time_threshold ) 
	 {
	 sprintf(buffer, "undercurrent counter value is 5 production off \r\n");
   HAL_UART_Transmit(&huart2, (uint8_t *)buffer, sizeof(buffer), 100);
	 undercurrent_standby_counter=0;
	 elapsed_time=0;
	 start_time_s = 0;
	 start_time_m = 0;
   undercurrent=true;

   //
   //exit from production
   productionON_sequence=false;
   Flushing_sequence=true;

	// move to undercurrent fault state
	 
	 }
	 
	 //check if value is less than threshold for more than 10 seconds
	 if (system_time.systemtime_s - start_time_s >= time_threshold) {
		        sprintf(buffer, "Production off \r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer, sizeof(buffer), 100);
		        undercurrent=true;		 
            // Reset start_time_s and start_time_m
            start_time_s = 0;
		        start_time_m = 0;
            elapsed_time=0;		 
		        undercurrent_standby_counter=0;

            //exit from production
            productionON_sequence=false;
            Flushing_sequence=true;

					  // move to undercurrent fault state
                		 
	}
	 
}	else
	{
	//turn on the cell fill solenoid 
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);	
	}
}


void current_high_threshold_for_duration()
{
  const float threshold_current_value=17.0;
	const uint32_t time_threshold=5;
  float average_current = measure_current_voltage().cell_current;
	static uint32_t start_time_m=0,start_time_s=0;
	static uint32_t overcurrent_standby_counter=0;
	static uint32_t elapsed_time=0;
	char buffer[100];
	SystemTime over_current_standby_system_time;
	TotalCycleTime over_current_standby_cycle_time;
	
	if (average_current>threshold_current_value)
	{
	
	if (start_time_m==0 && start_time_s==0)
		{
		start_time_m = system_time.systemtime_m;
		start_time_s = system_time.systemtime_s;
		sprintf(buffer, " ----start time of cell current >17 is %d------ \r\n", start_time_s);					  
		HAL_UART_Transmit(&huart2,(uint8_t*)buffer,sizeof(buffer),160);
    
		}
		//convert into seconds 
	    elapsed_time = (system_time.systemtime_m * 60 + system_time.systemtime_s) - (start_time_m * 60 + start_time_s);
        
		//check if value is greater than threshold for more than 5 seconds
    if (elapsed_time> time_threshold)
	 {
	  sprintf(buffer, "overcurrent condition occurs \r\n");
      HAL_UART_Transmit(&huart2, (uint8_t *)buffer, sizeof(buffer), 100);
	   overcurrent_standby_counter++;
		 overcurrent=true;
	   elapsed_time=0;
	   start_time_s = 0;
	   start_time_m = 0;
     productionON_sequence=false;
     Flushing_sequence=true;
	// move to overcurrent fault state
	 
	 }	  
}
}


CellsMeasurement measure_cells_current_voltages()
{   
	  // take the 10 samples for each each of the cell A ,B and C and calculate the current and voltages values. and update the structure
	  CellsMeasurement cells;
    uint16_t readvalue;
    const int num_samples = 10;        
    uint16_t sum_readvalue = 0;
    float average_readvalue;
    const float sensitivity = 0.1;
	
    // calculation for  Cell A
    // Take 10 samples of analog reading
    for (int i = 0; i < num_samples; i++) {
        HAL_ADC_PollForConversion(&hadc1, 1000);
        readvalue = HAL_ADC_GetValue(&hadc1);
        sum_readvalue += readvalue;
    }	
    // Calculate average of the readings
    average_readvalue = sum_readvalue / num_samples;
    // Convert average ADC value to voltage
    cells.cellA_voltage = average_readvalue * 3.3 / 4096;

    // Adjust for tolerance in voltage divider resistor & ADC accuracy
    if (cells.cellA_voltage != 2.5) {
        cells.cellA_voltage *= 1.035;
    }
    // Calculate current based on average voltage
    cells.cellA_current= (cells.cellA_voltage - 2.5) / sensitivity;
    
		sum_readvalue=0;
		average_readvalue=0;
		

    // calculation for Cell B
    // Take 10 samples of analog reading
    for (int i = 0; i < num_samples; i++) {
        HAL_ADC_PollForConversion(&hadc1, 1000);
        readvalue = HAL_ADC_GetValue(&hadc1);
        sum_readvalue += readvalue;
    }	
    // Calculate average of the readings
    average_readvalue = sum_readvalue / num_samples;
    // Convert average ADC value to voltage
    cells.cellB_voltage = average_readvalue * 3.3 / 4096;

    // Adjust for tolerance in voltage divider resistor & ADC accuracy
    if (cells.cellB_voltage != 2.5) {
        cells.cellB_voltage *= 1.035;
    }
    // Calculate current based on average voltage
    cells.cellB_current = (cells.cellB_voltage - 2.5) / sensitivity;

		sum_readvalue=0;
		average_readvalue=0;
		
    // calculation for Cell C
    // Take 10 samples of analog reading
    for (int i = 0; i < num_samples; i++) {
        HAL_ADC_PollForConversion(&hadc1, 1000);
        readvalue = HAL_ADC_GetValue(&hadc1);
        sum_readvalue += readvalue;
    }	
    // Calculate average of the readings
    average_readvalue = sum_readvalue / num_samples;
    // Convert average ADC value to voltage
    cells.cellC_voltage = average_readvalue * 3.3 / 4096;

    // Adjust for tolerance in voltage divider resistor & ADC accuracy
    if (cells.cellC_voltage != 2.5) {
        cells.cellC_voltage *= 1.035;
    }
    // Calculate current based on average voltage
    cells.cellC_current = (cells.cellC_voltage - 2.5) / sensitivity;
		
    // Print voltage on uart
    char voltage_value[150];
    sprintf(voltage_value, "Average voltage of cell A %f , cell B %f,cell C %f \t\n\r ", cells.cellA_voltage,cells.cellB_voltage,cells.cellC_voltage);
    HAL_UART_Transmit(&huart2, (uint8_t*)voltage_value, sizeof(voltage_value), 300);

    // Print current on uart
    char current_value[150];
    sprintf(current_value, "Average current of cell A %f , cell B %f,cell C %f \t\n\r ", cells.cellA_current,cells.cellB_current,cells.cellC_current);
    HAL_UART_Transmit(&huart2, (uint8_t*)current_value, sizeof(current_value), 300);		
    
    return cells;		
}


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



bool controlboard_power()
{
  //read the control board power on pin PC15
	GPIO_PinState status =HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15);

  //print the the salt tube bottom sensor on uart
	if (status ==GPIO_PIN_SET)
	{
		  uint8_t tx_buffer[100]=" control board power is high \r\n"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	else
	{
		  uint8_t tx_buffer[100]=" control board power is low \r\n"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	return status;
}



// monitor salt tube bottom sensor
bool salt_tube_bottom()
{
  //read the salt tube bottom sensor on pin PA0
	GPIO_PinState status =HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11);

  //print the the salt tube bottom sensor on uart
	if (status ==GPIO_PIN_SET)
	{
		  uint8_t tx_buffer[100]=" salt tube bottom sensor is high \r\n"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	else
	{
		  uint8_t tx_buffer[100]=" salt tube bottom sensor is low \r\n"; 
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
		  uint8_t tx_buffer[100]=" salt tube top sensor is high \r\n"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	else
	{
		  uint8_t tx_buffer[100]=" salt tube top sensor is low \r\n"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	return status;
}

//function to fill if the salt tube
bool fill_salt_tube()
{
    static uint32_t fill_time_limit = 90;
    static uint32_t start_time_s, start_time_m;
    static bool filling_started = false;
    static uint32_t elapsed_time=0;
    char tx_buffer[150];
    int n = sprintf(tx_buffer, "----In fill salt tube function--- start time is :%d:%d system time seconds are :%d\r\n", start_time_m, start_time_s, system_time.systemtime_s);
    HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, sizeof(tx_buffer), 130);

    if (!salt_tube_bottom() && !salt_tube_top())
    {
        if (!filling_started)
        {
            start_time_s = system_time.systemtime_s;
            start_time_m = system_time.systemtime_m; // Set the start time when filling starts
            n = sprintf(tx_buffer, "------- start time is :%d:%d \r\n", start_time_m, start_time_s);
            filling_started = true;
        }

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET); // Start filling
        uint8_t buff[100] = "Salt tube filling started \r\n";
        HAL_UART_Transmit(&huart2, buff, sizeof(buff), 120);

        // Calculate elapsed time in seconds
        elapsed_time = (system_time.systemtime_m * 60 + system_time.systemtime_s) - (start_time_m * 60 + start_time_s);
        n = sprintf(tx_buffer, "elapsed time is :%d\r\n", elapsed_time);
        //HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, sizeof(tx_buffer), 150);

        // Check if the salt tube is filled within 90 seconds
        if (salt_tube_top() && elapsed_time <= fill_time_limit)
        {
            
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET); //stop filling as salt tube is filled
					  filling_started = false;
            return true;
        }
        else if (elapsed_time > fill_time_limit)
        {
            uint8_t buffer[100] = "-----Salt tube not filled in 90 seconds------\r\n";
            HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 100);
					  //elapsed_time=0;
        }
    }

    // Stop filling when both sensors are high
    if (salt_tube_bottom() && salt_tube_top())
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET); // Stop filling
        uint8_t buff[100] = "Salt tube filling stopped \r\n";
        //HAL_UART_Transmit(&huart2, buff, sizeof(buff), 50);
        filling_started = false;
			  start_time_s=0;
			  start_time_m=0;
        return false;
    }

    // If function reaches here, it means filling wasn't successful within the time limit
    return false;
}



//function to monitor HOCL tank level
bool HOCL_tank_level()
{
  //read the HOCL tank level on pin PA0
	GPIO_PinState status =HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);

  //print the tank level on uart
	if (status ==GPIO_PIN_SET)
	{
		  uint8_t tx_buffer[100]=" HOCL_tank_level is high \r\n"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	else
	{
		  uint8_t tx_buffer[100]=" HOCL_tank_level is low \r\n"; 
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
		  uint8_t tx_buffer[100]=" pressure is high \r\n"; 
      HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), 100);
	}
	else
	{
		  uint8_t tx_buffer[100]=" pressure is low \r\n"; 
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

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // Turn on solenoid control pin
      drain_solenoidState = 1;

      // Set the target time for 1 minute
      drain_solenoidTargetTime = HAL_GetTick() + 5000; //  5 seconds in milliseconds
    }
  }

  // Check if 5 sec have passed
  if (drain_solenoidState == 1 && HAL_GetTick() >= drain_solenoidTargetTime)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // Turn off solenoid control pin
    drain_solenoidState = 0;
    drain_solenoidTime = HAL_GetTick();
  }
}

//function to start communication with HOCL flow controller   
void HOCL_flow_controller_initialization()
{
	//for HOCL flow controller 
  if(HAL_I2C_IsDeviceReady(&hi2c1,0X12,1,100)==HAL_OK)
	{
  uint8_t tx_buffer[100]="Communication started on I2C with HOCL_flow_controller \r\n"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);

}
	else
	{
	  uint8_t tx_buffer[100]="Error in Communicating with HOCL_flow_controller on I2C \r\n"; 
	  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	}
}

//function to start communication with NAOH flow controller   
void NAOH_flow_controller_initialization()
{
	//for NAOH flow controller 
  if(HAL_I2C_IsDeviceReady(&hi2c1,0X12,1,100)==HAL_OK)
	{
  uint8_t tx_buffer[100]="Communication started on I2C with NAOH_flow_controller \r\n"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);

}
	else
	{
	  uint8_t tx_buffer[100]="Error in Communicating with NAOH_flow_controller on I2C \r\n"; 
	  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	}
}

void NAOH_flow_controller()
{
  // send data to the NAOH flow controller command 



}

void HOCL_flow_controller()
{
  // send data to the HOCL flow controller command 

}


void salt_pump_flow_controller()
{
  // send data to the saltpump flow controller command 

}

//function to start I2C communication with ph sensor for NAOH on I2C
void pH_NAOH_initialization()
{ 
	//for NAOH ph sensor
  if(HAL_I2C_IsDeviceReady(&hi2c1,0X65,1,100)==HAL_OK)
	{
  uint8_t tx_buffer[100]="Communication started on I2C with pH NAOH sensor\r\n"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);

}
	else
	{
	  uint8_t tx_buffer[100]="Error in Communicating with pH NAOH sensor on I2C \r\n"; 
	  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	}
	
}

//function to start I2C communication with ph sensor for HOCL on I2C
void pH_HOCL_initialization()

{
  if(HAL_I2C_IsDeviceReady(&hi2c1,0X65,1,100)==HAL_OK)
	{
  uint8_t tx_buffer[100]="Communication started on I2C with pH HOCL sensor\r\n"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	
}
	else
	{
	uint8_t tx_buffer[100]="Error in Communicating with pH HOCL sensor on I2C \r\n"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);

	}
}

void salttube_brineconductivity_initialization()
{
  if(HAL_I2C_IsDeviceReady(&hi2c1,0X65,1,100)==HAL_OK)
	{
  uint8_t tx_buffer[100]="Communication started on I2C with salt tube brine conductivity sensor\r\n"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	
}
	else
	{
	uint8_t tx_buffer[100]="Error in Communicating with pH salt tube brine conductivity sensor on I2C \r\n"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);

	}
}


//function to start I2C communication with flowmeter NAOH on I2C
void measure_flow_NAOH_initialization()
{

  if(HAL_I2C_IsDeviceReady(&hi2c1,0X01,1,100)==HAL_OK)
	{
  uint8_t tx_buffer[100]="Communication started on I2C with flowmeter NAOH sensor\r\n"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	
}
	else
	{
	uint8_t tx_buffer[100]="Error in Communicating with flowmeter NAOH sensor on I2C \r\n"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);

	}

}
//function to start I2C communication with flowmeter HOCL on I2C
void measure_flow_HOCL_initialization()
{
  if(HAL_I2C_IsDeviceReady(&hi2c1,0X02,1,100)==HAL_OK)
	{
  uint8_t tx_buffer[100]="Communication started on I2C with flowmeter HOCL sensor\r\n"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	
}
	else
	{
	uint8_t tx_buffer[100]="Error in Communicating with flowmeter HOCL sensor on I2C \r\n"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);

	}

}
//function to start I2C communication with flowmeter saltpump on I2C
void measure_flow_saltpump_initialization()
{
  if(HAL_I2C_IsDeviceReady(&hi2c1,0X03,1,100)==HAL_OK)
	{
  uint8_t tx_buffer[100]="Communication started on I2C with flowmeter saltpump sensor\r\n"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);
	
}
	else
	{
	uint8_t tx_buffer[100]="Error in Communicating with flowmeter saltpump sensor on I2C \r\n"; 
  HAL_UART_Transmit(&huart2,tx_buffer,sizeof(tx_buffer),100);

	}

}
//function to turn on the salt pump every for 1 minute every 60 minutes
void salt_pump(void)
{
	//run salt pump for 1 minute every 60 minutes

  //check the status of salt pump is already on or the production flag is already on	
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

//function to monitor the flowrate of NAOH
bool monitor_flow_NAOH(float flow_NAOH_value)
{

    if (flow_NAOH_value <80)
    {
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "The flow value of NaOH is less than 80 \r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
        return true;
    }
    return false;
}

//function to monitor the flowrate of HOCL
bool monitor_flow_HOCL(float flow_HOCL_value)
{

    if (flow_HOCL_value <80)
    {
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "The flow value of HOCL is less than 80 \r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
        return true;
    }
    return false;
}


//function to monitor the flowrate of saltpump
bool monitor_flow_saltpump(float flow_saltpump_value)
{

    if (flow_HOCL_value <30)
    {
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "The flow value of salt pump is less than 30 \r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
        return true;
    }
    return false;
}

//function to check if the ph value of NAOH is between 11.5 and 13
bool monitor_pH_NAOH(float ph_NAOH)
{
    if (11.5 < ph_NAOH && ph_NAOH < 13)
    {
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "The pH value of NaOH is between 11.5 and 13 \r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
        return true;
    }
    return false;
}

//function to monitor the flow 
bool monitor_flow(float flow_value) {
    // Check if flow_value is greater than 0
    if (flow_value > 0 && Idle) {
			  uint8_t buffer[50]="The flow value is greater than 0 cc /10 seconds \r\n";
        return true;
    } else {
        return false;
    }
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
  uint8_t EndMSG[] = "Done! \r\n";
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
    time_converted.systemtime_h = elapsed_hours;

    char time_converted_buffer[150];
    snprintf(time_converted_buffer, sizeof(time_converted_buffer), "time converted is : %02d:%02d:%02d:%03d \r\n",
             time_converted.systemtime_h, time_converted.systemtime_m,
             time_converted.systemtime_s, time_converted.ticks);
    HAL_UART_Transmit(&huart2, (uint8_t*)time_converted_buffer, strlen(time_converted_buffer), 100);

    return time_converted;
}

// Function to check if the system time is valid
bool isvalid_system_time(const SystemTime *systemTime) {
    // Check for valid hours, minutes, seconds
	  //systemTime->systemtime_h < 24 &&
    if (systemTime->systemtime_h >= 0 && 
        systemTime->systemtime_m >= 0 && systemTime->systemtime_m < 60 &&
        systemTime->systemtime_s >= 0 && systemTime->systemtime_s < 60 ) 
		{
        return true; // Valid time
    } else {
        return false; // Invalid time
    }
}

bool isvalid_totalcycle_time(const TotalCycleTime *cycletime) {
	  // cycletime->cycletime_h < 24 &&
    // Check for valid hours, minutes, seconds
    if (cycletime->cycletime_h >= 0 && 
        cycletime->cycletime_m >= 0 && cycletime->cycletime_m < 60 &&
        cycletime->cycletime_s >= 0 && cycletime->cycletime_s < 60 )
         {
        return true; // Valid time
    } else {
        return false; // Invalid time
    }
}
SystemTime read_systemtime_in_flashmemory()
{
    const uint32_t system_time_address = 0x08020000;
    SystemTime read_system_time;

    if (isvalid_system_time((SystemTime*)system_time_address)) {
        read_system_time = *(SystemTime*)system_time_address; // Read SystemTime from flash memory

        // Check for invalid time values
        if (read_system_time.systemtime_h < 0 || read_system_time.systemtime_m < 0 || read_system_time.systemtime_s < 0 )
            
        {
            // Initialize to 00:00:00.000 if any value is invalid
            read_system_time.systemtime_h = 0;
            read_system_time.systemtime_m = 0;
            read_system_time.systemtime_s = 0;
            read_system_time.ticks = 0;
        }
    }
    
    
    else {
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
    const uint32_t system_hours_address = 0x08030000; //sector 4
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
    snprintf(system_time_buffer, sizeof(system_time_buffer), " --- current system time: %02d:%02d:%02d:%03d \r\n",
             system_time.systemtime_h, system_time.systemtime_m,
             system_time.systemtime_s, system_time.ticks);
    HAL_UART_Transmit(&huart2, (uint8_t*)system_time_buffer, strlen(system_time_buffer), 100);

    // Calculate total seconds for current system time
	  total_seconds_current = system_time.systemtime_h * 3600 + system_time.systemtime_m * 60 + system_time.systemtime_s;
		char total_seconds_current_buffer[150];
		snprintf(total_seconds_current_buffer, sizeof(total_seconds_current_buffer), "--- Total seconds current system time: %u \r\n", total_seconds_current);
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
             "!!!!! total system time calculated is : %02d:%02d:%02d:%03d \r\n",
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
    snprintf(buffer, sizeof(buffer), "System Time written on flash: %02d:%02d:%02d:%03d \r\n",
             total_system_time.systemtime_h, total_system_time.systemtime_m,
             total_system_time.systemtime_s, total_system_time.ticks);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
}


//function to read total cycle time from flash memory
TotalCycleTime read_total_cycle_time_in_flashmemory()
{
    
    const uint32_t total_cycle_time_address = 0x08010000;
	  TotalCycleTime read_totalcycle_time; 
    
    if (isvalid_totalcycle_time((TotalCycleTime*)total_cycle_time_address)) {
        read_totalcycle_time = *(TotalCycleTime*)total_cycle_time_address; // Read SystemTime from flash memory

        // Check for invalid time values
        if (read_totalcycle_time.cycletime_h < 0 || read_totalcycle_time.cycletime_m < 0 || read_totalcycle_time.cycletime_s < 0 )
            
        {
            // Initialize to 00:00:00.000 if any value is invalid
            read_totalcycle_time.cycletime_h = 0;
            read_totalcycle_time.cycletime_m = 0;
            read_totalcycle_time.cycletime_s = 0;
            //read_totalcycle_time.ticks = 0;
        }
    }
    
    
    else {
        // Initialize to 00:00:00.000 if the system time in flash is not valid
        read_totalcycle_time.cycletime_h = 0;
        read_totalcycle_time.cycletime_m = 0;
        read_totalcycle_time.cycletime_s = 0;
       // read_totalcycle_time.ticks = 0;
    }
    
    char buffer[100];
	 // Format and transmit system time
    snprintf(buffer, sizeof(buffer), "Totalcycle Time read: %02d:%02d:%02d \r\n",
             read_totalcycle_time.cycletime_h, read_totalcycle_time.cycletime_m,
             read_totalcycle_time.cycletime_s);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
    return read_totalcycle_time;
	}	
	
//write total cycle hours to flash memory
void write_total_cycle_time_in_flashmemory()
{
    const uint32_t total_cycle_hours_address = 0x08010000;
	  static SystemTime start_time, end_time; 
    uint32_t size_of_totalcycle_structure = sizeof(TotalCycleTime);
    uint8_t totalcycletime_byte_array[size_of_totalcycle_structure];
	  static bool read_time=false;
	  static uint32_t total_seconds_flash=0;
	  static uint32_t total_seconds=0;
    static uint32_t total_seconds_current=0;	  
	  //read total cycle hours from flash memory
	  TotalCycleTime totalcycle_time_fromflash;

	  if (!read_time)
		{
		totalcycle_time_fromflash = read_total_cycle_time_in_flashmemory();	
    char read_cycle_time_fromflash_buffer[150];
    snprintf(read_cycle_time_fromflash_buffer, sizeof(read_cycle_time_fromflash_buffer), 
             "!!!!! total Cycle Time received from flash: %02d:%02d:%02d\r\n",
             totalcycle_time_fromflash.cycletime_h, totalcycle_time_fromflash.cycletime_m,
             totalcycle_time_fromflash.cycletime_s);
    HAL_UART_Transmit(&huart2, (uint8_t*)read_cycle_time_fromflash_buffer, strlen(read_cycle_time_fromflash_buffer), 100);			
	  
		// Convert system times to seconds
		total_seconds_flash = totalcycle_time_fromflash.cycletime_h * 3600 + totalcycle_time_fromflash.cycletime_m * 60 + totalcycle_time_fromflash.cycletime_s;
		char total_seconds_flash_buffer[150];
		snprintf(total_seconds_flash_buffer, sizeof(total_seconds_flash_buffer), "!!!!! Total seconds flash totalcycle time: %u\r\n", total_seconds_flash);
		HAL_UART_Transmit(&huart2, (uint8_t*)total_seconds_flash_buffer, strlen(total_seconds_flash_buffer), 100) ; 		
		read_time=true;
		}	

    //print the current cycle hours 
    char current_cycle_time_buffer[150];
    snprintf(current_cycle_time_buffer, sizeof(current_cycle_time_buffer), " --- current cycle time: %02d:%02d:%02d \r\n",
             current_cycle_time.cycletime_h, current_cycle_time.cycletime_m,
             current_cycle_time.cycletime_s);
    HAL_UART_Transmit(&huart2, (uint8_t*)current_cycle_time_buffer, strlen(current_cycle_time_buffer), 100);

    // Calculate total seconds for current cycle time
	  total_seconds_current = current_cycle_time.cycletime_h * 3600 + current_cycle_time.cycletime_m * 60 + current_cycle_time.cycletime_s;
		char total_seconds_current_buffer[150];
		snprintf(total_seconds_current_buffer, sizeof(total_seconds_current_buffer), "--- Total seconds current systecycle time: %u \r\n", total_seconds_current);
		HAL_UART_Transmit(&huart2, (uint8_t*)total_seconds_current_buffer, strlen(total_seconds_current_buffer), 100);

    // Add the seconds
    total_seconds = total_seconds_flash + total_seconds_current;
		char total_seconds_buffer[150];
		snprintf(total_seconds_buffer, sizeof(total_seconds_buffer), "--- Total seconds total cycle time: %u\r\n", total_seconds);
		HAL_UART_Transmit(&huart2, (uint8_t*)total_seconds_buffer, strlen(total_seconds_buffer), 100);  

    // Convert back to hours, minutes, and seconds
     total_cycle_time.cycletime_h= total_seconds / 3600;
     total_seconds %= 3600;
     total_cycle_time.cycletime_m = total_seconds / 60;
     total_cycle_time.cycletime_s = total_seconds % 60;

    char total_cycle_time_buffer[150];
    snprintf(total_cycle_time_buffer, sizeof(total_cycle_time_buffer), 
             "!!!!! Total cycle time calculated is : %02d:%02d:%02d \r\n",
             total_cycle_time.cycletime_h, total_cycle_time.cycletime_m,
             total_system_time.systemtime_s);
    HAL_UART_Transmit(&huart2, (uint8_t*)total_cycle_time_buffer, strlen(total_cycle_time_buffer), 100);

    // Convert systemtime structure into the byte array
    memcpy(totalcycletime_byte_array, &total_cycle_time, size_of_totalcycle_structure);

    // Erase the flash memory before writing
    erase_flash_memory(4, FLASH_VOLTAGE_RANGE_3); 

    // Write the byte array to flash memory
    write_to_flash(total_cycle_hours_address, totalcycletime_byte_array, size_of_totalcycle_structure, FLASH_TYPEPROGRAM_BYTE);

    char buffer[150];
    snprintf(buffer, sizeof(buffer), "Total cycle Time written on flash: %02d:%02d:%02d \r\n",
             total_cycle_time.cycletime_h, total_cycle_time.cycletime_m,
             total_cycle_time.cycletime_s);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);

    // Reset start_time and end_time for next cycle
    memset(&start_time, 0, sizeof(start_time));
    memset(&end_time, 0, sizeof(end_time));
}

// continously check if the current value is greater than 0 for 5 seconds then raise fault
bool cell_current_greater_than_zero() {
	  
    static uint8_t startTime = 0; //variable to hold the start time
    uint32_t threshold = 5; // 5 seconds
	  //uint32_t cell_current=0;
	  Measurements cell=measure_current_voltage();
	  
  	char tx_buff[60];	
    if (cell.cell_current > 0 && Idle) {
        if (startTime == 0) {
            
            startTime = system_time.systemtime_s ;
					  //tx_buff[120]="start time of cell current >0 is",startTime;
					  sprintf(tx_buff, " ----start time of cell current >0 is %d------ \r\n", startTime);					  
					  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buff,sizeof(tx_buff),160);
        } 
				
				else if (system_time.systemtime_s - startTime > threshold) {
					
            // cell_current has been greater than zero for more than 5 seconds
            char buffer[120]=("Current greater than zero for more than 5 seconds \r\n");
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

//function to monitor the cell voltage
bool monitor_cell_voltage()	
{
	
 Measurements cell;
 const uint32_t threshold_voltage=3; //add to a variable list
 if (cell.cell_voltage> threshold_voltage && Idle)
 {
 uint8_t buffer[150]="cell voltage is greater than 3V \r\n";
 HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
 return true;
 }
return false;
}


// function to monitor the brine conductivity value
bool monitor_salttube_brine_conductivity(float brine_conditivity_value)
{
   const float threshold = 15.6;
	 if (brine_conditivity_value < threshold)
	 {
	 uint8_t buffer[100]="value of brine_conditivity is less than threshold \r\n";
	 HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),125);
	 return true;
	 } 
	 
return false;
}

//salt tube brine conductivity
bool salt_tube_filled_time()
{
return false;
}

//function to check if the ph value of HOCL is between 4.5 and 6.5
bool monitor_pH_HOCL(float ph_HOCL)
{
    if (4.5 < ph_HOCL && ph_HOCL < 6.5)
    {
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "The pH value of HOCL is between 4.5 and 6.5 \r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
        return true;
    }
    return false;
}





//fucntion to check low pressure for 2 hours
bool monitor_low_pressure() {
    static uint32_t start_time_h = 0, start_time_m = 0;
    const  uint32_t threshold_seconds = 2 * 3600; // 2 hours in seconds (2*60*60)
	  static uint32_t start_time_seconds=0;
    static uint32_t current_time_seconds=0;
	  static uint32_t elapsed_time_seconds=0;
    if (!monitor_pressure()) {
        if (start_time_h == 0 && start_time_m == 0) {
            start_time_h = system_time.systemtime_h;
            start_time_m = system_time.systemtime_m;
        }

        // Convert start time and current time to seconds
         start_time_seconds = (start_time_h * 3600) + (start_time_m * 60);
         current_time_seconds = (system_time.systemtime_h * 3600) + (system_time.systemtime_m * 60);

        // Calculate elapsed time in seconds
         elapsed_time_seconds = current_time_seconds - start_time_seconds;

        if (elapsed_time_seconds >= threshold_seconds) {
            char buffer[120] = "Pressure is low for more than 2 hours \r\n";
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer, sizeof(buffer), 100);
            //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            // Reset start time
            start_time_h = 0;
            start_time_m = 0;
            return true;
        }
    } else {
        // Reset start time if pressure is not low
        start_time_h = 0;
        start_time_m = 0;
        return false;
    }
    return false;
}

void turn_outputs_off()
{
//   //drain solenoid
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
//   //cell fill solenoid
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//   //salt fill tube
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);

//   //cell power signal
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
//   //salt pump off  
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

     // add salt fill actuator

//   //NAOH flow controller

//   //HOCL flow controler
}




void idle_state()
{
	//In idle state function.IT will run continously when the device starts

   //Turn off all output signals off
  turn_outputs_off();
  monitor_faults();
  // Idle=true;

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
	
	static bool Tankfill_timeexceed=false;
	static uint32_t Tankfill_timeexceed_counter=0;


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
	
	uint32_t current_value=0;
  Measurements cell = measure_current_voltage();





	if (cell.cell_current==0)
	{
	 
	//system ready 
  if(Stop_signal && !Production && !MasterFault)
	{
	Idle_systemready_state=true;

	Idle_tanksfull_state=false;
	Idle_tankfill_timeexceeded_state=false;
	Idle_lowpressure_state=false;
		
	Autocycle=false;
	
	}
	
	//Tank full state
  // 
	if (HOCL_tank_level() && NAOH_tank_level() && Production)
	{
	Idle_tanksfull_state=true;
		
	Idle_systemready_state=false;	
	Idle_tankfill_timeexceeded_state=false;
	Idle_lowpressure_state=false;
		
	Autocycle=true;
	}
	
	//LOW Pressure state
	
	if (!monitor_pressure())
	{

	Idle_lowpressure_state=true;	
	Idle_systemready_state=false;	
	Idle_tankfill_timeexceeded_state=false;
	Idle_lowpressure_state=false;		
	low_pressure_counter++;
	low_pressure_system_hours=system_time;
	low_pressure_cycle_hours=total_cycle_time;
	//Turn off cell fill solenoid off for 30 seconds and on for 5 seconds

if (!Iscellfillsolenoid && system_time.systemtime_s - cellfill_solenoid_time >= 30) {
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
		Idle_tankfill_timeexceeded_state=true;
		Idle_lowpressure_state=false;		
		Idle_systemready_state=false;	
		Idle_lowpressure_state=false;		
		
		Tankfill_timeexceed=true;
		Tankfill_timeexceed_counter++;
		Tankfill_time_exceed_system_hours=system_time;
		Tankfill_time_exceed_cycle_hours=total_cycle_time;
		 
	  Tankfill_time_exceed_hours=system_time.systemtime_h;
    uint32_t total_elapsed_minutes = system_time.ticks / 60000; // Convert ticks to total elapsed minutes
    current_time_in_mins = total_elapsed_minutes;

		
    // Shift timestamps and add the new one
    for (int i = 3 - 1; i > 0; i--) {
        Tankfill_timeexceed_timestamps[i] = Tankfill_timeexceed_timestamps[i - 1];
    }
    Tankfill_timeexceed_timestamps[0] = current_time_in_mins;
		
		if (Tankfill_timeexceed_counter >= 3) {
			
    // Check if three occurrences happened within the last 48 hours (2880 minutes)
    if (Tankfill_timeexceed_timestamps[0] - Tankfill_timeexceed_timestamps[3 - 1] <= 2880) {
			Tankfill_timetrend_code=system_time.systemtime_h-total_cycle_time.cycletime_h;
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

char* monitor_faults()
{
    static char fault_name[200] = ""; // Static array to hold concatenated fault names
    strcpy(fault_name, ""); // Initialize to empty string

    bool cell_voltage_fault = monitor_cell_voltage();
    bool controlboard_power_fault = controlboard_power();  
    bool salt_tube_fault = fill_salt_tube();
    bool brine_conductivity_fault = monitor_salttube_brine_conductivity(brine_conc_value);
    bool low_pressure_fault = monitor_low_pressure();
    bool flow_fault = monitor_flow(flow);
    bool ph_HOCL_fault = monitor_pH_HOCL(pH_HOCL_value);
    bool ph_NAOH_fault = monitor_pH_NAOH(pH_NAOH_value);
    bool flow_NAOH_fault=monitor_flow_NAOH(flow_NAOH_value);
    bool flow_HOCL_fault=monitor_flow_HOCL(flow_HOCL_value);
    bool flow_saltpump=monitor_flow_saltpump(flow_saltpump_value);

  
    if (cell_voltage_fault)
    {
        strcat(fault_name, "cell_voltage ");
        fault_cellcontrol_status=true;
    }
     if (controlboard_power_fault)
     {
     strcat(fault_name, "control_board ");
     fault_circuit_board_status=true;
    }

    if (salt_tube_fault)
    {
        strcat(fault_name, "salt_tube ");
        fault_salt_tube_level_status=true;
    }

    if (brine_conductivity_fault)
    {
        strcat(fault_name, "brine_conductivity ");
        fault_brine_conductivity_status=true;
    }

    if (low_pressure_fault)
    {
        strcat(fault_name, "low_pressure ");
        fault_pressure_loss_status=true;
    }

    if (flow_fault)
    {
        strcat(fault_name, "flow_fault ");
        fault_brine_conductivity_status=true;
    }
    if (ph_HOCL_fault)
    {
        strcat(fault_name, "ph_HOCL ");
        fault_ph_hocl_status=true;
    }
    if (ph_NAOH_fault)
    {
        strcat(fault_name, "ph_NAOH ");
        fault_ph_naoh_status=true;
    }

    if (flow_NAOH_fault)
    {
        strcat(fault_name, "flow_NAOH ");
        fault_naoh_flowrate_status=false;
    }

    if (flow_HOCL_fault)
    {
        strcat(fault_name, "flow_HOCL ");
        fault_hocl_flowrate_status=true;
    }

    if (flow_saltpump)
    {
        strcat(fault_name, "flow_saltpump ");
        fault_saltpump_flowrate_status=true;
    }
    return fault_name;
}

void production_state()
{
   Production=true;
   Priming_sequence=true;
	 //variable for the salttube start time
	 SystemTime current_time=time_conversion();

	 static uint32_t salttube_start_time=0;
	 static uint32_t priming_start_time=0;
	 static uint32_t flushing_start_time=0;
	 priming_start_time = current_time.systemtime_s;
   
	 const uint32_t Priming_sequence_timer=4;
   const uint32_t Flushing_sequence_timer=4;
	
	 uint32_t HOCL_current_production_duration_m=0;
	 uint32_t HOCL_solution_production=0;
	 uint32_t NAOH_current_production_duration_m=0;
	 uint32_t NAOH_solution_production=0;
	
	 Measurements cell_reference;
	 CellsMeasurement cells_reference;
	 static uint32_t low_current_trend_counter=0;
	 static uint32_t under_current_standby_counter=0;

	 SystemTime low_current_trend_system_time;
	 TotalCycleTime low_current_trend_cycle_time;

	 //salttube_start_time=current_time.systemtime_s;
	 //Priming_sequence=true;
 	 // Priming conditions  for 4 seconds

    //In Priming state 
    if (Priming_sequence)
		{
	  if (current_time.systemtime_s - priming_start_time <= Priming_sequence_timer )
		{
			
		   cells_reference = measure_cells_current_voltages();
			 cell_reference = measure_current_voltage();
			//turning on the salt pump 
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
			//turning on the cell fill solenoid 
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
			//turn on the Salt pump control signal ON
      
			//turn on the salt tube for 2 seconds
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET); // Start filling
			salttube_start_time=current_time.systemtime_s;
		
    if (((current_time.systemtime_s - salttube_start_time) >= 2))
    {
        // Turn off the salt tube
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    productionON_sequence=true;
    Priming_sequence=false;
	}

		
}
		
 if (productionON_sequence)
 {
  monitor_low_pressure();
	//monitor ph naoh,hocl 
	//monitor naoh,hocl,salt pump flows
	 
	//start counting cycle hours.
	current_cycle_time.cycletime_h=system_time.systemtime_h - start_time.systemtime_h;
	current_cycle_time.cycletime_m=system_time.systemtime_m - start_time.systemtime_m;
  current_cycle_time.cycletime_s=system_time.systemtime_s - start_time.systemtime_s;
  
	//writing cycle hours to flash memory
	 
  write_total_cycle_time_in_flashmemory();
	 
	//turn on the main cell signal ON
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
	
	//check the current drops by 20%
  if (compare_current_with_reference(cell_reference.cell_current))
	{
	 low_current_trend_counter++;
	 low_current_trend_system_time=system_time;
	 low_current_trend_cycle_time=current_cycle_time;
	 lowcurrent_trend=true;
	 //log current system hours and total cycle hours
	}
	 
	//check if current is below threshold
	current_below_threshold_for_duration();
	
	//check if current is higher than threshold
	current_high_threshold_for_duration();
		

	if (!HOCL_tank_level())
	{
	HOCL_cycle_time.cycletime_h=system_time.systemtime_h - start_time.systemtime_h;
	HOCL_cycle_time.cycletime_m=system_time.systemtime_m - start_time.systemtime_m;
  HOCL_cycle_time.cycletime_s=system_time.systemtime_s - start_time.systemtime_s;
	HOCL_current_production_duration_m=HOCL_cycle_time.cycletime_m;


	//calculate the HOCL solution production calculation
		
	}

	if (!NAOH_tank_level())
	{
	NAOH_cycle_time.cycletime_h=system_time.systemtime_h - start_time.systemtime_h;
	NAOH_cycle_time.cycletime_m=system_time.systemtime_m - start_time.systemtime_m;
  NAOH_cycle_time.cycletime_s=system_time.systemtime_s - start_time.systemtime_s;
	NAOH_current_production_duration_m=HOCL_cycle_time.cycletime_m;
		
	//calculate the NAOH solution production calculation
	//monitor HOCL,NAOH and Saltpump flows
  //monitor PH of NAOH and HOCL
  monitor_pH_HOCL(pH_HOCL_value);
  monitor_pH_NAOH(pH_NAOH_value);		
	}
	
	
	
	 
 }
 
 if (Stop_signal || (NAOH_tank_level() && HOCL_tank_level()) || monitor_pressure() || overcurrent || undercurrent || lowcurrent_trend )
{
	flushing_start_time=system_time.systemtime_s;
	if(Flushing_sequence)
	{
   // In flushing condtion	 
 	//turn off the main cell signal ON
	if(system_time.systemtime_s - flushing_start_time <= Flushing_sequence_timer)
	{
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
	Production=false;
		
 }
  //exit conditions
	//turning on the salt pump 
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	//turning on the cell fill solenoid 
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);	
  Idle=true;

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
	
	//initialize I2C devices
	measure_flow_NAOH_initialization();
  measure_flow_HOCL_initialization();
  measure_flow_HOCL_initialization();
  pH_NAOH_initialization();
  pH_HOCL_initialization();
  salttube_brineconductivity_initialization();
  HOCL_flow_controller_initialization();
  NAOH_flow_controller_initialization();
	start_time=time_conversion();

	



  //erase_flash_memory(5, FLASH_VOLTAGE_RANGE_3);
  //write_systemtime_in_flashmemory();
	
  //testing for fault
 // fault_circuit_board.counter = 1;
 // fault_circuit_board.cycle_hours = total_cycle_hours;
///	fault_circuit_board.fault_status=true;
	//fault_circuit_board.system_hours = system_hours;
////	fault_circuit_board.real_time_clock=4;
//update_fault(&my_faults,FAULT_TYPE_CKT_B , &fault_circuit_board_count, fault_circuit_board);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 

	// Production=true;
   currentTime=system_time.systemtime_s;
	 system_time=time_conversion();	

		NAOH_tank_level();
		HOCL_tank_level();
		salt_tube_bottom();
		salt_tube_top();
		fill_salt_tube();
		cell_current_greater_than_zero();
		measure_current_voltage();
    monitor_cell_voltage();

		salt_pump();
    drain_solenoid();
    controlboard_power();
    monitor_pH_HOCL(pH_HOCL_value);
    monitor_pH_NAOH(pH_NAOH_value);
		
	     // char* faults = monitor_fault();
    // printf("Detected Faults: %s\n", faults);

		
   //write the system hours in flashmemory after every 1 minute
	if ((currentTime - lastSaveTime) >= 60) //write after 1 minute
    { 
			  uint8_t buffer[150]=" !!!! writing time into flash memory \r\n";
			  HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
        write_systemtime_in_flashmemory();
        lastSaveTime = currentTime; // Update the last save time
		}
			
		if (!Start_signal && !Production && !MasterFault && !Resume_signal && Idle)	
		{
			
    uint8_t buff[50]="---------In ideal condtion-------";
		HAL_UART_Transmit(&huart2,buff,sizeof(buff),100);
		// measure_current_voltage();
		// salt_pump();
    // drain_solenoid();
    // controlboard_power();
    // monitor_pH_HOCL(pH_HOCL_value);
    // monitor_pH_NAOH(pH_NAOH_value);

		idle_state();
	
		}



		

		
		// if ((Start_signal || (!NAOH_tank_level() && !HOCL_tank_level() )|| Resume_signal || Tankfill_time_trend_correctpasscode || !monitor_pressure() )&& !Idle_systemready_state && !Idle_tanksfull_state && !Idle_tankfill_timeexceeded_state && !Idle_lowpressure_state && !MasterFault && !Production)

		// move to production if their is start signal and all the idle state flags are not high
		if ((Start_signal || Resume_signal || Tankfill_time_trend_correctpasscode || !monitor_pressure() )&& !Idle_systemready_state && !Idle_tanksfull_state && !Idle_tankfill_timeexceeded_state && !Idle_lowpressure_state && !MasterFault && !Production)
		{
      //turn production on when turned entire
        //turn off when production off

		  uint8_t buff[50]="---------In production condtion-------";
			HAL_UART_Transmit(&huart2,buff,sizeof(buff),100);
      production_state();
		}
		
		
		//if any of the fault is detected move to the fault state

		if(cell_voltage_fault || flow_fault || low_pressure_fault || ph_HOCL_fault || ph_NAOH_fault || brine_conductivity || no_board_power_fault )
		{
		  uint8_t buff[50]="---------In fault condtion-------";
			HAL_UART_Transmit(&huart2,buff,sizeof(buff),100);
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
