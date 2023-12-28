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
  #include <stdint.h>
  #include "main.h"
 // #include "NAOH_tank.h"
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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
  RTC_TimeTypeDef sTime ;
  RTC_DateTypeDef sDate ;
  RTC_AlarmTypeDef sAlarm ;

  //global variables for the user settings
  const uint32_t drain_solenoid_cycletimer=24*60*60; //in seconds
  const float threshold_overcurrent_value=17.0; //in Amps
  const float threshold_undercurrent_value=7.0; //in Amps
  const uint32_t salttube_fill_time_limit = 90; //in seconds
  const uint32_t drain_solenoid_ontime=5; //in seconds
  const uint32_t threshold_flow_saltpump_value=30;
  const uint32_t threshold_flow_HOCL_value=80;
  const uint32_t threshold_flow_NAOH_value=80;
  const float threshold_ph_HOCL_low_value=4.5;
  const float threshold_ph_HOCL_high_value=6.5;
  const float threshold_ph_NAOH_low_value=11.5;
  const float threshold_ph_NAOH_high_value=13;
  const uint32_t threshold_voltage=3; //add to a variable list
  const float salttube_brine_conductivity_threshold = 15.6;
  const uint32_t low_pressure_threshold = 2 * 3600; // 2 hours in seconds (2*60*60)
  const uint32_t flow_controll_NAOH=10; 
  const uint32_t flow_controll_HOCL=10;
  const uint32_t flow_controll_saltpump=10;
  const uint32_t  saltfill_actuator_value=14;



  //pinouts for the sensors

   //drain solenoid output
   GPIO_TypeDef *drain_solenoid_pin_port=GPIOA;    
   uint16_t drain_solenoid_pin_no=GPIO_PIN_15;

   //cell fill solenoid output
   GPIO_TypeDef *cell_fill_solenoid_pin_port=GPIOB;
   uint16_t cell_fill_solenoid_pin_no=GPIO_PIN_13;

   //salt fill tube output
   GPIO_TypeDef *salt_fill_tube_pin_port=GPIOA;
   uint16_t salt_fill_tube_pin_no=GPIO_PIN_6;

   //cell power signal output
   GPIO_TypeDef *cell_power_pin_port=GPIOC;
   uint16_t cell_power_pin_no=GPIO_PIN_14;

   //salt pump output 
   GPIO_TypeDef *salt_pump_pin_port=GPIOB;
   uint16_t salt_pump_pin_no=GPIO_PIN_4;

   //NAOH tank level
   GPIO_TypeDef *tank_level_pin_port=GPIOA;
   uint16_t tank_level_pin_no=GPIO_PIN_1;

   //HOCL tank level
   GPIO_TypeDef *tank_level_HOCL_pin_port=GPIOA;
   uint16_t tank_level_HOCL_pin_no=GPIO_PIN_7;

   //salt tube bottom input
   GPIO_TypeDef *salt_tube_bottom_pin_port=GPIOB;
   uint16_t salt_tube_bottom_pin_no=GPIO_PIN_14;
   
   //salt tube top input
   GPIO_TypeDef *salt_tube_top_pin_port=GPIOB;
   uint16_t salt_tube_top_pin_no=GPIO_PIN_1;

    //pressure sensor input
    GPIO_TypeDef *pressure_pin_port=GPIOB;
    uint16_t pressure_pin_no=GPIO_PIN_15;

    // main cell signal input
    GPIO_TypeDef *main_cell_power_pin_port=GPIOB;
    uint16_t main_cell_power_pin_no=GPIO_PIN_0;

    //controlboard_power input
    GPIO_TypeDef *controlboard_power_pin_port=GPIOC;
    uint16_t controlboard_power_pin_no=GPIO_PIN_15;

    //salt tube actuator pwm output
    GPIO_TypeDef *salt_tube_actuator_pwm_pin_port=GPIOA;
    uint16_t salt_tube_actuator_pwm_pin_no=GPIO_PIN_0;

  //global variables for functions
  float voltage=0,current=0;
  float flow=0;	
  float pH_NAOH_value=12,pH_HOCL_value=6.2;
  float flow_NAOH_value=0,flow_HOCL_value=0,flow_saltpump_value=0;
  float brine_conc_value=0;
  bool saltpump_status=false;



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
  uint32_t drain_solenoidTime = 0;
  uint8_t  drain_solenoidState = 0; 
  uint32_t drain_solenoidTargetTime = 0;

  /// variable for drain solenoid global cycle
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


  SystemTime system_time={0},total_system_time={0}, start_time={0};


  typedef struct{
      uint32_t cycletime_s;
      uint32_t cycletime_m;
      uint32_t cycletime_h;
  } TotalCycleTime;

  TotalCycleTime total_cycle_time={0},current_cycle_time={0},HOCL_cycle_time={0},NAOH_cycle_time={0};

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
  RTC_TimeTypeDef real_time_clock; 
      
  }__attribute__((packed)) fault_t;

  typedef struct {
    
      fault_t faults[NUM_FAULT_TYPES][20];
  } __attribute__((packed)) fault_array_t; 



  // Initialize my_faults
  fault_array_t my_faults;



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

  //For tank full
  bool Autocycle=false;
  char state[50]; // Buffer to hold the state string

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
  bool Clear_fault=false;


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
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
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
      snprintf(Time,sizeof(Time),"RTC Time is %02d:%02d:%02d \r\n",sTime.Hours, sTime.Minutes, sTime.Seconds);
      HAL_UART_Transmit(&huart2, (uint8_t*)Time, strlen(Time), 100);

      // snprintf(Date,sizeof(Date),"Date is %02d-%02d-%02d \n\r ",sDate.Year,sDate.Month,sDate.Date);
      // HAL_UART_Transmit(&huart2, (uint8_t*)Date, strlen(Date), 100);	

      // snprintf(unixtime,sizeof(unixtime), "unix time is %d \n\r ", unixTime);
      // HAL_UART_Transmit(&huart2, (uint8_t *)unixtime, strlen(unixtime), 300);
      return unixTime;
  }


  //calculate current and voltage
  Measurements measure_current_voltage(void) {
      uint16_t readvalue;
      const int num_samples = 5;
      float sum_readvalue = 0;
      float average_readvalue;
      const float sensitivity = 0.1;
      char test_buff1[200];
      snprintf(test_buff1, sizeof(test_buff1), "In measure_current_voltage function \r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_buff1, strlen(test_buff1), 100);
      memset(test_buff1, 0, sizeof(test_buff1));  // Clear the buffer

      // Take 10 samples of analog reading
      for (int i = 0; i < num_samples; i++) {
          HAL_ADC_PollForConversion(&hadc1, 1000);
          readvalue = HAL_ADC_GetValue(&hadc1);
          sum_readvalue += readvalue;
          HAL_Delay(20);

      }

      // Calculate average of the readings
      average_readvalue = sum_readvalue / num_samples;

      // Convert average ADC value to voltage
      main_cell.cell_voltage = average_readvalue * 3.3 / 4096;
      main_cell.cell_voltage=5.1;//for testing
      // Adjust for tolerance in voltage divider resistor & ADC accuracy
      if (main_cell.cell_voltage != 2.5) {
          main_cell.cell_voltage *= 1.035;
      }

      // Calculate current based on average voltage
      main_cell.cell_current = (main_cell.cell_voltage - 2.5) / sensitivity;
      main_cell.cell_current=5.1;//for testing

      // Print voltage on uart
      //char voltage_value[200];
      snprintf(test_buff1,sizeof(test_buff1) ,"Average voltage is %f \r\n ", main_cell.cell_voltage);
      // HAL_UART_Transmit(&huart2, (uint8_t*)voltage_value, sizeof(voltage_value), 300);
      HAL_UART_Transmit(&huart2, (uint8_t*)test_buff1, strlen(test_buff1), 300);
      memset(test_buff1, 0, sizeof(test_buff1));

      // char buff_test[150]="testing print \r\n";
      // // HAL_UART_Transmit(&huart2, (uint8_t*)buff_test, sizeof(buff_test), 100);
      // HAL_UART_Transmit(&huart2, (uint8_t*)buff_test, strlen(buff_test), 100);

      // Print current on uart
      char current_value[200];
      snprintf(test_buff1,sizeof(test_buff1), "Average current is %f \r\n ", main_cell.cell_current);
      HAL_UART_Transmit(&huart2, (uint8_t*)test_buff1, strlen(test_buff1), 300);
      memset(test_buff1, 0, sizeof(test_buff1));

      return main_cell;
  }


  bool compare_current_with_reference(float current_reference) {
      float average_current = measure_current_voltage().cell_current;
      char tx_buffer[100];
      if (average_current < current_reference * 0.2)

      { // Check if 20% below reference
          snprintf(tx_buffer,sizeof(tx_buffer),"Current value is 20 percent below the reference value.\r\n");
          HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);
          return true;
      }
      else 
      {
          snprintf(tx_buffer,sizeof(tx_buffer),"Current value is within acceptable range.\r\n");
          HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);
          return false;
      }
      return false;
  }


  bool current_below_threshold_for_duration()
  {
    const uint32_t time_threshold=10;
    const int duration= 10;
    float average_current = measure_current_voltage().cell_current;
    static uint32_t start_time_m=0,start_time_s=0;
    static uint32_t undercurrent_standby_counter=0;
    static uint32_t elapsed_time=0;
    const  uint32_t undercurrent_time_threshold=3600;
    char tx_buffer[100];
    SystemTime under_current_standby_system_time;
    TotalCycleTime under_current_standby_cycle_time;
    
    if (average_current<threshold_undercurrent_value)
    {
    

    //turn off  the  cell fill solenoid 
    HAL_GPIO_WritePin(cell_fill_solenoid_pin_port,cell_fill_solenoid_pin_no,GPIO_PIN_RESET);		
      
    if (start_time_m==0 && start_time_s==0)
      {
      start_time_m = system_time.systemtime_m;
      start_time_s = system_time.systemtime_s;
      snprintf(tx_buffer,sizeof (tx_buffer), " ----start time of cell current <7 is %d------ \r\n", start_time_s);					  
      HAL_UART_Transmit(&huart2,(uint8_t*)tx_buffer,strlen(tx_buffer),100);
      
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
    snprintf(tx_buffer,sizeof(tx_buffer), "undercurrent counter value is 5 production off \r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 100);
    undercurrent_standby_counter=0;
    elapsed_time=0;
    start_time_s = 0;
    start_time_m = 0;
    undercurrent=true;

    //
    //exit from production
    productionON_sequence=false;
    Flushing_sequence=true;
    return true;
    // move to undercurrent fault state
    
    }
    
    //check if value is less than threshold for more than 10 seconds
    if (system_time.systemtime_s - start_time_s >= time_threshold) {
              snprintf(tx_buffer,sizeof(tx_buffer), "Production off \r\n");
              HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 100);
              undercurrent=true;		 
              // Reset start_time_s and start_time_m
              start_time_s = 0;
              start_time_m = 0;
              elapsed_time=0;		 
              undercurrent_standby_counter=0;

              //exit from production
              productionON_sequence=false;
              Flushing_sequence=true; 
              return true;

              // move to undercurrent fault state
                      
    }
    
  }	else
    {
    //turn on the cell fill solenoid 
    HAL_GPIO_WritePin(cell_fill_solenoid_pin_port,cell_fill_solenoid_pin_no,GPIO_PIN_SET);	
    }
    return false;
  }



  void current_high_threshold_for_duration()
  {
    const uint32_t time_threshold=5;
    float average_current = measure_current_voltage().cell_current;
    static uint32_t start_time_m=0,start_time_s=0;
    static uint32_t overcurrent_standby_counter=0;
    static uint32_t elapsed_time=0;
    char tx_buffer[100];
    SystemTime over_current_standby_system_time;
    TotalCycleTime over_current_standby_cycle_time;
    
    if (average_current>threshold_overcurrent_value)
    {
    
    if (start_time_m==0 && start_time_s==0)
      {
      start_time_m = system_time.systemtime_m;
      start_time_s = system_time.systemtime_s;
      snprintf(tx_buffer,sizeof(tx_buffer), " ----start time of cell current >17 is %d------ \r\n", start_time_s);					  
      HAL_UART_Transmit(&huart2,(uint8_t*)tx_buffer,strlen(tx_buffer),160);
      
      }
      //convert into seconds 
        elapsed_time = (system_time.systemtime_m * 60 + system_time.systemtime_s) - (start_time_m * 60 + start_time_s);
          
      //check if value is greater than threshold for more than 5 seconds
      if (elapsed_time> time_threshold)
    {
      snprintf(tx_buffer,sizeof(tx_buffer), "overcurrent condition occurs \r\n");
      HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 100);
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
          HAL_Delay(50);
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
          HAL_Delay(50);

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
          HAL_Delay(50);

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
      char voltage_value[200];
      snprintf(voltage_value, sizeof(voltage_value), "Average voltage of cell A %f , cell B %f,cell C %f \t\n\r ", cells.cellA_voltage,cells.cellB_voltage,cells.cellC_voltage);
      HAL_UART_Transmit(&huart2, (uint8_t*)voltage_value, strlen(voltage_value), 300);

      // Print current on uart
      char current_value[200];
      snprintf(current_value, sizeof(current_value),"Average current of cell A %f , cell B %f,cell C %f \t\n\r ", cells.cellA_current,cells.cellB_current,cells.cellC_current);
      HAL_UART_Transmit(&huart2, (uint8_t*)current_value, strlen(current_value), 300);		
      
      return cells;		
  }

  
  //function to monitor NAOH tank level
  
  bool NAOH_tank_level()
  {

    //read the tank level on pin PA1
    GPIO_PinState status =HAL_GPIO_ReadPin(tank_level_pin_port,tank_level_pin_no);
    char tx_buffer[100]; 
    //print the tank level on uart
    if (status ==GPIO_PIN_SET)
    {
      
        snprintf(tx_buffer,sizeof(tx_buffer)," NAOH_tank_level is high \r\n"); 
        HAL_UART_Transmit(&huart2,(uint8_t *) tx_buffer, strlen(tx_buffer), 100);
    }
    else
    {

        snprintf(tx_buffer,sizeof(tx_buffer)," NAOH_tank_level is low \r\n"); 
        HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 100);
    }
    
    return status;
  }

  

  bool controlboard_power()
  {
    //read the control board power on pin PC15
    GPIO_PinState status =HAL_GPIO_ReadPin(controlboard_power_pin_port,controlboard_power_pin_no);
    char tx_buffer[100]; 
    //print the the salt tube bottom sensor on uart
    if (status ==GPIO_PIN_SET)
    {
        
        snprintf(tx_buffer,sizeof(tx_buffer),"control board power is high \r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 100);
    }
    else
    {
        snprintf(tx_buffer,sizeof(tx_buffer),"control board power is low \r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *) tx_buffer, strlen(tx_buffer), 100);
    }
    return status;
  }



  // monitor salt tube bottom sensor
  bool salt_tube_bottom()
  {

    //read the salt tube bottom sensor on pin PB14
    GPIO_PinState status =HAL_GPIO_ReadPin(salt_tube_bottom_pin_port,salt_tube_bottom_pin_no);
    char tx_buffer[100];
    //print the the salt tube bottom sensor on uart
    if (status ==GPIO_PIN_SET)
    {
         
        snprintf(tx_buffer,sizeof(tx_buffer),"salt tube bottom sensor is high \r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 100);
    }
    else
    {
        snprintf(tx_buffer,sizeof(tx_buffer),"salt tube bottom sensor is low \r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 100);
        
    }
    return status;
  }
  // monitor salt tube Top sensor
  bool salt_tube_top()
  {

    //read the salt tube top sensor on pin PB12
    GPIO_PinState status =HAL_GPIO_ReadPin(salt_tube_top_pin_port,salt_tube_top_pin_no);
    char tx_buffer[100];
    //print the the salt tube top sensor on uart
    if (status ==GPIO_PIN_SET)
    {
        snprintf(tx_buffer,sizeof(tx_buffer),"salt tube top sensor is high \r\n"); 
        HAL_UART_Transmit(&huart2,(uint8_t *) tx_buffer, strlen(tx_buffer), 100);
    }
    else
    {
        snprintf(tx_buffer,sizeof(tx_buffer),"salt tube top sensor is low \r\n"); 
        HAL_UART_Transmit(&huart2,(uint8_t *)  tx_buffer, strlen(tx_buffer), 100);
    }
    return status;
  }

  //function to fill if the salt tube
  bool fill_salt_tube()
  {
      static uint32_t start_time_s=0, start_time_m=0;
      static bool filling_started = false;
      static uint32_t elapsed_time=0;
      char tx_buffer[150];
      char buff[100];
      // int n = sprintf(tx_buffer, "----In fill salt tube function--- start time is :%d:%d system time seconds are :%d\r\n", start_time_m, start_time_s, system_time.systemtime_s);
      // HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, sizeof(tx_buffer), 130);

      if (!salt_tube_bottom() && !salt_tube_top())
      {
          if (!filling_started)
          {
              start_time_s = system_time.systemtime_s;
              start_time_m = system_time.systemtime_m; // Set the start time when filling starts
              int n = snprintf(tx_buffer,sizeof(tx_buffer), "----In fill salt tube function start time is :%d:%d system time seconds are :%d \r\n", start_time_m, start_time_s, system_time.systemtime_s);
              HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 130);
              filling_started = true;
          }

          HAL_GPIO_WritePin(salt_fill_tube_pin_port, salt_fill_tube_pin_no, GPIO_PIN_SET); // Start filling
          snprintf(buff,sizeof(buff),"Salt tube filling started \r\n");
          HAL_UART_Transmit(&huart2, (uint8_t *)buff, strlen(buff), 120);

          // Calculate elapsed time in seconds
          elapsed_time = (system_time.systemtime_m * 60 + system_time.systemtime_s) - (start_time_m * 60 + start_time_s);
          int n = snprintf(tx_buffer,sizeof(tx_buffer), "elapsed time of salt tube filling is :%d \r\n", elapsed_time);
          //HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, sizeof(tx_buffer), 150);

          // Check if the salt tube is filled within 90 seconds
          if (salt_tube_top() && elapsed_time <= salttube_fill_time_limit)
          {

              HAL_GPIO_WritePin(salt_fill_tube_pin_port, salt_fill_tube_pin_no, GPIO_PIN_RESET); //stop filling as salt tube is filled
              filling_started = false;
              return true;
          }
          else if (elapsed_time > salttube_fill_time_limit)
          {
              snprintf(tx_buffer,sizeof(tx_buffer),"-----Salt tube not filled in 90 seconds------ \r\n");
              HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 100);
              //elapsed_time=0;
          }
      }


      // Stop filling when both sensors are high
      if (salt_tube_bottom() && salt_tube_top())
      {
          HAL_GPIO_WritePin(salt_fill_tube_pin_port, salt_fill_tube_pin_no, GPIO_PIN_RESET); // Stop filling
          snprintf(buff,sizeof(buff),"Salt tube filling stopped \r\n");
          HAL_UART_Transmit(&huart2, (uint8_t *)buff, strlen(buff), 120);

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
    GPIO_PinState status =HAL_GPIO_ReadPin(tank_level_HOCL_pin_port,tank_level_HOCL_pin_no);
    char tx_buffer[100];
    //print the tank level on uart
    if (status ==GPIO_PIN_SET)
    {
        snprintf(tx_buffer,sizeof(tx_buffer),"HOCL_tank_level is high \r\n"); 
        HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 100);
    }
    else
    {
        snprintf(tx_buffer,sizeof(tx_buffer),"HOCL_tank_level is low \r\n");
        HAL_UART_Transmit(&huart2,(uint8_t *) tx_buffer, strlen(tx_buffer), 100);
    }
    return status;
  }

  //function to monitor pressure
  bool monitor_pressure()
  {

    //read the pressure on pin PB0
    GPIO_PinState status =HAL_GPIO_ReadPin(pressure_pin_port,pressure_pin_no);
    char tx_buffer[100];
    //print the pressure status on uart
      if (status ==GPIO_PIN_SET)
    {
        snprintf(tx_buffer,sizeof(tx_buffer),"pressure is high \r\n"); 
        HAL_UART_Transmit(&huart2,(uint8_t *) tx_buffer, strlen(tx_buffer), 100);
    }
    else
    { 
        snprintf(tx_buffer,sizeof(tx_buffer),"pressure is hilowgh \r\n"); 
        HAL_UART_Transmit(&huart2,(uint8_t *) tx_buffer, strlen(tx_buffer), 100);
    }
    return status;
  }

  //function to turn on or off cellfill solenoid
  void cellfill_solenoid(bool status)
  {


    if (status==true)
    {
    HAL_GPIO_WritePin(cell_fill_solenoid_pin_port,cell_fill_solenoid_pin_no,GPIO_PIN_SET);
    monitor_pressure();
    }
    else
    HAL_GPIO_WritePin(cell_fill_solenoid_pin_port,cell_fill_solenoid_pin_no,GPIO_PIN_RESET);

  }

  //function to turn on or off drain solenoid for 1 
  void drain_solenoid()
  {
    // Check if 24 hours have passed
    if ((HAL_GetTick() - drain_solenoidTime) >= (drain_solenoid_cycletimer))
    {

      if (drain_solenoidState == 0)
      {

        HAL_GPIO_WritePin(drain_solenoid_pin_port, drain_solenoid_pin_no, GPIO_PIN_SET); // Turn on solenoid control pin
        drain_solenoidState = 1;

        // Set the target time for 1 minute
        drain_solenoidTargetTime = HAL_GetTick() + drain_solenoid_ontime; //  5 seconds in milliseconds
      }
    }

    // Check if 5 sec have passed
    if (drain_solenoidState == 1 && HAL_GetTick() >= drain_solenoidTargetTime)
    {
      HAL_GPIO_WritePin(drain_solenoid_pin_port, drain_solenoid_pin_no, GPIO_PIN_RESET); // Turn off solenoid control pin
      drain_solenoidState = 0;
      drain_solenoidTime = HAL_GetTick();
    }
  }

  //function to start communication with HOCL flow controller   
  void HOCL_flow_controller_initialization()
  {
    char tx_buffer[100];
    //for HOCL flow controller 
    if(HAL_I2C_IsDeviceReady(&hi2c1,0X12,1,100)==HAL_OK)
    {
    snprintf(tx_buffer,sizeof(tx_buffer),"Communication started on I2C with HOCL_flow_controller \r\n");
    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);

  }
    else
    {
      snprintf(tx_buffer,sizeof(tx_buffer),"Error in Communicating with HOCL_flow_controller on I2C \r\n");
      HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);

    }
  }

  //function to start communication with NAOH flow controller   
  void NAOH_flow_controller_initialization()
  {
    //for NAOH flow controller 
    char tx_buffer[100];
    if(HAL_I2C_IsDeviceReady(&hi2c1,0X12,1,100)==HAL_OK)
    {
    snprintf(tx_buffer,sizeof(tx_buffer),"Communication started on I2C with NAOH_flow_controller \r\n"); 
    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);

  }
    else
    {
      snprintf(tx_buffer,sizeof(tx_buffer),"Error in Communicating with NAOH_flow_controller on I2C \r\n"); 
      HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);
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
    char tx_buffer[100];
    //for NAOH ph sensor
    if(HAL_I2C_IsDeviceReady(&hi2c1,0X65,1,100)==HAL_OK)
    {
    snprintf(tx_buffer,sizeof(tx_buffer),"Communication started on I2C with pH NAOH sensor \r\n"); 
    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);
  }
    else
    {
    snprintf(tx_buffer,sizeof(tx_buffer),"Error in Communicating with pH NAOH sensor on I2C \r\n"); 
    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);
    }
    
  }

  //function to start I2C communication with ph sensor for HOCL on I2C
  void pH_HOCL_initialization()

  {
    char tx_buffer[100];
    if(HAL_I2C_IsDeviceReady(&hi2c1,0X65,1,100)==HAL_OK)
    {
    snprintf(tx_buffer,sizeof(tx_buffer),"Communication started on I2C with pH HOCL sensor \r\n"); 
    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);
    
  }
    else
    {
    snprintf(tx_buffer,sizeof(tx_buffer),"Error in Communicating with pH HOCL sensor on I2C \r\n"); 
    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);
    }
  }

  void salttube_brineconductivity_initialization()
  {
    char tx_buffer[100];
    if(HAL_I2C_IsDeviceReady(&hi2c1,0X65,1,100)==HAL_OK)
    {
     snprintf(tx_buffer,sizeof(tx_buffer),"Communication started on I2C with salt tube brine conductivity sensor \r\n"); 
    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);   
  }
    else
    {

    snprintf(tx_buffer,sizeof(tx_buffer),"Error in Communicating with salt tube brine conductivity sensor on I2C \r\n"); 
    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100); 
    }
  }


  //function to start I2C communication with flowmeter NAOH on I2C
  void measure_flow_NAOH_initialization()
  {
    char tx_buffer[100];
    if(HAL_I2C_IsDeviceReady(&hi2c1,0X01,1,100)==HAL_OK)
    {

    snprintf(tx_buffer,sizeof(tx_buffer),"Communication started on I2C with flowmeter NAOH sensor \r\n"); 
    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100); 
    
  }
    else
    {

    snprintf(tx_buffer,sizeof(tx_buffer),"Error in Communicating with flowmeter NAOH sensor on I2C \r\n"); 
    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100); 
    }

  }
  //function to start I2C communication with flowmeter HOCL on I2C
  void measure_flow_HOCL_initialization()
  {
    char tx_buffer[100];
    if(HAL_I2C_IsDeviceReady(&hi2c1,0X02,1,100)==HAL_OK)
    {

    snprintf(tx_buffer,sizeof(tx_buffer),"Communication started on I2C with flowmeter HOCL sensor\r\n"); 
    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);     
  }
    else
    {

    snprintf(tx_buffer,sizeof(tx_buffer),"Error in Communicating with flowmeter HOCL sensor on I2C \r\n"); 
    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);  
    }

  }
  //function to start I2C communication with flowmeter saltpump on I2C
  void measure_flow_saltpump_initialization()
  {
    char tx_buffer[100];
    if(HAL_I2C_IsDeviceReady(&hi2c1,0X03,1,100)==HAL_OK)
    {

    snprintf(tx_buffer,sizeof(tx_buffer),"Communication started on I2C with flowmeter saltpump sensor \r\n"); 
    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);  
  }
    else
    {
    snprintf(tx_buffer,sizeof(tx_buffer),"Error in Communicating with flowmeter saltpump sensor on I2C \r\n"); 
    HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);  
    }
  }

   

  void saltfill_actuator(int saltfill_actuator_value)
  {
   	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,saltfill_actuator_value);
		HAL_Delay(2000);
  }
  //function to turn on the salt pump every for 1 minute every 60 minutes
  void salt_pump(void)
  {
    //run salt pump for 1 minute every 60 minutes

    //check the status of salt pump is already on or the production flag is already on	
    if (sTime.Minutes==1 && saltpump_status==false)
      {
      HAL_GPIO_WritePin(salt_pump_pin_port,salt_pump_pin_no,GPIO_PIN_SET);
      saltpump_status=true;
      
      }
    else
    {
    HAL_GPIO_WritePin(salt_pump_pin_port,salt_pump_pin_no,GPIO_PIN_RESET);
    saltpump_status=false;
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
      if (flow_NAOH_value <threshold_flow_NAOH_value)
      {
          char tx_buffer[100];
          snprintf(tx_buffer,sizeof(tx_buffer),"The flow value of NaOH is less than 80 \r\n"); 
          HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);  
          return true;
      }
      return false;
  }

  //function to monitor the flowrate of HOCL
  bool monitor_flow_HOCL(float flow_HOCL_value)
  {

      if (flow_HOCL_value <80)
      {
          char tx_buffer[100];
          snprintf(tx_buffer,sizeof(tx_buffer),"The flow value of HOCL is less than 80 \r\n"); 
          HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);
          return true;
      }
      return false;
  }


  //function to monitor the flowrate of saltpump
  bool monitor_flow_saltpump(float flow_saltpump_value)
  {

      if (flow_saltpump_value <threshold_flow_saltpump_value)
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

      if (threshold_ph_NAOH_low_value < ph_NAOH && ph_NAOH < threshold_ph_NAOH_high_value)
      {
          char buff_NAOH[100];
          //snprintf(buff_NAOH, sizeof(buff_NAOH), "The pH value of NaOH is between 11.5 and 13 \r\n");
          snprintf(buff_NAOH,sizeof(buff_NAOH), "The pH value of NaOH is between 11.5 and 13 \r\n");
          HAL_UART_Transmit(&huart2, (uint8_t *)buff_NAOH, strlen(buff_NAOH), 150);


          return true;
      }
      return false;
  }

  //function to monitor the flow 
  bool monitor_flow(float flow_value) {
      // Check if flow_value is greater than 0
      if (flow_value > 0 && Idle) {
          char tx_buffer[50]="The flow value is greater than 0 cc /10 seconds \r\n";
          snprintf(tx_buffer,sizeof(tx_buffer),"The flow value is greater than 0 cc /10 seconds \r\n");
          HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 100);
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
      //n += snprintf(uartStr + n, sizeof(uartStr) - n, "fault_type: %d, index: %d,fault_param.counter: %d,fault_param.cycle_hours: %d,fault_param.system_hours: %d,fault_param.fault_status: %d,fault_param.real_time_clock: %d,falult type %d,fault_array address %p \r\n", fault_type, *index,fault_param.counter,fault_param.cycle_hours,fault_param.system_hours,fault_param.fault_status,fault_param.real_time_clock,fault_type,fault_array);
      
      //read from flash memory
      fault_array_t temp_fault_array;
      memcpy(&temp_fault_array, (fault_array_t*)fault_log_address, sizeof(fault_array_t));
      
          
      if (fault_array && *index > 0 && *index < 21) {
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
              snprintf(uartStr + n, sizeof(uartStr) - n, "Flash Write: SUCCESS \r\n");
          } else {
              snprintf(uartStr + n, sizeof(uartStr) - n, "Flash Write: ERROR \r\n");
          }
          
        }
      }
      
  //function to scan connected devices on I2C 		
  void I2C_Scan()
  {
    char Buffer[25] = {0};
    //char Space[] = " - ";
    //char StartMSG[] = "Starting I2C Scanning: \r\n";
    // char EndMSG[] = "Done! \r\n";
    uint8_t i = 0, ret;

  /*-[ I2C Bus Scanning ]-*/
    snprintf(Buffer,sizeof(Buffer), "Starting I2C Scanning: \r\n");
    HAL_UART_Transmit(&huart2,(uint8_t *) Buffer, sizeof(Buffer), 10000);
    for(i=1; i<128; i++)
    {
    ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
    if (ret != HAL_OK) /* No ACK Received At That Address */
    { 
      snprintf(Buffer,sizeof(Buffer), " - ");
      HAL_UART_Transmit(&huart2, (uint8_t *) Buffer, strlen(Buffer), 10000);
      }
    else if(ret == HAL_OK)
    {
    snprintf(Buffer,sizeof(Buffer), "0x%X", i);
    HAL_UART_Transmit(&huart2, (uint8_t *)Buffer, strlen(Buffer), 10000);
    }
      }
      snprintf(Buffer,sizeof(Buffer), "Done! \r\n");
      HAL_UART_Transmit(&huart2, (uint8_t *) Buffer, strlen(Buffer), 10000);
      /*--[ Scanning Done ]--*/

  }

  SystemTime time_conversion()
  {
      
      SystemTime time_converted;
      uint32_t prev_ticks = 0;
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

      char time_converted_buffer[100];
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

      char buffer[200];
      // Format and transmit system time
      snprintf(buffer, sizeof(buffer), "System Time read: %02d:%02d:%02d:%03d \r\n",
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
      char read_system_time_fromflash_buffer[200];
      snprintf(read_system_time_fromflash_buffer, sizeof(read_system_time_fromflash_buffer), 
              "!!!!! System Time received from flash: %02d:%02d:%02d:%03d \r\n",
              system_time_fromflash.systemtime_h, system_time_fromflash.systemtime_m,
              system_time_fromflash.systemtime_s, system_time_fromflash.ticks);
      HAL_UART_Transmit(&huart2, (uint8_t*)read_system_time_fromflash_buffer, strlen(read_system_time_fromflash_buffer), 100);			
      
      // Convert system times to seconds
      total_seconds_flash = system_time_fromflash.systemtime_h * 3600 + system_time_fromflash.systemtime_m * 60 + system_time_fromflash.systemtime_s;
      char total_seconds_flash_buffer[200];
      snprintf(total_seconds_flash_buffer, sizeof(total_seconds_flash_buffer), "!!!!! Total seconds flash system time: %u \r\n", total_seconds_flash);
      HAL_UART_Transmit(&huart2, (uint8_t*)total_seconds_flash_buffer, strlen(total_seconds_flash_buffer), 100) ; 		
      read_time=true;
      }	
        
      //get the current system time
      SystemTime current_time=time_conversion();
      char system_time_buffer[200];
      snprintf(system_time_buffer, sizeof(system_time_buffer), " --- current system time: %02d:%02d:%02d:%03d \r\n",
              system_time.systemtime_h, system_time.systemtime_m,
              system_time.systemtime_s, system_time.ticks);
      HAL_UART_Transmit(&huart2, (uint8_t*)system_time_buffer, strlen(system_time_buffer), 100);

      // Calculate total seconds for current system time
      total_seconds_current = system_time.systemtime_h * 3600 + system_time.systemtime_m * 60 + system_time.systemtime_s;
      char total_seconds_current_buffer[200];
      snprintf(total_seconds_current_buffer, sizeof(total_seconds_current_buffer), "--- Total seconds current system time: %u \r\n", total_seconds_current);
      HAL_UART_Transmit(&huart2, (uint8_t*)total_seconds_current_buffer, strlen(total_seconds_current_buffer), 100);

      // Add the seconds
      total_seconds = total_seconds_flash + total_seconds_current;
      char total_seconds_buffer[200];
      snprintf(total_seconds_buffer, sizeof(total_seconds_buffer), "--- Total seconds total system time: %u \r\n", total_seconds);
      HAL_UART_Transmit(&huart2, (uint8_t*)total_seconds_buffer, strlen(total_seconds_buffer), 100);    
      
      // Convert back to hours, minutes, and seconds
      total_system_time.systemtime_h = total_seconds / 3600;
      total_seconds %= 3600;
      total_system_time.systemtime_m = total_seconds / 60;
      total_system_time.systemtime_s = total_seconds % 60;

      char total_system_time_buffer[200];
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

      char buffer[200];
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
      char read_cycle_time_fromflash_buffer[200];
      snprintf(read_cycle_time_fromflash_buffer, sizeof(read_cycle_time_fromflash_buffer), 
              "!!!!! total Cycle Time received from flash: %02d:%02d:%02d\r\n",
              totalcycle_time_fromflash.cycletime_h, totalcycle_time_fromflash.cycletime_m,
              totalcycle_time_fromflash.cycletime_s);
      HAL_UART_Transmit(&huart2, (uint8_t*)read_cycle_time_fromflash_buffer, strlen(read_cycle_time_fromflash_buffer), 100);			
      
      // Convert system times to seconds
      total_seconds_flash = totalcycle_time_fromflash.cycletime_h * 3600 + totalcycle_time_fromflash.cycletime_m * 60 + totalcycle_time_fromflash.cycletime_s;
      char total_seconds_flash_buffer[200];
      snprintf(total_seconds_flash_buffer, sizeof(total_seconds_flash_buffer), "!!!!! Total seconds flash totalcycle time: %u \r\n", total_seconds_flash);
      HAL_UART_Transmit(&huart2, (uint8_t*)total_seconds_flash_buffer, strlen(total_seconds_flash_buffer), 100) ; 		
      read_time=true;
      }	

      //print the current cycle hours 
      char current_cycle_time_buffer[200];
      snprintf(current_cycle_time_buffer, sizeof(current_cycle_time_buffer), " --- current cycle time: %02d:%02d:%02d \r\n",
              current_cycle_time.cycletime_h, current_cycle_time.cycletime_m,
              current_cycle_time.cycletime_s);
      HAL_UART_Transmit(&huart2, (uint8_t*)current_cycle_time_buffer, strlen(current_cycle_time_buffer), 100);

      // Calculate total seconds for current cycle time
      total_seconds_current = current_cycle_time.cycletime_h * 3600 + current_cycle_time.cycletime_m * 60 + current_cycle_time.cycletime_s;
      char total_seconds_current_buffer[200];
      snprintf(total_seconds_current_buffer, sizeof(total_seconds_current_buffer), "--- Total seconds current systecycle time: %u \r\n", total_seconds_current);
      HAL_UART_Transmit(&huart2, (uint8_t*)total_seconds_current_buffer, strlen(total_seconds_current_buffer), 100);

      // Add the seconds
      total_seconds = total_seconds_flash + total_seconds_current;
      char total_seconds_buffer[200];
      snprintf(total_seconds_buffer, sizeof(total_seconds_buffer), "--- Total seconds total cycle time: %u \r\n", total_seconds);
      HAL_UART_Transmit(&huart2, (uint8_t*)total_seconds_buffer, strlen(total_seconds_buffer), 100);  

      // Convert back to hours, minutes, and seconds
      total_cycle_time.cycletime_h= total_seconds / 3600;
      total_seconds %= 3600;
      total_cycle_time.cycletime_m = total_seconds / 60;
      total_cycle_time.cycletime_s = total_seconds % 60;

      char total_cycle_time_buffer[20];
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

      char buffer[200];
      snprintf(buffer, sizeof(buffer), "Total cycle Time written on flash: %02d:%02d:%02d \r\n",
              total_cycle_time.cycletime_h, total_cycle_time.cycletime_m,
              total_cycle_time.cycletime_s);
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);

      // Reset start_time and end_time for next cycle
      memset(&start_time, 0, sizeof(start_time));
      memset(&end_time, 0, sizeof(end_time));
  }

void write_state_to_flash(const char* state) {
    uint32_t address = 0x08008000; // Flash memory address
    uint8_t data[50]; // Buffer to hold the state string, 

    // Clear the buffer and copy the state to it
    memset(data, 0, sizeof(data));
    strncpy((char*)data, state, sizeof(data) - 1);

    // Unlock flash memory for writing
    HAL_FLASH_Unlock();

    // Erase the flash sector before writing
    FLASH_Erase_Sector(FLASH_SECTOR_2, VOLTAGE_RANGE_3); // Adjust sector number and voltage range if necessary

    // Write state to flash
     write_to_flash(address, data, sizeof(data), FLASH_TYPEPROGRAM_BYTE);


}
//function to read current state from flash memory
// Define a function to read current state from flash memory and print it
void read_state_from_flash() {
    uint32_t address = 0x08008000; // Flash memory address

    // Read state from flash
    memcpy(state, (char*)address, sizeof(state));
    state[sizeof(state) - 1] = '\0'; // Ensure null termination

    char buffer[50];
    if (strcmp(state, "production") == 0) {
        snprintf(buffer, sizeof(buffer), "State: Production \r\n");
    }
     else if (strcmp(state, "idle") == 0) {
        snprintf(buffer, sizeof(buffer), "State: Idle \r\n");
    }
    else if (strcmp(state, "faults") == 0) {
        snprintf(buffer, sizeof(buffer), "State: faults \r\n");
    }
     else {
        snprintf(buffer, sizeof(buffer), "State: Unknown \r\n");
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
}



  // continously check if the current value is greater than 0 for 5 seconds then raise fault
  bool cell_current_greater_than_zero() {
      
      static uint8_t startTime = 0; //variable to hold the start time
      uint32_t threshold = 5; // 5 seconds
      //uint32_t cell_current=0;
      Measurements cell=measure_current_voltage();
      cell.cell_current =5.1; //for testing
      char tx_buffer[100];	
      if (cell.cell_current > 0 && Idle) {
          if (startTime == 0) {
              
              startTime = system_time.systemtime_s ;
              //tx_buff[120]="start time of cell current >0 is",startTime;
              snprintf(tx_buffer,sizeof(tx_buffer), " ----start time of cell current greater than zero is %d------ \r\n", startTime);					  
              HAL_UART_Transmit(&huart2,(uint8_t*)tx_buffer,strlen(tx_buffer),160);
          } 
          
          else if (system_time.systemtime_s - startTime > threshold) {
            
              // cell_current has been greater than zero for more than 5 seconds
              snprintf(tx_buffer,sizeof(tx_buffer),"Current greater than zero for more than 5 seconds \r\n");
              HAL_UART_Transmit(&huart2,(uint8_t *)tx_buffer,strlen(tx_buffer),100);
              // Reset startTime
              startTime = 0;
              return true;
          }
      } 
      
      
      else {
          // Reset startTime if cell_current is not greater than zero
          
          startTime = 0;
          return false;
      }
      return 0;
  }

  //function to monitor the cell voltage
  bool monitor_cell_voltage()	
  {
  char test_buff[100];
  snprintf(test_buff,sizeof(test_buff),"In monitor_cell_voltage_function \r\n");
  HAL_UART_Transmit(&huart2,(uint8_t *)test_buff,strlen(test_buff),100); 
  memset(test_buff, 0, sizeof(test_buff));  // Clear the buffer

  Measurements cell;
  cell.cell_voltage=measure_current_voltage().cell_voltage;
  cell.cell_voltage=10.2;
  if (cell.cell_voltage> threshold_voltage && Idle)
  {
 // char buffer[100];
  snprintf(test_buff, sizeof(test_buff),"cell voltage is greater than 3V \r\n");
  HAL_UART_Transmit(&huart2,(uint8_t *)test_buff,strlen(test_buff),100);
  memset(test_buff, 0, sizeof(test_buff));  // Clear the buffer
  return true;
  }
  return false;
  }


  // function to monitor the brine conductivity value
  bool monitor_salttube_brine_conductivity(float brine_conditivity_value)
  {
    if (brine_conditivity_value < salttube_brine_conductivity_threshold)
    {
    char buffer[100]="value of brine_conditivity is less than threshold \r\n";
    snprintf(buffer, sizeof(buffer),"cell voltage is greater than 3V \r\n");
    HAL_UART_Transmit(&huart2,(uint8_t *)buffer,strlen(buffer),125);
    return true;
    } 
    
  return false;
  }


  //function to check if the ph value of HOCL is between 4.5 and 6.5
  bool monitor_pH_HOCL(float ph_HOCL)
  {
      if (threshold_ph_HOCL_low_value < ph_HOCL && ph_HOCL < threshold_ph_HOCL_high_value )
      {
          char buff_HOCL[100];
          // snprintf(buff_HOCL, sizeof(buff_HOCL), "The pH value of HOCL is between 4.5 and 6.5 \r\n");
          snprintf(buff_HOCL, sizeof(buff_HOCL), "The pH value of HOCL is between 4.5 and 6.5 \r\n");
          HAL_UART_Transmit(&huart2, (uint8_t *)buff_HOCL, strlen(buff_HOCL), 100);
          
          return true;
      }
      return false;
  }





  //fucntion to check low pressure for 2 hours
  bool monitor_low_pressure() {
      static uint32_t start_time_h = 0, start_time_m = 0;
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

          if (elapsed_time_seconds >= low_pressure_threshold) {
              char buffer[100];
              snprintf(buffer,sizeof(buffer),"Pressure is low for more than 2 hours \r\n");
              HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
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
      HAL_GPIO_WritePin(drain_solenoid_pin_port, drain_solenoid_pin_no, GPIO_PIN_RESET);
  //   //cell fill solenoid
      HAL_GPIO_WritePin(cell_fill_solenoid_pin_port, cell_fill_solenoid_pin_no, GPIO_PIN_RESET);
  //   //salt fill tube
      HAL_GPIO_WritePin(salt_fill_tube_pin_port, salt_fill_tube_pin_no, GPIO_PIN_RESET);

  //   //cell power signal
      HAL_GPIO_WritePin(cell_power_pin_port, cell_power_pin_no, GPIO_PIN_RESET);
  //   //salt pump off  
      HAL_GPIO_WritePin(salt_pump_pin_port, salt_pump_pin_no, GPIO_PIN_RESET);

      // add salt fill actuator
      saltfill_actuator(0);
  //   //NAOH flow controller

  //   //HOCL flow controler
  }


  char* monitor_faults()
  {
      write_state_to_flash("faults"); // Writing "idle" to flash
      read_state_from_flash(); // Reading state from flash
      static char fault_name[200] = ""; // Static array to hold concatenated fault names
      strcpy(fault_name, ""); // Initialize to empty string
      static bool saltpump_ON = false;


      bool cell_control_fault = monitor_cell_voltage();
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


      static bool prev_cell_control_status=false;
      static bool prev_controlboard_power_status=false;
      static bool prev_salt_tube_status=false;
      static bool prev_brine_conductivity_status=false;
      static bool prev_low_pressure_status=false;
      static bool prev_flow_status=false;
      static bool prev_ph_HOCL_status=false;
      static bool prev_ph_NAOH_status=false;
      static bool prev_flow_NAOH_status=false;
      static bool prev_flow_HOCL_status=false;
      static bool prev_flow_saltpump_status=false;

      static bool Iscellfillsolenoid = false;
			static uint32_t cellfill_solenoid_time=0;

      //counter for each fault occurs
      static uint32_t fault_cellcontrol_counter=0;
      static uint32_t fault_circuit_board_counter=0;
      static uint32_t fault_over_current_counter=0;
      static uint32_t fault_salt_tube_counter=0;
      static uint32_t fault_brine_conductivity_counter=0;
      static uint32_t fault_pressure_loss_counter=0;
      static uint32_t fault_flow_counter=0;
      static uint32_t fault_ph_hocl_counter=0;
      static uint32_t fault_ph_naoh_counter=0;
      static uint32_t fault_naoh_flowrate_counter=0;
      static uint32_t fault_hocl_flowrate_counter=0;
      static uint32_t fault_salt_pump_flowrate_counter=0;
      static uint32_t fault_salt_tube_level_counter=0; 


      if (cell_control_fault)
      {
          strcat(fault_name, "cell_control ");
          fault_cellcontrol_status=true;

        if (!prev_cell_control_status)
        {
          fault_cellcontrol_count++;
          //limit the number of faults
          //add the counter
          fault_cellcontrol_counter++;
          if (fault_cellcontrol_count>20){fault_cellcontrol_count=0;}
          
          //update fault_t structure
          fault_t fault_cellcontrol;
          fault_cellcontrol.counter=fault_cellcontrol_counter;
          fault_cellcontrol.fault_status=fault_cellcontrol_status;
          fault_cellcontrol.system_hours=system_time;
          fault_cellcontrol.cycle_hours=current_cycle_time;
          fault_cellcontrol.real_time_clock=sTime;
          update_fault(&my_faults,FAULT_TYPE_CELL_CONTROL , &fault_cellcontrol_count, fault_cellcontrol);

          //turn on the saltpump
          HAL_GPIO_WritePin(salt_pump_pin_port,salt_pump_pin_no,GPIO_PIN_SET);
          
          if (!Iscellfillsolenoid && system_time.systemtime_s - cellfill_solenoid_time >= 5) 
          {
          // Turn on the cell solenoid for 5 seconds
          cellfill_solenoid(true);
          Iscellfillsolenoid = true;
          cellfill_solenoid_time = system_time.systemtime_s; // Update the last change time
          }

          else if (Iscellfillsolenoid && system_time.ticks - cellfill_solenoid_time >= 5) 
          {
          // Turn off the cell solenoid for 5 seconds
          cellfill_solenoid(false);
          Iscellfillsolenoid = false;
          cellfill_solenoid_time = system_time.systemtime_s; // Update the last change time
          }
          prev_cell_control_status = true;
        }
      }
      else
      {
        prev_cell_control_status = false;
      }



      if  (!Clear_fault)
      {
      if (controlboard_power_fault)
      {
      strcat(fault_name, "control_board ");
      fault_circuit_board_status=true;
      if (!prev_controlboard_power_status)
      {
      fault_circuit_board_count++;
      //limit the number of faults
      if (fault_circuit_board_count) {fault_circuit_board_count=0;}
      //update fault_t structure
      fault_t fault_circuit_board;
      fault_circuit_board.counter=fault_circuit_board_counter;
      fault_circuit_board.fault_status=fault_circuit_board_status;
      fault_circuit_board.system_hours=system_time;
      fault_circuit_board.cycle_hours=current_cycle_time;
      fault_circuit_board.real_time_clock=sTime;
      update_fault(&my_faults,FAULT_TYPE_CKT_BOARD , &fault_circuit_board_count, fault_circuit_board);


      prev_controlboard_power_status=true;
      }
      }
      else
      {
        prev_controlboard_power_status = false;
      }


       //salt tube fault
      if (salt_tube_fault)
      {
          strcat(fault_name, "salt_tube ");
          fault_salt_tube_level_status=true;
          if (!prev_salt_tube_status)
          {
        
          fault_salt_tube_level_count++;
          if(fault_salt_tube_level_count>20){fault_salt_tube_level_count=0;}
          if (fault_salt_tube_level_count>20)

        //  fault_t fault_salt_tube_level;
         // fault_salt_tube_level.counter=fault_salt_tube_level_counter;
         // fault_salt_tube_level.fault_status=fault_salt_tube_level_status;
          //fault_salt_tube_level.system_hours=system_time;
          //fault_salt_tube_level.cycle_hours=current_cycle_time;
         // fault_salt_tube_level.real_time_clock=sTime;
         // update_fault(&my_faults,FAULT_TYPE_SALTTUBE_LEVEL , &fault_salt_tube_level_count, fault_salt_tube_level);
          prev_salt_tube_status = true;


      }
      }
      else
      {
        prev_salt_tube_status = false;
      }

       //brine conductivity fault
      if (brine_conductivity_fault)
      {
          strcat(fault_name, "brine_conductivity ");
          fault_brine_conductivity_status=true;
          if (!prev_brine_conductivity_status)
          {
          fault_brine_conductivity_count++;
          if(fault_brine_conductivity_count>20){fault_brine_conductivity_count=0;}
          //update fault_t structure
          fault_t fault_brine_conductivity;
          fault_brine_conductivity.counter=fault_brine_conductivity_counter;
          fault_brine_conductivity.fault_status=fault_brine_conductivity_status;
          fault_brine_conductivity.system_hours=system_time;
          fault_brine_conductivity.cycle_hours=current_cycle_time;
          fault_brine_conductivity.real_time_clock=sTime;
          update_fault(&my_faults,FAULT_TYPE_BRINE_CONDUCTIVITY , &fault_brine_conductivity_count, fault_brine_conductivity);
          prev_brine_conductivity_status=true;
      }
      }
      else
      {
        prev_brine_conductivity_status = false;
      }

      //low pressure fault
      if (low_pressure_fault)
      {
          strcat(fault_name, "low_pressure ");
          fault_pressure_loss_status=true;
          if (!prev_low_pressure_status)
          {
          fault_pressure_loss_count++;
          if(fault_pressure_loss_count>20){fault_pressure_loss_count=0;}
          //update fault_t structure
          fault_t fault_pressure_loss;
          fault_pressure_loss.counter=fault_pressure_loss_counter;
          fault_pressure_loss.fault_status=fault_pressure_loss_status;
          fault_pressure_loss.system_hours=system_time;
          fault_pressure_loss.cycle_hours=current_cycle_time;
          fault_pressure_loss.real_time_clock=sTime;
          update_fault(&my_faults,FAULT_TYPE_PRESSURE_LOSS , &fault_pressure_loss_count, fault_pressure_loss);
          prev_low_pressure_status=true;
          }
      }
      else
      {
        prev_low_pressure_status = false;
      }

      //flow fault
      if (flow_fault)
      {
          strcat(fault_name, "flow_fault ");
          fault_brine_conductivity_status=true;
      }
      //ph HOCL faults
      if (ph_HOCL_fault)
      {
          strcat(fault_name, "ph_HOCL ");
          fault_ph_hocl_status=true;
          
          if( !prev_ph_HOCL_status)
          {
          fault_ph_hocl_count++;
          if(fault_ph_hocl_count>20){fault_ph_hocl_count=0;}
          //update fault_t structure
          fault_t fault_ph_hocl;
          fault_ph_hocl.counter=fault_ph_hocl_counter;
          fault_ph_hocl.fault_status=fault_ph_hocl_status;
          fault_ph_hocl.system_hours=system_time;
          fault_ph_hocl.cycle_hours=current_cycle_time;
          fault_ph_hocl.real_time_clock=sTime;
          update_fault(&my_faults,FAULT_TYPE_PH_HOCL , &fault_ph_hocl_count, fault_ph_hocl);
          prev_ph_HOCL_status = true;
      }
      }
      else
      {
        prev_ph_HOCL_status = false;
      }

      //ph NAOH faults
      if (ph_NAOH_fault)
      {
          strcat(fault_name, "ph_NAOH ");
          fault_ph_naoh_status=true;
          if (!prev_ph_NAOH_status)
          {
          fault_ph_naoh_count++;
          if(fault_ph_naoh_count>20){fault_ph_naoh_count=0;}
          //update fault_t structure
          fault_t fault_ph_naoh;
          fault_ph_naoh.counter=fault_ph_naoh_counter;
          fault_ph_naoh.fault_status=fault_ph_naoh_status;
          fault_ph_naoh.system_hours=system_time;
          fault_ph_naoh.cycle_hours=current_cycle_time;
          fault_ph_naoh.real_time_clock=sTime;
          update_fault(&my_faults,FAULT_TYPE_PH_NAOH , &fault_ph_naoh_count, fault_ph_naoh);
          prev_ph_NAOH_status=true;
          }

      }
      else
      {
        prev_ph_NAOH_status = false;
      }


      //flow NAOH faults
      if (flow_NAOH_fault)
      {
          strcat(fault_name, "flow_NAOH ");
          fault_naoh_flowrate_status=false;
          if(!prev_flow_NAOH_status)
          {
          fault_naoh_flowrate_count++;
          if(fault_naoh_flowrate_count>20){fault_naoh_flowrate_count=0;}

          //update fault_t structure
          fault_t fault_naoh_flowrate;
          fault_naoh_flowrate.counter=fault_naoh_flowrate_counter;
          fault_naoh_flowrate.fault_status=fault_naoh_flowrate_status;
          fault_naoh_flowrate.system_hours=system_time;
          fault_naoh_flowrate.cycle_hours=current_cycle_time;
          fault_naoh_flowrate.real_time_clock=sTime;
          update_fault(&my_faults,FAULT_TYPE_FLOW_NAOH , &fault_naoh_flowrate_count, fault_naoh_flowrate);
          prev_flow_NAOH_status=true;
      }
      }
      else
      {
        prev_flow_NAOH_status = false;
      }

      //flow HOCL faults
      if (flow_HOCL_fault)
      {
          strcat(fault_name, "flow_HOCL ");
          fault_hocl_flowrate_status=true;
          if(!prev_flow_HOCL_status)
          {
          fault_hocl_flowrate_count++;
          if (fault_hocl_flowrate_count>20){fault_hocl_flowrate_count=0;}
          //update fault_t structure
          fault_t fault_hocl_flowrate;
          fault_hocl_flowrate.counter=fault_hocl_flowrate_counter;
          fault_hocl_flowrate.fault_status=fault_hocl_flowrate_status;
          fault_hocl_flowrate.system_hours=system_time;
          fault_hocl_flowrate.cycle_hours=current_cycle_time;
          fault_hocl_flowrate.real_time_clock=sTime;
          update_fault(&my_faults,FAULT_TYPE_FLOW_HOCL, &fault_hocl_flowrate_count, fault_hocl_flowrate);
          prev_flow_HOCL_status=true;
      }
      }
      else
      {
        prev_flow_HOCL_status = false;
      }


      //salt pump faults
      if (flow_saltpump)
      {
          strcat(fault_name, "flow_saltpump");
          fault_saltpump_flowrate_status=true;
          if(!prev_flow_saltpump_status)
          {
          fault_salt_pump_flowrate_count++;
          if(fault_salt_pump_flowrate_count>20){fault_salt_pump_flowrate_count=0;}
          //update fault_t structure
          fault_t fault_saltpump_flowrate;
          fault_saltpump_flowrate.counter=fault_salt_pump_flowrate_counter;
          fault_saltpump_flowrate.fault_status=fault_saltpump_flowrate_status;
          fault_saltpump_flowrate.system_hours=system_time;
          fault_saltpump_flowrate.cycle_hours=current_cycle_time;
          fault_saltpump_flowrate.real_time_clock=sTime;
          update_fault(&my_faults,FAULT_TYPE_FLOW_SALTPUMP , &fault_salt_pump_flowrate_count, fault_saltpump_flowrate);
          prev_flow_saltpump_status=true;
          
      }
      }
      else
      {
        prev_flow_saltpump_status = false;
      }

}
else
{
  //turn off the saltpump
  HAL_GPIO_WritePin(salt_pump_pin_port,salt_pump_pin_no,GPIO_PIN_RESET);
  fault_saltpump_flowrate_status=false;
  fault_hocl_flowrate_status=false;
  fault_naoh_flowrate_status=false;
  fault_ph_naoh_status=false;
  fault_ph_hocl_status=false;
  fault_salt_tube_level_status=false;
  fault_pressure_loss_status=false;
  fault_over_current_status=false;
  fault_under_current_status=false;
  fault_circuit_board_status=false;

}
      return fault_name;
  }

  void idle_state()
  {

    char buff_idle[200];
    snprintf(buff_idle,sizeof(buff_idle),"---------In idle state -------\r\n");
    HAL_UART_Transmit(&huart2,(uint8_t*)buff_idle,strlen(buff_idle),100);

    //In idle state function.IT will run continously when the device starts
    write_state_to_flash("idle"); // Writing "idle" to flash

    //Turn off all output signals off
    turn_outputs_off();
    monitor_faults();
    // Idle=true;

    // Turn of cell power signal off
    // monitor cell current.Equal to 0 Amps
    static bool cell_power_status =false;

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
    static bool prev_Tankfill_timeexceed_counter=false;

    
    static uint32_t Tankfill_time_exceed_hours=0,Tankfill_time_exceed_mins=0;
    static uint32_t Tankfill_timeexceed_timestamps[3] = {0};
    static uint32_t current_time_in_mins=0;
    static uint32_t Tankfill_timetrend_counter=0;
    static uint32_t Tankfill_timetrend_code=0;
    static uint32_t Tankfill_timetrend_passcode=0;
    uint32_t user_Tankfill_timetrend_passcode=0;
    static bool prev_Tankfill_time=false,prev_Tankfill_timeexceed=false;
    SystemTime system_time_fromflash_code;
    TotalCycleTime totalcycle_time_code;     
    uint32_t current_value=0;
    Measurements cell = measure_current_voltage();

    if (cell.cell_current==0)
    {
    
    //system ready 
    // if(Stop_signal && !Production && !MasterFault)

    if(Stop_signal)
    {
    snprintf(buff_idle,sizeof(buff_idle),"---------In system ready condtion-------\r\n");
    HAL_UART_Transmit(&huart2,(uint8_t*)buff_idle,strlen(buff_idle),100);     
    Idle_systemready_state=true;

    Idle_tanksfull_state=false;
    Idle_tankfill_timeexceeded_state=false;
    Idle_lowpressure_state=false;
      
    Autocycle=false;
    
    }
    
    //Tank full state
    // 
    if (HOCL_tank_level() && NAOH_tank_level() && Autocycle)
    {
    snprintf(buff_idle,sizeof(buff_idle),"---------In Tank full state condtion-------\r\n");
    HAL_UART_Transmit(&huart2,(uint8_t*)buff_idle,strlen(buff_idle),100);
    Idle_tanksfull_state=true;
    Idle_systemready_state=false;	
    Idle_tankfill_timeexceeded_state=false;
    Idle_lowpressure_state=false;
      
    //Autocycle=true;
    }

    if ((!HOCL_tank_level() || !NAOH_tank_level()) && Autocycle)
    {
    snprintf(buff_idle,sizeof(buff_idle),"---------In Tank full state condtion when low tank levels-------\r\n");
    HAL_UART_Transmit(&huart2,(uint8_t*)buff_idle,strlen(buff_idle),100);

    Idle_tanksfull_state=false;
    Idle_systemready_state=false;
    Idle_tankfill_timeexceeded_state=false;
    Idle_lowpressure_state=false;
    Production=true;
    Idle=false;
    }
    
    //LOW Pressure state
    
    if (!monitor_pressure())
    {
    snprintf(buff_idle,sizeof(buff_idle),"---------In LOW Pressure state-------\r\n");
    HAL_UART_Transmit(&huart2,(uint8_t*)buff_idle,strlen(buff_idle),100);
    Idle_lowpressure_state=true;	
    Idle_systemready_state=false;	
    Idle_tankfill_timeexceeded_state=false;
    Idle_lowpressure_state=false;		
    low_pressure_counter++;
    low_pressure_system_hours=system_time;
    low_pressure_cycle_hours=total_cycle_time;
    Autocycle=false;
    //Turn off cell fill solenoid off for 30 seconds and on for 5 seconds

  if (!Iscellfillsolenoid && system_time.systemtime_s - cellfill_solenoid_time >= 30) {
      // Turn on the solenoid after it has been off for 30 seconds

    snprintf(buff_idle,sizeof(buff_idle),"---------cellfill solenoid on for 5 seconds-------\r\n");
      HAL_UART_Transmit(&huart2,(uint8_t*)buff_idle,strlen(buff_idle),100);     
      cellfill_solenoid(true);
      Iscellfillsolenoid = true;
      cellfill_solenoid_time = system_time.systemtime_s; // Update the last change time
  }
  else if (Iscellfillsolenoid && system_time.ticks - cellfill_solenoid_time >= 5000) 
  {
      // Turn off the solenoid after it has been on for 5 seconds
      snprintf(buff_idle,sizeof(buff_idle),"---------cellfill solenoid off for 30 seconds-------\r\n");
      HAL_UART_Transmit(&huart2,(uint8_t*)buff_idle,strlen(buff_idle),100);
      cellfill_solenoid(false);
      Iscellfillsolenoid = false;
      cellfill_solenoid_time = system_time.systemtime_s; // Update the last change time
  }
    }	
    //Tank fill time exceed
    
    Tankfill_time=true;
    Tankfill_timeexceed=true;
    Tankfill_time_trend_correctpasscode=false;
    Resume_signal=false;  

    if (Tankfill_time && Tankfill_timeexceed && !Resume_signal && !Tankfill_time_trend_correctpasscode )
    {
      snprintf(buff_idle,sizeof(buff_idle),"---------tank fill time excedd condition -------\r\n");
      HAL_UART_Transmit(&huart2,(uint8_t*)buff_idle,strlen(buff_idle),100);  
      Idle_tankfill_timeexceeded_state=true;
      Idle_lowpressure_state=false;		
      Idle_systemready_state=false;	
      Idle_lowpressure_state=false;		
      Autocycle=false;
      Tankfill_timeexceed=true;
            // if(!prev_Tankfill_timeexceed_counter)
      // {
      Tankfill_timeexceed_counter++;


        // prev_Tankfill_timeexceed_counter=true;
      // }
      // else
      // {
      //   prev_Tankfill_timeexceed_counter=false;
      // }    
      Tankfill_time_exceed_system_hours=system_time;
      Tankfill_time_exceed_cycle_hours=total_cycle_time;
      
      Tankfill_time_exceed_hours=system_time.systemtime_h;
      uint32_t total_elapsed_minutes = system_time.ticks / 60000; // Convert ticks to total elapsed minutes
      current_time_in_mins = total_elapsed_minutes;

      char buff_elapsedtime[50];
      snprintf(buff_elapsedtime, sizeof(buff_elapsedtime), "current_time_in_mins: %02d:%02d\r\n",current_time_in_mins,Tankfill_timeexceed_counter);
      HAL_UART_Transmit(&huart2, (uint8_t*)buff_elapsedtime, strlen(buff_elapsedtime), 100);

       //read system time from flash
        system_time_fromflash_code = read_systemtime_in_flashmemory();
        char buffer_sys[200];
        // Format and transmit system time
        snprintf(buffer_sys, sizeof(buffer_sys), "System Time read: %02d:%02d:%02d:%03d\r\n",
                system_time_fromflash_code.systemtime_h, system_time_fromflash_code.systemtime_m,
                system_time_fromflash_code.systemtime_s, system_time_fromflash_code.ticks);
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer_sys, strlen(buffer_sys), 100);

        //read total cycle time from flash
         totalcycle_time_code = read_total_cycle_time_in_flashmemory();	
        char buffer_cyc[200];
      // Format and transmit system time
        snprintf(buffer_cyc, sizeof(buffer_cyc), "Totalcycle Time read: %02d:%02d:%02d \r\n",
                totalcycle_time_code.cycletime_h, totalcycle_time_code.cycletime_m,
                totalcycle_time_code.cycletime_s);
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer_cyc, strlen(buffer_cyc), 100);
      // Shift timestamps and add the new one
      for (int i = 3 - 1; i > 0; i--) {
          Tankfill_timeexceed_timestamps[i] = Tankfill_timeexceed_timestamps[i - 1];
      }
      Tankfill_timeexceed_timestamps[0] = current_time_in_mins;
      
      if (Tankfill_timeexceed_counter >= 3) {
        
      // Check if three occurrences happened within the last 48 hours (2880 minutes)
      if (Tankfill_timeexceed_timestamps[0] - Tankfill_timeexceed_timestamps[3 - 1] <= 2880) {
        Tankfill_timetrend_code=system_time_fromflash_code.systemtime_h-totalcycle_time_code.cycletime_h;
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



  void production_state()
  {
    write_state_to_flash("production"); // Writing "idle" to flash
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
    const uint32_t tankfill_time_limit=18000 ; //5 hours in seconds
    static uint32_t elapsed_time=0;
    uint32_t HOCL_current_production_duration_m=0;
    uint32_t HOCL_solution_production=0;
    uint32_t NAOH_current_production_duration_m=0;
    uint32_t NAOH_solution_production=0;
    
    static Measurements cell_reference = {0};
    static CellsMeasurement cells_reference ={0};
    static uint32_t low_current_trend_counter=0;
    static uint32_t under_current_standby_counter=0;

    static SystemTime low_current_trend_system_time;
    static TotalCycleTime low_current_trend_cycle_time={0},tankfill_timer={0};

    static float Total_HOCL_gallon_produced=0;
    static float Total_NAOH_gallon_produced=0;
    static uint32_t NAOH_cycle_time_elapsed_time_inminutes=0;
    static uint32_t HOCL_cycle_time_elapsed_time_inminutes=0;


      //In Priming state 
      if (Priming_sequence)
      {
      if (current_time.systemtime_s - priming_start_time <= Priming_sequence_timer)
      {
        
        cells_reference = measure_cells_current_voltages();
        cell_reference = measure_current_voltage();

        //turning on the salt pump 
        HAL_GPIO_WritePin(salt_pump_pin_port,salt_pump_pin_no,GPIO_PIN_SET);
        saltpump_status=true;

        //turning on the cell fill solenoid 
        HAL_GPIO_WritePin(cell_fill_solenoid_pin_port,cell_fill_solenoid_pin_no,GPIO_PIN_SET);
        //turn on the Salt pump control signal ON

        //turn on the salt tube for 2 seconds
        HAL_GPIO_WritePin(salt_fill_tube_pin_port, salt_fill_tube_pin_no, GPIO_PIN_SET); // Start filling

        salttube_start_time=current_time.systemtime_s;
      
      if (((current_time.systemtime_s - salttube_start_time) >= 2))
      {
          // Turn off the salt tube
          HAL_GPIO_WritePin(salt_fill_tube_pin_port, salt_fill_tube_pin_no, GPIO_PIN_RESET);
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
    HAL_GPIO_WritePin(main_cell_power_pin_port,main_cell_power_pin_no,GPIO_PIN_SET);

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
      
    // starting tankfill timer
    tankfill_timer.cycletime_h=system_time.systemtime_h - start_time.systemtime_h;
    tankfill_timer.cycletime_m=system_time.systemtime_m - start_time.systemtime_m;
    tankfill_timer.cycletime_s=system_time.systemtime_s - start_time.systemtime_s;
    elapsed_time=tankfill_timer.cycletime_s + tankfill_timer.cycletime_m * 60 + tankfill_timer.cycletime_h * 3600; 

    if (elapsed_time >= tankfill_time_limit && !HOCL_tank_level()  && !NAOH_tank_level())
    {
      uint8_t tank_buffer[100]="Tanks not filled within estimated time";
      HAL_UART_Transmit(&huart2,tank_buffer, sizeof(tank_buffer), 100);
      Flushing_sequence=true;
    }

    // 
    if (Stop_signal)
    {
      Flushing_sequence=true;
    }

    if (!HOCL_tank_level())
    {
    HOCL_cycle_time.cycletime_h=system_time.systemtime_h - start_time.systemtime_h;
    HOCL_cycle_time.cycletime_m=system_time.systemtime_m - start_time.systemtime_m;
    HOCL_cycle_time.cycletime_s=system_time.systemtime_s - start_time.systemtime_s;

    HOCL_cycle_time_elapsed_time_inminutes=(HOCL_cycle_time.cycletime_s/60) + HOCL_cycle_time.cycletime_m  + HOCL_cycle_time.cycletime_h * 60;
    //calculate the HOCL solution production calculation
    HOCL_current_production_duration_m=HOCL_cycle_time_elapsed_time_inminutes;
    Total_HOCL_gallon_produced=((flow_HOCL_value*6)/3758.411)*HOCL_current_production_duration_m;

    //calculate the HOCL solution production calculation
      
    }

    if (!NAOH_tank_level())
    {
    NAOH_cycle_time.cycletime_h=system_time.systemtime_h - start_time.systemtime_h;
    NAOH_cycle_time.cycletime_m=system_time.systemtime_m - start_time.systemtime_m;
    NAOH_cycle_time.cycletime_s=system_time.systemtime_s - start_time.systemtime_s;
    NAOH_current_production_duration_m=HOCL_cycle_time.cycletime_m;

    //calculate the NAOH solution production calculation
    NAOH_cycle_time_elapsed_time_inminutes=(NAOH_cycle_time.cycletime_s/60) + NAOH_cycle_time.cycletime_m  + NAOH_cycle_time.cycletime_h * 60;
    NAOH_current_production_duration_m=NAOH_cycle_time_elapsed_time_inminutes;
    Total_NAOH_gallon_produced=((flow_NAOH_value*6)/3758.411)*NAOH_current_production_duration_m;

    //monitor HOCL,NAOH and Saltpump flows
    //monitor PH of NAOH and HOCL

    monitor_pH_HOCL(pH_HOCL_value);
    monitor_pH_NAOH(pH_NAOH_value);		
    }
    
    
    
    
  }
  
  if (Stop_signal || (NAOH_tank_level() && HOCL_tank_level()) || monitor_pressure() || overcurrent || undercurrent || lowcurrent_trend )
  {

    if(NAOH_tank_level() && HOCL_tank_level()) {Autocycle=true;}

    flushing_start_time=system_time.systemtime_s;
    if(Flushing_sequence)
    {
    // In flushing condtion	 
    //turn off the main cell signal ON
    if(system_time.systemtime_s - flushing_start_time <= Flushing_sequence_timer)
    {
    HAL_GPIO_WritePin(main_cell_power_pin_port,main_cell_power_pin_no,GPIO_PIN_RESET);

    //reset tankfill timer
    tankfill_timer.cycletime_h=0;
    tankfill_timer.cycletime_m=0;
    tankfill_timer.cycletime_s=0;
    elapsed_time=0;
    NAOH_current_production_duration_m=0;
    NAOH_cycle_time_elapsed_time_inminutes=0;
    Production=false;
      
  }
    Idle=true;

    //exit conditions
    //turning off the salt pump 
    HAL_GPIO_WritePin(salt_pump_pin_port,salt_pump_pin_no,GPIO_PIN_RESET);
    saltpump_status=false;
 
    //turning off the cell fill solenoid 
    HAL_GPIO_WritePin(cell_fill_solenoid_pin_port,cell_fill_solenoid_pin_no,GPIO_PIN_RESET);	

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
    char time_buff[150];
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	//initialize the PWM
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	
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


    read_state_from_flash();
    //system_time structure time instance
    start_time=time_conversion();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
    // Production=true;
    currentTime=system_time.systemtime_s;
    system_time=time_conversion();
    //getUnixTime();
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    snprintf(time_buff,sizeof(time_buff),"RTC time is %d:%d:%d \r\n",sTime.Hours,sTime.Minutes,sTime.Seconds);
    HAL_UART_Transmit(&huart2,(uint8_t *)time_buff,strlen(time_buff),100);
    //HAL_Delay(5000);	

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

      
        // char* faults = monitor_faults();
      // printf("Detected Faults: %s\n", faults);

      
    //write the system hours in flashmemory after every 1 minute
    // if ((currentTime - lastSaveTime) >= 60) //write after 1 minute
    //   { 
    //       uint8_t buffer[150]=" !!!! writing time into flash memory \r\n";
    //       HAL_UART_Transmit(&huart2,buffer,sizeof(buffer),100);
    //       write_systemtime_in_flashmemory();
    //       lastSaveTime = currentTime; // Update the last save time
    //   }
       //  Entry in idle state
      // Start_signal Production MasterFault Resume_signal are all false and  idle signal is true       
      // exit from the idle state
         // if start signal or production or master fault or resumesignal or tankfilltimetrend correctpasscode or monitor pressure or any tank level is high

      // if (!Start_signal && !Production && !MasterFault && !Resume_signal && !Tankfill_time_trend_correctpasscode  ( || !monitor_pressure() || Idle))	

    //   if ((!Start_signal && !Production && !MasterFault && !Resume_signal && !Tankfill_time_trend_correctpasscode && !monitor_pressure()) || Idle)	
    //   {
    //   snprintf(buff_test,sizeof(buff_test),"---------In ideal condtion-------\r\n");
    //   HAL_UART_Transmit(&huart2,(uint8_t*)buff_test,strlen(buff_test),100);
    //   // idle_state();
    
    //   }


      

      
      // if ((Start_signal || (!NAOH_tank_level() && !HOCL_tank_level() )|| Resume_signal || Tankfill_time_trend_correctpasscode || !monitor_pressure() )&& !Idle_systemready_state && !Idle_tanksfull_state && !Idle_tankfill_timeexceeded_state && !Idle_lowpressure_state && !MasterFault && !Production)

      // move to production if their is start signal and all the idle state flags are not high
            // if ((Start_signal || Resume_signal || Tankfill_time_trend_correctpasscode || !monitor_pressure() || Autocycle ) && !Idle_systemready_state && !Idle_tanksfull_state && !Idle_tankfill_timeexceeded_state && !Idle_lowpressure_state && !MasterFault && Production)

    //   if ((Start_signal || Resume_signal || Tankfill_time_trend_correctpasscode || monitor_pressure() || Autocycle || !HOCL_tank_level() || !NAOH_tank_level() ) && !Idle_systemready_state && !Idle_tanksfull_state && !Idle_tankfill_timeexceeded_state && !Idle_lowpressure_state && !MasterFault )
    //   {
    //     // turn production on when turned entire
    //     // turn off when production off
    //     uint8_t buff[50]="---------In production condtion-------";
    //     HAL_UART_Transmit(&huart2,buff,sizeof(buff),100);
    //     production_state();

    //   }
      
      
      //if any of the fault is detected move to the fault state

      //

      
      // if(cell_voltage_fault || flow_fault || low_pressure_fault || ph_HOCL_fault || ph_NAOH_fault || brine_conductivity || no_board_power_fault )
      // {
      //   uint8_t buff[50]="---------In fault condtion-------";
      //   HAL_UART_Transmit(&huart2,buff,sizeof(buff),100);
      // }
      		
		
		
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
  sConfig.Channel = ADC_CHANNEL_4;
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
  sTime.Hours = 0x15;
  sTime.Minutes = 0x1;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  sDate.Month = RTC_MONTH_DECEMBER;
  sDate.Date = 0x22;
  sDate.Year = 0x23;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x18;
  sAlarm.AlarmTime.Minutes = 0x10;
  sAlarm.AlarmTime.Seconds = 0x15;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
  sAlarm.AlarmDateWeekDay = RTC_WEEKDAY_FRIDAY;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
