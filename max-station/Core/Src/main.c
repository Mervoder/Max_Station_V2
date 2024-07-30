/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwgps/lwgps.h"
#include "w25q.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LORA_RX_BUFFER_SIZE 70
#define RX_BUFFER_SIZE 128
#define HYI_BUFFER_SIZE 77

#define TAKIM_ID 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t lora_rx_buffer[LORA_RX_BUFFER_SIZE];
uint8_t rx_index_lora=0;
uint8_t rx_data_lora=0;

uint8_t HYI_BUFFER[HYI_BUFFER_SIZE];

uint8_t gps_rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index=0;
uint8_t rx_data=0;

uint8_t Cmd_End[3] = {0xff,0xff,0xff};

const double PI = 4.0*atan(1.0);
const double radius_of_earth = 6378136.0474;
double _distance=0;
double _angle=0;

///////////////sustainer
uint8_t sustgpssatsinview=0;
float sustgpsaltitude=0;
float sustgpslatitude=0;
float sustgpslongitude=0;
float sustspeed=0;
float sustaltitude=0;
float susttemperature=0;
float sustaccx=0;
float sustaccy=0;
float sustaccz=0;
float sustroll=0;
float sustpitch=0;
uint8_t sustv4_battery=0;
uint8_t sustv4_mod=0;
uint8_t suststage_communication=0;

//egu
uint8_t EGU_ARIZA=0;
uint8_t EGU_AYRILMA_TESPIT=0;
uint8_t EGU_MOTOR_ATESLEME_TALEP_IN=0;
uint8_t EGU_STAGE_DURUM=0;
uint8_t EGU_UCUS_BASLADIMI=0;
uint8_t EGU_FITIL =0;

float EGU_BATTERY=0;
float EGU_IRTIFA=0;
float EGU_ANGLE=0;

////////////BOOOSTER

uint8_t boostgpssatsinview=0;
float boostgpsaltitude=0;
float boostgpslatitude=0;
float boostgpslongitude=0;
float boostspeed=0;
float boostaltitude=0;
float boosttemperature=0;
float boostaccx=0;
float boostaccy=0;
float boostaccz=0;
float boostroll=0;
float boostpitch=0;
uint8_t boostv4_battery=0;
uint8_t boostv4_mod=0;
uint8_t booststage_communication=0;



uint32_t tim1=0;
uint32_t takim_sayac;


uint8_t BUTTON_STATE=0;

uint8_t writeData[50] = {0,1,1,1};
uint8_t readData[50] = {0};


lwgps_t gps;

typedef union{
  float fVal;
  unsigned char array[4];
}float2unit8;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void E220_CONFIG(uint8_t ADDH, uint8_t ADDL, uint8_t CHN, uint8_t MODE);
double distance_in_m(double lat1, double long1, double lat2, double long2);
double toRadians(double degree);
double calculateAngle(double lat1, double lon1, double lat2, double lon2);
void NEXTION_SendString (char *ID, char *string);
void NEXTION_SendNum (char *obj, int32_t num);
void NEXTION_SendFloat (char *obj, float num, int dp);
void HYI_BUFFER_Fill();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3){
	if(rx_data_lora != '\n'&& rx_index_lora < LORA_RX_BUFFER_SIZE){
	lora_rx_buffer[rx_index_lora++]=rx_data_lora;

	}
	else{
		rx_data_lora=0;
		rx_index_lora=0;

		}
HAL_UART_Receive_IT(&huart3, &rx_data_lora, 1);
	}

//	if(huart == &huart2) {
//			if( rx_index < 70) {
//				gps_rx_buffer[rx_index++] = rx_data;
//			} else {
//				lwgps_process(&gps, gps_rx_buffer, rx_index+1);
//				rx_index = 0;
//				rx_data = 0;
//			}
//			HAL_UART_Receive_IT(&huart2, &rx_data, 1);
//		}
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3, &rx_data_lora, 1);
//  HAL_UART_Receive_IT(&huart2,&rx_data, 1);
  E220_CONFIG(0x8,0x2A,0x10,1);
  lwgps_init(&gps);


  tim1=HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  if(HAL_GetTick()-tim1>500)
{
		  HYI_BUFFER_Fill();

	  if(lora_rx_buffer[3]==2){

		  sustgpssatsinview=lora_rx_buffer[4];

			 float2unit8 f2u8_gpsalt;
				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_gpsalt.array[i]=lora_rx_buffer[i+5];
					 HYI_BUFFER[10+i] =lora_rx_buffer[i+5]; // 10 11 12 13
				 }
				 sustgpsaltitude=f2u8_gpsalt.fVal;
			 float2unit8 f2u8_latitude;

				 for(uint8_t i=0;i<4;i++)
				 {
					f2u8_latitude.array[i]=lora_rx_buffer[i+9];
					HYI_BUFFER[14+i] =lora_rx_buffer[i+9]; // 14 15 16 17
				 }
				 sustgpslatitude=f2u8_latitude.fVal;

			 float2unit8 f2u8_longitude;
				 for(uint8_t i=0;i<4;i++)
				 {
					f2u8_longitude.array[i]=lora_rx_buffer[i+13];
					HYI_BUFFER[18+i] =lora_rx_buffer[i+13]; // 18 19 20 21
				 }
				 sustgpslongitude=f2u8_longitude.fVal;

			 float2unit8 f2u8_altitude;
				 for(uint8_t i=0;i<4;i++)
				 {
					f2u8_altitude.array[i]=lora_rx_buffer[i+17];
					HYI_BUFFER[6+i] =lora_rx_buffer[i+17]; // 6 7 8 9
				 }
				 sustaltitude=f2u8_altitude.fVal;

			 float2unit8 f2u8_speed;

				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_speed.array[i]=lora_rx_buffer[i+21];
				 }
				 sustspeed=f2u8_speed.fVal;

			 float2unit8 f2u8_temp;
				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_temp.array[i]=lora_rx_buffer[i+25];
				 }
				 susttemperature=f2u8_temp.fVal;

			 float2unit8 f2u8_accx;
				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_accx.array[i]=lora_rx_buffer[i+29];
					 HYI_BUFFER[58+i]=lora_rx_buffer[i+29]; //
				 }
				 sustaccx=f2u8_accx.fVal;

			float2unit8 f2u8_accy;
				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_accy.array[i]=lora_rx_buffer[i+33];
					 HYI_BUFFER[62+i]=lora_rx_buffer[i+33];
				 }
					 sustaccy=f2u8_accy.fVal;

			float2unit8 f2u8_accz;
			      for(uint8_t i=0;i<4;i++)
				 {
			    	  f2u8_accz.array[i]=lora_rx_buffer[i+37];
			    	  HYI_BUFFER[66+i]=lora_rx_buffer[i+37];
				 }
					 sustaccz=f2u8_accz.fVal;

			float2unit8 f2u8_roll;
				  for(uint8_t i=0;i<4;i++)
				 {
					  f2u8_roll.array[i]=lora_rx_buffer[i+41];
				 }
					 sustroll=f2u8_roll.fVal;

			float2unit8 f2u8_pitch;
				  for(uint8_t i=0;i<4;i++)
				 {
					  f2u8_pitch.array[i]=lora_rx_buffer[i+45];
				 }
					 sustpitch=f2u8_pitch.fVal;

					 sustv4_battery=lora_rx_buffer[49];
					 sustv4_mod=lora_rx_buffer[50];
					 suststage_communication=lora_rx_buffer[51];

					 //EGU PART
					 EGU_ARIZA=lora_rx_buffer[52];
					 EGU_AYRILMA_TESPIT=lora_rx_buffer[53];

			float2unit8 f2u8_EGU_BATTERY;
					for(uint8_t i=0;i<4;i++)
				 {
						f2u8_EGU_BATTERY.array[i]=lora_rx_buffer[i+54];
				 }
					 EGU_BATTERY=f2u8_EGU_BATTERY.fVal;

			float2unit8 f2u8_EGU_ANGLE;
					for(uint8_t i=0;i<4;i++)
				{
						f2u8_EGU_ANGLE.array[i]=lora_rx_buffer[i+58];
				 }
					  EGU_ANGLE=f2u8_EGU_ANGLE.fVal;

			float2unit8 f2u8_EGU_IRTIFA;
					for(uint8_t i=0;i<4;i++)
				{
						f2u8_EGU_IRTIFA.array[i]=lora_rx_buffer[i+62];
				}
					  EGU_IRTIFA=f2u8_EGU_IRTIFA.fVal;

					  EGU_FITIL=lora_rx_buffer[53];

					  EGU_UCUS_BASLADIMI=lora_rx_buffer[66];
					  EGU_STAGE_DURUM=lora_rx_buffer[67];
					  EGU_MOTOR_ATESLEME_TALEP_IN=lora_rx_buffer[68];
}

	  if(lora_rx_buffer[3]==1){

		  boostgpssatsinview=lora_rx_buffer[4];

			 float2unit8 f2u8_bgpsalt;
				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_bgpsalt.array[i]=lora_rx_buffer[i+5];
					 HYI_BUFFER[34+i]=lora_rx_buffer[i+5]; // 34 35 36 37
				 }
				 sustgpsaltitude=f2u8_bgpsalt.fVal;
			 float2unit8 f2u8_blatitude;

				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_blatitude.array[i]=lora_rx_buffer[i+9];
					 HYI_BUFFER[38+i]=lora_rx_buffer[i+9]; // 38 39 40 41
				 }
				 boostgpslatitude=f2u8_blatitude.fVal;

			 float2unit8 f2u8_blongitude;
				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_blongitude.array[i]=lora_rx_buffer[i+13];
					 HYI_BUFFER[42+i]=lora_rx_buffer[i+13]; // 42 43 44 45
				 }
				 boostgpslongitude=f2u8_blongitude.fVal;

			 float2unit8 f2u8_baltitude;
				 for(uint8_t i=0;i<4;i++)
				 {
					f2u8_baltitude.array[i]=lora_rx_buffer[i+17];
				 }
				 boostaltitude=f2u8_baltitude.fVal;

			 float2unit8 f2u8_bspeed;

				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_bspeed.array[i]=lora_rx_buffer[i+21];
				 }
				 boostspeed=f2u8_bspeed.fVal;

			 float2unit8 f2u8_btemp;
				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_btemp.array[i]=lora_rx_buffer[i+25];
				 }
				 boosttemperature=f2u8_btemp.fVal;

			 float2unit8 f2u8_baccx;
				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_baccx.array[i]=lora_rx_buffer[i+29];
				 }
				 boostaccx=f2u8_baccx.fVal;

			float2unit8 f2u8_baccy;
				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_baccy.array[i]=lora_rx_buffer[i+33];
				 }
					 boostaccy=f2u8_baccy.fVal;

			float2unit8 f2u8_baccz;
			      for(uint8_t i=0;i<4;i++)
				 {
			    	  f2u8_baccz.array[i]=lora_rx_buffer[i+37];
				 }
					 boostaccz=f2u8_baccz.fVal;

			float2unit8 f2u8_broll;
				  for(uint8_t i=0;i<4;i++)
				 {
					  f2u8_broll.array[i]=lora_rx_buffer[i+41];
				 }
					 boostroll=f2u8_broll.fVal;

			float2unit8 f2u8_bpitch;
				  for(uint8_t i=0;i<4;i++)
				 {
					  f2u8_bpitch.array[i]=lora_rx_buffer[i+45];
				 }
					 boostpitch=f2u8_bpitch.fVal;

					 boostv4_battery=lora_rx_buffer[49];
					 boostv4_mod=lora_rx_buffer[50];
					 booststage_communication=lora_rx_buffer[51];


}




	  tim1=HAL_GetTick();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  sConfig.Channel = ADC_CHANNEL_12;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, CS_Pin|BUZZER_Pin|GATE_D_Pin|GATE_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M0_Pin|M1_Pin|FN_Pin|LED2_Pin
                          |LED1_Pin|GATE_B_Pin|GATE_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin BUZZER_Pin GATE_D_Pin GATE_C_Pin */
  GPIO_InitStruct.Pin = CS_Pin|BUZZER_Pin|GATE_D_Pin|GATE_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M0_Pin M1_Pin FN_Pin LED2_Pin
                           LED1_Pin GATE_B_Pin GATE_A_Pin */
  GPIO_InitStruct.Pin = M0_Pin|M1_Pin|FN_Pin|LED2_Pin
                          |LED1_Pin|GATE_B_Pin|GATE_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SECINP_Pin */
  GPIO_InitStruct.Pin = SECINP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SECINP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void E220_CONFIG(uint8_t ADDH, uint8_t ADDL, uint8_t CHN, uint8_t MODE)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
    HAL_Delay(50);

    char cfg_buff[8] = {0}; // E220 için 8 elemanlı bir dizi kullanıyoruz
    enum {Transparent, Fixed} mode;
    mode = MODE;

    cfg_buff[0] = ADDH;
    cfg_buff[1] = ADDL;
    cfg_buff[2] = 0x62;
    cfg_buff[3] = 0x00;
    cfg_buff[4] = CHN;

    switch(mode){
        case Transparent:
            cfg_buff[5] = 0x00;  // opsiyon
            break;
        case Fixed:
            cfg_buff[5] = 0x11;
            break;
        default:
            cfg_buff[5] = 0x11;
     }

     cfg_buff[6] = 0x00;
     cfg_buff[7] = 0x00;


    HAL_UART_Transmit(&huart3, (uint8_t*) cfg_buff, 8, 1000);

    HAL_Delay(25);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
    HAL_Delay(25);
}
double distance_in_m(double lat1, double long1, double lat2, double long2) {


    double dlat1=lat1*(PI/180);

    double dlong1=long1*(PI/180);
    double dlat2=lat2*(PI/180);
    double dlong2=long2*(PI/180);

    double dLong=dlong1-dlong2;
    double dLat=dlat1-dlat2;

    double aHarv= pow(sin(dLat/2.0),2.0)+cos(dlat1)*cos(dlat2)*pow(sin(dLong/2),2);
    double cHarv=2*atan2(sqrt(aHarv),sqrt(1.0-aHarv));

    double distance=radius_of_earth*cHarv;
    return (distance);
    }
double toRadians(double degree) {
    return (degree * PI / 180.0);
}

double calculateAngle(double lat1, double lon1, double lat2, double lon2){
    double lat1_rad = toRadians(lat1);
    double lat2_rad = toRadians(lat2);
    double delta_lon = toRadians(lon2 - lon1);
    double y = sin(delta_lon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon);
    double angle_rad = atan2(y, x);
    double angle_deg = angle_rad * 180.0 / PI;
    return angle_deg;
}

void NEXTION_SendString (char *ID, char *string)
{
	char buf[50];
	int len = sprintf (buf, "%s.txt=\"%s\"", ID, string);
	HAL_UART_Transmit(&huart4, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);
}


void NEXTION_SendNum (char *obj, int32_t num)
{
	uint8_t *buffer = malloc(30*sizeof (char));
	int len = sprintf ((char *)buffer, "%s.val=%ld", obj, num);
	HAL_UART_Transmit(&huart4, buffer, len, 1000);
	HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);
	free(buffer);
}


void NEXTION_SendFloat (char *obj, float num, int dp)
{
	// convert to the integer
	int32_t number = num*(pow(10,dp));

	uint8_t *buffer = malloc(30*sizeof (char));
	int len = sprintf ((char *)buffer, "%s.vvs1=%d", obj, dp);
	HAL_UART_Transmit(&huart4, buffer, len, 1000);
	HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);


	len = sprintf ((char *)buffer, "%s.val=%ld", obj, number);
	HAL_UART_Transmit(&huart4, buffer, len, 1000);
	HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);
	free(buffer);
}

void HYI_BUFFER_Fill()
{
	HYI_BUFFER[0] =0xFF;
	HYI_BUFFER[1] =0xFF;
	HYI_BUFFER[3] =0x54;
	HYI_BUFFER[4] =TAKIM_ID;
	HYI_BUFFER[5] =takim_sayac;
	HYI_BUFFER[74]= EGU_AYRILMA_TESPIT;
	HYI_BUFFER[75]= 0; // CRC
	HYI_BUFFER[76]= 0x0D;
	HYI_BUFFER[77]= 0x0A;







}

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
