/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stn1110.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define cur_myELM327_STATUS curS;
#define pre_myELM327_STATUS preS;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
STN1110 stn;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //UART에 수신인터럽트 일어나면 이놈이 호출됨
{
 if (huart->Instance == USART6) //current UART
  {
	 stn.push(stn.rxData);
	 stn.rxFlag = true;
  }
 	 HAL_UART_Receive_IT(&huart6, &stn.rxData, 1); //다음 수신인터럽트를 준비
 }


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t rpm = 0;
uint32_t motorrpm = 0;
uint32_t kph = 0;
uint32_t engineLoad = 0;
uint16_t runTime = 0;
uint8_t fuelType = 0;
int32_t oilTemp = 0;
uint32_t relativePedalPos = 0;
uint32_t throttle = 0;
uint32_t relativeThrottle = 0;
int32_t intakeAirTemp = 0;
uint32_t fuelLevel = 0;
float mafRate = 0; // uint32_t
uint8_t obdStandards = 0;
uint16_t distTravelWithMIL = 0;
uint16_t distSinceCodesCleared = 0;
uint32_t ctrlModVoltage = 0;
int16_t ambientAirTemp = 0;
uint32_t manifoldPressure = 0;
int32_t engineCoolantTemp = 0;
float commandedThrottleActuator = 0;
float commandedAirFuelRatio = 0.0; //stn1110 Data Variables

static uint32_t counter_AccelYp;
static uint32_t counter_Fuel;
static uint32_t counter_Energy;
static uint32_t counter_Rpm;
static uint32_t counter_MotorRpm;
static float efficiencyScore;
static uint32_t counter_AccelX;
static uint32_t counter_AccelYm;
static uint32_t counter_AccelYm2;
static uint32_t counter_Time;
static uint32_t counter_KphRpm;
static uint32_t counter_KphMotorRpm;
static uint32_t counter_Throttle;
static float safetyScore;
static float drivingScore = 10.0;
static float averagedrivingScore = 10.0;
static int counter_N1 = 1;
static int counter_N2 = 1;
static int counter_N3 = 1;
static int counter_N4 = 1;
static int counter_Idle = 0;

static float AFR = 14.70;

float pre_f_accelX;
float pre_f_accelY;
float pre_instantFuelConsume;
float pre_instantEnergyConsume;
uint32_t pre_rpm;
uint32_t pre_motorrpm;
uint32_t pre_runTime;
int32_t pre_kph;
float pre_commandedThrottleActuator;
float instantFuelConsume = 12.0;
float averageFuelConsume = 12.0;
float averagecommandedThrottleActuator = 8;
float instantEnergyConsume = 12.0;
float averageEnergyConsume = 12.0;



bool flag_aws_publish = true; //flag

char car_state[30] = "";
int pwrCheck;
int rstCheck;
bool errorvaluecheck = true;
bool mafRatecheck = false;

static uint32_t elmstatusCheck = 0;

unsigned long c_time = 0;
unsigned long e_time = 0;

int HOUR = 0;
int MIN = 0;
int SEC = 0; // 자체 시간 시,분,초

int E_HOUR = 0;
int E_MIN = 0;
int E_SEC = 0; // 자체 시간 시,분,초
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  stn.queue_init();
  HAL_UART_Receive_IT(&huart6, &stn.rxData, 1);


	if (!stn.begin(&huart6, true, 2000))
	{
		printf("Couldn't connect to STN1110");
	}
	printf("Connected to STN1110\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    static int8_t cur_myELM327_STATUS; // 현재상태 정적변수
	    static int8_t pre_myELM327_STATUS; // 이전상태 정적변수

	    c_time = millis() / 1000; //시간

	    SEC = c_time % 60;
	    MIN = (c_time / 60) % 60;
	    HOUR = (c_time / (60 * 60)) % 24;

	    curS = stn.status; //현재상태 갱신

	    switch (curS) {
	         case ELM_SUCCESS:
	         strcpy(car_state, "ELM_SUCCESS");
	         break;

	         case ELM_NO_RESPONSE:
	         strcpy(car_state, "ELM_NO_RESPONSE");
	         break;

	         case ELM_BUFFER_OVERFLOW:
	         strcpy(car_state, "ELM_BUFFER_OVERFLOW");
	         break;

	         case ELM_UNABLE_TO_CONNECT:
	         strcpy(car_state, "ELM_UNABLE_TO_CONNECT");
	         break;

	         case ELM_NO_DATA:
	         strcpy(car_state, "ELM_NO_DATA");
	         break;

	         case ELM_STOPPED:
	         strcpy(car_state, "ELM_STOPPED");
	         break;

	         case ELM_TIMEOUT:
	         strcpy(car_state, "ELM_TIMEOUT");
	         break;

	         default:
	         strcpy(car_state, "UNKNOWN_ERROR");
	       }

	    float tempRPM = stn.rpm();
	    uint32_t tempVEHICLE_SPEED = stn.kph();
	    float tempENGINE_LOAD = stn.engineLoad();
	    uint16_t tempRUN_TIME_SINCE_ENGINE_START = stn.runTime();
	    uint8_t tempFUEL_TYPE = stn.fuelType();
	    float tempENGINE_OIL_TEMP = stn.oilTemp();
	    float tempENGINE_COOLANT_TEMP = stn.engineCoolantTemp();
	    float tempRELATIVE_ACCELERATOR_PEDAL_POS = stn.relativePedalPos();
	    float tempTHROTTLE_POSITION = stn.throttle();
	    float tempCOMMANDED_THROTTLE_ACTUATOR = stn.commandedThrottleActuator();
	    float tempRELATIVE_THROTTLE_POSITION = stn.relativeThrottle();
	    float tempINTAKE_AIR_TEMP = stn.intakeAirTemp();
	    uint8_t tempINTAKE_MANIFOLD_ABS_PRESSURE = stn.manifoldPressure();
	    float tempFUEL_TANK_LEVEL_INPUT = stn.fuelLevel();
	    uint8_t tempOBD_STANDARDS = stn.obdStandards();
	    float tempCONTROL_MODULE_VOLTAGE = stn.ctrlModVoltage();
	    float tempAMBIENT_AIR_TEMP = stn.ambientAirTemp();
	    uint16_t tempDISTANCE_TRAVELED_WITH_MIL_ON = stn.distTravelWithMIL();
	    uint16_t tempDIST_TRAV_SINCE_CODES_CLEARED = stn.distSinceCodesCleared();
	    float tempMAF_FLOW_RATE = stn.mafRate();
	    float tempFUEL_AIR_MANDED_EQUIV_RATIO = stn.commandedAirFuelRatio();

	    preS = curS; // 이전상태 현재상태로 갱신

	  if (curS == ELM_SUCCESS)
	  {
		  runTime = (uint16_t)tempRUN_TIME_SINCE_ENGINE_START; //엔진켜진시점 이후 운행시간
		  fuelType = (int32_t)tempFUEL_TYPE; //사용 연료정보
		  rpm = (uint32_t)tempRPM; //차량 RPM
		  kph = (uint32_t)tempVEHICLE_SPEED; //차량속도
		  engineLoad = (uint32_t)tempENGINE_LOAD; //엔진부하
		  oilTemp = (int32_t)tempENGINE_OIL_TEMP; //오일온도
		  engineCoolantTemp = (int32_t)tempENGINE_COOLANT_TEMP; //엔진 냉각수 온도
		  relativePedalPos = (uint32_t)tempRELATIVE_ACCELERATOR_PEDAL_POS;
		  throttle = (uint32_t)tempTHROTTLE_POSITION; //스로틀 포지션
		  relativeThrottle = (uint32_t)tempRELATIVE_THROTTLE_POSITION; //상대 스로틀 포지션
		  commandedThrottleActuator = (float)tempCOMMANDED_THROTTLE_ACTUATOR; //스로틀 엑츄에이터
		  intakeAirTemp = (int32_t)tempINTAKE_AIR_TEMP; //흡입공기 온도
		  mafRate = (float)tempMAF_FLOW_RATE; //공기유량
		  manifoldPressure = (uint8_t)tempINTAKE_MANIFOLD_ABS_PRESSURE; //흡기매니폴드 절대압력
		  ambientAirTemp = (int16_t)tempAMBIENT_AIR_TEMP; //외기온도
		  distTravelWithMIL = (uint16_t)tempDISTANCE_TRAVELED_WITH_MIL_ON; //경고등 점등이후 주행거리
		  distSinceCodesCleared = (uint16_t)tempDIST_TRAV_SINCE_CODES_CLEARED; //DTC 소거후 주행거리
		  fuelLevel = (uint32_t)tempFUEL_TANK_LEVEL_INPUT; //연료레벨
		  ctrlModVoltage = (uint32_t)tempCONTROL_MODULE_VOLTAGE; //컨트롤 모듈 전압
		  obdStandards = (uint8_t)tempOBD_STANDARDS; //OBD 정보 - 수치값 wikipedia 검색
		  commandedAirFuelRatio = (float)tempFUEL_AIR_MANDED_EQUIV_RATIO;
	   	  printf("RPM :");
	 	  printf("%lu\r\n", rpm);
 	  }

	  else
		  stn.printError(curS);
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
