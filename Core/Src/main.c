/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "rtc.h"
#include "tim.h"
#include "gpio.h"
//#include "app_mems.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_x-cube-mems1.h"
//Quelle est la cible?
#define __STR2__(x) #x
#define __STR1__(x) __STR2__(x)
#pragma message("***************************")
#if defined(STM32F401xE)
#pragma message("Compiling for NUCLEO_F401RE")     //with '-fno-diagnostics-show-caret'
//----- end F401RE -----
#elif defined(STM32L476xx)
#pragma message("Compiling for NUCLEO_L476RG")  //with '-fno-diagnostics-show-caret'
//----- end L476RG ------
#else
#pragma message("warning: Unknown TARGET")
#endif
#pragma message("le " __STR1__(__DATE__)" "__STR1__(__TIME__))
#pragma message("---------------------------")
#pragma message("program " __STR1__(yPROG))
#pragma message("version " __STR1__(yVER))
#pragma message("CubeMX  " __STR1__(yCubeMX))
#pragma message("***************************\n")

#include "com.h"
#include "DemoSerial.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "screen.h"
#include "VT100.h"
#include "iks01a3_motion_sensors.h"
#include "iks01a3_env_sensors.h"
#include "yMEMs.h"				//JPO utils sensors
#include "yI2CprogsLCD.h"		//gestion LCD-RGB
#include "gui.h"
#include "CubeMon.h"			//pour STM32CubeMonitor (Node-rED)

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
       set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

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

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define MAX_BUF_SIZE 256
char aTxBuffer[TMsg_MaxLen * 4];		    //buffer d'emission (UART & LCD)
uint8_t aRxBuffer[UART_RxBufferSize];		//buffer de reception

volatile uint8_t BP1Detected = 0;	//flag detection interrupt BP bleu
volatile int comVTcomGUI;			//0=comm via VT, 1=comm via Unicleo-GUI
volatile int yGPio;
float j,k;

extern I2C_HandleTypeDef hi2c1;

extern volatile uint8_t DataLoggerActive;		/* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
volatile uint8_t DataLoggerActive;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t sensorId1,sensorId2,sensorId3;
RTC_TimeTypeDef myTime;
RTC_DateTypeDef myDate;
extern UART_HandleTypeDef UartHandle;

/*
 * Allume/eteint la LED au rythme du TIM6
 *  avec/sans tracage de l'heure
 */
void yLedStartStop(int StartStop, int optTime) {
	if (StartStop == 1) {
		HAL_TIM_Base_Start_IT(&htim1);
		//optPrintTime = optTime;
	} else {
		HAL_TIM_Base_Stop_IT(&htim1);
		//optPrintTime = 0;
	}
}

/*
 * Display date & time from real Time Clock (inside RTC)
 */
void yPrintTime(void) {
	  HAL_RTC_GetTime(&hrtc, &myTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &myDate, RTC_FORMAT_BIN);	//need to read also the date!!!
	  snprintf(aTxBuffer, 1024, "\t==> %02d-%02d-%02d %02d:%02d:%02d\r\n",
			  	  	  	  	  	  	  myDate.Date, myDate.Month, myDate.Year,
									  myTime.Hours, myTime.Minutes, myTime.Seconds);
	  HAL_UART_Transmit(&UartHandle,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
}

/*
 * @brief  appelé cycliquement via TIM1
 * 			affiche infos MEMs sur LCD
 * @param  none
 * @retval none
 */
void yDisplayMEMs() {
	//snprintf(aTxBuffer, 9, "%02d:%02d:%02d", hh, mm, sc);
	snprintf(aTxBuffer, 12, "MEMs values");
	yI2C_LCD_locate(1,1);
	yI2C_LCD_Affich_Txt(aTxBuffer);
}

/*
 * EXTI15_10 callback
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/** PC13 (B1 blue button) */
	//if(GPIO_Pin == B1_Pin) {
	//if(GPIO_Pin == GPIO_PIN_13) {
	if(GPIO_Pin == USER_BUTTON_PIN) {
		//MU flag detection interrupt, traitement ds le while du main
		BP1Detected = 1;
	}

	/** PC1	GPIO_EXTI1	(STTS751_INT) */
	if(GPIO_Pin == STTS751_INT_Pin) {
		//snprintf(aTxBuffer, 1024, "\n\t--STTS751 Interrupt");
		/* trop repetitif!! */	//HAL_UART_Transmit(&UartHandle,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
		//yMEMS_Env_Sensors_Infos(IKS01A3_STTS751_0);
	}

	/** autre entree interrupt */
}

/*
 * TIMers callback
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	/** TIM1 - clignotement LD2 */
	if(htim->Instance == TIM1) {
		//HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		BSP_LED_Toggle(LED2);
	} //TIM1

	/** TIM11 - affichage heure sur LCD */
//	if(htim->Instance == TIM11) {
//		//Afficher date & heure
////		yDisplayDate();
////		yDisplayHeure();
//	} //TIM11

	/* manage an other TIM if any! */
}	//TIMs callback


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
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_MEMS_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize UART via com.c a la mode STM, pas via CubeMX*/
  USARTConfig();

  /* Initialize RTC */
//  RTC_Config();
//  RTC_TimeStampConfig();

  //message de bienvenue
   snprintf(aTxBuffer, 1024, clrscr homescr
 		  	  	  	  	   "\n\nBonjour maître!"
 		  	  	  	  	  	 "\n(c)Jean92, " yDATE
 							 "\n* " yPROG " * " yVER
 						     "\nCompil: " __TIME__" on "__DATE__
 						     "\nSTmicro NUCLEO_F401RE"
 							 "\r\n");
   HAL_UART_Transmit(&UartHandle,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);

   //complement de titre du programme
   snprintf(aTxBuffer, 1024, "==> IKS01A3_TestDataLogTerminal(vt/gui)\r\n");
   HAL_UART_Transmit(&UartHandle,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
   //test led Led on Nucleo
   for (int ii = 0; ii < 20; ++ii) {
     //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
     BSP_LED_On(LED_GREEN);
     HAL_Delay(30);
     //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
     BSP_LED_Off(LED_GREEN);
     HAL_Delay(30);
   }

   //init LCD, msg de bienvenue
   yI2C_LCD_init();
   HAL_Delay(50);
   yI2C_LCD_locate(0,0); yI2C_LCD_Affich_Txt(yPROG);
   yI2C_LCD_locate(0,1); yI2C_LCD_Affich_Txt(yVER);
   yI2C_LCD_locate(7,1); yI2C_LCD_Affich_Txt(__TIME__);
   HAL_Delay(500);

   //demarrer les timers
   HAL_TIM_Base_Start_IT(&htim1);

   /* other inits */
   BP1Detected = 1;
   comVTcomGUI = 1;
   //detailler les sensors MEMs
   snprintf(aTxBuffer, 1024, " Sensor\t\tId\tSts\r\n");
   HAL_UART_Transmit(&UartHandle,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
   yMEMS_Env_Sensors_Infos(IKS01A3_HTS221_0);
   yMEMS_Env_Sensors_Infos(IKS01A3_LPS22HH_0);
   yMEMS_Env_Sensors_Infos(IKS01A3_STTS751_0);
   yMEMS_Motion_Sensors_Infos(IKS01A3_LSM6DSO_0);
   yMEMS_Motion_Sensors_Infos(IKS01A3_LIS2DW12_0);
   yMEMS_Motion_Sensors_Infos(IKS01A3_LIS2MDL_0);

   //signature pour Node-RED
   snprintf(nr_Prog, 40, yPROG " * " yVER);		//yPROG " * " yVER);

   //afficher fin des inits
   snprintf(aTxBuffer, 1024, "\t--- fin inits ---\n" DECSC);
   HAL_UART_Transmit(&UartHandle,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
   if (yTRC)    printf("\n\n*********  fin inits  ***********");

   comVTcomGUI = 0;
   snprintf(aTxBuffer, 512, CUP(10,50) "Mode VT actif %d" ERASELINE DECRC, comVTcomGUI);
   HAL_UART_Transmit(&UartHandle,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);

   HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
   {
	   /* debounce & check BP1 when interrupt ou receive from STM32CubeMonitor */
	   if (BP1Detected != 0U)
	   {
		   /* Debouncing */
		   HAL_Delay(50);
		   /* Wait until the button is released */
		   //while ((BSP_PB_GetState( BUTTON_KEY ) == PushButtonState));
		   while ((BSP_PB_GetState( BUTTON_KEY ) == 1));
		   //while ( HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != GPIO_PIN_SET );
		   /* Debouncing */
		   HAL_Delay(50);
		   /* Reset Interrupt flag */
		   BP1Detected = 0;
		   /* define VT ou GUI */
		   if (comVTcomGUI == 0U) {	//comVT active?
			   //passer en comGUI
			   comVTcomGUI = 1U;
			   snprintf(aTxBuffer, 512, CUP(10,50) "Mode Unicleo-GUI actif %d" ERASELINE DECRC, comVTcomGUI);
			   HAL_UART_Transmit(&UartHandle,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
			   // erase sensors display
			   snprintf(aTxBuffer, 512, CUP(18,50) "\n" ERASELINE "\n" ERASELINE "\n" ERASELINE "\n" ERASELINE "\n" ERASELINE
					   "\n" ERASELINE "\n" ERASELINE "\n" ERASELINE "\n" ERASELINE "\n" ERASELINE "\n" ERASELINE "\n" ERASELINE
					   "\n" ERASELINE "\n" ERASELINE "\n" ERASELINE "\n" ERASELINE DECRC);
			   HAL_UART_Transmit(&UartHandle,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
			   //			yI2C_LCD_clear(); HAL_Delay(45);
			   //			snprintf(aTxBuffer, 16, "-> GUI %d %d      ", comVTcomGUI, DataLoggerActive);
			   //			yI2C_LCD_locate(0,0); yI2C_LCD_Affich_Txt(aTxBuffer);
			   //			I2C_LCD_locate(0,0); yI2C_LCD_Affich_Txt("-> com via GUI  ");
		   }
		   else {						//comGUI active
			   //passer en comVT
			   comVTcomGUI = 0U;
			   snprintf(aTxBuffer, 512, CUP(10,50) "Mode VT actif %d" ERASELINE DECRC, comVTcomGUI);
			   HAL_UART_Transmit(&UartHandle,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
			   //			snprintf(aTxBuffer, 16, "<- VT  %d %d      ", comVTcomGUI, DataLoggerActive);
			   //			yI2C_LCD_locate(0,1); yI2C_LCD_Affich_Txt(aTxBuffer);
			   //			yI2C_LCD_clear(); HAL_Delay(45);
			   //			yI2C_LCD_locate(0,1); yI2C_LCD_Affich_Txt("<- com via VT  ");
		   }
	   }	//BP1detected

	   if (comVTcomGUI == 0) {		//comVT active?
		   //MX_MEMS_Process();
		   //write from STM32CubeMon
		   snprintf(aTxBuffer, 1024, CUP(15,50) "nr_Prog  : %s" ERASELINE DECRC, nr_Prog);
		   HAL_UART_Transmit(&UartHandle,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
		   snprintf(aTxBuffer, 1024, CUP(17,50) "nr_w_Cpt : %d" ERASELINE DECRC, nr_w_Compteur);
		   HAL_UART_Transmit(&UartHandle,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);

		   HAL_Delay(500);
	   }
	   else {						//comGUI Unicleo-GUI active
		   (void) GUI_DataLog_Manage();
	   }
	   /* check GPIO pin for debug */
	   yGPio = HAL_GPIO_ReadPin(STTS751_INT_GPIO_Port, STTS751_INT_Pin);
	   printf("\n--STTS751_int etat: %x", yGPio);

	   //dans tous les cas VT ou GUI
	   /* pour Node-RED */
	   // lire l'horloge
	   HAL_RTC_GetTime(&hrtc, &myTime, RTC_FORMAT_BIN);
	   HAL_RTC_GetDate(&hrtc, &myDate, RTC_FORMAT_BIN);	//need to read also the date!!!
	   // MEMS sensors
	   IKS01A3_ENV_SENSOR_GetValue(IKS01A3_STTS751_0, ENV_TEMPERATURE, &nr_STTS751_Temp);
	   IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_TEMPERATURE, &nr_HTS221_Temp);
	   IKS01A3_ENV_SENSOR_GetValue(IKS01A3_LPS22HH_0, ENV_TEMPERATURE, &nr_LPS22HH_Temp);


    /* USER CODE END WHILE */

  MX_MEMS_Process();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
