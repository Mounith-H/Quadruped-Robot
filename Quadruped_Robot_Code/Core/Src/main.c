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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pca9685.h"
#include "MPU6050.h"
#include "Robot.h"
//#include "Common_Functions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  char* command;
  int radius;
  int length;
  int interPoints;
  int softDelay;
} UARTData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
#define TRUE  1
#define FALSE 0
#define PI 3.14
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#define rx_data_Size 13
#define UART_DELAY 10 //in ms
#define Selfsoftdelay 50 // in ms
#define SelfInterpoints 40
#define neutralPosLength 15 // in cm
#define sitDownLength 15 // in cm

// Struct holding information of UART Data Received

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
UARTData uartData;

uint8_t rx_data[rx_data_Size];
bool DataReceaved = FALSE;
bool runOnceFlag = FALSE;
bool calibrateFlag = FALSE;
bool neutralFlag = FALSE;
char * delimeter = ",";
uint8_t initialHeight = 15;
double t1[128];
double t2[128];

double th1[40] = {113.1953,114.5020,115.7947,117.0730,118.3369,119.5862,120.8205,122.0396,123.2431,124.4306,125.6017,126.7560,127.8929,129.0121,130.1129,131.1948,132.2574,133.3000,134.3221,135.3232,136.3026,137.2597,138.1941,139.1052,139.9923,140.8549,141.6925,142.5045,143.2903,144.0496,144.7816,145.4860,146.1622,146.8098,147.4281,148.0168,148.5754,149.1033,149.6001,150.0652};

double th2[40] = {104.1674,103.4252,102.7242,102.0640,101.4441,100.8641,100.3238,99.8230,99.3613,98.9387,98.5549,98.2098,97.9033,97.6353,97.4057,97.2145,97.0616,96.9469,96.8705,96.8323,96.8323,96.8705,96.9469,97.0616,97.2145,97.4057,97.6353,97.9033,98.2098,98.5549,98.9387,99.3613,99.8230,100.3238,100.8641,101.4441,102.0640,102.7242,103.4252,104.1674};

double ts1[40] = {113.1953,120.0954,123.4125,126.1872,128.7008,131.0592,133.3146,135.4960,137.6205,139.6980,141.7337,143.7297,145.6859,147.5999,149.4683,151.2863,153.0483,154.7477,156.3777,157.9309,159.3997,160.7764,162.0532,163.2225,164.2765,165.2074,166.0076,166.6688,167.1825,167.5388,167.7267,167.7322,167.5374,167.1182,166.4394,165.4470,164.0491,162.0658,159.0311,150.0652};

double ts2[40] = {104.1674,90.3440,84.9274,80.9240,77.6719,74.9164,72.5285,70.4329,68.5818,66.9427,65.4932,64.2171,63.1026,62.1411,61.3260,60.6526,60.1174,59.7180,59.4526,59.3201,59.3201,59.4526,59.7180,60.1174,60.6526,61.3260,62.1411,63.1026,64.2171,65.4932,66.9427,68.5818,70.4329,72.5285,74.9164,77.6719,80.9240,84.9274,90.3440,104.1674};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

double sind(double x){return sin(x * M_PI / 180);}

double asind(double x){return asin(x) / M_PI * 180;}

double cosd(double x){return cos(x * M_PI / 180);}

double acosd(double x){return acos(x) / M_PI * 180;}

double tand(double x){return tan(x * M_PI / 180);}

double atand(double x){return atan(x) / M_PI * 180;}

void DisplayInitialMsg()
{
  HAL_UART_Transmit(&huart1, (uint8_t *)"------------------------------------\r\n", 38, UART_DELAY);
  HAL_UART_Transmit(&huart1, (uint8_t *)"INITILIZING QUADRUPED ROBOT\r\n", 29, UART_DELAY);
  HAL_UART_Transmit(&huart1, (uint8_t *)"PROJECT BY\r\n", 12, UART_DELAY);
  HAL_UART_Transmit(&huart1, (uint8_t *)"MOUNITH H          -- 1RV23EC406\r\n", 34, UART_DELAY); // Mounith H -- 1RV23EC406
  HAL_UART_Transmit(&huart1, (uint8_t *)"JEEVOTTAM HEBLE    -- 1RV22EC072\r\n", 34, UART_DELAY); // Jeevottam Heble, 1RV22EC072
  HAL_UART_Transmit(&huart1, (uint8_t *)"KUSHAL P KOUNDINYA -- 1RV22EC083\r\n", 34, UART_DELAY); // Kushal P Koundinya -- 1RV22EC083
  HAL_UART_Transmit(&huart1, (uint8_t *)"------------------------------------\r\n\n\n", 40, UART_DELAY);
}

void parseUARTMessage(char* message, char* token)
{
  char* _token = strtok(message, token);
  uartData.command = _token; // Assign the value to the command member of uartData

  for(int i = 1; i <= 4; i++)
  {
    _token = strtok(NULL, token);
    if(_token == NULL) break;
    if(i == 1) uartData.length = atoi(_token);
    else if(i == 2) uartData.interPoints = atoi(_token);
  }
}

double* linspace(double a, double b, int n, double u[])
{
  double c;
  int i;
  if(n < 2 || u == 0) return (void*)0;
  c = (b - a)/(n - 1);
  for(i = 0; i < n - 1; ++i) u[i] = a + i*c;
  u[n - 1] = b;
  return u;
}

void IK_Move_interPoints(uint8_t radiusofcircle, uint8_t xCoordinateCenterSemicircle, uint8_t  yCoordinateCenterSemicircle, uint8_t interPoints, double _t1[], double _t2[])
{
  double x[40];
  double y[40];
  linspace(xCoordinateCenterSemicircle-radiusofcircle, xCoordinateCenterSemicircle+radiusofcircle, interPoints, x);
  for(int i = 0; i < interPoints; i++)
  {
    y[i] = (yCoordinateCenterSemicircle -  pow(pow(radiusofcircle,2)-pow((x[i] + xCoordinateCenterSemicircle),2),0.5));
    _t1[i] = asind((pow(x[i],2) + pow(y[i],2) + pow(upperLegLength,2) - pow(lowerLegLength,2))/(2*upperLegLength*pow(pow(x[i],2) + pow(y[i],2),0.5))) - atand(x[i]/y[i]);
    _t2[i] = acosd(((x[i] - upperLegLength*cosd(_t1[i]))/lowerLegLength)) - _t1[i];
    _t1[i] = 180 - _t1[i];
    _t2[i] = 180 - _t2[i];
  }
}

void IK_UpDown_interPoints(uint8_t finalHeight,  uint8_t interPoints, double _t1[], double _t2[])
{
  double x[100];

  linspace(finalHeight, initialHeight, interPoints, x);
  for(int i = interPoints-1; i >= 0 ; i--)
  {
    _t1[i] = acosd((pow(upperLegLength,2) + pow(x[i],2) - pow(lowerLegLength,2))/(2*upperLegLength*x[i]));
    _t2[i] = acosd((pow(upperLegLength,2) + pow(lowerLegLength,2) - pow(x[i],2))/(2*upperLegLength*lowerLegLength));
    _t1[i] = 180 - _t1[i];
    _t2[i] = 180 - _t2[i];
  }
  initialHeight = finalHeight;
}

void IMU_Calibration()
{
  HAL_UART_Transmit(&huart1, (uint8_t *)"CALIBRATING IMU... \r\n", 20, UART_DELAY);
  HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
  MPU_calibrateGyro(&hi2c1, 1500);
  HAL_UART_Transmit(&huart1, (uint8_t *)"CALIBRATION COMPLETE \r\n", 23, UART_DELAY);
}

void softPool(double _t1[], double _t2[], int softPoolDelay)
{
  for (int i=uartData.interPoints; i>=0; i--)
  {
    PCA9685_SetServoAngle(ServoFRT1, 180-_t1[i]); //front right
    PCA9685_SetServoAngle(ServoFRB2, 180-_t2[i]); //front right
    PCA9685_SetServoAngle(ServoFLT1, _t1[i]); //front left
    PCA9685_SetServoAngle(ServoFLB2, _t2[i]); //front left

    PCA9685_SetServoAngle(ServoBLT1, _t1[i]); //back left
    PCA9685_SetServoAngle(ServoBLB2, _t2[i]); //back left
    PCA9685_SetServoAngle(ServoBRT1, 180-_t1[i]); //back right
    PCA9685_SetServoAngle(ServoBRB2, 180-_t2[i]); //back right
    HAL_Delay(softPoolDelay);
  }
}

void robotHeightPos(double _t1[],  double _t2[], int softPoolDelay)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)"Moving robot to Specified height position...\r\n", 46, UART_DELAY);
  for(int i = 0; i < uartData.interPoints; i++)
  {

    PCA9685_SetServoAngle(ServoFRT1, (180-t1[i])+5); //front right
    PCA9685_SetServoAngle(ServoFRB2, (180-t2[i])); //front right
    PCA9685_SetServoAngle(ServoFLT1, (t1[i])+5); //front left
    PCA9685_SetServoAngle(ServoFLB2, (t2[i])+5); //front left

    PCA9685_SetServoAngle(ServoBLT1, t1[i]); //back left
    PCA9685_SetServoAngle(ServoBLB2, t2[i]); //back left
    PCA9685_SetServoAngle(ServoBRT1, 180-t1[i]); //back right
    PCA9685_SetServoAngle(ServoBRB2, 180-t2[i]); //back right
    HAL_Delay(softPoolDelay);
  }
  HAL_UART_Transmit(&huart1, (uint8_t *)"Robot is now on Specified height position\r\n", 42, UART_DELAY);
}

void robotSitDown(double _t1[],  double _t2[], int softPoolDelay)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)"Sitting down robot...\r\n", 23, UART_DELAY);
  for(int i = 0; i < uartData.interPoints; i++)
  {
    PCA9685_SetServoAngle(ServoFRT1, 180-_t1[i]); //front right
    PCA9685_SetServoAngle(ServoFRB2, 180-_t2[i]); //front right
    PCA9685_SetServoAngle(ServoFLT1, _t1[i]); //front left
    PCA9685_SetServoAngle(ServoFLB2, _t2[i]); //front left

    PCA9685_SetServoAngle(ServoBLT1, _t1[i]); //back left
    PCA9685_SetServoAngle(ServoBLB2, _t2[i]); //back left
    PCA9685_SetServoAngle(ServoBRT1, 180-_t1[i]); //back right
    PCA9685_SetServoAngle(ServoBRB2, 180-_t2[i]); //back right
    HAL_Delay(softPoolDelay);
  }
  HAL_UART_Transmit(&huart1, (uint8_t *)"Robot is now on Sitting Position\r\n", 34, UART_DELAY);

}

void robotNeutralPos(double _t1[],  double _t2[], int softPoolDelay)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)"Moving robot to neutral position...\r\n", 36, UART_DELAY);
  for(int i = 0; i < uartData.interPoints; i++)
  {
    PCA9685_SetServoAngle(ServoFRT1, (180-t1[i])+5); //front right
    PCA9685_SetServoAngle(ServoFRB2, (180-t2[i])); //front right
    PCA9685_SetServoAngle(ServoFLT1, (t1[i])+5); //front left
    PCA9685_SetServoAngle(ServoFLB2, (t2[i])+5); //front left

    PCA9685_SetServoAngle(ServoBLT1, t1[i]); //back left
    PCA9685_SetServoAngle(ServoBLB2, t2[i]); //back left
    PCA9685_SetServoAngle(ServoBRT1, 180-t1[i]); //back right
    PCA9685_SetServoAngle(ServoBRB2, 180-t2[i]); //back right
    HAL_Delay(softPoolDelay);
  }
  HAL_UART_Transmit(&huart1, (uint8_t *)"Robot is now in neutral position\r\n", 34, UART_DELAY);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1, rx_data, rx_data_Size);
  DisplayInitialMsg();
  PCA9685_Init(&hi2c1);
  robot_init();
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);

  // Check if IMU configured properly and block if it didn't
  HAL_UART_Transmit(&huart1, (uint8_t *)"INITIALIZING IMU.........", 24, UART_DELAY);
  if(MPU_begin(&hi2c1, AD0_LOW, AFSR_4G, GFSR_500DPS, 0.98, 0.004) == TRUE)
  {
    HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, 1);
    HAL_UART_Transmit(&huart1, (uint8_t *)"IMU INITIALIZED\r\n", 17, UART_DELAY);
  }
  else
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)"ERROR!! -- IMU not Initialized\r\n", 33, UART_DELAY);
    while (1)
    {
      HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
      HAL_Delay(250);
    }
  }
  HAL_UART_Transmit(&huart1, (uint8_t *)"WAITING FOR SERIAL COMMANDS..\r\n", 31, UART_DELAY);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(DataReceaved)
    {
      DataReceaved = FALSE;
      parseUARTMessage(rx_data, delimeter);
      HAL_UART_Receive_IT(&huart1, rx_data, rx_data_Size);
      switch (*uartData.command)
      {
        case 'c': // Calibrate the IMU
        calibrateFlag = FALSE;
        runOnceFlag = TRUE;
        break;

        case 'h': // robot to height position
        IK_UpDown_interPoints(uartData.length, uartData.interPoints, t1, t2);
        calibrateFlag = FALSE;
        neutralFlag = FALSE;
        runOnceFlag = TRUE;
        break;

        case 'n': // move robot to neutral position
        uartData.interPoints = 50;
        IK_UpDown_interPoints(20, 50, t1, t2);
        neutralFlag = FALSE;
        runOnceFlag = TRUE;
        break;

        case 's': // sit down robot
        runOnceFlag = TRUE;
        break;

        case 'm': // move gait
        break;

        case 'b': // balance robot
        break;

        case 'u': // situp robot
        initialHeight = 10;
        uartData.interPoints = 50;
        break;

        case 'f': // Standing robot
        runOnceFlag = TRUE;
        break;

        default:
        break;
      }
    }

    if(!DataReceaved)
    {
      switch (*uartData.command)
      {
        case 'c': // Calibrate the IMU
        if(runOnceFlag)
        {
          IMU_Calibration();
          calibrateFlag = TRUE;
          runOnceFlag = FALSE;
        }
        break;

        case 'h': // robot to height position
        if(runOnceFlag)
        {
          robotHeightPos(t1, t2, Selfsoftdelay);
          runOnceFlag = FALSE;
        }
        break;

        case 'n': // move robot to neutral position
        if(runOnceFlag)
        {
          robotHeightPos(t1, t2, Selfsoftdelay);
          neutralFlag = TRUE;
          runOnceFlag = FALSE;
        }
        break;

        case 'f': // move robot to Standing position
        if(runOnceFlag)
        {
          PCA9685_SetServoAngle(ServoFRT1, (90+5)); //front right
          PCA9685_SetServoAngle(ServoFRB2, (0)); //front right
          PCA9685_SetServoAngle(ServoFLT1, (90)+10); //front left
          PCA9685_SetServoAngle(ServoFLB2, (180)+5); //front left

          PCA9685_SetServoAngle(ServoBLT1, 90); //back left
          PCA9685_SetServoAngle(ServoBLB2, 180); //back left
          PCA9685_SetServoAngle(ServoBRT1, 90-10); //back right
          PCA9685_SetServoAngle(ServoBRB2, 0); //back right
          HAL_Delay(50);
          runOnceFlag = FALSE;
        }
        break;

        case 's': // sit down robot
        if(runOnceFlag)
        {
          PCA9685_SetServoAngle(ServoFRT1, (90+5)); //front right
          PCA9685_SetServoAngle(ServoFRB2, (90)); //front right
          PCA9685_SetServoAngle(ServoFLT1, (90)+5); //front left
          PCA9685_SetServoAngle(ServoFLB2, (90)+5); //front left

          PCA9685_SetServoAngle(ServoBLT1, 90); //back left
          PCA9685_SetServoAngle(ServoBLB2, 90); //back left
          PCA9685_SetServoAngle(ServoBRT1, 90); //back right
          PCA9685_SetServoAngle(ServoBRB2, 90-5); //back right
          HAL_Delay(50);
          runOnceFlag = FALSE;
        }
        break;

        case 'u': // sit up robot
        IK_UpDown_interPoints(22, 50, t1, t2);
        robotHeightPos(t1, t2, Selfsoftdelay);
        IK_UpDown_interPoints(10, 50, t1, t2);
        robotHeightPos(t1, t2, Selfsoftdelay);

        break;

        case 'm': // move gait
        for(int i = 39; i >= 0; i--)
        {
          PCA9685_SetServoAngle(ServoFRT1, (180-ts1[i])+5); //front right
          PCA9685_SetServoAngle(ServoFRB2, (180-ts2[i])); //front right
          PCA9685_SetServoAngle(ServoFLT1, (th1[39-i])+5); //front left
          PCA9685_SetServoAngle(ServoFLB2, (th2[39-i])+5); //front left

          PCA9685_SetServoAngle(ServoBLT1, ts1[i]); //back left
          PCA9685_SetServoAngle(ServoBLB2, ts2[i]); //back left
          PCA9685_SetServoAngle(ServoBRT1, 180-th1[39-i]); //back right
          PCA9685_SetServoAngle(ServoBRB2, 180-th2[39-i]); //back right
          HAL_Delay(50);
        }

        for(int i = 0; i < 39; i++)
        {
          PCA9685_SetServoAngle(ServoFRT1, (180-th1[i])+5); //front right
          PCA9685_SetServoAngle(ServoFRB2, (180-th2[i])); //front right
          PCA9685_SetServoAngle(ServoFLT1, (ts1[39-i])+5); //front left
          PCA9685_SetServoAngle(ServoFLB2, (ts2[39-i])+5); //front left

          PCA9685_SetServoAngle(ServoBLT1, th1[i]); //back left
          PCA9685_SetServoAngle(ServoBLB2, th2[i]); //back left
          PCA9685_SetServoAngle(ServoBRT1, 180-ts1[39-i]); //back right
          PCA9685_SetServoAngle(ServoBRB2, 180-ts2[39-i]); //back right
          HAL_Delay(50);
        }

        break;

        case 'b': // balance robot
        if(neutralFlag)
        {
          if(calibrateFlag)
          {
            // MPU_readProcessedData(&hi2c1);
            MPU_readProcessedDataLPF(&hi2c1);
            robot_balance();
          }
          else
          {
            HAL_UART_Transmit(&huart1, (uint8_t *)"ERROR!! -- IMU not Calibrated\r\n", 33, UART_DELAY);
            HAL_UART_Transmit(&huart1, (uint8_t *)"Resolving ERROR........\r\n", 27, UART_DELAY);
            IMU_Calibration();
            HAL_UART_Transmit(&huart1, (uint8_t *)"ERROR Resolved -- Balancing Robot\r\n", 33, UART_DELAY);
            calibrateFlag = TRUE;
          }
        }
        else
        {
          HAL_UART_Transmit(&huart1, (uint8_t *)"ERROR!! -- Robot not in neutral position\r\n", 42, UART_DELAY);
          HAL_UART_Transmit(&huart1, (uint8_t *)"ERROR!! -- IMU not Calibrated\r\n", 31, UART_DELAY);
          HAL_UART_Transmit(&huart1, (uint8_t *)"Resolving ERROR........\r\n", 25, UART_DELAY);
          IK_UpDown_interPoints(neutralPosLength, SelfInterpoints, t1, t2);
          robotNeutralPos(t1, t2, Selfsoftdelay);
          IMU_Calibration();
          HAL_UART_Transmit(&huart1, (uint8_t *)"ERROR Resolved -- Balancing Robot\r\n", 34, UART_DELAY);
          neutralFlag = TRUE;
          calibrateFlag = TRUE;
        }
        break;

        default:
        break;
      }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UNUSED(huart);
  DataReceaved = TRUE;
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
