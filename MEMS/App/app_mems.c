/**
  ******************************************************************************
  * File Name          : app_mems.c
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-MEMS1.10.0.0 instances.
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

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_mems.h"
#include <stdio.h>
#include <lcd.h>

#include "iks01a3_motion_sensors.h"
#include "iks01a3_env_sensors.h"
#include "math.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct displayFloatToInt_s {
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t  out_int;
  uint32_t  out_dec;
} displayFloatToInt_t;

/* Private define ------------------------------------------------------------*/
#define MAX_BUF_SIZE 256

/* Private variables ---------------------------------------------------------*/
static volatile uint8_t PushButtonDetected = 0;
static IKS01A3_MOTION_SENSOR_Capabilities_t MotionCapabilities[IKS01A3_MOTION_INSTANCES_NBR];
static IKS01A3_ENV_SENSOR_Capabilities_t EnvCapabilities[IKS01A3_ENV_INSTANCES_NBR];
static char dataOut[MAX_BUF_SIZE];
uint8_t choosen_screen;

/* Private function prototypes -----------------------------------------------*/
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec);
void Temp_Sensor_Handler(uint32_t Instance,uint8_t jednostka_temperatury);
void Hum_Sensor_Handler(uint32_t Instance);
void Press_Sensor_Handler(uint32_t Instance);
static void MX_IKS01A3_DataLogTerminal_Init(void);
static void MX_IKS01A3_DataLogTerminal_Process(void);

void MX_MEMS_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Init_PreTreatment */

  /* USER CODE END MEMS_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */
  MX_IKS01A3_DataLogTerminal_Init();

  /* USER CODE BEGIN MEMS_Init_PostTreatment */

  /* USER CODE END MEMS_Init_PostTreatment */
}

/*
 * LM background task
 */
void MX_MEMS_Process(void)
{
  /* USER CODE BEGIN MEMS_Process_PreTreatment */

  /* USER CODE END MEMS_Process_PreTreatment */

  MX_IKS01A3_DataLogTerminal_Process();

  /* USER CODE BEGIN MEMS_Process_PostTreatment */

  /* USER CODE END MEMS_Process_PostTreatment */
}

/**
  * @brief  Initialize the DataLogTerminal application
  * @retval None
  */
void MX_IKS01A3_DataLogTerminal_Init(void)
{
  displayFloatToInt_t out_value_odr;
  int i;

  IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_ACCELERO | MOTION_GYRO);

  IKS01A3_MOTION_SENSOR_Init(IKS01A3_LIS2DW12_0, MOTION_ACCELERO);

  IKS01A3_MOTION_SENSOR_Init(IKS01A3_LIS2MDL_0, MOTION_MAGNETO);

  for(i = 0; i < IKS01A3_MOTION_INSTANCES_NBR; i++)
  {
    IKS01A3_MOTION_SENSOR_GetCapabilities(i, &MotionCapabilities[i]);
    snprintf(dataOut, MAX_BUF_SIZE,
             "\r\nMotion Sensor Instance %d capabilities: \r\n ACCELEROMETER: %d\r\n GYROSCOPE: %d\r\n MAGNETOMETER: %d\r\n LOW POWER: %d\r\n",
             i, MotionCapabilities[i].Acc, MotionCapabilities[i].Gyro, MotionCapabilities[i].Magneto, MotionCapabilities[i].LowPower);
    printf("%s", dataOut);
    floatToInt(MotionCapabilities[i].AccMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX ACC ODR: %d.%03d Hz, MAX ACC FS: %d\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec, (int)MotionCapabilities[i].AccMaxFS);
    printf("%s", dataOut);
    floatToInt(MotionCapabilities[i].GyroMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX GYRO ODR: %d.%03d Hz, MAX GYRO FS: %d\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec, (int)MotionCapabilities[i].GyroMaxFS);
    printf("%s", dataOut);
    floatToInt(MotionCapabilities[i].MagMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX MAG ODR: %d.%03d Hz, MAX MAG FS: %d\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec, (int)MotionCapabilities[i].MagMaxFS);
    printf("%s", dataOut);
  }

  IKS01A3_ENV_SENSOR_Init(IKS01A3_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);

  IKS01A3_ENV_SENSOR_Init(IKS01A3_LPS22HH_0, ENV_TEMPERATURE | ENV_PRESSURE);

  IKS01A3_ENV_SENSOR_Init(IKS01A3_STTS751_0, ENV_TEMPERATURE);

  for(i = 0; i < IKS01A3_ENV_INSTANCES_NBR; i++)
  {
    IKS01A3_ENV_SENSOR_GetCapabilities(i, &EnvCapabilities[i]);
    snprintf(dataOut, MAX_BUF_SIZE,
             "\r\nEnvironmental Sensor Instance %d capabilities: \r\n TEMPERATURE: %d\r\n PRESSURE: %d\r\n HUMIDITY: %d\r\n LOW POWER: %d\r\n",
             i, EnvCapabilities[i].Temperature, EnvCapabilities[i].Pressure, EnvCapabilities[i].Humidity, EnvCapabilities[i].LowPower);
    printf("%s", dataOut);
    floatToInt(EnvCapabilities[i].TempMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX TEMP ODR: %d.%03d Hz\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec);
    printf("%s", dataOut);
    floatToInt(EnvCapabilities[i].PressMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX PRESS ODR: %d.%03d Hz\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec);
    printf("%s", dataOut);
    floatToInt(EnvCapabilities[i].HumMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX HUM ODR: %d.%03d Hz\r\n", (int)out_value_odr.out_int, (int)out_value_odr.out_dec);
    printf("%s", dataOut);
  }
}

/**
  * @brief  BSP Push Button callback
  * @param  Button Specifies the pin connected EXTI line
  * @retval None.
  */

/**
  * @brief  Process of the DataLogTerminal application
  * @retval None
  */
void MX_IKS01A3_DataLogTerminal_Process(void)
{
  int i;

  if (PushButtonDetected != 0U)
  {
    /* Debouncing */
    /* Wait until the button is released */
    //while ((BSP_PB_GetState( BUTTON_KEY ) == PushButtonState));

    /* Debouncing */
    HAL_Delay(50);

    /* Reset Interrupt flag */
    PushButtonDetected = 0;

    /* Do nothing */
  }
  //HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
  for(i = 0; i < IKS01A3_ENV_INSTANCES_NBR; i++)
  {
    if(EnvCapabilities[i].Humidity)
    {
      Hum_Sensor_Handler(i);
    }
    if(EnvCapabilities[i].Temperature)
    {
      Temp_Sensor_Handler(i,0);
    }
    if(EnvCapabilities[i].Pressure)
    {
      Press_Sensor_Handler(i);
    }
  }
  HAL_Delay( 1000 );
}

/**
  * @brief  Splits a float into two integer values.
  * @param  in the float value as input
  * @param  out_value the pointer to the output integer structure
  * @param  dec_prec the decimal precision to be used
  * @retval None
  */
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec)
{
  if(in >= 0.0f)
  {
    out_value->sign = 0;
  }else
  {
    out_value->sign = 1;
    in = -in;
  }

  in = in + (0.5f / pow(10, dec_prec));
  out_value->out_int = (int32_t)in;
  in = in - (float)(out_value->out_int);
  out_value->out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

/**
  * @brief  Handles the accelerometer axes data getting/sending
  * @param  Instance the device instance
  * @retval None
  */




void Temp_Sensor_Handler(uint32_t Instance, uint8_t jednostka_temperatury)
{
  float temperature;
  displayFloatToInt_t out_value;
  if (IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_TEMPERATURE, &temperature))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "T: Error");
  }
  else
  {
    floatToInt(temperature, &out_value, 2);
    switch(jednostka_temperatury)
    {
    case 0:
        snprintf(dataOut, MAX_BUF_SIZE, "T:%c%02d.%02dC",((out_value.sign) ? '-' : '+'), (int)out_value.out_int,
                 (int)out_value.out_dec);
  	  break;
    case 1:
    	int fahrenheit_decimal=(int)out_value.out_int*1.8+32;
    	uint8_t fahrenheit_float=(int)out_value.out_dec*1.8;
    	fahrenheit_decimal+=fahrenheit_float/100;
    	fahrenheit_float%=100;
        snprintf(dataOut, MAX_BUF_SIZE, "T:%c%02d.%02dF",((out_value.sign) ? '-' : '+'), fahrenheit_decimal,fahrenheit_float);
  	  break;
    }
  }
  lcd_clear();
  if ((int)out_value.out_int>=30)
  {
  	lcd_print(2,3,"Slonecznie");
  	lcd_char(2,1,3);
  }
  else if((int)out_value.out_int<30 && ((int)out_value.out_int>=24))
  {
  	lcd_print(2,3,"Za chmurami");
  	lcd_char(2,1,4);
  }
  else
  {
  	lcd_print(2,3,"Zachmurzenie");
  	lcd_char(2,1,5);
  }

  lcd_print(1,1, dataOut);
  lcd_char(1,16,2);
  HAL_Delay(1000);
  lcd_clear();
}

void Press_Sensor_Handler(uint32_t Instance)
{
  float pressure;
  displayFloatToInt_t out_value;
  char buf[16]="";

  if (IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_PRESSURE, &pressure))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "P: Error");
  }
  else
  {
    floatToInt(pressure, &out_value, 2);
    snprintf(dataOut, MAX_BUF_SIZE, "P:%dhPa",(int)out_value.out_int);
    if((int)out_value.out_int==1013)
    {
    	sprintf(buf,"W normie");
    }
    if((int)out_value.out_int>1013)
    {
    	sprintf(buf,"Za wysokie");
    }
    else
    {
    	sprintf(buf,"Za niskie");
    }
  }
  lcd_clear();
  lcd_print(1,1, dataOut);
  lcd_char(1,16,1);
  lcd_print(2,1,buf);
  HAL_Delay(1000);
  lcd_clear();
}
void Hum_Sensor_Handler(uint32_t Instance)
{
  float humidity;
  displayFloatToInt_t out_value;

  if (IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_HUMIDITY, &humidity))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "H: Error");
  }
  else
  {
    floatToInt(humidity, &out_value, 2);
    snprintf(dataOut, MAX_BUF_SIZE, "H:%d.%02d%%", (int)out_value.out_int,
             (int)out_value.out_dec);
  }
  lcd_clear();
  lcd_print(1,1, dataOut);
  lcd_char(1,16,0);
  lcd_char(2,3,'[');
  int wartosc=(int)out_value.out_int/10;
  for(int i=1;i<wartosc+1;i++)
  {
	  lcd_char(2,i+3,0);
  }
  lcd_char(2,14,']');
  HAL_Delay(1000);
  lcd_clear();
}

#ifdef __cplusplus
}
#endif
