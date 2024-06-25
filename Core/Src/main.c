/* USER CODE BEGIN Header */
/**
  ****************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ****************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ****************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "rtc.h"
#include "gpio.h"
#include "app_mems.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdbool.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct przyciski
{
	  uint16_t select;
	  uint16_t left;
	  uint16_t right;
	  uint16_t up;
	  uint16_t down;
};
bool przerwanie=false;
uint8_t jednostka_temperatury=0;
uint8_t mnoznik=10;

uint32_t adc_konwersja(bool wyswietl)
{
	char buf[16];
	uint32_t raw;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
	raw=HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	if(wyswietl){
		sprintf(buf,"Odczytano: %lu",raw);
		lcd_print(1,1,buf);
		HAL_Delay(1000);
		lcd_clear();
	}
	return raw;
}
uint8_t zmiana_temp(struct przyciski p)
{
	HAL_Delay(150);
	bool petla=true;
	uint8_t j=0;
	while(petla)
	{
		lcd_clear();
		lcd_print(1,1,"Jednostka");
		switch(j)
		{
		case 0:
			lcd_print(2,1,"Celsjusz");
			break;
		case 1:
			lcd_print(2,1,"Fahrenheit");
			break;
		}
		HAL_Delay(100);
		uint16_t raw=adc_konwersja(false);
		if (raw>= p.up-150 && raw<=p.up+150)
		{
			if(j==1)
			{
				j=0;
			}
			else
				j++;
		}
		if(raw>= p.down-150 && raw<=p.down+150)
		{
			if(j==0)
			{
				j=1;
			}
			else
				j--;
			}
		if (raw>= p.select-150 && raw<=p.select+150)
		{
			petla=false;
		}
		lcd_clear();
		}
		return j;

}
void wyswietl_date()
{
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	char buf[17];
	HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&date, RTC_FORMAT_BIN);
	for(int i=0;i<2;i++)
	{
		lcd_char(1,16,6);
		if(i%2==0)
		{
			sprintf(buf,"%02d:%02d %02d.%02d.20%02d",time.Hours,time.Minutes,date.Date,date.Month,date.Year);
		}
		else
		{
			sprintf(buf,"%02d %02d %02d.%02d.20%02d",time.Hours,time.Minutes,date.Date,date.Month,date.Year);
		}
		lcd_print(1,1,"Godzina i data");
		lcd_print(2,1,buf);
		HAL_Delay(1000);
		lcd_clear();
	}

}
void wywolaj_funkcje(uint8_t licznik)
{
	switch(licznik)
	{
		case 0:
			Temp_Sensor_Handler(0,jednostka_temperatury);
			break;
		case 1:
			Hum_Sensor_Handler(0);
			break;
		case 2:
			Press_Sensor_Handler(1);
			break;
		case 3:
			wyswietl_date();
			break;

	}
}
void menu_acykliczne(uint8_t *licz,struct przyciski p){
	uint8_t licznik=*licz;
	uint32_t raw;
	bool petla=true;
	while(petla)
	{
		switch(licznik){
			case 0:
				wywolaj_funkcje(licznik);
				break;
			case 1:
				wywolaj_funkcje(licznik);
				break;
			case 2:
				wywolaj_funkcje(licznik);
				break;
			case 3:
				wywolaj_funkcje(licznik);
				break;
		}
		raw=adc_konwersja(false);
		if(raw>=p.right && raw<=p.right+150)
		{
			  if(licznik==3)
			  {
				  licznik=0;
			  }
			  else
				  licznik++;
		}
		if(raw>=p.left-150 && raw<=p.left+150)
		{
			  if(licznik==0)
			  {
				  licznik=3;
			  }
			  else
			  {
				  licznik--;
			  }
		}
		if (raw>=p.select-150 && raw<=p.select+150)
		{
			petla=false;

		}
	}

}
void zmiana_mnoznika(struct przyciski p)
{
	uint16_t raw;
	uint8_t j=mnoznik;
	bool edytowanie_nieukonczone=true;
	char buf[17];
	HAL_Delay(150);
	do
	{
		raw=adc_konwersja(false);
		if (raw>= p.up-150 && raw<=p.up+150)
		{
			if(j==60)
			{
				j=1;
			}
			else
				j++;
			  }
		if (raw>= p.down-150 && raw<=p.down+150)
		{
			if(j==1)
			{
				j=60;
			}
			else
				j--;
		}
		if (raw>= p.select-150 && raw<=p.select+150)
		{
			edytowanie_nieukonczone=false;
			mnoznik=j;
		}
		sprintf(buf,"%d sekund",j);
		lcd_print(1,1,"Czas wysw");
		lcd_print(2,1,buf);
		HAL_Delay(150);
		lcd_clear();

	}
	while(edytowanie_nieukonczone);
}
void lcd_create_custom_char(uint8_t char_addr, char* char_data) {
 char_addr &= 0x07;
 lcd_cmd(0x40 + (char_addr * 8));
 for (int i = 0; i < 8; ++i) {
 lcd_char_cp(char_data[i]);
 }}
void edytowanie_godziny(RTC_TimeTypeDef *time,uint8_t *migniecie,struct przyciski p)
{
	uint16_t raw;
	char buf[16];
	lcd_print(1,1,"HH:MM");
			if(*migniecie<=6)
			{
				sprintf(buf,"%02d:%02d",time->Hours,time->Minutes);
				*migniecie+=1;
			}
			else if(*migniecie<=9)
			{
				sprintf(buf,"  :%02d",time->Minutes);
				*migniecie+=1;
			}else
			{
				sprintf(buf,"  :%02d",time->Minutes);
				*migniecie=0;
			}
				lcd_print(2,1,buf);
				HAL_Delay(100);

			raw=adc_konwersja(false);
			if(raw>=p.up-150 && raw<=p.up+150)
			{
				  if(time->Hours==23)
				  {
					  time->Hours=0;
				  }
				  else
					  time->Hours++;
			}
			if(raw>=p.down-150 && raw<=p.down+150)
			{
				  if(time->Hours==0)
				  {
					  time->Hours=23;
				  }
				  else
				  {
					  time->Hours--;
				  }
			}

}
void edytowanie_minuty(RTC_TimeTypeDef *time,uint8_t *migniecie,struct przyciski p)
{
	uint16_t raw;
	char buf[16];
	lcd_print(1,1,"HH:MM");
			if(*migniecie<=6)
			{
				sprintf(buf,"%02d:%02d",time->Hours,time->Minutes);
				*migniecie+=1;
			}
			else if(*migniecie<=9)
			{
				sprintf(buf,"%02d:  ",time->Hours);
				*migniecie+=1;
			}else
			{
				sprintf(buf,"%02d:  ",time->Hours);
				*migniecie=0;
			}
				lcd_print(2,1,buf);
				HAL_Delay(100);

			raw=adc_konwersja(false);
			if(raw>=p.up-150 && raw<=p.up+150)
			{
				  if(time->Minutes==59)
				  {
					  time->Minutes=0;
				  }
				  else
					  time->Minutes++;
			}
			if(raw>=p.down-150 && raw<=p.down+150)
			{
				  if(time->Minutes==0)
				  {
					  time->Minutes=59;
				  }
				  else
				  {
					  time->Minutes--;
				  }
			}
}
void edytowanie_dnia(RTC_DateTypeDef *date,uint8_t *migniecie,struct przyciski p)
{
	uint16_t raw;
	char buf[16];
	lcd_print(1,1,"DD.MM.YYYY");
			if(*migniecie<=6)
			{
				sprintf(buf,"%02d.%02d.20%02d",date->Date,date->Month,date->Year);
				*migniecie+=1;
			}
			else if(*migniecie<=9)
			{
				sprintf(buf,"  .%02d.20%02d",date->Month,date->Year);
				*migniecie+=1;
			}else
			{
				sprintf(buf,"  .%02d.20%02d",date->Month,date->Year);
				*migniecie=0;
			}
				lcd_print(2,1,buf);
				HAL_Delay(100);

			raw=adc_konwersja(false);
			if(raw>=p.up-150 && raw<=p.up+150)
			{
				  if(date->Date==31)
				  {
					  date->Date=1;
				  }
				  else
					  date->Date++;
			}
			if(raw>=p.down-150 && raw<=p.down+150)
			{
				  if(date->Date==1)
				  {
					  date->Date=31;
				  }
				  else
				  {
					  date->Date--;
				  }
			}
}
void edytowanie_miesiaca(RTC_DateTypeDef *date,uint8_t *migniecie,struct przyciski p)
{
	uint16_t raw;
	char buf[16];
	lcd_print(1,1,"DD.MM.YYYY");
			if(*migniecie<=6)
			{
				sprintf(buf,"%02d.%02d.20%02d",date->Date,date->Month,date->Year);
				*migniecie+=1;
			}
			else if(*migniecie<=9)
			{
				sprintf(buf,"%02d.  .20%02d",date->Date,date->Year);
				*migniecie+=1;
			}else
			{
				sprintf(buf,"%02d.  .20%02d",date->Date,date->Year);
				*migniecie=0;
			}
				lcd_print(2,1,buf);
				HAL_Delay(100);

			raw=adc_konwersja(false);
			if(raw>=p.up-150 && raw<=p.up+150)
			{
				  if(date->Month==12)
				  {
					  date->Month=1;
				  }
				  else
					  date->Month++;
			}
			if(raw>=p.down-150 && raw<=p.down+150)
			{
				  if(date->Month==1)
				  {
					  date->Month=12;
				  }
				  else
				  {
					  date->Month--;
				  }
			}
}
void edytowanie_roku(RTC_DateTypeDef *date,uint8_t *migniecie,struct przyciski p)
{
	uint16_t raw;
	char buf[16];
	lcd_print(1,1,"DD.MM.YYYY");
			if(*migniecie<=6)
			{
				sprintf(buf,"%02d.%02d.20%02d",date->Date,date->Month,date->Year);
				*migniecie+=1;
			}
			else if(*migniecie<=9)
			{
				sprintf(buf,"%02d.%02d.    ",date->Date,date->Month);
				*migniecie+=1;
			}else
			{
				sprintf(buf,"%02d.%02d.    ",date->Date,date->Month);
				*migniecie=0;
			}
				lcd_print(2,1,buf);
				HAL_Delay(100);

			raw=adc_konwersja(false);
			if(raw>=p.up-150 && raw<=p.up+150)
			{
				  if(date->Year==99)
				  {
					  date->Year=0;
				  }
				  else
					  date->Year++;
			}
			if(raw>=p.down-150 && raw<=p.down+150)
			{
				  if(date->Year==0)
				  {
					  date->Year=99;
				  }
				  else
				  {
					  date->Year--;
				  }
			}
}

void konfiguracja_daty(RTC_TimeTypeDef *time,RTC_DateTypeDef *date,struct przyciski p)
{
	lcd_clear();
	lcd_print(1,1,"Konfiguracja");
	lcd_print(2,1,"daty i godziny");
	HAL_Delay(2000);
	lcd_clear();
	uint8_t edycja=0;
	bool edytowanie_nieukonczone=true;
	uint8_t migniecie=0;
	uint16_t raw;
	do
	{
		switch(edycja)
		{
			case 0:
				edytowanie_dnia(date, &migniecie, p);
				break;
			case 1:
				edytowanie_miesiaca(date, &migniecie, p);
				break;
			case 2:
				edytowanie_roku(date, &migniecie, p);
				break;
		}
		raw=adc_konwersja(false);
		if(raw>=p.left-150 && raw<=p.left+150)
		{
			if(edycja!=0)
			{
				edycja--;
			}
		}
		if(raw>=p.right && raw<=p.right+150)
		{
			if(edycja!=2)
			{
				edycja++;
			}
		}
		if(raw>=p.select-150 && raw<=p.select+150)
		{
			edytowanie_nieukonczone=false;
		}
	}
	while(edytowanie_nieukonczone);
	edytowanie_nieukonczone=true;
	HAL_Delay(300);
	lcd_clear();
	edycja=0;
	do
	{
		switch(edycja)
		{
			case 0:
				edytowanie_godziny(time,&migniecie,p);
				break;
			case 1:
				edytowanie_minuty(time, &migniecie,p);
				break;
		}
		raw=adc_konwersja(false);
		if(raw>=p.left-150 && raw<=p.left+150)
		{
			edycja=0;
		}
		if(raw>=p.right && raw<=p.right+150)
		{
			edycja=1;
		}
		if(raw>=p.select-150 && raw<=p.select+150)
		{
			edytowanie_nieukonczone=false;
		}
	}
	while(edytowanie_nieukonczone);
	HAL_RTC_SetTime(&hrtc,time,RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc,date,RTC_FORMAT_BIN);
	HAL_Delay(150);
	lcd_clear();
}
void wywolaj_menu(uint8_t *licznik,struct przyciski p)
{
	uint8_t j=0;
	while(przerwanie==true)
	{
		uint32_t raw=adc_konwersja(false);
		lcd_print(1,1,"wybierz opcje:");
		if (raw>= p.up-150 && raw<=p.up+150)
		{
			if(j==4)
			{
				j=0;
			}
			else
				j++;
			  }
		if (raw>= p.down-150 && raw<=p.down+150)
		{
			if(j==0)
			{
				j=4;
			}
			else
				j--;
		}
		if (raw>= p.select-150 && raw<=p.select+150)
		{
			switch(j)
			{
				case 0:
					przerwanie=false;
					HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_RESET);
					break;
				case 1:
					menu_acykliczne(licznik,p);
					HAL_Delay(150);
					break;
				case 2:
					jednostka_temperatury=zmiana_temp(p);
					break;
				case 3:
					RTC_TimeTypeDef time;
					RTC_DateTypeDef date;
					HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);
					HAL_RTC_GetDate(&hrtc,&date, RTC_FORMAT_BIN);
					konfiguracja_daty(&time, &date, p);
					break;
				case 4:
					zmiana_mnoznika(p);
					break;
			}
		}
		switch(j)
		{
			case 0:
				lcd_print(2,1,"wysw cyklicznie");
			 	break;
			case 1:
				lcd_print(2,1,"wysw acyklicznie");
		  		break;
		  	case 2:
		  		lcd_print(2,1,"jednostka temp");
		  		break;
		  	case 3:
		  		lcd_print(2,1,"zmiana daty");
		  		break;
		  	case 4:
		  		lcd_print(2,1,"czas wysw");
		  		break;
		}
		HAL_Delay(150);
		lcd_clear();
	}
}

uint16_t wcisniety_przycisk()
{
	uint16_t raw;
	do
	{
		raw=adc_konwersja(true);
	}
	while(raw>4090);
	return raw;
}




void konfiguracja_przyciskow(struct przyciski *p)
{
	lcd_print(1,1,"Konfiguracja");
	lcd_print(2,1,"przyciskow");
	HAL_Delay(2000);
	lcd_clear();
	lcd_print(1,1,"wcisnij SELECT");
	HAL_Delay(1000);
	p->select=wcisniety_przycisk();
	lcd_clear();
	lcd_print(1,1,"SELECT zapisany");
	HAL_Delay(1000);
	lcd_clear();
	lcd_print(1,1,"wcisnij LEFT");
	HAL_Delay(1000);
	p->left=wcisniety_przycisk();
	lcd_clear();
	lcd_print(1,1,"LEFT zapisany");
	HAL_Delay(1000);
	lcd_clear();
	lcd_print(1,1,"wcisnij RIGHT");
	HAL_Delay(1000);
	p->right=wcisniety_przycisk();
	lcd_clear();
	lcd_print(1,1,"RIGHT zapisany");
	HAL_Delay(1000);
	lcd_clear();
	lcd_print(1,1,"wcisnij UP");
	HAL_Delay(1000);
	p->up=wcisniety_przycisk();
	lcd_clear();
	lcd_print(1,1,"UP zapisany");
	HAL_Delay(1000);
	lcd_clear();
	lcd_print(1,1,"wcisnij DOWN");
	HAL_Delay(1000);
	p->down=wcisniety_przycisk();
	lcd_clear();
	lcd_print(1,1,"DOWN zapisany");
	HAL_Delay(1000);
	lcd_clear();
	lcd_print(1,1,"Skonfigurowano");
	lcd_print(2,1,"przyciski");
	HAL_Delay(2000);
	lcd_clear();
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  char kropelka[] = {
	    0B00010,
	    0B00110,
	    0B01110,
	    0B01110,
	    0B11110,
	    0B11111,
	    0B11111,
	    0B01110
	  };
	  char termometr[] = {
	    0B00100,
	    0B01010,
	    0B01010,
	    0B01010,
	    0B01010,
	    0B10001,
	    0B10001,
	    0B01110
	  };
	  char cisnienie[] = {
	    0B00100,
	    0B00100,
	    0B00100,
	    0B10101,
	    0B01110,
	    0B00100,
	    0B00000,
	    0B11111
	  };
	  char chmura[] = {
	    0B00000,
	    0B00000,
	    0B00000,
	    0B00000,
	    0B01110,
	    0B11111,
	    0B11111,
	    0B01110
	  };
	  char sloncechmura[] = {
	    0B00000,
	    0B00100,
	    0B10101,
	    0B01110,
	    0B01010,
	    0B11111,
	    0B11111,
	    0B01110
	  };
	  char slonce[] = {
	    0B00000,
	    0B00100,
	    0B10101,
	    0B01110,
	    0B11011,
	    0B01110,
	    0B10101,
	    0B00100
	  };
	  char zegar[] = {
	    0B00000,
	    0B00100,
	    0B01110,
	    0B10101,
	    0B10111,
	    0B10001,
	    0B01110,
	    0B00000
	  };

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
  MX_RTC_Init();
  MX_MEMS_Init();
  /* USER CODE BEGIN 2 */
  lcd_init(_LCD_4BIT,_LCD_FONT_5x8, _LCD_2LINE);
  lcd_create_custom_char(0, kropelka);
  lcd_create_custom_char(1, cisnienie);
  lcd_create_custom_char(2, termometr);
  lcd_create_custom_char(3, slonce);
  lcd_create_custom_char(4, sloncechmura);
  lcd_create_custom_char(5, chmura);
  lcd_create_custom_char(6, zegar);
  struct przyciski p;
  uint8_t licznik;
  p.select=3694;
  p.left=2405;
  p.right=0;
  p.up=558;
  p.down=1510;
  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;
  time.Hours=0;
  time.Minutes=0;
  date.Date=1;
  date.Month=1;
  date.Year=0;

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  konfiguracja_przyciskow(&p);
  konfiguracja_daty(&time,&date,p);
  while (1)
  {
    /* USER CODE END WHILE */
		for(licznik=0;licznik<3;licznik++){
			for(int i=0;i<mnoznik;i++)
			{
			wywolaj_menu(&licznik,p);
			wywolaj_funkcje(licznik);
			}
		}
		licznik=3;
		for(int i=-1;i<mnoznik/2;i++)
		{

			wywolaj_menu(&licznik,p);
			wywolaj_funkcje(licznik);
		}
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==Button_Blue_IT_Pin)
	{
		przerwanie=true;
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_SET);
	}
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
