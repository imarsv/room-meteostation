/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include <math.h>
#include "ssd1306/ssd1306.h"
#include "ssd1306/fonts.h"
#include "ssd1306/images.h"
#include "BMP280/bmp280.h"
#include "HTU21/htu21.h"
#include "MHZ19/mhz19.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define slog(msg) \
		{ /*printf("info: %s\n", msg);*/ }

#define elog(msg) \
		{ printf("error: %s\n", msg); }

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))
#define MIN(a,b)   (((a)<(b))?(a):(b))
#define MAX(a,b)   (((a)>(b))?(a):(b))
#define PA2mmHG(p) (((float) (p) * 760 / 101325))

#define BUFF_LEN          (32)

#define MAIN_LOOP_DELAY           (2)

#define SHOW_LOGO_DELAY           (4)
#define SHOW_MEASUREMENTS_DELAY   (8)
#define SHOW_BATTERY_STATUS_DELAY (2)

#define MEASUREMENTS_ACTIVE_DELAY (2)
#define MEASUREMENTS_SLEEP_DELAY  (20)

#define STATE_ACTIVE_DELAY        (30)
#define STATE_SLEEP_DELAY         (10*60)

#define HIDE_SCREEN_DELAY         (20)

// Max ADC value ~ voltage
#define ADC_TO_VOLTAGE_BASE (4.95f)
#define BATTERY_MAX_VOLTAGE (4.1f)
#define BATTERY_MIN_VOLTAGE (2.8f)

// Segments
#define LED_SEGMENT_WIDTH (SSD1306_WIDTH / 2)

struct bmp280_t BMP280;

char buffer[BUFF_LEN] = {};

int audio_activation = 0;

typedef enum { ACTIVE, SLEEP } State;
typedef enum { LOGO, MEASUREMENTS, BATTERY, HIDE } Screen;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is going to be read
 *	\param reg_data : This is the data read from the sensor, which is held in an array
 *	\param cnt : The no of bytes of data to be read
 */
s8 BMP280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is to be written
 *	\param reg_data : It is a value held in the array,
 *		which is written in the register
 *	\param cnt : The no of bytes of data to be written
 */
s8 BMP280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*
 * \Brief: SPI/I2C init routine
*/
s8 I2C_routine(void);

/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMP280_delay_msek(u32 msek);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == AUDIO_Pin){
		audio_activation++;
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	HAL_RTC_DeactivateAlarm(hrtc, RTC_ALARM_A);
}

uint32_t RTC_To_Timestamp(RTC_TimeTypeDef *time) {
	/* Convert time in seconds */
	return (uint32_t)(((uint32_t)time->Hours * 3600) +
			((uint32_t)time->Minutes * 60) +
			((uint32_t)time->Seconds));
}

void Timestamp_To_RTC(uint32_t timestamp, RTC_TimeTypeDef *time) {
	/* Fill the structure fields with the read parameters */
	time->Hours = timestamp / 3600;
	time->Minutes  = (uint8_t)((timestamp % 3600) / 60);
	time->Seconds  = (uint8_t)((timestamp % 3600) % 60);
}

uint32_t RTC_GetTimestamp() {
	RTC_TimeTypeDef time;
	if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK) {
		return 0;
	}

	return RTC_To_Timestamp(&time);
}

void sleep(uint32_t delay) {
	RTC_AlarmTypeDef alarm;
	alarm.Alarm = RTC_ALARM_A;

	Timestamp_To_RTC(RTC_GetTimestamp() + delay - 1, &alarm.AlarmTime);

	if (HAL_RTC_SetAlarm_IT(&hrtc, &alarm, FORMAT_BIN) == HAL_OK) {
		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		HAL_ResumeTick();
	} else {
		HAL_Delay(delay * 1000);
	}
}

float getBatteryVoltage() {

	if (HAL_ADC_Start(&hadc1) != HAL_OK) {
		elog("ADC start fail");

		return 0.0;
	}

	if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) {
		elog("ADC poll for conversion fail");
	}

	uint32_t value = HAL_ADC_GetValue(&hadc1);

	if (HAL_ADC_Stop(&hadc1) != HAL_OK) {
		elog("ADC stop fail");
	}

	// (0 to 4095) 0x00 ... 0x0FFF
//	printf("ADC1: (0 to 4095)  %lu \n", value);

	return (float) ADC_TO_VOLTAGE_BASE * value / 0x0FFF;
}

int getBatteryCharge(float voltage) {

	int charge = rintf((voltage - BATTERY_MIN_VOLTAGE) * 100.0f / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE));

	if (charge > 100) {
		charge = 100;
	} else if (charge < 0) {
		charge = 0;
	}

	return charge;
}

void showLogo() {
	slog("Show logo");

	SSD1306_Fill(SSD1306_COLOR_BLACK);

	ssd1306_image(logo, 0, 0, 0);

	SSD1306_UpdateScreen();
}

void showBatteryCharge(int charge) {
	slog("Show battery charge");

	SSD1306_Fill(SSD1306_COLOR_BLACK);

	ssd1306_image(battery, 0, 0, 0);

	for (int i = 0; i < charge / 10; i++) {
		ssd1306_image(battery_point, 0, 16 + i * 9, 13);
	}

	snprintf(buffer, BUFF_LEN, "%d%%", charge);
	int messageWidth = strlen(buffer) * Font_10x16.FontWidth;

	int shiftX = (SSD1306_WIDTH - messageWidth * 1.2) / 2;
	int shiftY = (SSD1306_HEIGHT - Font_10x16.FontHeight * 1.2) / 2;
	SSD1306_DrawFilledRectangle(shiftX, shiftY,
			messageWidth * 1.2, Font_10x16.FontHeight * 1.2, SSD1306_COLOR_BLACK);

	shiftX = (SSD1306_WIDTH - messageWidth) / 2;
	shiftY = (SSD1306_HEIGHT - Font_10x16.FontHeight) / 2;
	SSD1306_GotoXY(shiftX, shiftY);
	SSD1306_Puts(buffer, &Font_10x16, SSD1306_COLOR_WHITE);

	SSD1306_UpdateScreen();
}

void showMeasurements(float temperature, float humidity, float pressure, uint16_t co2_ppm) {
	slog("Show measurements");

	SSD1306_Fill(SSD1306_COLOR_BLACK);

	SSD1306_GotoXY(0, Font_7x10.FontHeight * 0);
	SSD1306_Puts("   ����    �����", &Font_7x10, SSD1306_COLOR_WHITE);

	snprintf(buffer, BUFF_LEN, "%.1f%c�", temperature, 176);
	int shift = (LED_SEGMENT_WIDTH - strlen(buffer) * Font_10x16.FontWidth) / 2;
	SSD1306_GotoXY(shift, Font_7x10.FontHeight * 1 + 3);
	SSD1306_Puts(buffer, &Font_10x16, SSD1306_COLOR_WHITE);

	snprintf(buffer, BUFF_LEN, "%.1f%%", humidity);
	shift = (LED_SEGMENT_WIDTH - strlen(buffer) * Font_10x16.FontWidth) / 2;
	SSD1306_GotoXY(shift + LED_SEGMENT_WIDTH, Font_7x10.FontHeight * 1 + 3);
	SSD1306_Puts(buffer, &Font_10x16, SSD1306_COLOR_WHITE);



	char *header1 = "   ����   CO";
	char *header2 = " ppm";
	SSD1306_GotoXY(0, Font_7x10.FontHeight * 3 + 3);
	SSD1306_Puts(header1, &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_Puts(header2, &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_GotoXY(Font_7x10.FontWidth * strlen(header1), Font_7x10.FontHeight * 3 + Font_4x6.FontHeight / 2 + 3);
	SSD1306_Puts("2 ", &Font_4x6, SSD1306_COLOR_WHITE);

	snprintf(buffer, BUFF_LEN, "%.1f", PA2mmHG(pressure));
	shift = (LED_SEGMENT_WIDTH - strlen(buffer) * Font_10x16.FontWidth) / 2;
	SSD1306_GotoXY(shift, Font_7x10.FontHeight * 4 + 6);
	SSD1306_Puts(buffer, &Font_10x16, SSD1306_COLOR_WHITE);

	snprintf(buffer, BUFF_LEN, "%d", co2_ppm);
	shift = (LED_SEGMENT_WIDTH - strlen(buffer) * Font_10x16.FontWidth) / 2;
	SSD1306_GotoXY(shift + LED_SEGMENT_WIDTH, Font_7x10.FontHeight * 4 + 6);
	SSD1306_Puts(buffer, &Font_10x16, SSD1306_COLOR_WHITE);

	SSD1306_UpdateScreen();
}

void updateMeasurements(float *temperature, float *humidity, float *pressure, uint16_t *co2_ppm) {
	slog("Update measurements");

	if (htu21_temperature(temperature) != 0) {
		elog("HTU21 read temperature failed");
	}
	if (htu21_humidity(humidity) != 0) {
		elog("HTU21 read humidity failed");
	}

	s32 temp;
	if (bmp280_read_uncomp_pressure(&temp) != 0) {
		elog("BMP280 read uncomp pressure failed");
	}
	*pressure = bmp280_compensate_pressure_double(temp);

	if (mhz19_get(co2_ppm) != 0) {
		elog("MHT19 get gas concentration failed");
	}
}

void setScreenState(Screen screen) {

}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_IWDG_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
	/**
	* SSD1306
	*/
	SSD1306_Init();

	/**
	* HTU21
	*/
	float humidity = 0.0f;
	float temperature = 0.0f;

	if (htu21_reset() != 0) {
		elog("HTU21 reset fail");
	}

	/**
	* BMP280
	*/
	float pressure = 0.0f;

	I2C_routine();

	bmp280_set_soft_rst();

	if (bmp280_init(&BMP280) != 0) {
		elog("BMP280 init failed");
	}

	/*	For initialization it is required to set the mode of
	 *	the sensor as "NORMAL"
	 *	data acquisition/read/write is possible in this mode
	 *	by using the below API able to set the power mode as NORMAL*/
	/* Set the power mode as NORMAL*/
	if (bmp280_set_power_mode(BMP280_NORMAL_MODE) != 0) {
		elog("BMP280 set NORMAL mode failed");
	}

	/*	For reading the pressure and temperature data it is required to
	 *	set the work mode
	 *	The measurement period in the Normal mode is depends on the setting of
	 *	over sampling setting of pressure, temperature and standby time
	 *
	 *	OSS				pressure OSS	temperature OSS
	 *	ultra low power			x1			x1
	 *	low power			x2			x1
	 *	standard resolution		x4			x1
	 *	high resolution			x8			x2
	 *	ultra high resolution		x16			x2
	 */
	/* The oversampling settings are set by using the following API*/
	if (bmp280_set_work_mode(BMP280_ULTRA_HIGH_RESOLUTION_MODE) != 0) {
		elog("BMP280 ULTRA_HIGH_RESOLUTION work mode failed");
	}

	HAL_Delay(100);

	if (bmp280_set_work_mode(BMP280_STANDARD_RESOLUTION_MODE) != 0) {
		elog("BMP280 STANDARD_RESOLUTION work mode failed");
	}

	/**
	 * MH-Z19
	 */
	uint16_t co2_ppm = 0;

	/**
	* ADC (battery)
	*/
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
		elog("ADC calibration fail");
	}

	printf("Battery voltage: %1.2fv \n", getBatteryVoltage());

	/**
	 * For battery voltage calibration
	 */
//	while(1) {
//		float voltage = getBatteryVoltage();
//
//		printf("Battery voltage: %1.2fv \n", voltage);
//
//		HAL_Delay(200);
//	}

	/**
	 * Watch DOG
	 */
	HAL_IWDG_Start(&hiwdg);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	State state = ACTIVE;
	Screen screen = LOGO;

	uint32_t nextScreenUpdateTimestamp = RTC_GetTimestamp() + SHOW_LOGO_DELAY;
	uint32_t nextMeasurementTimestamp = 0;
	uint32_t nextStateUpdateTimestamp = RTC_GetTimestamp() + STATE_ACTIVE_DELAY;

	int screen_is_on = 0;
	audio_activation = 0;

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//    printf("\n[%d] %lu %lu\n", __LINE__, HAL_GetTick(), RTC_GetTimestamp());

    uint32_t timestamp = RTC_GetTimestamp();

    /**
     * State
     */
	if (audio_activation) {
		slog("Audio activation");

		audio_activation = 0;

		state = ACTIVE;
		nextStateUpdateTimestamp = timestamp + STATE_ACTIVE_DELAY;

		if (screen == HIDE) {
			screen = MEASUREMENTS;
			nextScreenUpdateTimestamp = timestamp + SHOW_MEASUREMENTS_DELAY;
			nextMeasurementTimestamp = 0;
		}
	}

    if (timestamp >= nextStateUpdateTimestamp) {
		switch(state) {
		case ACTIVE:
			slog("Time to go sleep");

			state = SLEEP;
			screen = HIDE;
			nextStateUpdateTimestamp = timestamp + STATE_SLEEP_DELAY;
			break;

		case SLEEP:
			slog("Time to show me");

			state = ACTIVE;
			nextStateUpdateTimestamp = timestamp + STATE_ACTIVE_DELAY;

			if (screen == HIDE) {
				screen = MEASUREMENTS;
				nextScreenUpdateTimestamp = timestamp + SHOW_MEASUREMENTS_DELAY;
			}
			break;
		}
    }

    /**
     * Measurements
     */
    if (timestamp >= nextMeasurementTimestamp) {
    	updateMeasurements(&temperature, &humidity, &pressure, &co2_ppm);

    	printf("%lu;%.1f;%.1f;%.1f;%d;\n",
    			timestamp, temperature, humidity, PA2mmHG(pressure), co2_ppm);

		switch(state) {
		case ACTIVE:
			nextMeasurementTimestamp = timestamp + MEASUREMENTS_ACTIVE_DELAY;
			break;

		case SLEEP:
			nextMeasurementTimestamp = timestamp + MEASUREMENTS_SLEEP_DELAY;
			break;
		}
    }

    /**
     * Screen
     */
    if (timestamp >= nextScreenUpdateTimestamp) {
    	switch (screen) {
    	case LOGO:
    		screen = MEASUREMENTS;
    		nextScreenUpdateTimestamp = timestamp + SHOW_MEASUREMENTS_DELAY;
    		break;

    	case MEASUREMENTS:
    		screen = BATTERY;
    		nextScreenUpdateTimestamp = timestamp + SHOW_BATTERY_STATUS_DELAY;
    		break;

    	case BATTERY:
    		screen = MEASUREMENTS;
    		nextScreenUpdateTimestamp = timestamp + SHOW_MEASUREMENTS_DELAY;
    		break;

    	case HIDE:
    		nextScreenUpdateTimestamp = UINT32_MAX;
    		break;
    	}
    }

	switch (screen) {
	case LOGO:
	case MEASUREMENTS:
	case BATTERY:
		if (!screen_is_on) {
			SSD1306_ON();
			screen_is_on = 1;
		}
		break;

	case HIDE:
		if (screen_is_on) {
			SSD1306_OFF();
			screen_is_on = 0;
		}
		break;
	}

	switch (screen) {
	case LOGO:
		showLogo();
		break;

	case MEASUREMENTS:
		showMeasurements(temperature, humidity, pressure, co2_ppm);
		break;

	case BATTERY:
		showBatteryCharge(getBatteryCharge(getBatteryVoltage()));
		break;

	default:
		break;
	}

//	printf("[%d] M:%lu (%lu) S:%lu (%lu)  T:%lu  Sleep: (%lu) \n", __LINE__,
//			nextMeasurementTimestamp, nextMeasurementTimestamp - timestamp,
//			nextScreenUpdateTimestamp, nextScreenUpdateTimestamp - timestamp,
//			timestamp, MIN(nextMeasurementTimestamp, nextScreenUpdateTimestamp) - timestamp );

	HAL_IWDG_Refresh(&hiwdg);
	sleep( (MIN(nextMeasurementTimestamp, nextScreenUpdateTimestamp) - timestamp) );
	HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV128;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void __io_putchar(uint8_t ch) {
	/**
	 * \brief		__io_putchar - A routine to transmit a single character out the serial port
	 * \return		void
	 * \param[in]	ch - uint8_t the character to transmit
	 * \author		andreichichak
	 * \date		Oct 16, 2015
	 * \details		This routine assumes that we will be using UART2. A timeout value of 1ms (4th parameter)
	 * 				gives a retry if the transmit buffer is full when back to back characters are transmitted,
	 * 				avoiding dropping characters.
	 */

	HAL_UART_Transmit(&huart2, &ch, 1, 1);
}

/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bmp280_t
*-------------------------------------------------------------------------*/
s8 I2C_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bmp280 the following structure parameter can be accessed
 *	Bus write function pointer: BMP280_WR_FUNC_PTR
 *	Bus read function pointer: BMP280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	BMP280.bus_write  = BMP280_I2C_bus_write;
	BMP280.bus_read   = BMP280_I2C_bus_read;
	BMP280.dev_addr   = (BMP280_I2C_ADDRESS1 << 1);
	BMP280.delay_msec = BMP280_delay_msek;

	return BMP280_INIT_VALUE;
}

#define	I2C_BUFFER_LEN 8
#define BMP280_DATA_INDEX	1

/*	\Brief: The function is used as I2C bus write
*	\Return : Status of the I2C write
*	\param dev_addr : The device address of the sensor
*	\param reg_addr : Address of the first register, where data is to be written
*	\param reg_data : It is a value held in the array,
*		which is written in the register
*	\param cnt : The no of bytes of data to be written
*/
s8  BMP280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP280_INIT_VALUE;

	/*
	* Please take the below function as your reference for
	* write the data using I2C communication
	* "IERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write function here
	* iError is an return value of I2C read function
	* Please select your valid return value
	* In the driver SUCCESS defined as BMP280_INIT_VALUE
	* and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated.Thus cnt+1 operation done in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/

	if (HAL_I2C_Mem_Write(&hi2c2, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, cnt, 200) != HAL_OK) {
		iError = ERROR;
	}

	return (s8)iError;
}

/*	\Brief: The function is used as I2C bus read
*	\Return : Status of the I2C read
*	\param dev_addr : The device address of the sensor
*	\param reg_addr : Address of the first register, where data is going to be read
*	\param reg_data : This is the data read from the sensor, which is held in an array
*	\param cnt : The no of data to be read
*/
s8  BMP280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP280_INIT_VALUE;

	/* Please take the below function as your reference
	 * to read the data using I2C communication
	 * add your I2C rad function here.
	 * "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
	 * iError is an return value of SPI write function
	 * Please select your valid return value
	 * In the driver SUCCESS defined as BMP280_INIT_VALUE
	 * and FAILURE defined as -1
	 */

	if (HAL_I2C_Mem_Read(&hi2c2, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, cnt, 200) != HAL_OK) {
		iError = ERROR;
	}

	return (s8)iError;
}

/*	Brief : The delay routine
*	\param : delay in ms
*/
void  BMP280_delay_msek(u32 msek) {
	HAL_Delay(msek);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
