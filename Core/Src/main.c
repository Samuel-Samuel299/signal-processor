/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS_PORT GPIOB
#define RS_PIN GPIO_PIN_14
#define RW_PORT GPIOB
#define RW_PIN GPIO_PIN_15
#define EN_PORT GPIOB
#define EN_PIN GPIO_PIN_1
#define D4_PORT GPIOA
#define D4_PIN GPIO_PIN_10
#define D5_PORT GPIOA
#define D5_PIN GPIO_PIN_12
#define D6_PORT GPIOC
#define D6_PIN GPIO_PIN_6
#define D7_PORT GPIOC
#define D7_PIN GPIO_PIN_8
#define PI 3.1415926
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern volatile int ADC_trigger;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//student number
uint8_t sn[13] = "@,23594780,!\n";
uint8_t marker[6] = "Marker";
uint8_t rx[1];

//Status
uint8_t receiveCommand = 0;
uint8_t measurementMode[2] = "DV";
uint8_t outputType[1] = "d";
uint8_t outputState = 1;
uint8_t measureParameter;
uint8_t commandType = 0;
enum measureTypes{t, a, o, f, d, c};
enum measureTypes measure_type;


//ADC variables
uint32_t adc_measure = 0;
uint32_t prev_measure = 0;
uint8_t max_reached = 1;
uint8_t min_reached = 0 ;

double period = 0;
double freq = 0;
double pk_volt = 0;
double offset = 0;
uint32_t nr_measured_dv = 1;
uint32_t nr_measured_av = 1;
uint32_t total_measured = 0;
double avg_dv_val = 0;
double avg_av_val = 0;

uint32_t max = 0 ;
uint32_t min = 4095 ;
uint32_t hundred= 100;
uint8_t sampling_enabled = 0;
double nr_samples = 0;
double sampling_freq = 10000;
uint32_t avg_cross = 0;

uint8_t type = 100;
uint64_t freq_int = 0;
uint64_t avg_dv_int = 1000;
uint64_t offset_int = 0;
uint64_t amp_int = 0;


//Call back function
uint8_t counter = 0;
uint8_t input[20];
uint8_t numBytes;

//System state is in Menu state (indicated by a 0)
uint8_t systemState = 1;
uint8_t prevSystemState = 2;

//DAC variables
uint8_t startDMA = 0;
uint8_t stopDMA = 0;
uint32_t signal_buffer[1000];

float amplitude_DAC = 1000;
uint32_t frequency_DAC = 1000;
float DC_offset_DAC = 1000;
float S_offset_DAC = 1200;
uint32_t duty_cycle_DAC = 0;
uint8_t signal_type_DAC[1] = "d";
uint8_t parameter_type_DAC[1] = "o";


//LCD variables
uint8_t clearFlag = 0;
uint8_t lcdDisplay = 1;
uint8_t stateOne = 0;
uint8_t stateTwo = 0;


//Menu variables
uint8_t display_state = 1;
uint32_t lastTick = 20000;
uint8_t measurement_state = 1;
uint8_t output_branch_state = 1;
int amp;
int offset1;
int freq1;
int dutcyc;

//LCD display flags
uint8_t measurement_state_has_displayed = 0;
uint8_t system_state_1_has_displayed = 0;
uint8_t output_state_has_displayed = 0;
uint8_t display_state_2_has_displayed = 0;
uint8_t output_LED_has_displayed = 0;

//Button variables
int downButtonPressed;
int upButtonPressed;
int rightButtonPressed;
int leftButtonPressed;
int buttonPressed;

uint32_t lastTick0 = 0;
uint32_t lastTick1 = 0;
uint32_t lastTick2 = 0;
uint32_t lastTick3 = 0;
uint32_t lastTick4 = 0;

//Helping comment
/*
measurement_state_has_displayed = 0;
system_state_1_has_displayed = 0;
output_state_has_displayed = 0;
display_state_2_has_displayed = 0;
systemState = 1;
measurement_state = 1;
output_branch_state = 1;
display_state = 1;
*/

void delay (uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
	if (GPIO_Pin == GPIO_PIN_4) {
		if ((HAL_GetTick() - lastTick0) > 100) {
			buttonPressed = 1;
			lastTick0 = HAL_GetTick();
		}
	} else if (GPIO_Pin == GPIO_PIN_6) {
		if (HAL_GetTick() - lastTick1 > 100) {
			downButtonPressed = 1;
			lastTick1 = HAL_GetTick();
		}
	} else if (GPIO_Pin == GPIO_PIN_7) {
		if (HAL_GetTick() - lastTick2 > 100) {
			rightButtonPressed = 1;
			lastTick2 = HAL_GetTick();
		}
	} else if (GPIO_Pin == GPIO_PIN_8) {
		if (HAL_GetTick() - lastTick3 > 100) {
			leftButtonPressed = 1;
			lastTick3 = HAL_GetTick();
		}
	} else if (GPIO_Pin == GPIO_PIN_9) {
		if ((HAL_GetTick() - lastTick4) > 100) {
			upButtonPressed = 1;
			lastTick4 = HAL_GetTick();
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (rx[0] != 10)
	{
		input[counter] = rx[0];
		counter++;
	}
	else
	{
		input[counter] = 10;
		numBytes = counter+1;
		counter = 0;
		receiveCommand = 1;
	}

	HAL_UART_Receive_IT(&huart2, rx, 1);
}

void determineCommandType()
{

	if (input[0] == 64 && input[2] == 35)
	{
		commandType = 1;
	}
	else if (input[0] == 64 && input[2] == 36)
	{
		commandType = 2;
	}
	else if (input[0] == 64 && input[2] == 94)
	{
		commandType = 3;
	}
	else if (input[0] == 64 && input[2] == 42 && input[4] == 115)
	{
		commandType = 4;
	}
	else if (input[0] == 64 && input[2] == 42 && input[4] == 109)
	{
		commandType = 5;
	}
	else
	{
		commandType = 0;
	}
}

void adc_Measure() {

	HAL_ADC_Start(&hadc1);
	HAL_StatusTypeDef res = HAL_ADC_PollForConversion(&hadc1, 50);



	if (res == HAL_OK) {


		adc_measure = HAL_ADC_GetValue(&hadc1);


		//calibrating error of adc value:
		if (adc_measure > 100 && adc_measure < 200) {
			adc_measure += 50;
		}
		else if (adc_measure >= 200 && adc_measure < 300)
		{
			adc_measure += 60;
		}
		else if (adc_measure >= 300 && adc_measure < 400)
		{
			adc_measure += 75;
		}
		else if (adc_measure >= 400 && adc_measure <1000)
		{
			adc_measure += 110;
		}
		else if (adc_measure >= 1000)
		{
			adc_measure += 120;
		}



		if (measurementMode[0] == 'D' && measurementMode[1] == 'V') {

			//for dc count measurements and sum measurements
			//avg = total/count
			total_measured += adc_measure;
			if (nr_measured_dv == 2000) {
				avg_dv_val = total_measured / nr_measured_dv;
				avg_dv_val = avg_dv_val / 4095 * 3300;
				avg_dv_int = (uint64_t) (avg_dv_val*2);
				total_measured = 0;
				nr_measured_dv = 1;

			}
			nr_measured_dv++;
		}

		if (measurementMode[0] == 'A' && measurementMode[1] == 'V') {
			//get max
			if (adc_measure >= max) {
				max = adc_measure;
			}
			//get min
			if (adc_measure <= min) {
				min = adc_measure;
			}

			//counts how many times avg crossed
			//only triggered to count once every period (min_reached controls this)
			if ((adc_measure >= avg_av_val) && (min_reached)) {
				max_reached = 1;
				avg_cross++;

				//freq avg in 100 waves
				if (avg_cross == 100) {
					//freg = asamp_freq/(nr_samples since last avg cross)
					freq = (sampling_freq / nr_samples) * 100;
					freq_int = (uint64_t) (freq*2);
					nr_samples = 0;
					avg_cross = 0;
				}
				min_reached = 0;
			}

			nr_samples += 1;

			if ((adc_measure <= avg_av_val) && (max_reached)) {
				min_reached = 1;
				max_reached = 0;
			}

			total_measured += adc_measure;
			nr_measured_av++;

			if (nr_measured_av == 7000) {
				avg_av_val = total_measured / nr_measured_av;
				pk_volt = max - min;
				pk_volt = (pk_volt) / 4095 * 3300;

				amp_int = (uint64_t) (pk_volt*2);

				offset = avg_av_val / 4095 * 3300;
				offset_int = (uint64_t) (offset*2);

				total_measured = 0;
				nr_measured_av = 0;
				max = 0;
				min = 4095;
			}

		}
	}
	HAL_ADC_Stop(&hadc1);
}

void commandFourResponse()
{
	outputState = input[6]-48;
	if (outputState == 1)
	{
		stopDMA = 0;
	}

	uint8_t statusMessage[11] = "@,xx,x,x,!x";
	statusMessage[2] = measurementMode[0];
	statusMessage[3] = measurementMode[1];
	statusMessage[5] = signal_type_DAC[0];
	statusMessage[7] = input[6];
	statusMessage[10] = 10;
	HAL_UART_Transmit(&huart2, statusMessage, 11, 50);
}

void send_to_lcd (char data, int rs)
{
	HAL_GPIO_WritePin(RS_PORT, RS_PIN, rs);  // rs = 1 for data, rs=0 for command

	/* write the data to the respective pin */
	HAL_GPIO_WritePin(D7_PORT, D7_PIN, ((data>>3)&0x01));
	HAL_GPIO_WritePin(D6_PORT, D6_PIN, ((data>>2)&0x01));
	HAL_GPIO_WritePin(D5_PORT, D5_PIN, ((data>>1)&0x01));
	HAL_GPIO_WritePin(D4_PORT, D4_PIN, ((data>>0)&0x01));

	/* Toggle EN PIN to send the data
	 * if the HCLK > 100 MHz, use the  20 us delay
	 * if the LCD still doesn't work, increase the delay to 50, 80 or 100..
	 */
	HAL_GPIO_WritePin(EN_PORT, EN_PIN, 1);
	delay(20);
	HAL_GPIO_WritePin(EN_PORT, EN_PIN, 0);
	delay(20);

}

void lcd_send_cmd (char cmd)
{
    char datatosend;
    /* send upper nibble first */
    datatosend = ((cmd>>4)&0x0f);
    send_to_lcd(datatosend,0);  // RS must be while sending command
    /* send Lower Nibble */
    datatosend = ((cmd)&0x0f);
    send_to_lcd(datatosend, 0);
}

void lcd_send_data (char data)
{
    char datatosend;

    /* send higher nibble */
    datatosend = ((data>>4)&0x0f);
    send_to_lcd(datatosend, 1);  // rs =1 for sending data
    /* send Lower nibble */
    datatosend = ((data)&0x0f);
    send_to_lcd(datatosend, 1);
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_cmd (col);
}

void lcd_init (void)
{
    // 4 bit initialisation
    HAL_Delay(50);  // wait for >40ms
    lcd_send_cmd (0x30);
    HAL_Delay(5);  // wait for >4.1ms
    lcd_send_cmd (0x30);
    HAL_Delay(1);  // wait for >100us
    lcd_send_cmd (0x30);
    HAL_Delay(10);
    lcd_send_cmd (0x20);  // 4bit mode
    HAL_Delay(10);

  // dislay initialisation
    lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
    HAL_Delay(1);
    lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
    HAL_Delay(1);
    lcd_send_cmd (0x01);  // clear display
    HAL_Delay(1);
    HAL_Delay(1);
    lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
    HAL_Delay(1);
    lcd_send_cmd (0x0E); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void lcd_clear (void)
{
	lcd_send_cmd(0x01);
	HAL_Delay(2);
}

void lcd_scroll_left_or_right(int left)
{
	if (left == 1)
	{
		lcd_send_cmd(0x1c);
	}
	else
	{
		lcd_send_cmd(0x18);
	}
}

void calculateDCSignalBuffer(uint32_t offset)
{

	for (int i = 0; i < 1000; i++)
	{
		signal_buffer[i] = ((offset*0.49)*4095.0/3300.0);
	}
}

void calculateSinusoidalSignalBuffer(float amplitude, uint32_t frequency, float offset)
{
	int ns = 100000/frequency;
	offset = offset/1000;
	amplitude = amplitude/1000;

	for (int i = 0; i < 1000; i++)
	{
		signal_buffer[i] = (amplitude*sin(i*2*PI/ns)/3.3 + offset*0.571)*(4096/2);
	}
}

void calculateSinusoidalSignalBuffer1(float amplitude, uint32_t frequency, float offset)
{
	int ns = 100000/frequency;
	offset = offset/1000;
	amplitude = amplitude/1000;

	for (int i = 0; i < 1000; i++)
	{
		signal_buffer[i] = (amplitude*0.246*sin(i*2*PI/ns)+(0.5*offset))*(4096/3.3);
	}
}

void calculateSinusoidalSignalBuffer2(float amplitude, uint32_t frequency, float offset)
{
	int signal_frequency = 200000/frequency;

	for (int i = 0; i < 2000; i++)
	{
		signal_buffer[i] = (((amplitude/2000) * sin(i*2*PI/signal_frequency) + offset/2000) * (4096/3.3));
	}
}

void calculatePulseSignalBuffer(float amplitude, uint32_t frequency, float offset, uint32_t duty_cycle)
{
	int num;
	int num1 = 100000/frequency;
	num = num1*(duty_cycle/100.0);

	for (int i = 0; i < 1000; i++)
	{
		if (i%num1 < num)
		{
			signal_buffer[i] = (((amplitude+offset)*0.49)*4095.0/3300.0);
		}
		else
		{
			signal_buffer[i] = ((offset*0.49)*4095.0/3300.0);
		}

	}
}

void fillBuffer(uint8_t signal_type_DAC[], float amplitude, uint32_t frequency, float offset)
{
	if (signal_type_DAC[0] == 'd')
	{
	calculateDCSignalBuffer(offset);
	}
	else if (signal_type_DAC[0] == 's')
	{
		calculateSinusoidalSignalBuffer(amplitude, frequency, offset);
	}
}

void setDACValue() {
	if (input[4] == 't') {
		signal_type_DAC[0] = input[6];
	} else {
		int value = ((input[6] - 48) * 1000) + ((input[7] - 48) * 100)
				+ ((input[8] - 48) * 10) + (input[9] - 48);

		if (parameter_type_DAC[0] == 'a') {
			amplitude_DAC = value;
		} else if (parameter_type_DAC[0] == 'o') {
			if (signal_type_DAC[0] == 'd') {
				DC_offset_DAC = value;
			} else if (signal_type_DAC[0] == 's') {
				S_offset_DAC = value;
			}
		} else if (parameter_type_DAC[0] == 'f') {
			frequency_DAC = value;
		} else if (parameter_type_DAC[0] == 'd') {
			duty_cycle_DAC = value;
		}

	}
}

void outputDAC()
{
	if (startDMA == 0)
	{
		if (signal_type_DAC[0] == 'd')
			{
			HAL_TIM_Base_Start(&htim3);
			calculateDCSignalBuffer(DC_offset_DAC);
			HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, signal_buffer, 1000, DAC_ALIGN_12B_R);
			}
			else if (signal_type_DAC[0] == 's')
			{
				HAL_TIM_Base_Start(&htim3);
				calculateSinusoidalSignalBuffer1(amplitude_DAC, frequency_DAC, S_offset_DAC);
				HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, signal_buffer, 1000, DAC_ALIGN_12B_R);
			} else if (signal_type_DAC[0] == 'p')
			{
				HAL_TIM_Base_Start(&htim3);
				calculatePulseSignalBuffer(amplitude_DAC, frequency_DAC, S_offset_DAC, duty_cycle_DAC);
				HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, signal_buffer, 1000, DAC_ALIGN_12B_R);
			}
		startDMA = 1;
	}


}

void stopOutputDAC()
{
	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
	startDMA = 0;

}

int determineValue()
{
	int16_t value = 0;

	if (input[6] == 116) {
		value = -1;
	} else if (input[6] == 97) {
		value = amp_int;
	} else if (input[6] == 111) {
		if (measurementMode[0] == 'D' && measurementMode[1] == 'V')
		{
			value = avg_dv_int;
		}
		else if (measurementMode[0] == 'A' && measurementMode[1] == 'V')
		{
			value = offset_int;
		}

	} else if (input[6] == 102) {
		value = freq_int;
	}

	return value;
}

void displayMeasurementMessage(int16_t value)
{
	uint8_t measurementMessage[13];

	if (value < 0) {
		measurementMessage[0] = 64;
		measurementMessage[1] = 44;
		measurementMessage[2] = 42;
		measurementMessage[3] = 44;
		measurementMessage[4] = measureParameter;
		measurementMessage[5] = 44;
		measurementMessage[6] = 32;
		measurementMessage[7] = type;
		measurementMessage[8] = 32;
		measurementMessage[9] = 32;
		measurementMessage[10] = 44;
		measurementMessage[11] = 33;
		measurementMessage[12] = 10;
	} else {
		uint8_t valueInStringForm[4] = "0000";
		valueInStringForm[0] = ((value / 1000) % 10) + 48;
		valueInStringForm[1] = ((value / 100) % 10) + 48;
		valueInStringForm[2] = ((value / 10) % 10) + 48;
		valueInStringForm[3] = (value % 10) + 48;
		measurementMessage[0] = 64;
		measurementMessage[1] = 44;
		measurementMessage[2] = 42;
		measurementMessage[3] = 44;
		measurementMessage[4] = measureParameter;
		measurementMessage[5] = 44;
		measurementMessage[6] = valueInStringForm[0];
		measurementMessage[7] = valueInStringForm[1];
		measurementMessage[8] = valueInStringForm[2];
		measurementMessage[9] = valueInStringForm[3];
		measurementMessage[10] = 44;
		measurementMessage[11] = 33;
		measurementMessage[12] = 10;
	}
	HAL_UART_Transmit(&huart2, measurementMessage, 13, 50);
}

void executeLCDCommand()
{
	uint8_t byte;
	byte = input[6];

}

void executeCommand()
{
	if (commandType == 1) {
		executeLCDCommand();
	} else if (commandType == 2) {
		measurementMode[0] = input[4];
		measurementMode[1] = input[5];
		measurement_state_has_displayed = 0;
		system_state_1_has_displayed = 0;
		output_state_has_displayed = 0;
		display_state_2_has_displayed = 0;
		systemState = 2;
		if (measurementMode[0] == 'D' && measurementMode[1] == 'V')
		{
			measurement_state = 2;
		}
		else if (measurementMode[0] == 'A' && measurementMode[1] == 'V')
		{
			measurement_state = 3;
		}
		output_branch_state = 1;
		display_state = 1;
		lcd_clear();
	} else if (commandType == 3) {
		parameter_type_DAC[0] = input[4];
		measurement_state_has_displayed = 0;
		system_state_1_has_displayed = 0;
		output_state_has_displayed = 0;
		display_state_2_has_displayed = 0;
		systemState = 3;
		measurement_state = 1;
		display_state = 1;
		setDACValue();
		if (parameter_type_DAC[0] == 't') {
			if (signal_type_DAC[0] == 'd') {
				output_branch_state = 3;
			} else if (signal_type_DAC[0] == 's') {
				output_branch_state = 4;
			} else if (signal_type_DAC[0] == 'p') {
				output_branch_state = 5;
			}
		} else if (parameter_type_DAC[0] == 'a') {
				output_branch_state = 8;
		} else if (parameter_type_DAC[0] == 'o') {
				output_branch_state = 10;
		} else if (parameter_type_DAC[0] == 'f') {
				output_branch_state = 12;
		} else if (parameter_type_DAC[0] == 'd') {
				output_branch_state = 14;
		}
		lcd_clear();
	} else if (commandType == 4) {
		commandFourResponse();
		measurement_state_has_displayed = 0;
		system_state_1_has_displayed = 0;
		output_state_has_displayed = 0;
		display_state_2_has_displayed = 0;
		systemState = 3;
		measurement_state = 1;
		display_state = 1;
		if (outputState == 1)
		{
			output_branch_state = 16;
		}
		else if (outputState == 0)
		{
			output_branch_state = 17;
		}
		lcd_clear();

	} else if (commandType == 5) {
		measureParameter = input[6];
		int16_t value = determineValue();
		displayMeasurementMessage(value);
		measurement_state_has_displayed = 0;
		system_state_1_has_displayed = 0;
		output_state_has_displayed = 0;
		display_state_2_has_displayed = 0;
		systemState = 1;
		output_branch_state = 1;
		display_state = 2;
		measurement_state = 1;
		lcd_clear();
	}
}

void measure_ADC_output_DAC()
{
	if (ADC_trigger == 1) {
		adc_Measure();
		ADC_trigger = 0;
	}

	if (outputState == 1) {
		outputDAC();
	} else {
		if (stopDMA == 0)
		{
		stopOutputDAC();
		stopDMA = 1;
		}
	}
}

void executeMeasurementOutput()
{
	if (display_state_2_has_displayed == 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
		display_state_2_has_displayed = 1;
	}

	if (HAL_GetTick() - lastTick > 7000) {
		lcd_clear();
		lastTick = HAL_GetTick();

		if (measurementMode[0] == 'D' && measurementMode[1] == 'V') {
			char displayString[] = "X.XXXV";
			displayString[0] = (avg_dv_int / 1000) + 48;
			displayString[2] = ((avg_dv_int % 1000) / 100) + 48;
			displayString[3] = ((avg_dv_int % 100) / 10) + 48;
			displayString[4] = (avg_dv_int % 10) + 48;
			lcd_send_string(displayString);
		} else if (measurementMode[0] == 'A' && measurementMode[1] == 'V') {
			char displayString[] = "O:x.xxxV,A:x.xxxV,F:xxxxHz";
			displayString[2] = (offset_int/1000)+48;
			displayString[4] = ((offset_int%1000)/100)+48;
			displayString[5] = ((offset_int%100)/10)+48;
			displayString[6] = (offset_int%10)+48;
			displayString[11] = (amp_int/1000)+48;
			displayString[13] = ((amp_int%1000)/100)+48;
			displayString[14] = ((amp_int%100)/10)+48;
			displayString[15] = (amp_int%10)+48;
			displayString[20] = (freq_int/1000)+48;
			displayString[21] = ((freq_int%1000)/100)+48;
			displayString[22] = ((freq_int%100)/10)+48;
			displayString[23] = (freq_int%10)+48;
			lcd_send_string(displayString);
		}

		lcd_put_cur(1, 0);
		if (outputState == 1) {
			if (signal_type_DAC[0] == 'd')
			{
			char displayString[] = "X.XXXV";
			displayString[0] = (((int)DC_offset_DAC) / 1000) + 48;
			displayString[2] = ((((int)DC_offset_DAC) % 1000) / 100) + 48;
			displayString[3] = ((((int)DC_offset_DAC) % 100) / 10) + 48;
			displayString[4] = (((int)DC_offset_DAC) % 10) + 48;
			lcd_send_string(displayString);
			}
			else if (signal_type_DAC[0] == 's')
			{
				char displayString[] = "O:x.xxxV,A:x.xxxV,F:xxxxHz";
				displayString[2] = (((int)S_offset_DAC) / 1000) + 48;
				displayString[4] = ((((int)S_offset_DAC) % 1000) / 100) + 48;
				displayString[5] = ((((int)S_offset_DAC) % 100) / 10) + 48;
				displayString[6] = (((int)S_offset_DAC) % 10) + 48;
				displayString[11] = (((int)amplitude_DAC) / 1000) + 48;
				displayString[13] = ((((int)amplitude_DAC) % 1000) / 100) + 48;
				displayString[14] = ((((int)amplitude_DAC) % 100) / 10) + 48;
				displayString[15] = (((int)amplitude_DAC) % 10) + 48;
				displayString[20] = (((int)frequency_DAC) / 1000) + 48;
				displayString[21] = ((((int)frequency_DAC) % 1000) / 100) + 48;
				displayString[22] = ((((int)frequency_DAC) % 100) / 10) + 48;
				displayString[23] = (((int)frequency_DAC) % 10) + 48;
				lcd_send_string(displayString);
			}
			else if (signal_type_DAC[0] == 'p')
			{
				char displayString[] = "”O:x.xxxV,A:x.xxxV,F:xxxxHz,D:xxx%";
				displayString[2] = (((int)S_offset_DAC) / 1000) + 48;
				displayString[4] = ((((int)S_offset_DAC) % 1000) / 100) + 48;
				displayString[5] = ((((int)S_offset_DAC) % 100) / 10) + 48;
				displayString[6] = (((int)S_offset_DAC) % 10) + 48;
				displayString[11] = (((int)amplitude_DAC) / 1000) + 48;
				displayString[13] = ((((int)amplitude_DAC) % 1000) / 100) + 48;
				displayString[14] = ((((int)amplitude_DAC) % 100) / 10) + 48;
				displayString[15] = (((int)amplitude_DAC) % 10) + 48;
				displayString[20] = (((int)frequency_DAC) / 1000) + 48;
				displayString[21] = ((((int)frequency_DAC) % 1000) / 100) + 48;
				displayString[22] = ((((int)frequency_DAC) % 100) / 10) + 48;
				displayString[23] = (((int)frequency_DAC) % 10) + 48;
				displayString[28] = (((int)duty_cycle_DAC) / 100) + 48;
				displayString[29] = (((int)duty_cycle_DAC)%100)+48;
				displayString[30] = (((int)duty_cycle_DAC)%10)+48;
				lcd_send_string(displayString);
			}
		} else if (outputState == 0) {
			lcd_put_cur(1, 0);
			lcd_send_string("OUTPUT OFF");
		}
	}

	if (leftButtonPressed == 1)
	{
		leftButtonPressed = 0;
		lcd_scroll_left_or_right(1);
	}
	else if (rightButtonPressed == 1)
	{
		rightButtonPressed = 0;
		lcd_scroll_left_or_right(0);
	}



	if (buttonPressed == 1) {
		display_state = 1;
		buttonPressed = 0;
		display_state_2_has_displayed = 0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
		lcd_clear();
	}
}

void executeStartMessage()
{
	if (system_state_1_has_displayed == 0)
		{
		//lcd_send_string("Sam's Menu");
		lcd_send_string("Snozzi's Menu");
		lcd_put_cur(1,0);
		lcd_send_string("Measure/Output");
		system_state_1_has_displayed = 1;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
		buttonPressed = 0;
		}
		if (buttonPressed == 1)
		{
			buttonPressed = 0;
			display_state = 2;
			lcd_clear();
			system_state_1_has_displayed = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
		} else if(downButtonPressed == 1)
		{
			systemState = 2;
			downButtonPressed = 0;
			lcd_clear();
			system_state_1_has_displayed = 0;
		}
}

void executeMeasurementState()
{
		if (measurement_state == 1)
		{
			if (measurement_state_has_displayed == 0)
			{
			lcd_send_string("MEASURE");
			lcd_put_cur(1,0);
			lcd_send_string("MODE");
			measurement_state_has_displayed = 1;
			}
			if (downButtonPressed == 1)
			{
				downButtonPressed = 0;
				measurement_state = 2;
				lcd_clear();
				measurement_state_has_displayed = 0;
			} else if (rightButtonPressed == 1)
			{
				rightButtonPressed = 0;
				systemState = 3;
				lcd_clear();
				measurement_state_has_displayed = 0;
			} else if (upButtonPressed == 1)
			{
				upButtonPressed = 0;
				systemState = 1;
				lcd_clear();
				measurement_state_has_displayed = 0;
				measurement_state = 1;
			}
		}
		else if (measurement_state == 2)
		{
			if (measurement_state_has_displayed == 0)
			{
			lcd_send_string("DC VOLTAGE");
			measurement_state_has_displayed = 1;
				if (measurementMode[0] == 'D' && measurementMode[1] == 'V')
				{
					lcd_put_cur(1,0);
					lcd_send_string("SET");
				}
			}
			if (buttonPressed == 1)
			{
				buttonPressed = 0;
				measurementMode[0] = 'D';
				measurementMode[1] = 'V';
				measurement_state_has_displayed = 0;
				lcd_clear();
			}
			else if (rightButtonPressed == 1)
			{
				rightButtonPressed = 0;
				measurement_state = 3;
				lcd_clear();
				measurement_state_has_displayed = 0;
			} else if (upButtonPressed == 1)
			{
				upButtonPressed = 0;
				measurement_state = 1;
				lcd_clear();
				measurement_state_has_displayed = 0;
			}
		}
		else if (measurement_state == 3)
		{
			if (measurement_state_has_displayed == 0)
			{
			lcd_send_string("AC VOLTAGE");
			measurement_state_has_displayed = 1;
			if (measurementMode[0] == 'A' && measurementMode[1] == 'V')
			{
				lcd_put_cur(1,0);
				lcd_send_string("SET");
			}
			}
			if (buttonPressed == 1)
			{
				buttonPressed = 0;
				measurementMode[0] = 'A';
				measurementMode[1] = 'V';
				measurement_state_has_displayed = 0;
				lcd_clear();
			}
			else if (leftButtonPressed == 1)
			{
				leftButtonPressed = 0;
				measurement_state = 2;
				lcd_clear();
				measurement_state_has_displayed = 0;
			}
			else if (upButtonPressed == 1)
			{
				upButtonPressed = 0;
				measurement_state = 1;
				lcd_clear();
				measurement_state_has_displayed = 0;
			}
		}
}

void branchesOneToSix()
{
	if (output_branch_state == 1) {
		if (output_state_has_displayed == 0) {
			lcd_send_string("SIG GEN");
			output_state_has_displayed = 1;
		}

		if (downButtonPressed == 1) {
			downButtonPressed = 0;
			output_branch_state = 2;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (upButtonPressed == 1) {
			upButtonPressed = 0;
			systemState = 1;
			output_branch_state = 1;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (leftButtonPressed == 1) {
			leftButtonPressed = 0;
			output_branch_state = 1;
			systemState = 2;
			output_state_has_displayed = 0;
			lcd_clear();
		}

	} else if (output_branch_state == 2) {
		if (output_state_has_displayed == 0) {
			lcd_send_string("TYPE");
			output_state_has_displayed = 1;
		}

		if (downButtonPressed == 1) {
			downButtonPressed = 0;
			output_branch_state = 3;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (rightButtonPressed == 1) {
			rightButtonPressed = 0;
			output_branch_state = 6;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (upButtonPressed == 1) {
			upButtonPressed = 0;
			output_branch_state = 1;
			output_state_has_displayed = 0;
			lcd_clear();
		}
	} else if (output_branch_state == 3) {
		if (output_state_has_displayed == 0) {
			lcd_send_string("DC");
			output_state_has_displayed = 1;
			if (signal_type_DAC[0] == 'd')
			{
				lcd_put_cur(1,0);
				lcd_send_string("SET");
			}
		}

		if (buttonPressed == 1) {
			buttonPressed = 0;
			signal_type_DAC[0] = 'd';
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (upButtonPressed == 1) {
			upButtonPressed = 0;
			output_branch_state = 2;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (rightButtonPressed == 1) {
			rightButtonPressed = 0;
			output_branch_state = 4;
			output_state_has_displayed = 0;
			lcd_clear();
		}
	} else if (output_branch_state == 4) {
		if (output_state_has_displayed == 0) {
			lcd_send_string("SINUSOIDAL");
			output_state_has_displayed = 1;
			if (signal_type_DAC[0] == 's')
			{
				lcd_put_cur(1,0);
				lcd_send_string("SET");
			}
		}

		if (buttonPressed == 1) {
			buttonPressed = 0;
			output_state_has_displayed = 0;
			signal_type_DAC[0] = 's';
			lcd_clear();
		} else if (upButtonPressed == 1) {
			upButtonPressed = 0;
			output_branch_state = 2;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (rightButtonPressed == 1) {
			rightButtonPressed = 0;
			output_branch_state = 5;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (leftButtonPressed == 1) {
			leftButtonPressed = 0;
			output_branch_state = 3;
			output_state_has_displayed = 0;
			lcd_clear();
		}
	} else if (output_branch_state == 5) {
		if (output_state_has_displayed == 0) {
			lcd_send_string("PULSE");
			output_state_has_displayed = 1;
			if (signal_type_DAC[0] == 'p')
			{
				lcd_put_cur(1,0);
				lcd_send_string("SET");
			}
		}

		if (buttonPressed == 1) {
			buttonPressed = 0;
			output_state_has_displayed = 0;
			signal_type_DAC[0] = 'p';
			lcd_clear();
		} else if (upButtonPressed == 1) {
			upButtonPressed = 0;
			output_branch_state = 2;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (leftButtonPressed == 1) {
			leftButtonPressed = 0;
			output_branch_state = 4;
			output_state_has_displayed = 0;
			lcd_clear();
		}
	} else if (output_branch_state == 6) {
		if (output_state_has_displayed == 0) {
			lcd_send_string("PARAMETER");
			output_state_has_displayed = 1;
		}

		if (downButtonPressed == 1) {
			downButtonPressed = 0;
			output_branch_state = 7;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (upButtonPressed == 1) {
			upButtonPressed = 0;
			output_branch_state = 1;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (rightButtonPressed == 1) {
			rightButtonPressed = 0;
			output_branch_state = 15;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (leftButtonPressed == 1) {
			leftButtonPressed = 0;
			output_branch_state = 2;
			output_state_has_displayed = 0;
			lcd_clear();
		}
	}
}

void branchesSevenToTen()
{
	if (output_branch_state == 7) {
		if (output_state_has_displayed == 0) {
			lcd_send_string("AMPLITUDE");
			output_state_has_displayed = 1;
		}

		if (downButtonPressed == 1) {
			downButtonPressed = 0;
			output_branch_state = 8;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (upButtonPressed == 1) {
			upButtonPressed = 0;
			output_branch_state = 6;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (rightButtonPressed == 1) {
			rightButtonPressed = 0;
			output_branch_state = 9;
			output_state_has_displayed = 0;
			lcd_clear();
		}
	} else if (output_branch_state == 8) {

		if (output_state_has_displayed == 0) {
			lcd_send_string("AMPLITUDE");
			lcd_put_cur(1, 0);
			amp = amplitude_DAC;
			char send_string[] = "XXXXmV";
			send_string[0] = (amp / 1000) + 48;
			send_string[1] = ((amp % 1000) / 100) + 48;
			send_string[2] = ((amp % 100) / 10) + 48;
			send_string[3] = (amp % 10) + 48;
			lcd_send_string(send_string);
			output_state_has_displayed = 1;
		}

		if (leftButtonPressed == 1) {
			leftButtonPressed = 0;
			lcd_put_cur(1, 0);
			if ((amp - 100) < 0) {
				amp = 2200 - (100 - amp);
			} else {
				amp = amp - 100;
			}
			char send_string[] = "XXXXmV";
			send_string[0] = (amp / 1000) + 48;
			send_string[1] = ((amp % 1000) / 100) + 48;
			send_string[2] = ((amp % 100) / 10) + 48;
			send_string[3] = (amp % 10) + 48;
			lcd_send_string(send_string);
		} else if (upButtonPressed == 1) {
			upButtonPressed = 0;
			output_branch_state = 7;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (rightButtonPressed == 1) {
			rightButtonPressed = 0;
			lcd_put_cur(1, 0);
			if ((amp + 100 > 2200)) {
				amp = (amp + 100) - 2200;
			}
			else
			{
				amp = amp + 100;
			}
			char send_string[] = "XXXXmV";
			send_string[0] = (amp / 1000) + 48;
			send_string[1] = ((amp % 1000) / 100) + 48;
			send_string[2] = ((amp % 100) / 10) + 48;
			send_string[3] = (amp % 10) + 48;
			lcd_send_string(send_string);
		} else if (buttonPressed == 1) {
			buttonPressed = 0;
			amplitude_DAC = amp;
			lcd_put_cur(1, 8);
			lcd_send_string("SET");
		}
	} else if (output_branch_state == 9) {
		if (output_state_has_displayed == 0) {
			lcd_send_string("OFFSET");
			output_state_has_displayed = 1;
		}

		if (downButtonPressed == 1) {
			downButtonPressed = 0;
			output_branch_state = 10;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (upButtonPressed == 1) {
			upButtonPressed = 0;
			output_branch_state = 6;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (rightButtonPressed == 1) {
			rightButtonPressed = 0;
			output_branch_state = 11;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (leftButtonPressed == 1) {
			leftButtonPressed = 0;
			output_branch_state = 7;
			output_state_has_displayed = 0;
			lcd_clear();
		}

	} else if (output_branch_state == 10) {
		if (output_state_has_displayed == 0) {
			lcd_send_string("OFFSET");
			lcd_put_cur(1, 0);
			if (signal_type_DAC[0] == 'd')
			{
				offset1 = DC_offset_DAC;
			}
			else
			{
				offset1 = S_offset_DAC;
			}
			char send_string[] = "XXXXmV";
			send_string[0] = (offset1 / 1000) + 48;
			send_string[1] = ((offset1 % 1000) / 100) + 48;
			send_string[2] = ((offset1 % 100) / 10) + 48;
			send_string[3] = (offset1 % 10) + 48;
			lcd_send_string(send_string);
			output_state_has_displayed = 1;
		}

		if (leftButtonPressed == 1) {
			leftButtonPressed = 0;
			lcd_put_cur(1, 0);
			if ((offset1 - 100) < 0) {
				offset1 = 2200 - (100 - offset1);
			} else {
				offset1 = offset1 - 100;
			}
			char send_string[] = "XXXXmV";
			send_string[0] = (offset1 / 1000) + 48;
			send_string[1] = ((offset1 % 1000) / 100) + 48;
			send_string[2] = ((offset1 % 100) / 10) + 48;
			send_string[3] = (offset1 % 10) + 48;
			lcd_send_string(send_string);
		} else if (upButtonPressed == 1) {
			upButtonPressed = 0;
			output_branch_state = 9;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (rightButtonPressed == 1) {
			rightButtonPressed = 0;
			lcd_put_cur(1, 0);
			if ((offset1 + 100) > 2200) {
				offset1 = (offset1 + 100) - 2200;
			} else {
				offset1 = offset1 + 100;
			}
			char send_string[] = "XXXXmV";
			send_string[0] = (offset1 / 1000) + 48;
			send_string[1] = ((offset1 % 1000) / 100) + 48;
			send_string[2] = ((offset1 % 100) / 10) + 48;
			send_string[3] = (offset1 % 10) + 48;
			lcd_send_string(send_string);
		} else if (buttonPressed == 1) {
			buttonPressed = 0;
			if (signal_type_DAC[0] == 'd')
			{
				DC_offset_DAC = offset1;
			}
			else
			{
				S_offset_DAC = offset1;
			}

			lcd_put_cur(1, 8);
			lcd_send_string("SET");
		}

	}
}

void branchesElevenToFourteen()
{
	if (output_branch_state == 11) {
		if (output_state_has_displayed == 0) {
			lcd_send_string("FREQUENCY");
			output_state_has_displayed = 1;
		}

		if (downButtonPressed == 1) {
			downButtonPressed = 0;
			output_branch_state = 12;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (upButtonPressed == 1) {
			upButtonPressed = 0;
			output_branch_state = 6;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (rightButtonPressed == 1) {
			rightButtonPressed = 0;
			output_branch_state = 13;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (leftButtonPressed == 1) {
			leftButtonPressed = 0;
			output_branch_state = 9;
			output_state_has_displayed = 0;
			lcd_clear();
		}
	} else if (output_branch_state == 12) {

		if (output_state_has_displayed == 0) {
			lcd_send_string("FREQUENCY");
			lcd_put_cur(1, 0);
			freq1 = frequency_DAC;
			char send_string[] = "XXXXHz";
			send_string[0] = (freq1 / 1000) + 48;
			send_string[1] = ((freq1 % 1000) / 100) + 48;
			send_string[2] = ((freq1 % 100) / 10) + 48;
			send_string[3] = (freq1 % 10) + 48;
			lcd_send_string(send_string);
			output_state_has_displayed = 1;
		}

		if (leftButtonPressed == 1) {
			leftButtonPressed = 0;
			lcd_put_cur(1, 0);
			if ((freq1 - 100) < 0) {
				freq1 = 2200 - (100 - freq1);
			} else {
				freq1 = freq1 - 100;
			}
			char send_string[] = "XXXXHz";
			send_string[0] = (freq1 / 1000) + 48;
			send_string[1] = ((freq1 % 1000) / 100) + 48;
			send_string[2] = ((freq1 % 100) / 10) + 48;
			send_string[3] = (freq1 % 10) + 48;
			lcd_send_string(send_string);
		} else if (upButtonPressed == 1) {
			upButtonPressed = 0;
			output_branch_state = 11;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (rightButtonPressed == 1) {
			rightButtonPressed = 0;
			lcd_put_cur(1, 0);
			if ((freq1 + 100 > 2200)) {
				freq1 = (freq1 + 100) - 2200;
			}
			else
			{
				freq1 = freq+100;
			}
			char send_string[] = "XXXXHz";
			send_string[0] = (freq1 / 1000) + 48;
			send_string[1] = ((freq1 % 1000) / 100) + 48;
			send_string[2] = ((freq1 % 100) / 10) + 48;
			send_string[3] = (freq1 % 10) + 48;
			lcd_send_string(send_string);
		} else if (buttonPressed == 1) {
			buttonPressed = 0;
			frequency_DAC = freq1;
			lcd_put_cur(1, 8);
			lcd_send_string("SET");
		}
	}
	else if (output_branch_state == 13)
	{
		if (output_state_has_displayed == 0) {
					lcd_send_string("DUTY CYCLE");
					output_state_has_displayed = 1;
				}

				if (downButtonPressed == 1) {
					downButtonPressed = 0;
					output_branch_state = 14;
					output_state_has_displayed = 0;
					lcd_clear();
				} else if (upButtonPressed == 1) {
					upButtonPressed = 0;
					output_branch_state = 6;
					output_state_has_displayed = 0;
					lcd_clear();
				} else if (leftButtonPressed == 1) {
					leftButtonPressed = 0;
					output_branch_state = 11;
					output_state_has_displayed = 0;
					lcd_clear();
				}
	}
	else if (output_branch_state == 14)
	{
		if (output_state_has_displayed == 0) {
					lcd_send_string("DUTY CYCLE");
					lcd_put_cur(1, 0);
					dutcyc = duty_cycle_DAC;
					char send_string[] = "XXX%";
					send_string[0] = (dutcyc/100) + 48;
					send_string[1] = ((dutcyc % 100)/10) + 48;
					send_string[2] = (dutcyc % 10) + 48;
					lcd_send_string(send_string);
					output_state_has_displayed = 1;
				}

				if (leftButtonPressed == 1) {
					leftButtonPressed = 0;
					if ((dutcyc - 10) < 0) {
						dutcyc = 100 - (10 - dutcyc);
					} else {
						dutcyc = dutcyc - 10;
					}
					lcd_put_cur(1, 0);
					char send_string[] = "XXX%";
					send_string[0] = (dutcyc/100) + 48;
					send_string[1] = ((dutcyc % 100)/10) + 48;
					send_string[2] = (dutcyc % 10) + 48;
					lcd_send_string(send_string);
				} else if (upButtonPressed == 1) {
					upButtonPressed = 0;
					output_branch_state = 13;
					output_state_has_displayed = 0;
					lcd_clear();
				} else if (rightButtonPressed == 1) {
					rightButtonPressed = 0;
					lcd_put_cur(1, 0);
					if ((dutcyc + 10 > 100)) {
						dutcyc = (dutcyc + 10) - 100;
					}
					else
					{
						dutcyc = dutcyc+10;
					}
					char send_string[] = "XXX%";
					send_string[0] = (dutcyc/100) + 48;
					send_string[1] = ((dutcyc % 100)/10) + 48;
					send_string[2] = (dutcyc % 10) + 48;
					lcd_send_string(send_string);
				} else if (buttonPressed == 1) {
					buttonPressed = 0;
					duty_cycle_DAC = dutcyc;
					lcd_put_cur(6, 0);
					lcd_send_string("SET");
				}
	}
}

void branchesFifteenToSeventeen()
{
	if (output_branch_state == 15)
	{
		if (output_state_has_displayed == 0) {
					lcd_send_string("OUTPUT");
					output_state_has_displayed = 1;
				}
				if (downButtonPressed == 1) {
					downButtonPressed = 0;
					output_branch_state = 16;
					output_state_has_displayed = 0;
					lcd_clear();
				} else if (leftButtonPressed == 1) {
					leftButtonPressed = 0;
					output_branch_state = 6;
					output_state_has_displayed = 0;
					lcd_clear();
				} else if (upButtonPressed == 1) {
					upButtonPressed = 0;
					output_branch_state = 1;
					output_state_has_displayed = 0;
					lcd_clear();
				}
	}
	else if (output_branch_state == 16)
	{
		if (output_state_has_displayed == 0) {
			lcd_send_string("OUTPUT ON");
			if (outputState == 1)
			{
				stopDMA = 0;
				lcd_put_cur(1, 0);
				lcd_send_string("SET");
			}
			output_state_has_displayed = 1;
		}

		if (rightButtonPressed == 1) {
			rightButtonPressed = 0;
			output_branch_state = 17;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (upButtonPressed == 1) {
			upButtonPressed = 0;
			output_branch_state = 15;
			output_state_has_displayed = 0;
			lcd_clear();
		} else if (buttonPressed == 1) {
			buttonPressed = 0;
			outputState = 1;
			stopDMA = 0;
			output_state_has_displayed = 0;
			lcd_clear();
		}
	}
	else if (output_branch_state == 17)
	{
		if (output_state_has_displayed == 0) {
					lcd_send_string("OUTPUT OFF");
					if (outputState == 0)
					{
						lcd_put_cur(1,0);
						lcd_send_string("SET");
					}
					output_state_has_displayed = 1;
				}

				if (leftButtonPressed == 1) {
					leftButtonPressed = 0;
					output_branch_state = 16;
					output_state_has_displayed = 0;
					lcd_clear();
				} else if (upButtonPressed == 1) {
					upButtonPressed = 0;
					output_branch_state = 15;
					output_state_has_displayed = 0;
					lcd_clear();
				} else if (buttonPressed == 1) {
					buttonPressed = 0;
					outputState = 0;
					output_state_has_displayed = 0;
					output_LED_has_displayed = 0;
					lcd_clear();
				}
	}
}

void executeOutputState()
{
	branchesOneToSix();
	branchesSevenToTen();
	branchesElevenToFourteen();
	branchesFifteenToSeventeen();
}

void executeDisplayAndButtons()
{
	if (display_state == 1) {
		if (systemState == 1) {
				executeStartMessage();
		} else if (systemState == 2) {
				executeMeasurementState();
		} else if (systemState == 3) {
				executeOutputState();
		}
	} else if (display_state == 2) {
			executeMeasurementOutput();
		}
	if (outputState == 1)
	{
		if (output_LED_has_displayed == 0)
		{
			output_LED_has_displayed = 1;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
		}
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
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
	ADC_trigger = 0;
	buttonPressed = 0;
	systemState = 1;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Transmit(&huart2, sn, 13, 50);
	HAL_UART_Receive_IT(&huart2, rx, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	HAL_TIM_Base_Start_IT(&htim2);

	HAL_TIM_Base_Start(&htim1);
	lcd_init();
	lcd_send_cmd(0x28);

	/*HAL_TIM_Base_Start(&htim3);
	calculateDCSignalBuffer(1000);
	HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, signal_buffer, 1000, DAC_ALIGN_12B_R);
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	 //Menu and state execution



		//Commands received from TS
			if (receiveCommand == 1) {
		 receiveCommand = 0;
		 determineCommandType();
		 executeCommand();
		 }

			executeDisplayAndButtons();

		//receive and measure the input signal and then store the value measured in the correct variables
		// Output a signal based on certain parameters if output state is on.
		measure_ADC_output_DAC();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T3_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_REMAPTRIGGER_ENABLE(HAL_REMAPTRIGGER_DAC1_TRIG);
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB10 PB14 PB15 
                           PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
