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
#include "fonts.h"
#include "image.h"
#include "ssd1306.h"
#include "ssd1306_defines.h"
#include "main.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M_PI		3.14159265358979323846
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// int32_t prevCounter = 0;

// Для движения
float k1 = 1, k2 = 1, k3 = 1, delta; // Коэффициенты k1 - обратная связь по д. холла, k2 - при потери синхронизации, k3 - резерв test
float U_PWM, V_PWM, W_PWM; // ШИМ на обмотках
float target_angle, current_angle; // Угол цели для которой должно быть сделано предсказание, виртуальный угол тяги
float remaining_angle = 0; // Угол, который осталось пройти в 1 режиме
float dimensional_factor = 84; // Фактор размерности нужен для учета особеностей реального двигателя. Например у двигателя в проекте 4 полюса или 2 пары разной полярности, и редуктор на 42
float increment = 0; // Угол на который будет инкрементироваться текущий угол
float rotor_speed; // Скорость ротора в об/сек
float offset = 15; // Сдвиг от виртуального вектора движения current_angle
GPIO_PinState hal_U,hal_V,hal_W; // Переменные для хранения состояния датчиков холла
uint16_t last_pin, mode, stator_state; // Последний сработавший ...
float mode_arr[6] = {75, 135, 195, 255, 315, 15};

// Старый код
float target_frq, current_frq, sum_angle, angle_speed = 60/15, t, k;

// Для датчика тока
uint16_t aRaw[3]; // Массив с напряжениями с датчиков, измеряющих токи на обмотках соответствующих мостов
uint16_t H_I, N_I, V_I; // Токи на обмотках соответствующих мостов
uint8_t DMA_Flag = 0; // Флаг готовности данных
float Sensitivity_I = 0.0066; // (мВ/мА) Для модели ACS712ELS-30A
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */
void change_mode(int set_mode);
void set_RPM(float rotations);
void move_rotor(float to_angle);
void check_sync();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//static UART_HandleTypeDef s_UARTHandle = UART_HandleTypeDef();


#define IDLE   0
#define DONE   1
#define F_CLK  60000000UL

volatile uint8_t gu8_State = IDLE;
volatile uint8_t gu8_MSG[35] = {'\0'};
volatile uint32_t last_tim, cur_tim, gu32_T1 = 0;
volatile uint32_t gu32_T2 = 0;
volatile uint32_t gu32_Ticks = 0;
volatile uint16_t tim_ovc = 0, gu16_TIM13_OVC = 0;
volatile uint32_t gu32_Freq = 0;

char *buffer = "tes";

// Срабатывает, по завершению считывания данных с датчиков тока
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint8_t DMA_Flag = 1;
}

void change_mode(int set_mode) {
	switch (set_mode){
	case 0:
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1); // ШИМ TIM1_CH1N
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);    // ШИМ TIM1_CH1
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2); // ШИМ TIM1_CH2N
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);    // ШИМ TIM1_CH2
		HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1); // ШИМ TIM8_CH1N
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);    // ШИМ TIM8_CH1
		// +Добавить оставшиеся таймеры

		//установка шима в 0
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM8->CCR1 = 0;
	default:
		offset = 15;
		break;
	}
	mode = set_mode;
};

void USART2_IRQHandler()
{
    HAL_UART_IRQHandler(&huart3);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}

char* convertIntegerToChar(int N)
{
    // Count digits in number N
    int m = N;
    int digit = 0;
    while (m) {

        // Increment number of digits
        digit++;

        // Truncate the last
        // digit from the number
        m /= 10;
    }

    // Declare char array for result
    char* arr;

    // Declare duplicate char array
    char arr1[digit];

    // Memory allocation of array
    arr = (char*)malloc(digit);

    // Separating integer into digits and
    // accommodate it to character array
    int index = 0;
    while (N) {

        // Separate last digit from
        // the number and add ASCII
        // value of character '0' is 48
        arr1[++index] = N % 10 + '0';

        // Truncate the last
        // digit from the number
        N /= 10;
    }

    // Reverse the array for result
    int i;
    for (i = 0; i < index; i++) {
        arr[i] = arr1[index - i];
    }

    // Char array truncate by null
    arr[i] = '\0';

    // Return char array
    return (char*)arr;
}

void set_RPM(float rotations) {
	// Расчитываем необходимую скорость движения
	// Требуемуе кол-во градусов в секунду делим на кол-во прирываний таймера 14 в секунду и получаем необходимый инкремент
	increment = rotations * dimensional_factor * 360 / (60000000 / ((TIM14->ARR + 1) * (TIM14->PSC+1)));
	rotor_speed = rotations;
}

void move_rotor(float to_angle) {
	if (increment >= 0) to_angle += offset;
	else to_angle -= offset;

	// Расчет потенциалов и заполнения шима для фаз test
	U_PWM = k1*(sin((to_angle)     * M_PI/180) + sin((to_angle)     * M_PI/60)/4);
	V_PWM = k1*(sin((to_angle+120) * M_PI/180) + sin((to_angle+120) * M_PI/60)/4);
	W_PWM = k1*(sin((to_angle+240) * M_PI/180) + sin((to_angle+240) * M_PI/60)/4);

	// Перенастройка шима на фазах
	if(U_PWM >= 0) {
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		TIM1->CCR1 = U_PWM;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	}
	else {
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		TIM1->CCR1 = -U_PWM;
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	}

	if(V_PWM >= 0) {
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		TIM1->CCR2 = V_PWM;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	}
	else {
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		TIM1->CCR2 = -V_PWM;
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	}

	if(W_PWM >= 0) {
		HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
		TIM8->CCR1 = W_PWM;
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	}
	else {
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
		TIM8->CCR1 = -W_PWM;
		HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
	}

	// Сброс счетчиков таймеров для синхронизации
	TIM1->CNT = 0;
	TIM8->CNT = 0;
	return;
}

// Отвечает за работу таймеров и переключение обмоток
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	// Прерывание отвечающего за движежние таймера
	if (htim == &htim14) {
		//HAL_TIM_PeriodElapsedCallback(0);

		/* 0 - Ожидание
		 * 1 - Плавный разгон
		 * 2 - Поворот на угол
		 * 3 - поддержание скорости
		 * 4 - бесконечный разгон
		 * 5 - динамическое торможение
		 * 6 - Калибровка*/
		switch (mode) {
		case 1:
			if (remaining_angle>increment) current_angle += increment;
			else if (remaining_angle!=0) {
				current_angle += increment-remaining_angle;
				remaining_angle = 0;
				offset = 0;
			}
			current_angle = remainder(current_angle, 360);
			move_rotor(current_angle);
			break;
		case 2:
		case 3:
		case 4:
		case 6:
			current_angle = remainder((current_angle + increment), 360); // Получаем новый угол тяги
			move_rotor(current_angle); // Изменяем угол тяги на новый
			break;
		case 5:
			// Установка максимального ШИМ
			k1 = 2;
			move_rotor(current_angle);
			break;
		default:
			break;
		}
	}

	if (htim == 0)
	{
		//HAL_TIM_Base_Stop_IT(&htim14); //остановка таймера 14

		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1); // ШИМ TIM1_CH1N
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);    // ШИМ TIM1_CH1
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2); // ШИМ TIM1_CH2N
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);    // ШИМ TIM1_CH2
		HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1); // ШИМ TIM8_CH1N
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);    // ШИМ TIM8_CH1

		//установка шима в 0
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM8->CCR1 = 0;

		//обнуление счётчиков таймеров для синхронизации
		TIM1->CNT = 0;
		TIM8->CNT = 0;
		TIM14->CNT = 0;
	}
}

// Функция для замера скорости
void Capture() {
    if(gu8_State == IDLE) {
        gu32_T1 = TIM13->CCR1;
        gu16_TIM13_OVC = 0;
        gu8_State = DONE;
    }
    else if(gu8_State == DONE) {
        gu32_T2 = TIM13->CCR1;
        gu32_Ticks = (gu32_T2 + (gu16_TIM13_OVC * 65536)) - gu32_T1;
        gu32_Freq = (uint32_t)(F_CLK/gu32_Ticks);
        if(gu32_Freq != 0) {
          //sprintf(gu8_MSG, "Frequency = %lu Hz\n\r", gu32_Freq);
          //HAL_UART_Transmit(&huart3, gu8_MSG, sizeof(gu8_MSG), 100);
        	buffer = "";
        	buffer = convertIntegerToChar(gu32_Freq);
        	//char* arr = convertIntegerToChar(N);
        }
        gu8_State = IDLE;
    }
}

void check_sync() {
	delta = 0;
	hal_U = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5);
	hal_V = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
	hal_W = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);

	// СОСТОЯНИЕ 1
	if((hal_U == GPIO_PIN_SET)&&(hal_V == GPIO_PIN_RESET)&&(hal_W == GPIO_PIN_SET)) {
		stator_state = 1;
		if (increment>=0) {
			delta = mode_arr[1] - (current_angle - increment*(TIM14->ARR - TIM14->CNT)/TIM14->ARR); // Вычисляем дельту между
			target_angle = mode_arr[2]; // Следующеее прерывание будет в этом угле

		}
		else {
			target_angle = mode_arr[1];
			delta = (current_angle - increment*(TIM14->ARR - TIM14->CNT)/TIM14->ARR) - mode_arr[2];
		}

		// sum_angle = 75;
	}
	// СОСТОЯНИЕ 2
	if((hal_U == GPIO_PIN_SET)&&(hal_V == GPIO_PIN_RESET)&&(hal_W == GPIO_PIN_RESET)) {
		stator_state = 2;
		if (increment>=0) {
			delta = mode_arr[2] - (current_angle - increment*(TIM14->ARR - TIM14->CNT)/TIM14->ARR); // Вычисляем дельту между
			target_angle = mode_arr[3]; // Следующеее прерывание будет в этом угле

		}
		else {
			target_angle = mode_arr[2];
			delta = (current_angle - increment*(TIM14->ARR - TIM14->CNT)/TIM14->ARR) - mode_arr[3];
		}
		// sum_angle = 135;
	}

	// СОСТОЯНИЕ 3
	if((hal_U == GPIO_PIN_SET)&&(hal_V == GPIO_PIN_SET)&&(hal_W == GPIO_PIN_RESET)) {
		stator_state = 3;
		if (increment>=0) {
			delta = mode_arr[3] - (current_angle - increment*(TIM14->ARR - TIM14->CNT)/TIM14->ARR); // Вычисляем дельту между
			target_angle = mode_arr[4]; // Следующеее прерывание будет в этом угле

		}
		else {
			target_angle = mode_arr[3];
			delta = (current_angle - increment*(TIM14->ARR - TIM14->CNT)/TIM14->ARR) - mode_arr[4];
		}
		// sum_angle = 195;
	}
	// СОСТОЯНИЕ 4
	if((hal_U == GPIO_PIN_RESET)&&(hal_V == GPIO_PIN_SET)&&(hal_W == GPIO_PIN_RESET)) {
		stator_state = 4;
		if (increment>=0) {
			delta = mode_arr[4] - (current_angle - increment*(TIM14->ARR - TIM14->CNT)/TIM14->ARR); // Вычисляем дельту между
			target_angle = mode_arr[5]; // Следующеее прерывание будет в этом угле

		}
		else {
			target_angle = mode_arr[4];
			delta = (current_angle - increment*(TIM14->ARR - TIM14->CNT)/TIM14->ARR) - mode_arr[5];
		}
		// sum_angle = 255;
	}
	// СОСТОЯНИЕ 5
	if((hal_U == GPIO_PIN_RESET)&&(hal_V == GPIO_PIN_SET)&&(hal_W == GPIO_PIN_SET)) {
		stator_state = 5;
		if (increment>=0) {
			delta = mode_arr[5] - (current_angle - increment*(TIM14->ARR - TIM14->CNT)/TIM14->ARR); // Вычисляем дельту между
			target_angle = mode_arr[6]; // Следующеее прерывание будет в этом угле

		}
		else {
			target_angle = mode_arr[5];
			delta = (current_angle - increment*(TIM14->ARR - TIM14->CNT)/TIM14->ARR) - mode_arr[6];
		}
		// sum_angle = 315;
	}
	// СОСТОЯНИЕ 6
	if((hal_U == GPIO_PIN_RESET)&&(hal_V == GPIO_PIN_RESET)&&(hal_W == GPIO_PIN_SET)) {
		stator_state = 6;
		if (increment>=0) {
			delta = mode_arr[6] - (current_angle - increment*(TIM14->ARR - TIM14->CNT)/TIM14->ARR); // Вычисляем дельту между
			target_angle = mode_arr[1]; // Следующеее прерывание будет в этом угле

		}
		else {
			target_angle = mode_arr[6];
			delta = (current_angle - increment*(TIM14->ARR - TIM14->CNT)/TIM14->ARR) - mode_arr[1];
		}
		// sum_angle = 15;
	}
	// Обработчики для аварийных состояний датчика холла
	// АВАРИЙНОЕ СОСТОЯНИЕ 7
	if((hal_U == GPIO_PIN_SET)&&(hal_V == GPIO_PIN_SET)&&(hal_W == GPIO_PIN_SET)) {
		stator_state = 7;
		HAL_TIM_PeriodElapsedCallback(0);
		change_mode(0);
	}
	// АВАРИЙНОЕ СОСТОЯНИЕ 8
	if((hal_U == GPIO_PIN_RESET)&&(hal_V == GPIO_PIN_RESET)&&(hal_W == GPIO_PIN_RESET)) {
		stator_state = 8;
		HAL_TIM_PeriodElapsedCallback(0);
		change_mode(0);
	}

	HAL_TIM_PeriodElapsedCallback(0);
}

// Обработчик прерываний GPIO. Например для прерываний от датчиков холла.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	/* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);
	/* NOTE: This function Should not be modified, when the callback is needed,
	the HAL_GPIO_EXTI_Callback could be implemented in the user file
	*/

	switch(GPIO_Pin) {

	// Обработчик прерываний с датчиков холла
	case GPIO_PIN_5:
	case GPIO_PIN_0:
	case GPIO_PIN_1:
		check_sync();
		break;
	}

	// Проверка состояния кнопки?
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==GPIO_PIN_SET) {
		HAL_TIM_Base_Stop_IT(&htim14);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
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
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM14_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, (uint16_t *) aRaw, 3); // инициализация записи значений в буфер с помощью DMA

  	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_SR_UIF); // очищаем флаг, но зачем?

	// Запускаем двигатель
	change_mode(3); // 3 - поддержание скорости
	set_RPM(2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if (DMA_Flag) { // Когда преобразование закончено
			// Получаем данные с датчиков тока
			H_I = aRaw[1] * Sensitivity_I * 3.3 * (6.8+10)/10 / 4096;
			N_I = aRaw[0] * Sensitivity_I * 3.3 * (6.8+10)/10 / 4096;
			V_I = aRaw[2] * Sensitivity_I * 3.3 * (6.8+10)/10 / 4096;

			if ((H_I + N_I + V_I)>800){ // Если сумма токов больше 800 мА
				change_mode(0);         // - Останавливаем двигатель
			}
			DMA_Flag = 0;
		}


		/* Что-то связаное с энкодером и экраном. Я это не хочу трогать. Я брезгаю
		//HAL_UART_Transmit(&huart3, buffer, sizeof(buffer),15);
		int32_t currCounter = __HAL_TIM_GET_COUNTER(&htim4);
		//int32_t currCounter = TIM4->CNT;
		currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;
		if(currCounter > 32768/2)
		{
			// Преобразуем значения счетчика из:
			//  ... 32766, 32767, 0, 1, 2 ...
			// в значения:
			//  ... -2, -1, 0, 1, 2 ...
			currCounter = currCounter - 32768;
		}
		if(currCounter != prevCounter)
		{
			//int32_t delta = currCounter-prevCounter;
			if(currCounter < prevCounter)
			{
				strr = "<-";
			}else{
				strr = "->";
			}
			//HAL_UART_Transmit(&huart3, strr, sizeof(strr),15);
			ssd1306_Clear();
			ssd1306_SetCursor(20, 20);
			ssd1306_WriteString(strr, Font_16x26);

			prevCounter = currCounter;
			/*
			Capture();
			ssd1306_SetColor(Black);
			ssd1306_FillRect(14,50,7,10);
			ssd1306_SetColor(White);
			ssd1306_SetCursor(0, 50);
			ssd1306_WriteString(buffer, Font_7x10);
			ssd1306_UpdateScreen();
			*/
			//strr = currCounter;
			//ssd1306_WriteString(strr, Font_7x10);
			//ssd1306_UpdateScreen();


		//HAL_UART_Transmit(&huart3, gu32_Freq, sizeof(gu32_Freq),15);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 119;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 49;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 119;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim8.Init.Period = 49;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 59999;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 50;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 59;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 59;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 499;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart3.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PA0 PA1 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
