/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <math.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float coeff;      // Constante précalculée
    float Q1;         // État précédent N-1
    float Q2;         // État précédent N-2
    uint32_t samples; // Compteur pour réinitialiser le filtre
} Goertzel_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265358979f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
#define TARGET_FREQ 25000.0f    // Fréquence cible (25 kHz)
#define SAMPLING_FREQ 160000.0f // Fréquence d'exécution de Goertzel (800kHz / 5)

// --- CONFIGURATION HAUTE DENSITÉ (2 ms à 800 kHz) ---
#define SAMPLES_PER_CH 1600     // 2 ms * 800 000 Hz = 1600 échantillons par canal
#define BUFFER_SIZE (SAMPLES_PER_CH * 3) // 4800 uint32_t (19.2 Ko de RAM)
#define BLOCK_SIZE_INSTANTS (SAMPLES_PER_CH / 2) // Interruption à la moitié (1 ms)

#define GOERTZEL_N 32
#define SEUIL_DETECTION 1000000000.0f // Seuil empirique de détection

uint32_t adc_buffer[BUFFER_SIZE];
Goertzel_t hydrophones[5];
volatile float power_results[5] = {0.0f};

// --- MACHINE À ÉTATS DU SONAR ---
typedef enum {
    MODE_VEILLE,    // Cherche le pinger
    MODE_CAPTURE,   // Pinger trouvé, capture la milliseconde suivante
    MODE_ANALYSE    // DMA stoppé, mémoire figée pour TDOA
} EtatSonar_t;

volatile EtatSonar_t etat_sonar = MODE_VEILLE;
volatile uint8_t pinger_detecte = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void SendValNextion(UART_HandleTypeDef *uart,char *nom,uint32_t val);
void SendTxtNextion(UART_HandleTypeDef *uart,char *nom,char *txt);
void SendBCONextion(UART_HandleTypeDef *uart,char *nom,uint16_t bco);

void Goertzel_Init(Goertzel_t *g, float target_freq, float sampling_freq);
float Goertzel_ProcessSample(Goertzel_t *g, float sample, uint32_t N_reset);
void Process_Audio_Block(uint32_t *buffer, uint16_t block_size);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialisation de l'algorithme pour l'hydrophone 0
  for(int i=0;i<5;i++){
	  Goertzel_Init(&hydrophones[i], TARGET_FREQ, SAMPLING_FREQ);
  }

  // Démarrage matériel
  HAL_ADC_Start(&hadc2);
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc_buffer, BUFFER_SIZE);
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (etat_sonar == MODE_ANALYSE) {
		  // LA MÉMOIRE EST FIGÉE. Les 2ms de signal haute-définition sont dans adc_buffer.
		  // C'est ici que l'on fera le calcul de l'angle TDOA.

		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // Allume la LED

		  // Simulation d'un temps de calcul de 2 secondes
		  HAL_Delay(2000);

		  // Remise à zéro pour la prochaine chasse
		  etat_sonar = MODE_VEILLE;
		  pinger_detecte = 0;
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

		  // Relance l'acquisition matérielle
		  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc_buffer, BUFFER_SIZE);
	  }

	  // Petit délai d'anti-saturation pour la boucle
	  HAL_Delay(10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 80;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 921600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SendValNextion(UART_HandleTypeDef *uart,char *nom,uint32_t val){
	//On attend que la dernière valeur soit envoyée
	while (uart->gState != HAL_UART_STATE_READY);
	int i=0,j=0;
	static char nexbuff[64];
	char charbuff[16];

	//On ajoute le nom
	while(nom[i] != '\0'){nexbuff[i] = nom[i];i++;}

	//On ajoute le .val=
	nexbuff[i]='.';i++;nexbuff[i]='v';i++;nexbuff[i]='a';i++;
	nexbuff[i]='l';i++;nexbuff[i]='=';i++;

	//On converti la valeur en suite de charactères ASCII
	do{charbuff[j] = (val%10)+'0';j++;val = val/10;}while(val>0);

	//On ajoute la valeur
	while((j)>0){nexbuff[i] = charbuff[j-1];i++;j--;}

	//On ajoute les bytes de fin
	nexbuff[i]=0xFF;i++;
	nexbuff[i]=0xFF;i++;
	nexbuff[i]=0xFF;i++;

	//On envoi la valeur à afficher
	HAL_UART_Transmit_DMA(uart,(uint8_t*)nexbuff,i);
}

void SendTxtNextion(UART_HandleTypeDef *uart,char *nom,char *txt){
	//On attend que la dernière valeur soit envoyée
	while (uart->gState != HAL_UART_STATE_READY);
	int i=0,j=0;
	static char nexbuff[64];

	//On ajoute le nom
	while(nom[i] != '\0'){nexbuff[i] = nom[i];i++;}

	//On ajoute le .txt=
	nexbuff[i]='.';i++;nexbuff[i]='t';i++;nexbuff[i]='x';i++;
	nexbuff[i]='t';i++;nexbuff[i]='=';i++;nexbuff[i]='\"';i++;

	//On ajoute le texte
	while(txt[j] != '\0'){nexbuff[i] = txt[j];i++;j++;}
	nexbuff[i]='\"';i++;

	//On ajoute les bytes de fin
	nexbuff[i]=0xFF;i++;
	nexbuff[i]=0xFF;i++;
	nexbuff[i]=0xFF;i++;

	//On envoi le texte à afficher
	HAL_UART_Transmit_DMA(uart,(uint8_t*)nexbuff,i);
}

void SendBCONextion(UART_HandleTypeDef *uart,char *nom,uint16_t bco)
{
	//On attend que la dernière valeur soit envoyée
	while (uart->gState != HAL_UART_STATE_READY);
	int i=0,j=0;
	static char nexbuff[64];
	char charbuff[16];

	//On ajoute le nom
	while(nom[i] != '\0'){nexbuff[i] = nom[i];i++;}

	//On ajoute le .bco=
	nexbuff[i]='.';i++;nexbuff[i]='b';i++;nexbuff[i]='c';i++;
	nexbuff[i]='o';i++;nexbuff[i]='=';i++;

	//On converti la valeur en suite de charactères ASCII
	do{charbuff[j] = (bco%10)+'0';j++;bco = bco/10;}while(bco>0);

	//On ajoute la valeur
	while((j)>0){nexbuff[i] = charbuff[j-1];i++;j--;}

	//On ajoute les bytes de fin
	nexbuff[i]=0xFF;i++;
	nexbuff[i]=0xFF;i++;
	nexbuff[i]=0xFF;i++;

	//On envoi la valeur à afficher
	HAL_UART_Transmit_DMA(uart,(uint8_t*)nexbuff,i);
}

void Goertzel_Init(Goertzel_t *g, float target_freq, float sampling_freq) {
    float omega = (2.0f * PI * target_freq) / sampling_freq;
    g->coeff = 2.0f * cosf(omega);
    g->Q1 = 0.0f;
    g->Q2 = 0.0f;
    g->samples = 0;
}

float Goertzel_ProcessSample(Goertzel_t *g, float sample, uint32_t N_reset) {
    float Q0 = (g->coeff * g->Q1) - g->Q2 + sample;
    g->Q2 = g->Q1;
    g->Q1 = Q0;
    g->samples++;

    float power = (g->Q1 * g->Q1) + (g->Q2 * g->Q2) - (g->coeff * g->Q1 * g->Q2);

    if (g->samples >= N_reset) {
        g->Q1 = 0.0f;
        g->Q2 = 0.0f;
        g->samples = 0;
    }
    return power;
}

void Process_Audio_Block(uint32_t *buffer, uint16_t block_size) {
	// Si on a déjà tout capturé, on ignore le DMA
	if (etat_sonar == MODE_ANALYSE) return;

	// Si on vient de finir la capture d'1ms supplémentaire après la détection
	if (etat_sonar == MODE_CAPTURE) {
		HAL_ADC_Stop_DMA(&hadc1); // GÈLE LA MÉMOIRE IMMÉDIATEMENT
		etat_sonar = MODE_ANALYSE;
		return;
	}

	uint8_t signal_present_dans_ce_bloc = 0;
	static uint8_t decimation_counter = 0;

	for (uint16_t i = 0; i < block_size; i++) {
		// Le DMA stocke à 800 kHz, mais on ne calcule Goertzel qu'une fois sur 5 (160 kHz)
		decimation_counter++;
		if (decimation_counter >= 5) {
			decimation_counter = 0;

			uint32_t rang1 = buffer[i * 3 + 0];
			uint32_t rang2 = buffer[i * 3 + 1];
			uint32_t rang3 = buffer[i * 3 + 2];

			// Pré-casting en float avant l'entrée dans l'algorithme
			float h_samples[5];
			h_samples[0] = (float)(rang1 & 0xFFFF) - 2048.0f;
			h_samples[1] = (float)((rang1 >> 16) & 0xFFFF) - 2048.0f;
			h_samples[2] = (float)(rang2 & 0xFFFF) - 2048.0f;
			h_samples[3] = (float)((rang2 >> 16) & 0xFFFF) - 2048.0f;
			h_samples[4] = (float)(rang3 & 0xFFFF) - 2048.0f;

			for (uint8_t ch = 0; ch < 5; ch++) {
				float current_calc = Goertzel_ProcessSample(&hydrophones[ch], h_samples[ch], GOERTZEL_N);

				if (hydrophones[ch].samples == 0) {
					power_results[ch] = current_calc;
					if (power_results[ch] > SEUIL_DETECTION) {
						signal_present_dans_ce_bloc = 1;
					}
				}
			}
		}
	}

	// Déclencheur du "Piège"
	if (signal_present_dans_ce_bloc == 1 && etat_sonar == MODE_VEILLE) {
		etat_sonar = MODE_CAPTURE; // La prochaine interruption gèlera le tableau
		pinger_detecte = 1;
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc == &hadc1) {
        Process_Audio_Block(&adc_buffer[0], BLOCK_SIZE_INSTANTS);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc == &hadc1) {
        Process_Audio_Block(&adc_buffer[BUFFER_SIZE / 2], BLOCK_SIZE_INSTANTS);
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
#ifdef USE_FULL_ASSERT
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
