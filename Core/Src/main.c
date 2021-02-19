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
#include "i2c.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

static void GPIO_Init(void);
static void TIM_Init(void);
static void DMA_Init(void);
static void ADC_Init(void);
static void PGA_Init(void);
static void DAC_Init(void);
static void Calibrate_Init(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

__IO uint16_t ADC2ConvertedVault[64];				//定义储存数组
uint32_t	OverSampling_15bit;								//15位过采样值
uint32_t 	OverSampling_20bit;								//20位过采样值
uint16_t	Avg_H_Vault;											//90%样本平均值
uint16_t	Avg_L_Vault;											//0%样本平均值
uint16_t	Sum_H_Vault;											//90%样本和值
uint16_t	Sum_L_Vault;											//0%样本和值
uint16_t	Actual_Gain;											//实际增益值
uint16_t	Actual_Offset;										//实际截止值
uint16_t	H_Vault[20];											//90%样本存放数组
uint16_t	L_Vault[20];											//0%样本存放数组

int TimeBase = 0;														//秒级时基

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
	
	/* USER CODE BEGIN 2 */
	GPIO_Init();
	TIM_Init();
	DMA_Init();
	ADC_Init();
	PGA_Init();
	DAC_Init();
	I2C_Init();
	Calibrate_Init();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */


static void GPIO_Init(void)
{
	RCC->AHBENR |= 1<<17;				//GPIOA 时钟使能
	GPIOA->MODER |= 1<<8;				//PA4 输出模式
	GPIOA->MODER |= 3<<12;			//PA6	模拟模式
	GPIOA->MODER |= 3<<14;			//PA7 模拟模式
	GPIOA->AFR[0] |= 13<<24;			//PA6 复用使能
}

static void DMA_Init(void)
{
	RCC->AHBENR |= 1<<0; 									//DMA1 时钟使能
	DMA1_Channel2->CPAR = (uint32_t)&ADC2->DR;							//设置外设地址
	DMA1_Channel2->CMAR = (uint32_t)&ADC2ConvertedVault;		//设置内存地址
	DMA1_Channel2->CNDTR = 64;						//每周期传输数据量
	DMA1_Channel2->CCR &= 0<<4;						//设置传输方向
	DMA1_Channel2->CCR |= 1<<5;						//开启循环模式
	DMA1_Channel2->CCR |= 1<<7;						//内存递增模式
	DMA1_Channel2->CCR |= 1<<8;						//外设数据16位
	DMA1_Channel2->CCR |= 1<<10;					//DMA 存储数据16位
	DMA1_Channel2->CCR |= 3<<12;					//通道设置为最高优先级
	DMA1_Channel2->CCR |= 1<<0;						//DMA1 通道2使能
}

static void ADC_Init(void)
{
	
	RCC->AHBENR |= 1<<28; 			//ADC2 时钟使能
	RCC->AHBRSTR |= 1<<28;			//ADC2 复位
	RCC->AHBRSTR &= 0<<28;			//ADC2 复位结束
	
	ADC1_2_COMMON->CCR |= 3<<16;//ADC2 时钟设置为 HCLK/4 
	
	ADC2->CFGR |= 1<<1;					//DMA 循环模式
	ADC2->CFGR |= 1<<13;				//ADC2 循环转换模式
	ADC2->SQR1 &= 0<<0;					//ADC2 总通道数设为1
	ADC2->SQR1 |=	3<<6;					//ADC2 通道3使能
	ADC2->CR |= 1<<0;						//ADC2 转换使能
	ADC2->CFGR |= 1<<0;					//DMA 信号功能使能
	ADC2->CR |= 1<<2;						//规则通道转换使能
}

static void TIM_Init(void)
{
	RCC->APB1ENR |= 1<<0;				//TIM2 时钟使能
	TIM2->ARR = 2000-1;					//重装载值
	TIM2->PSC = 36000-1;				//预分频系数
	TIM2->DIER = 1<<0;					//TIM2 允许更新中断使能
	TIM2->CR1 |= 1<<0;					//TIM2 使能	
	NVIC_EnableIRQ(TIM2_IRQn);	//TIM2 中断使能
	
	RCC->APB1ENR |= 1<<1;				//TIM3 时钟使能
	TIM3->ARR = 200-1;					//重装载值
	TIM3->PSC = 72-1;						//预分频系数
	TIM3->DIER = 1<<0;					//TIM3 允许更新中断使能
	TIM3->CR1 |= 1<<0;					//TIM3 使能	
	NVIC_EnableIRQ(TIM3_IRQn);	//TIM3 中断使能
}

void TIM2_IRQHandler(void)
{
	int	mk_20bit;								//20位过采样值生成计数
	uint32_t sum_20bit = 0;			//20位过采样值生成求和
	
	if(TIM2->SR&1)							//判断是否溢出
	{
		TimeBase++;
		if(TimeBase == 5)
		{
			for(mk_20bit=0;mk_20bit<1024;mk_20bit++)
			{
				sum_20bit += OverSampling_15bit;
			}
			OverSampling_20bit = sum_20bit >> 10;
			TimeBase = 0;
		}
	}
	TIM2->SR &= 0<<0;						//清除中断标志
}

static void PGA_Init(void)
{
	RCC->APB2ENR |= 1<<0;				//SYSCFG 时钟使能
	OPAMP2->CSR |= 3<<2;				//PA7 设为OPAMP2非反相输入端
	OPAMP2->CSR |= 3<<5;				//设置为跟随模式
	OPAMP2->CSR |= 1<<0;				//OPAMP2  使能
}

static void DAC_Init(void)
{
	RCC->APB1ENR |= 1<<29;			//DAC1 时钟使能
	DAC1->CR |= 1<<0;						//DAC1 通道1使能
}


void TIM3_IRQHandler(void)
{
	int mk_15bit = 0;						//15位过采样值生成计数
	uint32_t sum_15bit = 0;			//15位过采样值生成求和
	uint32_t	DMA1_ISR_Value = 0;//DMA1->ISR 寄存器状态	
	
	if(TIM3->SR&1)							//判断是否溢出
	{
		ADC2->CR |= 1<<2;					//ADC2 规则通道转换使能
		
		DMA1_ISR_Value = DMA1->ISR;//复制 DMA1->ISR 寄存器值
		if((DMA1_ISR_Value&1<<5) == 1)		//判断DMA是否传输完成
			ADC2->CR |= 1<<4;				//ADC2 规则通道转化停止
		
		for(mk_15bit=0;mk_15bit<64;mk_15bit++)
		{
			sum_15bit += ADC2ConvertedVault[mk_15bit];
		}  

		OverSampling_15bit = sum_15bit >> 6;
		sum_15bit = 0;
		
		if(ADC2->DR>3277)
		{
			if(OPAMP2->CSR == 0x1078404D)			//X4模式
				{
				OPAMP2->CSR = 0x1078004D;				//X2模式
				}
			else
			{
				if(OPAMP2->CSR == 0x1078004D)		//X2模式
				{
					OPAMP2->CSR = 0x1078006D;			//X1模式
				}
			}
		}
			
		if(ADC2->DR<1229)
		{
			if(OPAMP2->CSR == 0x1078006D)			//X1模式
			{
				OPAMP2->CSR = 0x1078004D;				//X2模式
			}				
			else
			{
				if(OPAMP2->CSR == 0x1078004D)		//X2模式
				{
					OPAMP2->CSR = 0x1078404D;			//X4模式
				}
			}
		}
			
		DAC1->DHR12R1 = ADC2->DR;
	}
	
	TIM3->SR &= 0<<0;						//清除中断标志
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
