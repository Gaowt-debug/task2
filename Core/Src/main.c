/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

/* ============================ USART1 收发逻辑（自 task1 移植） ============================
 * 不定长接收 + 回显发送，数据流：
 *   DMA 收数据到缓冲池 → IDLE 中断收尾（算帧长/入队/切缓冲）
 *   → 队列 usart1_receive_data 传"缓冲区编号" → usart1receive 任务拼帧
 *   → 队列 usart1_send_frame 传整帧 → usart1send 任务 DMA 回发
 *   （发送用二值信号量 uart_tx_sem 防重入，TxCplt 回调释放）
 * 与 task1 的差异：task1 由 Task3 一个任务做完"取帧+发送"，
 * task2 拆分为 usart1receive（取帧拼包）与 usart1send（DMA 发送）两个任务。
 * ========================================================================================= */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 待发送整帧：usart1receive 任务拼好 → 队列 → usart1send 任务发出 */
typedef struct
{
	uint16_t len;        // 帧总长度（报头 + 数据 + 换行）
	uint8_t  data[47];   // 13 字节 "Receive Data:" + 最多 32 字节数据 + "\n" + 余量
} usart1_tx_frame;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for usart1send */
osThreadId_t usart1sendHandle;
const osThreadAttr_t usart1send_attributes = {
  .name = "usart1send",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for mpu6050read */
osThreadId_t mpu6050readHandle;
const osThreadAttr_t mpu6050read_attributes = {
  .name = "mpu6050read",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for usart1receive */
osThreadId_t usart1receiveHandle;
const osThreadAttr_t usart1receive_attributes = {
  .name = "usart1receive",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/*==================== USART1 DMA 接收缓冲池（自 task1 移植） ====================*/
/* 环形缓冲池：5 个缓冲区轮流给 DMA 写入。IDLE 中断收到一帧后切换到
 * 下一个缓冲区，避免 DMA 覆盖任务还未读完的数据 */
uint8_t usart1_receive_pool[5][33]={0};   // 接收缓冲池（每格最大 32 字节 + 1 字节余量）
uint8_t usart1_receive_len[5]={0};        // 各缓冲区本次实际接收的帧长（字节），与缓冲池编号对应
                                        // 注：task1 原代码为 [3]，池却有 5 格，编号 3/4 时越界写——移植时已修正为 [5]
uint8_t usart1_receive_pool_idx=0;        // 当前 DMA 正在写入的缓冲区编号（0~4 环形递增）

/*==================== 队列 / 信号量句柄（在 main 中创建） ====================*/
/* IDLE 中断 → usart1receive：传缓冲区编号（索引而非数据本体） */
osMessageQueueId_t usart1_receive_dataHandle;
/* usart1receive → usart1send：传拼装好的整帧（按值拷贝） */
osMessageQueueId_t usart1_send_frameHandle;
/* UART 发送同步信号量：usart1send 发送前 acquire，DMA 发送完成中断里 release，
 * 保证同一时刻只有一次 DMA 发送在进行，防止下次发送覆盖上次还没发完的数据 */
osSemaphoreId_t uart_tx_sem;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
void Startusart1sendTask(void *argument);
void Startmpu6050readTask(void *argument);
void Startusart1receiveTask(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* 创建 UART 发送完成信号量：max=1, 初值=1（二值信号量），
   * 用于 usart1send 任务与 DMA 发送完成中断之间的同步 */
  uart_tx_sem = osSemaphoreNew(1, 1, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* IDLE 中断 → usart1receive：传缓冲区编号（uint8_t，深度 3；
   * 队列满时非阻塞 put 直接丢帧索引，不阻塞 DMA 接收） */
  usart1_receive_dataHandle = osMessageQueueNew(3, sizeof(uint8_t), NULL);
  /* usart1receive → usart1send：传整帧 usart1_tx_frame（深度 3） */
  usart1_send_frameHandle = osMessageQueueNew(3, sizeof(usart1_tx_frame), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of usart1send */
  usart1sendHandle = osThreadNew(Startusart1sendTask, NULL, &usart1send_attributes);

  /* creation of mpu6050read */
  mpu6050readHandle = osThreadNew(Startmpu6050readTask, NULL, &mpu6050read_attributes);

  /* creation of usart1receive */
  usart1receiveHandle = osThreadNew(Startusart1receiveTask, NULL, &usart1receive_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  /* ============ USART1 空闲中断 + DMA 不定长接收（自 task1 移植） ============ */
  /* 使能 IDLE 空闲中断：一帧数据发完后总线空闲（1 字节时间无数据）
   * 触发中断，在 USART1_IRQHandler 里收尾（详见 stm32f1xx_it.c） */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  /* 启动 DMA 接收：最多收 32 字节，写入缓冲池当前格，实际帧长由 IDLE 中断算出 */
  HAL_UART_Receive_DMA(&huart1, usart1_receive_pool[usart1_receive_pool_idx], 32);
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief USART1 DMA 发送完成回调（中断上下文，自 task1 移植）
 *        一帧数据 DMA 发完后释放信号量，通知 usart1send 可以发起下一次发送
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    osSemaphoreRelease(uart_tx_sem);   // 释放发送权
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Startusart1sendTask */
/**
  * @brief  Function implementing the usart1sendTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Startusart1sendTask */
void Startusart1sendTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  usart1_tx_frame frame = {0};   // 从发送队列取出的整帧
  /* Infinite loop */
  for(;;)
  {
    /* 阻塞等待 usart1receive 拼装好的整帧（队列空时挂起，不占 CPU） */
    osMessageQueueGet(usart1_send_frameHandle, &frame, NULL, HAL_MAX_DELAY);

    /* 拿到发送权才能发（DMA 忙则阻塞），发完由 TxCplt 回调释放信号量 */
    osSemaphoreAcquire(uart_tx_sem, HAL_MAX_DELAY);
    HAL_UART_Transmit_DMA(&huart1, frame.data, frame.len);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Startmpu6050readTask */
/**
* @brief Function implementing the mpu6050readTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startmpu6050readTask */
void Startmpu6050readTask(void *argument)
{
  /* USER CODE BEGIN Startmpu6050readTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Startmpu6050readTask */
}

/* USER CODE BEGIN Header_Startusart1receiveTask */
/**
* @brief Function implementing the usart1receive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startusart1receiveTask */
void Startusart1receiveTask(void *argument)
{
  /* USER CODE BEGIN Startusart1receiveTask */
  usart1_tx_frame frame = {0};  // 拼装中的发送帧
  uint8_t *receive;             // 指向缓冲池中本次收到的数据
  uint8_t receive_idx;          // IDLE 中断传来的缓冲区编号
  uint8_t receive_len;          // 本帧实际长度
  uint16_t offset;              // memcpy 拼接游标：当前写入位置
  /* Infinite loop */
  for(;;)
  {
    /* 阻塞等待 IDLE 中断投递的缓冲区编号（队列空时挂起） */
    osMessageQueueGet(usart1_receive_dataHandle, &receive_idx, NULL, HAL_MAX_DELAY);

    /* 清空发送帧，防止上次残留 */
    memset(&frame, 0, sizeof(frame));

    /* 按编号从缓冲池取出数据和帧长 */
    receive = usart1_receive_pool[receive_idx];
    receive_len = usart1_receive_len[receive_idx];

    /* 用 memcpy+offset 拼接（不用 strcat：数据里可能有 0x00）：
     * "Receive Data:" + 原始数据 + "\n" */
    offset = 0;
    memcpy(frame.data + offset, "Receive Data:", 13);
    offset += 13;
    memcpy(frame.data + offset, receive, receive_len);
    offset += receive_len;
    memcpy(frame.data + offset, "\n", 1);
    offset += 1;
    frame.len = offset;

    /* 整帧入队交给 usart1send 发送（非阻塞：队列满则丢帧，不阻塞接收链路） */
    osMessageQueuePut(usart1_send_frameHandle, &frame, 0, 0);
  }
  /* USER CODE END Startusart1receiveTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
