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
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "sw_i2c.h"
#include <stdio.h>
/* ============================ USART1 收发逻辑（自 task1 移植�??? ============================
 * 不定长接�??? + DMA 发�?�接口，数据流：
 *   DMA 收数据到缓冲�??? �??? IDLE 中断收尾（算帧长/入队/切缓冲）
 *   �??? 队列 usart1_receive_data �???"缓冲区编�???" �??? usart1receive 任务取数
 *   �??? 后续处理自行添加
 * 发�?�：usart1send 任务�??? DMA 发�?�（二�?�信号量 uart_tx_sem 防重入，
 *   TxCplt 回调释放�???
 * ========================================================================================= */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {mpu_pry,mpu_xyz} usart_data_type ;

typedef struct 
{
float pitch;
float roll;
float yaw;
} mpu6050_pry;


typedef struct 
{
short gyro[3];
short accel[3];
} mpu6050_xyz;



typedef struct 
{
  usart_data_type type;
union 
{
  mpu6050_xyz xyz;
  mpu6050_pry pry;
}data;

}usart_datatosend;

typedef enum {mpu_mode_set} usart_command_type ;

typedef struct 
{
  usart_command_type type;
  int data[2];
}command;


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
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for mpu6050read */
osThreadId_t mpu6050readHandle;
const osThreadAttr_t mpu6050read_attributes = {
  .name = "mpu6050read",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for usart1receive */
osThreadId_t usart1receiveHandle;
const osThreadAttr_t usart1receive_attributes = {
  .name = "usart1receive",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal4,
};
/* Definitions for usartsend */
osMessageQueueId_t usartsendHandle;
const osMessageQueueAttr_t usartsend_attributes = {
  .name = "usartsend"
};
/* USER CODE BEGIN PV */

/*==================== USART1 DMA 接收缓冲池（�??? task1 移植�??? ====================*/
/* 环形缓冲池：5 个缓冲区轮流�??? DMA 写入。IDLE 中断收到�???帧后切换�???
 * 下一个缓冲区，避�??? DMA 覆盖任务还未读完的数�??? */
uint8_t usart1_receive_pool[5][33]={0};   // 接收缓冲池（每格�???�??? 32 字节 + 1 字节余量�???
uint8_t usart1_receive_len[5]={0};        // 各缓冲区本次实际接收的帧长（字节），与缓冲池编号对应
uint8_t usart1_receive_pool_idx=0;        // 当前 DMA 正在写入的缓冲区编号�???0~4 环形递增�???

/*==================== 队列 / 信号量句柄（�??? main 中创建） ====================*/
/* IDLE 中断 �??? usart1receive：传缓冲区编号（索引而非数据本体�??? */
osMessageQueueId_t usart1_receive_dataHandle;
/* UART 发�?�同步信号量：usart1send 发�?�前 acquire，DMA 发�?�完成中断里 release�???
 * 保证同一时刻只有�???�??? DMA 发�?�在进行，防止下次发送覆盖上次还没发完的数据 */
osSemaphoreId_t uart_tx_sem;


volatile char mpu_usart_sendmode=0;//0原始1欧拉
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
  /* 创建 UART 发�?�完成信号量：max=1, 初�??=1（二值信号量），
   * 用于 usart1send 任务�??? DMA 发�?�完成中断之间的同步 */
  uart_tx_sem = osSemaphoreNew(1, 1, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of usartsend */
  usartsendHandle = osMessageQueueNew (5, sizeof(usart_datatosend), &usartsend_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* IDLE 中断 �??? usart1receive：传缓冲区编号（uint8_t，深�??? 3�???
   * 队列满时非阻�??? put 直接丢帧索引，不阻塞 DMA 接收�??? */
  usart1_receive_dataHandle = osMessageQueueNew(3, sizeof(uint8_t), NULL);
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
  /* ============ USART1 空闲中断 + DMA 不定长接收（�??? task1 移植�??? ============ */
  /* 使能 IDLE 空闲中断：一帧数据发完后总线空闲�???1 字节时间无数据）
   * 触发中断，在 USART1_IRQHandler 里收尾（详见 stm32f1xx_it.c�??? */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  /* 启动 DMA 接收：最多收 32 字节，写入缓冲池当前格，实际帧长�??? IDLE 中断算出 */
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
 * @brief USART1 DMA 发�?�完成回调（中断上下文，�??? task1 移植�???
 *        �???帧数�??? DMA 发完后释放信号量，�?�知 usart1send 可以发起下一次发�???
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    osSemaphoreRelease(uart_tx_sem);   // 释放发�?�权
  }
}

static const struct
{
  const char *name;
  usart_command_type type;
} command_table[] = 
{
  {"mpu_mode_set", mpu_mode_set},
};

int command_parse(const char *str, command *cmd)
{
  char name[32];
  int cnt, i;

  if (str == NULL || cmd == NULL)
    return -1;
  if (str[0] != '/')
    return -1;

  cmd->data[0] = 0;
  cmd->data[1] = 0;

  cnt = sscanf(str, "/%31s %d %d", name, &cmd->data[0], &cmd->data[1]);
  if (cnt < 1) 
    return -1;

  for (i = 0; i < (int)(sizeof(command_table) / sizeof(command_table[0])); i++)
  {
    if (strcmp(name, command_table[i].name) == 0)
    {
      cmd->type = command_table[i].type;
      return 0;
    }
  }
  return -2;
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
  uint8_t tosend[64] = {0}; // 发�?�缓冲：数据来源与内容自行填�???
  uint16_t tosend_len = 0;  // 发�?�长度（字节�???
  usart_datatosend datatosend;
  /* Infinite loop */
  for (;;)
  {
    osMessageQueueGet(usartsendHandle, &datatosend, NULL, HAL_MAX_DELAY);

    if (datatosend.type == mpu_xyz)
      tosend_len = (uint16_t)sprintf((char *)tosend, "G:%d,%d,%d A:%d,%d,%d\n",
                                     datatosend.data.xyz.gyro[0], datatosend.data.xyz.gyro[1], datatosend.data.xyz.gyro[2],
                                     datatosend.data.xyz.accel[0], datatosend.data.xyz.accel[1], datatosend.data.xyz.accel[2]);
    else
      tosend_len = (uint16_t)sprintf((char *)tosend, "P:%.2f R:%.2f Y:%.2f\n",
                                     datatosend.data.pry.pitch, datatosend.data.pry.roll, datatosend.data.pry.yaw);

    osSemaphoreAcquire(uart_tx_sem, HAL_MAX_DELAY);

    HAL_UART_Transmit_DMA(&huart1, tosend, tosend_len);
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
  /* ====== 初始�??? ====== */
  sw_i2c_init();

  struct int_param_s int_param = {0};
  mpu_init(&int_param);
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(200);

  dmp_load_motion_driver_firmware();
  dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL |
                     DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO);
  dmp_set_fifo_rate(200);
  mpu_set_dmp_state(1);

  /* ====== 循环读取 ====== */
  mpu6050_xyz xyz; // 六个原始数据：陀螺仪 + 加�?�度�???
  mpu6050_pry pry; // 解算后的欧拉�???
  usart_datatosend tosend;
  short sensors;
  unsigned char more;
  long quat[4];
  unsigned long timestamp;
  uint32_t tick = osKernelGetTickCount();

  for (;;)
  {
    tick += 5;
    osDelayUntil(tick);

    /* 直接把结构体成员数组喂给 dmp_read_fifo */
    if (dmp_read_fifo(xyz.gyro, xyz.accel, quat, &timestamp, &sensors, &more) == 0)
    {
      /* 四元数转欧拉角（度） */
      float q0 = quat[0] / 1073741824.0f; // / 2^30
      float q1 = quat[1] / 1073741824.0f;
      float q2 = quat[2] / 1073741824.0f;
      float q3 = quat[3] / 1073741824.0f;

      /* 原始数据：xyz.gyro[0~2] 角�?�度，xyz.accel[0~2] 加�?�度 */
      /* 欧拉角结果写入结构体 */
      pry.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.29578f;
      pry.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.29578f;
      pry.yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.29578f;
      if (mpu_usart_sendmode == 0)
      {
        tosend.type = mpu_xyz;
        tosend.data.xyz = xyz;
      }
      else
      {
        tosend.type = mpu_pry;
        tosend.data.pry = pry;
      }
      osMessageQueuePut(usartsendHandle, &tosend, 0, 0);
      /* TODO: �??? xyz / pry 通过 USART1 或其他方式输�??? */
    }
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
  uint8_t *receive;   
  uint8_t receive_idx; 
  uint8_t receive_len;
  char cmdbuf[34];
  command cmd;    

  for (;;)
  {

    osMessageQueueGet(usart1_receive_dataHandle, &receive_idx, NULL, HAL_MAX_DELAY);
    receive = usart1_receive_pool[receive_idx];
    receive_len = usart1_receive_len[receive_idx];

    if (receive_len > 32)
      receive_len = 32;
    memcpy(cmdbuf, receive, receive_len);
    cmdbuf[receive_len] = '\0';

    if (command_parse(cmdbuf, &cmd) == 0)
    {
      switch (cmd.type)
      {
      case mpu_mode_set:

        mpu_usart_sendmode = (char)cmd.data[0];
        break;
      default:
        break;
      }
    }
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
