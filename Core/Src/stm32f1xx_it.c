/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN EV */
/* USART1 鎺ユ敹缂撳啿姹犱笌闃熷垪鍙ユ焺锛堣嚜 task1 绉绘锛屽畾涔夊�? main.c�?? */
extern uint8_t usart1_receive_pool[5][33];
extern uint8_t usart1_receive_len[5];
extern uint8_t usart1_receive_pool_idx;
extern osMessageQueueId_t usart1_receive_dataHandle;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  /* ============ USART1 绌洪棽涓柇�?跺抚锛堜笉�?�氶暱鎺ユ敹鏍稿績閫昏緫锛岃�? task1 绉绘锛? ============
   * 鍘熺悊锛欴MA 寰幆鎶婃敹鍒扮殑�?�楄妭鍐欏叆缂撳啿姹犲綋鍓嶆牸锛涗竴甯ф暟鎹彂�?�屽悗鎬荤嚎
   * 绌洪棽瑙�?�? IDLE 涓柇锛屾澶勬敹灏撅細绠楀嚭�?�為檯甯ч�? �?? 閫氱煡浠诲姟 �?? 鍒囨�?
   * 鍒颁笅涓?鏍肩紦鍐查噸�?? DMA�??
   */
  uint8_t len_e;
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);            // 娓呴�? IDLE 鏍囧�?
    HAL_DMA_Abort(&hdma_usart1_rx);                     // 鍋滄鏈 DMA 浼犺�?
    /* DMA 鍓╀綑璁℃暟 = 32 - 瀹為檯鏀跺埌瀛楄妭鏁帮紝鍙嶆帹鍑烘湰甯ч暱�?? */
    len_e = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    usart1_receive_len[usart1_receive_pool_idx] = 32 - len_e;

    /* �??"缂撳啿鍖虹紪�??"锛堜笉鏄暟鎹湰浣擄級鍏ラ槦锛屼换鍔�?�嚟缂栧彿鍘荤紦鍐叉睜鍙栨暟鎹�?
     * 闈為樆濉? put锛氶槦鍒楁弧锛堣繑鍥? osErrorResource锛夋椂涓㈠純鏈抚绱㈠紩 */
    if(osMessageQueuePut(usart1_receive_dataHandle, &usart1_receive_pool_idx, NULL, 0)!=osErrorResource)
    {

    }
    /* 鏃犺鍏ラ槦鏄惁鎴愬姛閮藉垏鎹㈠埌涓嬩竴鏍硷細闃熷垪婊℃椂鏂版暟鎹鐩栨棫鏍硷紝
     * 涓嶉樆濉? DMA 鎺ユ敹锛堜涪鏂颁繚鏃х瓥鐣ョ殑宸茬煡鍙栬垗�?? */
    usart1_receive_pool_idx = (usart1_receive_pool_idx + 1) % 5;
    /* 鐢ㄦ柊鐨勭紦鍐插尯閲嶅惎 DMA 鎺ユ敹锛�?瓑涓嬩竴�?? */
    huart1.RxState = HAL_UART_STATE_READY;  /* HAL_DMA_Abort does not reset RxState; reset manually before restart */
    HAL_UART_Receive_DMA(&huart1, usart1_receive_pool[usart1_receive_pool_idx], 32);
  }

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
