/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "freertos_vars.h"
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
/* USER CODE BEGIN Variables */

// Message Queues
osMessageQueueId_t q_can1_rxHandle;
osMessageQueueId_t q_optflow_dataHandle;
osMessageQueueId_t q_motor_fbHandle;
osMessageQueueId_t q_imu_dataHandle;

// Semaphores
osSemaphoreId_t sem_can_txHandle;
osSemaphoreId_t sem_ctrl_triggerHandle;
osSemaphoreId_t sem_imu_readyHandle;
osSemaphoreId_t sem_spi_triggerHandle;

// Mutexes
osMutexId_t mtx_robot_stateHandle;
osMutexId_t mtx_spi_bufferHandle;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OptFlowRxTask */
osThreadId_t OptFlowRxTaskHandle;
const osThreadAttr_t OptFlowRxTask_attributes = {
  .name = "OptFlowRxTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for CrtlTask */
osThreadId_t CrtlTaskHandle;
const osThreadAttr_t CrtlTask_attributes = {
  .name = "CrtlTask",
  .stack_size = 6114 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for MotorRxTask */
osThreadId_t MotorRxTaskHandle;
const osThreadAttr_t MotorRxTask_attributes = {
  .name = "MotorRxTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for ImuRxTask */
osThreadId_t ImuRxTaskHandle;
const osThreadAttr_t ImuRxTask_attributes = {
  .name = "ImuRxTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for SpiExchangeTask */
osThreadId_t SpiExchangeTaskHandle;
const osThreadAttr_t SpiExchangeTask_attributes = {
  .name = "SpiExchangeTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for HealthTask */
osThreadId_t HealthTaskHandle;
const osThreadAttr_t HealthTask_attributes = {
  .name = "HealthTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartOptFlowRxTask(void *argument);
void StartCrtlTask(void *argument);
void StartMotorRxTask(void *argument);
void StartImuRxTask(void *argument);
void StartSpiExchangeTask(void *argument);
void StartHealthTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  mtx_robot_stateHandle = osMutexNew(NULL);
  mtx_spi_bufferHandle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  sem_can_txHandle = osSemaphoreNew(1, 1, NULL);
  sem_ctrl_triggerHandle = osSemaphoreNew(1, 0, NULL);
  sem_imu_readyHandle = osSemaphoreNew(1, 0, NULL);
  sem_spi_triggerHandle = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  q_optflow_dataHandle = osMessageQueueNew(8, 32, NULL);  // 8 snapshots, 32 bytes each (DualOptFlowSnapshot_t)
  q_motor_fbHandle = osMessageQueueNew(25, 16, NULL);     // 25 messages for 5 motors
  q_imu_dataHandle = osMessageQueueNew(8, 36, NULL);      // 8 messages, 36 bytes (9 floats)
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of OptFlowRxTask */
  OptFlowRxTaskHandle = osThreadNew(StartOptFlowRxTask, NULL, &OptFlowRxTask_attributes);

  /* creation of CrtlTask */
  CrtlTaskHandle = osThreadNew(StartCrtlTask, NULL, &CrtlTask_attributes);

  /* creation of MotorRxTask */
  MotorRxTaskHandle = osThreadNew(StartMotorRxTask, NULL, &MotorRxTask_attributes);

  /* creation of ImuRxTask */
  ImuRxTaskHandle = osThreadNew(StartImuRxTask, NULL, &ImuRxTask_attributes);

  /* creation of SpiExchangeTask */
  SpiExchangeTaskHandle = osThreadNew(StartSpiExchangeTask, NULL, &SpiExchangeTask_attributes);

  /* creation of HealthTask */
  HealthTaskHandle = osThreadNew(StartHealthTask, NULL, &HealthTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartOptFlowRxTask */
/**
* @brief Function implementing the OptFlowRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOptFlowRxTask */
__weak void StartOptFlowRxTask(void *argument)
{
  /* USER CODE BEGIN StartOptFlowRxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartOptFlowRxTask */
}

/* USER CODE BEGIN Header_StartCrtlTask */
/**
* @brief Function implementing the CrtlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCrtlTask */
__weak void StartCrtlTask(void *argument)
{
  /* USER CODE BEGIN StartCrtlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCrtlTask */
}

/* USER CODE BEGIN Header_StartMotorRxTask */
/**
* @brief Function implementing the MotorRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorRxTask */
__weak void StartMotorRxTask(void *argument)
{
  /* USER CODE BEGIN StartMotorRxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorRxTask */
}

/* USER CODE BEGIN Header_StartImuRxTask */
/**
* @brief Function implementing the ImuRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuRxTask */
__weak void StartImuRxTask(void *argument)
{
  /* USER CODE BEGIN StartImuRxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartImuRxTask */
}

/* USER CODE BEGIN Header_StartSpiExchangeTask */
/**
* @brief Function implementing the SpiExchangeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSpiExchangeTask */
__weak void StartSpiExchangeTask(void *argument)
{
  /* USER CODE BEGIN StartSpiExchangeTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSpiExchangeTask */
}

/* USER CODE BEGIN Header_StartHealthTask */
/**
* @brief Function implementing the HealthTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHealthTask */
__weak void StartHealthTask(void *argument)
{
  /* USER CODE BEGIN StartHealthTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartHealthTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

