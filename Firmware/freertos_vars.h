#ifndef __FREERTOS_VARS_H
#define __FREERTOS_VARS_H

#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>

// Message Queues
extern osMessageQueueId_t q_can1_rxHandle;
extern osMessageQueueId_t q_optflow_dataHandle;
extern osMessageQueueId_t q_motor_fbHandle;
extern osMessageQueueId_t q_imu_dataHandle;

// Semaphores
extern osSemaphoreId_t sem_can_txHandle;
extern osSemaphoreId_t sem_ctrl_triggerHandle;
extern osSemaphoreId_t sem_imu_readyHandle;
extern osSemaphoreId_t sem_spi_triggerHandle;

// Mutexes
extern osMutexId_t mtx_robot_stateHandle;
extern osMutexId_t mtx_spi_bufferHandle;

// OptFlow health status
extern volatile bool g_optflow_available;
extern volatile uint32_t g_optflow_last_update_ms;

#endif // __FREERTOS_VARS_H