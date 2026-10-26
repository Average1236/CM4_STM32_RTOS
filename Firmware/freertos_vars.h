#ifndef __FREERTOS_VARS_H
#define __FREERTOS_VARS_H

#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>

// Message Queues
extern osMessageQueueId_t q_can1_rxHandle;
extern osMessageQueueId_t q_optflow_dataHandle;
extern osMessageQueueId_t q_motor_fbHandle;

// Semaphores
extern osSemaphoreId_t sem_can_txHandle;
extern osSemaphoreId_t sem_ctrl_triggerHandle;
extern osSemaphoreId_t sem_imu_readyHandle;
extern osSemaphoreId_t sem_spi_triggerHandle;

// Mutexes
extern osMutexId_t mtx_robot_stateHandle;
extern osMutexId_t mtx_spi_bufferHandle;


#endif // __FREERTOS_VARS_H
