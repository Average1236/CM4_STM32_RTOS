#include "cmsis_os.h"
#include "freertos_vars.h"
#include "z_main.h"
#include "spi.h"
#include "main.h"
#include <cstring>

volatile uint32_t spi_trigger_dt_us = 0;
volatile uint32_t spi_trigger_last_cnt = 0;
volatile uint32_t spi_trigger_count = 0;
volatile uint32_t spi_timeout_count = 0;
volatile uint32_t spi_error_irq_count = 0;
volatile uint32_t spi_dma_start_fail_count = 0;
volatile uint32_t spi_recover_count = 0;
volatile uint32_t spi_mutex_skip_count = 0;
volatile uint32_t spi_last_hal_status = HAL_OK;
volatile uint32_t spi_last_error = HAL_SPI_ERROR_NONE;
volatile uint32_t spi_last_sr = 0;
volatile uint8_t spi_recover_requested = 0;

namespace {

constexpr uint32_t kSpiWaitTimeoutMs = 20;

void record_spi_status(HAL_StatusTypeDef status) {
    spi_last_hal_status = static_cast<uint32_t>(status);
    spi_last_error = HAL_SPI_GetError(&hspi1);
    spi_last_sr = hspi1.Instance->SR;
}

void clear_spi1_error_flags() {
    if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_OVR)) {
        __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    }
    if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_MODF)) {
        __HAL_SPI_CLEAR_MODFFLAG(&hspi1);
    }
    if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_FRE)) {
        __HAL_SPI_CLEAR_FREFLAG(&hspi1);
    }
}

void recover_spi1_dma(bool reinit_peripheral) {
    spi_recover_count++;

    (void)HAL_SPI_DMAStop(&hspi1);
    (void)HAL_SPI_Abort(&hspi1);
    clear_spi1_error_flags();

    if (reinit_peripheral) {
        __HAL_RCC_SPI1_FORCE_RESET();
        __HAL_RCC_SPI1_RELEASE_RESET();
        MX_SPI1_Init();
        clear_spi1_error_flags();
    }

    record_spi_status(HAL_OK);
}

bool start_spi1_dma_exchange(bool recover_first) {
    if (recover_first) {
        recover_spi1_dma(true);
    }

    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
            &hspi1, robot.spi_tx_data, robot.spi_rx_data, SPI_LENGTH);
        record_spi_status(status);
        if (status == HAL_OK) {
            return true;
        }

        spi_dma_start_fail_count++;
        recover_spi1_dma(attempt >= 1);
    }

    return false;
}

void update_spi_trigger_timing() {
    uint32_t current_cnt = TIM13->CNT;
    spi_trigger_dt_us = (current_cnt >= spi_trigger_last_cnt)
                        ? (current_cnt - spi_trigger_last_cnt)
                        : (current_cnt + ((1u << 16) - spi_trigger_last_cnt) + 1u);
    spi_trigger_last_cnt = current_cnt;
    spi_trigger_count++;
}

void process_completed_spi_frame() {
    if (osMutexAcquire(mtx_spi_bufferHandle, 10) != osOK) {
        spi_mutex_skip_count++;
        return;
    }

    if (osMutexAcquire(mtx_robot_stateHandle, 10) == osOK) {
        robot.pi_encode_spi();
        robot.pi_decode_spi();
        robot.watchdog_feed();
        osMutexRelease(mtx_robot_stateHandle);
    } else {
        spi_mutex_skip_count++;
    }

    osMutexRelease(mtx_spi_bufferHandle);
}

} // namespace

extern "C" {

// SpiExchangeTask - Handle SPI communication with CM4
void StartSpiExchangeTask(void *argument) {
    osDelay(100);  // Wait for initialization
    spi_trigger_last_cnt = TIM13->CNT;

    // Start first SPI transfer, then run as interrupt-triggered loop.
    start_spi1_dma_exchange(true);

    for(;;) {
        const osStatus_t wait_status = osSemaphoreAcquire(sem_spi_triggerHandle, kSpiWaitTimeoutMs);
        bool recover_before_restart = false;

        if (wait_status == osOK) {
            if (spi_recover_requested != 0) {
                spi_recover_requested = 0;
                recover_before_restart = true;
            } else {
                update_spi_trigger_timing();
                process_completed_spi_frame();
            }
        } else if (wait_status == osErrorTimeout) {
            spi_timeout_count++;
            recover_before_restart = true;
        } else {
            recover_before_restart = true;
        }

        start_spi1_dma_exchange(recover_before_restart);
    }
}

} // extern "C"
