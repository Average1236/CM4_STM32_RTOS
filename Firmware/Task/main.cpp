#include "z_main.h"
#include "iwdg.h"
#include "usart.h"

#if defined(STM32F405xx)
// Place FreeRTOS heap in core coupled memory for better performance
__attribute__((section(".ccmram")))
#endif
uint8_t ucHeap[configTOTAL_HEAP_SIZE];

extern "C" void MX_FREERTOS_Init(void); // Defined in freertos.c

Robot robot{};

ZCAN can1_bus;
ZCAN can2_bus;
IMU imu(IMU::Model::kIcm42688, &hspi2, &huart4, GPIOB, GPIO_PIN_9);
OptFlow opt_flow;

extern "C" int main(void) {
    // System initialization code...
    system_init();

    if (!board_init()) {
        // Handle board initialization failure
        while (1) {}
    }

    
    // Start FreeRTOS scheduler
    osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
    MX_FREERTOS_Init();
    osKernelStart();
    
    // Should never reach here
    while (1) {}
}