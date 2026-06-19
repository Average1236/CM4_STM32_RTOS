#ifndef __Z_CAN_HPP
#define __Z_CAN_HPP

#include <cmsis_os.h>

#include "canbus.hpp"
#include "can_simple.hpp"
#include <interfaces.hpp>

#include <array>

#define CAN_CLK_HZ (42000000)
#define CAN_CLK_MHZ (42)

// Anonymous enum for defining the most common CAN baud rates
enum {
    CAN_BAUD_125K = 125000,
    CAN_BAUD_250K = 250000,
    CAN_BAUD_500K = 500000,
    CAN_BAUD_1000K = 1000000,
    CAN_BAUD_1M = 1000000
};

class ZCAN : public CanBusBase, public RobotIntf::CanIntf {
public:
    struct Config_t {
        uint32_t baud_rate = CAN_BAUD_1M;
        uint32_t time_seg1 = 9;  // in time quanta
        uint32_t time_seg2 = 4;  // in time quanta
        uint32_t sjw = 1;        // in time quanta
        Protocol protocol = PROTOCOL_SIMPLE;

        ZCAN* parent = nullptr; // set in apply_config()
        void set_baud_rate(uint32_t value) { parent->set_baud_rate(value); }
    };

    ZCAN() {}

    bool apply_config();
    bool start(CAN_HandleTypeDef* handle);

    bool send_message(const can_Message_t& message) final;
    bool subscribe(const MsgIdFilterSpecs& filter, on_can_message_cb_t callback, void* ctx, CanSubscription** handle) final;
    bool unsubscribe(CanSubscription* handle) final;
    void process_rx_fifo(uint32_t fifo);
    bool set_baud_rate(uint32_t baud_rate);

    Error error_ = ERROR_NONE;

    Config_t config_;
    CANSimple can_simple_{this};

    osThreadId thread_id_;
    const uint32_t stack_size_ = 1024;  // Bytes

private:
    static const uint8_t kCanFifoNone = 0xff;
    static constexpr uint32_t kSlaveStartFilterBank = 14;

    struct ZCANSubscription : CanSubscription {
        uint8_t fifo = kCanFifoNone;
        on_can_message_cb_t callback;
        void* ctx;
    };

    bool reinit();
    
    // Hardware supports at most 28 filters unless we do optimizations. For now
    // we don't need that many.
    std::array<ZCANSubscription, 28> subscriptions_;
    // Filter banks are shared between CAN1 and CAN2 on STM32F4xx. We allocate
    // half the banks to each CAN peripheral.
    uint32_t start_bank_ = 0;
    CAN_HandleTypeDef *handle_ = nullptr;
};

#endif  // __ODRIVE_CAN_HPP
