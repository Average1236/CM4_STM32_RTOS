#include <can.h>
#include "z_can.hpp"

#include <cmsis_os.h>

#include "freertos_vars.h"
// #include "utils.hpp"

#include "board.h"


bool ZCAN::apply_config() {
    config_.parent = this;
    set_baud_rate(config_.baud_rate);
    return true;
}

bool ZCAN::reinit() {
    HAL_CAN_Stop(handle_);
    HAL_CAN_ResetError(handle_);
    const uint32_t rx_pending_it = (handle_->Instance == CAN1)
                                 ? CAN_IT_RX_FIFO0_MSG_PENDING
                                 : CAN_IT_RX_FIFO1_MSG_PENDING;
    return (HAL_CAN_Init(handle_) == HAL_OK)
        && (HAL_CAN_Start(handle_) == HAL_OK)
        && (HAL_CAN_ActivateNotification(handle_, rx_pending_it) == HAL_OK);
}

bool ZCAN::start(CAN_HandleTypeDef* handle) {
    handle_ = handle;

    #if defined(STM32F405xx)
    start_bank_ = (handle_->Instance == CAN1) ? 0 : kSlaveStartFilterBank;
    #endif

    handle_->Init.Prescaler = CAN_FREQ / config_.baud_rate / (config_.sjw + config_.time_seg1 + config_.time_seg2);
    if (!reinit()) {
        return false;
    }

    return true;
}

// Set one of only a few common baud rates.  CAN doesn't do arbitrary baud rates well due to the time-quanta issue.
// 21 TQ allows for easy sampling at exactly 80% (recommended by Vector Informatik GmbH for high reliability systems)
// Conveniently, the CAN peripheral's 42MHz clock lets us easily create 21TQs for all common baud rates
bool ZCAN::set_baud_rate(uint32_t baud_rate) {
    uint32_t prescaler = CAN_FREQ / baud_rate;
    if (prescaler * baud_rate == CAN_FREQ) {
        // valid baud rate
        config_.baud_rate = baud_rate;
        if (handle_) {
            handle_->Init.Prescaler = prescaler;
            return reinit();
        }
        return true;
    } else {
        // invalid baud rate - ignore
        return false;
    }
}

void ZCAN::process_rx_fifo(uint32_t fifo) {
    while (HAL_CAN_GetRxFifoFillLevel(handle_, fifo)) {
        CAN_RxHeaderTypeDef header;
        can_Message_t rxmsg;
        HAL_CAN_GetRxMessage(handle_, fifo, &header, rxmsg.buf);

        rxmsg.isExt = header.IDE;
        rxmsg.id = rxmsg.isExt ? header.ExtId : header.StdId;  // If it's an extended message, pass the extended ID
        rxmsg.len = header.DLC;
        rxmsg.rtr = header.RTR;

        // TODO: this could be optimized with an ahead-of-time computed
        // index-to-filter map

        size_t fifo0_idx = 0;
        size_t fifo1_idx = 0;

        // Find the triggered subscription item based on header.FilterMatchIndex
        auto it = std::find_if(subscriptions_.begin() + start_bank_, subscriptions_.end(), [&](auto& s) {
            if (s.fifo == kCanFifoNone) return false;
            size_t current_idx = (s.fifo == 0 ? fifo0_idx : fifo1_idx)++;
            return (header.FilterMatchIndex == current_idx) && (s.fifo == fifo);
        });

        if (it == subscriptions_.end()) {
            continue;
        }

        it->callback(it->ctx, rxmsg);
    }
}

// Send a CAN message on the bus
bool ZCAN::send_message(const can_Message_t &txmsg) {
    if (HAL_CAN_GetError(handle_) != HAL_CAN_ERROR_NONE) {
        return false;
    }

    CAN_TxHeaderTypeDef header;
    header.StdId = txmsg.id;
    header.ExtId = txmsg.id;
    header.IDE = txmsg.isExt ? CAN_ID_EXT : CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = txmsg.len;
    header.TransmitGlobalTime = FunctionalState::DISABLE;

    uint32_t retTxMailbox = 0;
    if (!HAL_CAN_GetTxMailboxesFreeLevel(handle_)) {
        return false;
    }
    
    return HAL_CAN_AddTxMessage(handle_, &header, (uint8_t*)txmsg.buf, &retTxMailbox) == HAL_OK;
}

//void ZCAN::set_error(Error error) {
//    error_ |= error;
//}

bool ZCAN::subscribe(const MsgIdFilterSpecs& filter, on_can_message_cb_t callback, void* ctx, CanSubscription** handle) {
    auto it = std::find_if(subscriptions_.begin() + start_bank_, subscriptions_.end(), [](auto& subscription) {
        return subscription.fifo == kCanFifoNone;
    });

    if (it == subscriptions_.end()) {
        return false; // all subscription slots in use
    }

    it->callback = callback;
    it->ctx = ctx;
    it->fifo = filter.fifo; // TODO: make customizable
    if (handle) {
        *handle = &*it;
    }

    bool is_extended = filter.id.index() == 1;
    uint32_t id = is_extended ?
                  ((std::get<1>(filter.id) << 3) | (1 << 2)) :
                  (std::get<0>(filter.id) << 21);
    uint32_t mask = (is_extended ? (filter.mask << 3) : (filter.mask << 21))
                  | (1 << 2); // care about the is_extended bit

    CAN_FilterTypeDef hal_filter = {};
    hal_filter.FilterActivation = ENABLE;
    hal_filter.FilterBank = &*it - &subscriptions_[0];
    hal_filter.FilterFIFOAssignment = it->fifo;
    hal_filter.FilterIdHigh = (id >> 16) & 0xffff;
    hal_filter.FilterIdLow = id & 0xffff;
    hal_filter.FilterMaskIdHigh = (mask >> 16) & 0xffff;
    hal_filter.FilterMaskIdLow = mask & 0xffff;
    hal_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    hal_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    hal_filter.SlaveStartFilterBank = kSlaveStartFilterBank;

    if (HAL_CAN_ConfigFilter(handle_, &hal_filter) != HAL_OK) {
        return false;
    }
    return true;
}

bool ZCAN::unsubscribe(CanSubscription* handle) {
    ZCANSubscription* subscription = static_cast<ZCANSubscription*>(handle);
    if (subscription < subscriptions_.begin() || subscription >= subscriptions_.end()) {
        return false;
    }
    if (subscription->fifo == kCanFifoNone) {
        return false; // not in use
    }

    subscription->fifo = kCanFifoNone;

    CAN_FilterTypeDef hal_filter = {};
    hal_filter.FilterActivation = DISABLE;
    hal_filter.FilterBank = subscription - &subscriptions_[0];
    hal_filter.SlaveStartFilterBank = kSlaveStartFilterBank;
    return HAL_CAN_ConfigFilter(handle_, &hal_filter) == HAL_OK;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) {}
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {}
// void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {}
// void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {}
// void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan) {}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    //HAL_CAN_ResetError(hcan);
}
