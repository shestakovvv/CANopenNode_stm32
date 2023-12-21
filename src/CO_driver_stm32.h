//
// Created by shestakovvv on 13.12.2023.
//

#ifndef TEST_CAN_CO_DRIVER_STM32_H
#define TEST_CAN_CO_DRIVER_STM32_H

#include "main.h"

/* (un)lock critical section in CO_CANsend() */
// Why disabling the whole Interrupt
#define CO_LOCK_CAN_SEND(CAN_MODULE)                                                                                   \
    do {                                                                                                               \
        (CAN_MODULE)->primask_send = __get_PRIMASK();                                                                  \
        __disable_irq();                                                                                               \
    } while (0)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE) __set_PRIMASK((CAN_MODULE)->primask_send)

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE)                                                                                       \
    do {                                                                                                               \
        (CAN_MODULE)->primask_emcy = __get_PRIMASK();                                                                  \
        __disable_irq();                                                                                               \
    } while (0)
#define CO_UNLOCK_EMCY(CAN_MODULE) __set_PRIMASK((CAN_MODULE)->primask_emcy)

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE)                                                                                         \
    do {                                                                                                               \
        (CAN_MODULE)->primask_od = __get_PRIMASK();                                                                    \
        __disable_irq();                                                                                               \
    } while (0)
#define CO_UNLOCK_OD(CAN_MODULE) __set_PRIMASK((CAN_MODULE)->primask_od)

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)                                                                                             \
    do {                                                                                                               \
        CO_MemoryBarrier();                                                                                            \
        rxNew = (void*)1L;                                                                                             \
    } while (0)
#define CO_FLAG_CLEAR(rxNew)                                                                                           \
    do {                                                                                                               \
        CO_MemoryBarrier();                                                                                            \
        rxNew = NULL;                                                                                                  \
    } while (0)

#endif //TEST_CAN_CO_DRIVER_STM32_H
