#ifndef CANOPENSTM32_CO_APP_STM32_H_
#define CANOPENSTM32_CO_APP_STM32_H_

/* CANHandle : Pass in the CAN Handle to this function and it wil be used for
 * all CAN Communications. It can be FDCan or CAN
 * and CANOpenSTM32 Driver will take of care of handling that
 *
 * HWInitFunction : Pass in the function that initialize the CAN peripheral,
 * usually MX_CAN_Init
 *
 * timerHandle : Pass in the timer that is going to be used for generating 1ms
 * interrupt for tmrThread function,
 * please note that CANOpenSTM32 Library will override
 * HAL_TIM_PeriodElapsedCallback function, if you also need this function
 * in your codes, please take required steps
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "CANopen.h"

typedef enum CO_app_Status {
        CO_APP_UNDEFINED,
        CO_APP_OK,
        CO_APP_ERROR,
        CO_APP_ERROR_CAN_NOT_ALLOCATE_MEMORY
} CO_app_Status;

typedef struct {
        uint8_t desiredNodeID;
        uint8_t activeNodeID; /* Assigned Node ID */
        uint16_t baudrate;
        TIM_HandleTypeDef *timerHandle;
#ifdef CO_STM32_FDCAN_Driver
        FDCAN_HandleTypeDef* CANHandle;
#else
        CAN_HandleTypeDef *CANHandle;
#endif

        void (*CANInitFunction)(void);

        CO_t *canOpen_Obj;
        CO_config_t *canOpen_Config;
        uint32_t canOpen_HeapMemoryUsed;
        uint32_t canOpen_PrevProcessTime;
} CANopenNodeHandle;

#ifndef CO_CAN_MAX_RETRY
#define CO_CAN_MAX_RETRY 15000
#endif

/* This function will initialize the required CANOpen Stack objects,
 * allocate the memory and prepare stack for communication reset*/
CO_app_Status CANopenNode_Init(CANopenNodeHandle *hCANopenNode);

/* This function will reset the CAN communication periperhal and
 * also the CANOpen stack variables */
CO_app_Status CANopenNode_ResetCommunication(CANopenNodeHandle *hCANopenHandle);

/* This function will check the input buffers and any outstanding
 * tasks that are not time critical, this function should be called regurarly
 * from your code (i.e from your while(1))*/
void CANopenNode_Process(CANopenNodeHandle *hCANopenHandle);

/* Thread function executes in constant intervals, this function can be called
 * from FreeRTOS tasks or Timers ********/
void CANopenNode_IRQ(CANopenNodeHandle *canopenSTM32);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CANOPENSTM32_CO_APP_STM32_H_ */
