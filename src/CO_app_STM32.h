#ifndef CANOPENSTM32_CO_APP_STM32_H_
#define CANOPENSTM32_CO_APP_STM32_H_

#include "CANopen.h"
#include "main.h"

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

typedef struct {
        /*This is the Node ID that you ask the CANOpen stack to assign
         * to your device, although it might not always
	 * be the final NodeID, after calling canopen_app_init()
         * you should check ActiveNodeID of CANopenNodeSTM32 structure
         * for assigned Node ID.
	 */
    uint8_t desiredNodeID;
    uint8_t activeNodeID; /* Assigned Node ID */
        /* This is the baudrate you've set in your CubeMX Configuration */
    uint8_t baudrate;
        /*Pass in the timer that is going to be used for generating 1ms
         * interrupt for tmrThread function,
         * please note that CANOpenSTM32 Library will override
         * HAL_TIM_PeriodElapsedCallback function, if you also need this
         * function in your codes, please take required steps
         */
    TIM_HandleTypeDef* timerHandle;

    /* Pass in the CAN Handle to this function and it wil be used for all CAN Communications. It can be FDCan or CAN
	 * and CANOpenSTM32 Driver will take of care of handling that*/
#ifdef CO_STM32_FDCAN_Driver
    FDCAN_HandleTypeDef* CANHandle;
#else
    CAN_HandleTypeDef* CANHandle;
#endif

    void (*HWInitFunction)(void); /* Pass in the function that initialize the CAN peripheral, usually MX_CAN_Init */

    CO_t* canOpenStack;
} CANopenNodeHandle;


/* This function will initialize the required CANOpen Stack objects,
 * allocate the memory and prepare stack for communication reset*/
int CANopenNode_Init(CANopenNodeHandle* canopenSTM32);
/* This function will reset the CAN communication periperhal and
 * also the CANOpen stack variables */
int CANopenNode_ResetCommunication(CANopenNodeHandle* canopenSTM32);
/* This function will check the input buffers and any outstanding
 * tasks that are not time critical, this function should be called regurarly
 * from your code (i.e from your while(1))*/
void CANopenNode_Process(CANopenNodeHandle* canopenSTM32);
/* Thread function executes in constant intervals, this function can be called
 * from FreeRTOS tasks or Timers ********/
void CANopenNode_IRQ(CANopenNodeHandle* canopenSTM32);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CANOPENSTM32_CO_APP_STM32_H_ */
