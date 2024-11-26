/*
 * CANopen main program file.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        main_generic.c
 * @author      Valeriy Shestakov       2023
 *              Hamed Jafarzadeh 	2022
 * 		Tilen Marjerle		2021
 * 		Janez Paternoster	2020
 * @copyright   2021 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include "CO_app_STM32.h"
//#include "CO_storageBlank.h"

#ifdef CO_MULTIPLE_OD
#ifndef CO_OD_COUNT
#error Provide count of CAN ports you use
#endif

#include "OD1.h"
#include "OD2.h"

#else
#include "OD.h"
#define CO_OD_COUNT 1
#endif
static CANopenNodeHandle* hCANopenNode_List[CO_OD_COUNT];
static uint8_t hCANopenNode_Counter = 0;


/* Printf function of CanOpen app */
#ifndef CAN_OPEN_NODE_PRINTF
#define CAN_OPEN_NODE_PRINTF(...)
#endif

/* default values for CO_CANopenInit() */
#ifndef NMT_CONTROL
#define NMT_CONTROL     (CO_NMT_ERR_ON_BUSOFF_HB | CO_NMT_ERR_ON_ERR_REG)
#endif
#ifndef FIRST_HB_TIME
#define FIRST_HB_TIME        500
#endif
#ifndef SDO_SRV_TIMEOUT_TIME
#define SDO_SRV_TIMEOUT_TIME 1000
#endif
#ifndef SDO_CLI_TIMEOUT_TIME
#define SDO_CLI_TIMEOUT_TIME 500
#endif
#ifndef SDO_CLI_BLOCK
#define SDO_CLI_BLOCK        false
#endif
#ifndef OD_STATUS_BITS
#define OD_STATUS_BITS       NULL
#endif

/* This function will basically setup the CANopen node */
CO_app_Status CANopenNode_Init(CANopenNodeHandle *hCANopenNode) {
        hCANopenNode_List[hCANopenNode_Counter++] = hCANopenNode;
        hCANopenNode->activeNodeID = 0;
        hCANopenNode->canOpen_Obj = NULL;
        hCANopenNode->canOpen_Config = NULL;
        hCANopenNode->canOpen_HeapMemoryUsed = 0;
        hCANopenNode->canOpen_PrevProcessTime = 0;

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
        CO_storage_t storage;
        CO_storage_entry_t storageEntries[] = {{.addr = &OD_PERSIST_COMM,
                                                .len = sizeof(OD_PERSIST_COMM),
                                                .subIndexOD = 2,
                                                .attr = CO_storage_cmd | CO_storage_restore,
                                                .addrNV = NULL}};
        uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);
        uint32_t storageInitError = 0;
#endif
        /* Allocate memory */
#ifdef CO_MULTIPLE_OD
        hCANopenNode->canOpen_Config = malloc(sizeof(CO_config_t));
        if (hCANopenNode->CANHandle->Instance == CAN1) {
                OD1_INIT_CONFIG(*hCANopenNode->canOpen_Config);
        } else if (hCANopenNode->CANHandle->Instance == CAN2) {
                OD2_INIT_CONFIG(*hCANopenNode->canOpen_Config);
        }

        hCANopenNode->canOpen_Config->CNT_LEDS = true;
        hCANopenNode->canOpen_Config->CNT_LSS_SLV = true;
#endif /* CO_MULTIPLE_OD */

        hCANopenNode->canOpen_Obj = CO_new(hCANopenNode->canOpen_Config, &hCANopenNode->canOpen_HeapMemoryUsed);
        if (hCANopenNode->canOpen_Obj == NULL) {
                CAN_OPEN_NODE_PRINTF("Error: Can't allocate memory\n");
                return CO_APP_ERROR_CAN_NOT_ALLOCATE_MEMORY;
        } else {
                CAN_OPEN_NODE_PRINTF("Allocated %u bytes for CANopen objects\n", hCANopenNode->canOpen_HeapMemoryUsed);
        }

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
        err = CO_storageBlank_init(&storage, CO->CANmodule, OD_ENTRY_H1010_storeParameters,
                                   OD_ENTRY_H1011_restoreDefaultParameters, storageEntries, storageEntriesCount,
                                   &storageInitError);

        if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
            CAN_OPEN_NODE_PRINTF("Error: Storage %d\n", storageInitError);
            return 2;
        }
#endif

        return CANopenNode_ResetCommunication(hCANopenNode);
}

CO_app_Status
CANopenNode_ResetCommunication(CANopenNodeHandle *hCANopenHandle) {
        /* CANopen communication reset - initialize CANopen objects *******************/
        CAN_OPEN_NODE_PRINTF("CANopenNode - Reset communication...\n");

        /* Wait rt_thread. */
        hCANopenHandle->canOpen_Obj->CANmodule->CANnormal = false;

        /* Enter CAN configuration. */
        CO_CANsetConfigurationMode((void *) hCANopenHandle);
        CO_CANmodule_disable(hCANopenHandle->canOpen_Obj->CANmodule);

        /* initialize CANopen */
        CO_ReturnError_t err = CO_CANinit(hCANopenHandle->canOpen_Obj,
                                          hCANopenHandle,
                                          0); // Bitrate for STM32 microcontroller is being set in MXCube Settings
        if (err != CO_ERROR_NO) {
                CAN_OPEN_NODE_PRINTF("Error: CAN initialization failed: %d\n", err);
                return 1;
        }

#ifdef CO_MULTIPLE_OD
        CO_LSS_address_t lssAddress = {0};
        if (hCANopenHandle->CANHandle->Instance == CAN1) {
                lssAddress.identity.vendorID = OD1_PERSIST_COMM.x1018_identity.vendor_ID;
                lssAddress.identity.productCode = OD1_PERSIST_COMM.x1018_identity.productCode;
                lssAddress.identity.revisionNumber = OD1_PERSIST_COMM.x1018_identity.revisionNumber;
                lssAddress.identity.serialNumber = OD1_PERSIST_COMM.x1018_identity.serialNumber;
        } else if (hCANopenHandle->CANHandle->Instance == CAN2) {
                lssAddress.identity.vendorID = OD2_PERSIST_COMM.x1018_identity.vendor_ID;
                lssAddress.identity.productCode = OD2_PERSIST_COMM.x1018_identity.productCode;
                lssAddress.identity.revisionNumber = OD2_PERSIST_COMM.x1018_identity.revisionNumber;
                lssAddress.identity.serialNumber = OD2_PERSIST_COMM.x1018_identity.serialNumber;
        }
#else
        CO_LSS_address_t lssAddress = {
                .identity = {
                        .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                        .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                        .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                        .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber
                }
        };
#endif
        err = CO_LSSinit(
                hCANopenHandle->canOpen_Obj,
                &lssAddress,
                &hCANopenHandle->desiredNodeID,
                (uint16_t *) &hCANopenHandle->baudrate
        );
        if (err != CO_ERROR_NO) {
                CAN_OPEN_NODE_PRINTF("Error: LSS slave initialization failed: %d\n", err);
                return 2;
        }

        hCANopenHandle->activeNodeID = hCANopenHandle->desiredNodeID;
        uint32_t errInfo = 0;

#ifdef CO_MULTIPLE_OD
        if (hCANopenHandle->CANHandle->Instance == CAN1)
                err = CO_CANopenInit(
                        hCANopenHandle->canOpen_Obj,                   /* CANopen object */
                        NULL,                 /* alternate NMT */
                        NULL,                 /* alternate em */
                        OD1,                   /* Object dictionary */
                        OD_STATUS_BITS,       /* Optional OD_statusBits */
                        NMT_CONTROL,          /* CO_NMT_control_t */
                        FIRST_HB_TIME,        /* firstHBTime_ms */
                        SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                        SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                        SDO_CLI_BLOCK,        /* SDOclientBlockTransfer */
                        hCANopenHandle->activeNodeID, &errInfo);
        else if (hCANopenHandle->CANHandle->Instance == CAN2) {
                err = CO_CANopenInit(
                        hCANopenHandle->canOpen_Obj,                   /* CANopen object */
                        NULL,                 /* alternate NMT */
                        NULL,                 /* alternate em */
                        OD2,                   /* Object dictionary */
                        OD_STATUS_BITS,       /* Optional OD_statusBits */
                        NMT_CONTROL,          /* CO_NMT_control_t */
                        FIRST_HB_TIME,        /* firstHBTime_ms */
                        SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                        SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                        SDO_CLI_BLOCK,        /* SDOclientBlockTransfer */
                        hCANopenHandle->activeNodeID, &errInfo);
        }
#else
        err = CO_CANopenInit(hCANopenHandle->canOpen_Obj,                   /* CANopen object */
                         NULL,                 /* alternate NMT */
                         NULL,                 /* alternate em */
                         OD,                   /* Object dictionary */
                         OD_STATUS_BITS,       /* Optional OD_statusBits */
                         NMT_CONTROL,          /* CO_NMT_control_t */
                         FIRST_HB_TIME,        /* firstHBTime_ms */
                         SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                         SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                         SDO_CLI_BLOCK,        /* SDOclientBlockTransfer */
                         hCANopenHandle->activeNodeID, &errInfo);
#endif
        if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
                if (err == CO_ERROR_OD_PARAMETERS) {
                        CAN_OPEN_NODE_PRINTF("Error: Object Dictionary entry 0x%X\n", errInfo);
                } else {
                        CAN_OPEN_NODE_PRINTF("Error: CANopen initialization failed: %d\n", err);
                }
                return 3;
        }

#ifdef CO_MULTIPLE_OD
        if (hCANopenHandle->CANHandle->Instance == CAN1) {
                err = CO_CANopenInitPDO(hCANopenHandle->canOpen_Obj,
                                        hCANopenHandle->canOpen_Obj->em, OD1,
                                        hCANopenHandle->activeNodeID, &errInfo);
        } else if (hCANopenHandle->CANHandle->Instance == CAN2) {
                err = CO_CANopenInitPDO(hCANopenHandle->canOpen_Obj,
                                        hCANopenHandle->canOpen_Obj->em, OD2,
                                        hCANopenHandle->activeNodeID, &errInfo);
        }
#else
        err = CO_CANopenInitPDO(hCANopenHandle->canOpen_Obj, hCANopenHandle->canOpen_Obj->em, OD, hCANopenHandle->activeNodeID, &errInfo);
#endif
        if (err != CO_ERROR_NO) {
                if (err == CO_ERROR_OD_PARAMETERS) {
                        CAN_OPEN_NODE_PRINTF("Error: Object Dictionary entry 0x%X\n", errInfo);
                } else {
                        CAN_OPEN_NODE_PRINTF("Error: PDO initialization failed: %d\n", err);
                }
                return 4;
        }


        /* Configure Timer interrupt function for execution every 1 millisecond */
        HAL_TIM_Base_Start_IT(hCANopenHandle->timerHandle); //1ms interrupt

        /* Configure CAN transmit and receive interrupt */

        /* Configure CANopen callbacks, etc */
        if (!hCANopenHandle->canOpen_Obj->nodeIdUnconfigured) {

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
                if (storageInitError != 0) {
                    CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, storageInitError);
                }
#endif
        } else {
                CAN_OPEN_NODE_PRINTF("CANopenNode - Node-id not initialized\n");
        }

        /* start CAN */
        CO_CANsetNormalMode(hCANopenHandle->canOpen_Obj->CANmodule);

        CAN_OPEN_NODE_PRINTF("CANopenNode - Running...\n");
        fflush(stdout);
        hCANopenHandle->canOpen_PrevProcessTime = HAL_GetTick();
        return 0;
}

void
CANopenNode_Process(CANopenNodeHandle *hCANopenHandle) {
        /* loop for normal program execution ******************************************/
        /* get time difference since last function call */
        uint32_t time_current = HAL_GetTick();

        uint32_t time_old = hCANopenHandle->canOpen_PrevProcessTime;

        if ((time_current - time_old) > 0) { // Make sure more than 1ms elapsed
                /* CANopen process */
                CO_NMT_reset_cmd_t reset_status;
                uint32_t timeDifference_us = (time_current - time_old) * 1000;
                hCANopenHandle->canOpen_PrevProcessTime = time_current;
                reset_status = CO_process(hCANopenHandle->canOpen_Obj, false,
                                          timeDifference_us, NULL);

                if (reset_status == CO_RESET_COMM) {
                        /* delete objects from memory */
                        CO_CANsetConfigurationMode((void *) hCANopenHandle);
                        CO_delete(hCANopenHandle->canOpen_Obj);
#ifdef CAN_OPEN_NODE_PRINTF
                        CAN_OPEN_NODE_PRINTF("CANopenNode Reset Communication request\n");
#endif
                        CANopenNode_ResetCommunication(
                                hCANopenHandle); // Reset Communication routine
                } else if (reset_status == CO_RESET_APP) {
#ifdef CAN_OPEN_NODE_PRINTF
                        CAN_OPEN_NODE_PRINTF("CANopenNode Device Reset\n");
#endif
                        HAL_NVIC_SystemReset(); // Reset the STM32 Microcontroller
                }
        }
}

/* Thread function executes in constant intervals, this function can be called from FreeRTOS tasks or Timers ********/
void
CANopenNode_IRQ(CANopenNodeHandle *hCANopenHandle) {
        CO_LOCK_OD(hCANopenHandle->canOpen_Obj->CANmodule);
        if (!hCANopenHandle->canOpen_Obj->nodeIdUnconfigured &&
            hCANopenHandle->canOpen_Obj->CANmodule->CANnormal) {
                bool_t syncWas = false;
                /* get time difference since last function call */
                #if BOARD_TYPE==BOARD_TYPE_CENTRAL_BOARD
                uint32_t timeDifference_us = 10000; // 1ms second
                #else
                uint32_t timeDifference_us = 1000; // 1ms second
                #endif

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
                syncWas = CO_process_SYNC(hCANopenHandle->canOpen_Obj,
                                          timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
                CO_process_RPDO(hCANopenHandle->canOpen_Obj, syncWas,
                                timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
                CO_process_TPDO(hCANopenHandle->canOpen_Obj, syncWas,
                                timeDifference_us, NULL);
#endif

                /* Further I/O or nonblocking application code may go here. */
        }
        CO_UNLOCK_OD(hCANopenHandle->canOpen_Obj->CANmodule);
}

#ifndef CAN_OPEN_NODE_CALLBACKS_OVERRIDE 
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
#ifdef CO_MULTIPLE_OD
        for (uint8_t i=0; i<CO_OD_COUNT; i++) {
                if (hcan == hCANopenNode_List[i]->CANHandle) {
                        CO_CANinterrupt_TX(hCANopenNode_List[i]->canOpen_Obj->CANmodule, CAN_TX_MAILBOX0);
                        break;
                }
        }
#else
        if (hcan == hCANopenNode_List[0]->CANHandle) {
                CO_CANinterrupt_TX(hCANopenNode_List[0]->canOpen_Obj->CANmodule, CAN_TX_MAILBOX0);
        }
#endif
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
        #ifdef CO_MULTIPLE_OD
        for (uint8_t i=0; i<CO_OD_COUNT; i++) {
                if (hcan == hCANopenNode_List[i]->CANHandle) {
                        CO_CANinterrupt_TX(hCANopenNode_List[i]->canOpen_Obj->CANmodule, CAN_TX_MAILBOX1);
                        break;
                }
        }
        #else
        if (hcan == hCANopenNode_List[0]->CANHandle) {
                CO_CANinterrupt_TX(hCANopenNode_List[0]->canOpen_Obj->CANmodule, CAN_TX_MAILBOX1);
        }
#endif
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
        #ifdef CO_MULTIPLE_OD
        for (uint8_t i=0; i<CO_OD_COUNT; i++) {
                if (hcan == hCANopenNode_List[i]->CANHandle) {
                        CO_CANinterrupt_TX(hCANopenNode_List[i]->canOpen_Obj->CANmodule, CAN_TX_MAILBOX2);
                        break;
                }
        }
        #else
        CO_CANinterrupt_TX(hCANopenNode_List[0]->canOpen_Obj->CANmodule, CAN_TX_MAILBOX2);

#endif
}

uint32_t can_timeInterruptPoint=0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
//Watchdog protection
        can_timeInterruptPoint = HAL_GetTick();
//Watchdog protection
        #ifdef CO_MULTIPLE_OD
        for (uint8_t i=0; i<CO_OD_COUNT; i++) {
                if (hcan == hCANopenNode_List[i]->CANHandle) {
                        CO_CANinterrupt_RX(hCANopenNode_List[i]->canOpen_Obj->CANmodule, CAN_RX_FIFO0);
                        break;
                }
        }
        #else
        CO_CANinterrupt_RX(hCANopenNode_List[0]->canOpen_Obj->CANmodule, CAN_RX_FIFO0);
#endif
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
//Watchdog protection
        can_timeInterruptPoint = HAL_GetTick();
//Watchdog protection
        #ifdef CO_MULTIPLE_OD
        for (uint8_t i=0; i<CO_OD_COUNT; i++) {
                if (hcan == hCANopenNode_List[i]->CANHandle) {
                        CO_CANinterrupt_RX(hCANopenNode_List[i]->canOpen_Obj->CANmodule, CAN_RX_FIFO1);
                        break;
                }
        }
        #else
        CO_CANinterrupt_RX(hCANopenNode_List[0]->canOpen_Obj->CANmodule, CAN_RX_FIFO1);

#endif
}
#endif
