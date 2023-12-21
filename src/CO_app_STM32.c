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

#include "CO_app_STM32.h"
//#include "CO_storageBlank.h"

#ifdef CO_MULTIPLE_OD
#include "OD1.h"
#include "OD2.h"
#else
#include "OD.h"
#endif

/* Printf function of CanOpen app */
#ifdef CAN_OPEN_NODE_USE_PRINTF
#define log_printf(macropar_message, ...) CAN_OPEN_NODE_USE_PRINTF(macropar_message, ##__VA_ARGS__)
#else
#define log_printf(macropar_message, ...) void
#endif

/* default values for CO_CANopenInit() */
#define NMT_CONTROL     (CO_NMT_STARTUP_TO_OPERATIONAL \
    | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)
#define FIRST_HB_TIME        500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK        false
#define OD_STATUS_BITS       NULL

#ifdef CO_MULTIPLE_OD
uint32_t prevTime[] = {0, 0};
CO_config_t coConfig[] = {{0}, {0}};

#define OD1_INDEX 0
#define OD1_CAN CAN1
#define OD2_INDEX 1
#define OD2_CAN CAN2
#else
uint32_t prevTime = 0;
#endif

/* This function will basically setup the CANopen node */
int
CANopenNode_Init(CANopenNodeHandle* canopenSTM32) {

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
    CO_config_t* config_ptr = NULL;
#ifdef CO_MULTIPLE_OD
    if (canopenSTM32->CANHandle->Instance == CAN1) {
            config_ptr = &coConfig[OD1_INDEX];
            OD1_INIT_CONFIG(*config_ptr);
    } else if (canopenSTM32->CANHandle->Instance == CAN2) {
            config_ptr = &coConfig[OD2_INDEX];
            OD2_INIT_CONFIG(*config_ptr);
    }

        config_ptr->CNT_LEDS = 1;
        config_ptr->CNT_LSS_SLV = 1;
#endif /* CO_MULTIPLE_OD */

    uint32_t heapMemoryUsed;
        canopenSTM32->canOpenStack = CO_new(config_ptr, &heapMemoryUsed);
    if (canopenSTM32->canOpenStack == NULL) {
        log_printf("Error: Can't allocate memory\n");
        return 1;
    } else {
        log_printf("Allocated %u bytes for CANopen objects\n", heapMemoryUsed);
    }

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    err = CO_storageBlank_init(&storage, CO->CANmodule, OD_ENTRY_H1010_storeParameters,
                               OD_ENTRY_H1011_restoreDefaultParameters, storageEntries, storageEntriesCount,
                               &storageInitError);

    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
        log_printf("Error: Storage %d\n", storageInitError);
        return 2;
    }
#endif

        CANopenNode_ResetCommunication(canopenSTM32);
    return 0;
}

int
CANopenNode_ResetCommunication(CANopenNodeHandle* canopenSTM32) {
    /* CANopen communication reset - initialize CANopen objects *******************/
    log_printf("CANopenNode - Reset communication...\n");

    /* Wait rt_thread. */
        canopenSTM32->canOpenStack->CANmodule->CANnormal = false;

    /* Enter CAN configuration. */
    CO_CANsetConfigurationMode((void*)canopenSTM32);
    CO_CANmodule_disable(canopenSTM32->canOpenStack->CANmodule);

    /* initialize CANopen */
    CO_ReturnError_t err = CO_CANinit(canopenSTM32->canOpenStack, canopenSTM32, 0); // Bitrate for STM32 microcontroller is being set in MXCube Settings
    if (err != CO_ERROR_NO) {
        log_printf("Error: CAN initialization failed: %d\n", err);
        return 1;
    }

#ifdef CO_MULTIPLE_OD
    CO_LSS_address_t lssAddress = {0};
    if (canopenSTM32->CANHandle->Instance == CAN1) {
            lssAddress.identity.vendorID = OD1_PERSIST_COMM.x1018_identity.vendor_ID;
            lssAddress.identity.productCode = OD1_PERSIST_COMM.x1018_identity.productCode;
            lssAddress.identity.revisionNumber = OD1_PERSIST_COMM.x1018_identity.revisionNumber;
            lssAddress.identity.serialNumber = OD1_PERSIST_COMM.x1018_identity.serialNumber;
    }
    else if (canopenSTM32->CANHandle->Instance == CAN2) {
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
            canopenSTM32->canOpenStack,
            &lssAddress,
            &canopenSTM32->desiredNodeID,
            (uint16_t*)&canopenSTM32->baudrate
    );
    if (err != CO_ERROR_NO) {
        log_printf("Error: LSS slave initialization failed: %d\n", err);
        return 2;
    }

        canopenSTM32->activeNodeID = canopenSTM32->desiredNodeID;
    uint32_t errInfo = 0;

#ifdef CO_MULTIPLE_OD
    if (canopenSTM32->CANHandle->Instance == CAN1)
        err = CO_CANopenInit(canopenSTM32->canOpenStack,                   /* CANopen object */
                         NULL,                 /* alternate NMT */
                         NULL,                 /* alternate em */
                         OD1,                   /* Object dictionary */
                         OD_STATUS_BITS,       /* Optional OD_statusBits */
                         NMT_CONTROL,          /* CO_NMT_control_t */
                         FIRST_HB_TIME,        /* firstHBTime_ms */
                         SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                         SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                         SDO_CLI_BLOCK,        /* SDOclientBlockTransfer */
                         canopenSTM32->activeNodeID, &errInfo);
    else if (canopenSTM32->CANHandle->Instance == CAN2) {
        err = CO_CANopenInit(canopenSTM32->canOpenStack,                   /* CANopen object */
                         NULL,                 /* alternate NMT */
                         NULL,                 /* alternate em */
                         OD2,                   /* Object dictionary */
                         OD_STATUS_BITS,       /* Optional OD_statusBits */
                         NMT_CONTROL,          /* CO_NMT_control_t */
                         FIRST_HB_TIME,        /* firstHBTime_ms */
                         SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                         SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                         SDO_CLI_BLOCK,        /* SDOclientBlockTransfer */
                         canopenSTM32->activeNodeID, &errInfo);
    }
#else
        err = CO_CANopenInit(canopenSTM32->canOpenStack,                   /* CANopen object */
                         NULL,                 /* alternate NMT */
                         NULL,                 /* alternate em */
                         OD,                   /* Object dictionary */
                         OD_STATUS_BITS,       /* Optional OD_statusBits */
                         NMT_CONTROL,          /* CO_NMT_control_t */
                         FIRST_HB_TIME,        /* firstHBTime_ms */
                         SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                         SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                         SDO_CLI_BLOCK,        /* SDOclientBlockTransfer */
                         canopenSTM32->activeNodeID, &errInfo);
#endif
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        if (err == CO_ERROR_OD_PARAMETERS) {
            log_printf("Error: Object Dictionary entry 0x%X\n", errInfo);
        } else {
            log_printf("Error: CANopen initialization failed: %d\n", err);
        }
        return 3;
    }

#ifdef CO_MULTIPLE_OD
        if (canopenSTM32->CANHandle->Instance == CAN1) {
                err = CO_CANopenInitPDO(canopenSTM32->canOpenStack, canopenSTM32->canOpenStack->em, OD1, canopenSTM32->activeNodeID, &errInfo);
        }
        else if (canopenSTM32->CANHandle->Instance == CAN2) {
                err = CO_CANopenInitPDO(canopenSTM32->canOpenStack, canopenSTM32->canOpenStack->em, OD2, canopenSTM32->activeNodeID, &errInfo);
        }
#else
        err = CO_CANopenInitPDO(canopenSTM32->canOpenStack, canopenSTM32->canOpenStack->em, OD, canopenSTM32->activeNodeID, &errInfo);
#endif
        if (err != CO_ERROR_NO) {
                if (err == CO_ERROR_OD_PARAMETERS) {
                        log_printf("Error: Object Dictionary entry 0x%X\n", errInfo);
                } else {
                        log_printf("Error: PDO initialization failed: %d\n", err);
                }
                return 4;
        }


    /* Configure Timer interrupt function for execution every 1 millisecond */
    HAL_TIM_Base_Start_IT(canopenSTM32->timerHandle); //1ms interrupt

    /* Configure CAN transmit and receive interrupt */

    /* Configure CANopen callbacks, etc */
    if (!canopenSTM32->canOpenStack->nodeIdUnconfigured) {

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
        if (storageInitError != 0) {
            CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, storageInitError);
        }
#endif
    } else {
        log_printf("CANopenNode - Node-id not initialized\n");
    }

    /* start CAN */
    CO_CANsetNormalMode(canopenSTM32->canOpenStack->CANmodule);

    log_printf("CANopenNode - Running...\n");
    fflush(stdout);
#ifdef CO_MULTIPLE_OD
    if (canopenSTM32->CANHandle->Instance == CAN1)
        prevTime[OD1_INDEX] = HAL_GetTick();
    else if (canopenSTM32->CANHandle->Instance == CAN2)
        prevTime[OD2_INDEX] = HAL_GetTick();

#else
        prevTime = HAL_GetTick();
#endif
    return 0;
}

void
CANopenNode_Process(CANopenNodeHandle* canopenSTM32) {
    /* loop for normal program execution ******************************************/
    /* get time difference since last function call */
    uint32_t time_current = HAL_GetTick();

    uint32_t time_old;
#ifdef CO_MULTIPLE_OD
    if (canopenSTM32->CANHandle->Instance == CAN1)
            time_old = prevTime[OD1_INDEX];
    else if (canopenSTM32->CANHandle->Instance == CAN2)
            time_old = prevTime[OD2_INDEX];
    else
            time_old = 0;
#else
    time_old = prevTime;
#endif

    if ((time_current - time_old) > 0) { // Make sure more than 1ms elapsed
        /* CANopen process */
        CO_NMT_reset_cmd_t reset_status;
        uint32_t timeDifference_us = (time_current - time_old) * 1000;
#ifdef CO_MULTIPLE_OD
            if (canopenSTM32->CANHandle->Instance == CAN1)
                    prevTime[OD1_INDEX] = time_current;
            else if (canopenSTM32->CANHandle->Instance == CAN2)
                    prevTime[OD2_INDEX] = time_current;
#else
            prevTime = time_current;
#endif
        reset_status = CO_process(canopenSTM32->canOpenStack, false, timeDifference_us, NULL);

        if (reset_status == CO_RESET_COMM) {
            /* delete objects from memory */
            CO_CANsetConfigurationMode((void*)canopenSTM32);
            CO_delete(canopenSTM32->canOpenStack);
            log_printf("CANopenNode Reset Communication request\n");
                CANopenNode_ResetCommunication(canopenSTM32); // Reset Communication routine
        } else if (reset_status == CO_RESET_APP) {
            log_printf("CANopenNode Device Reset\n");
            HAL_NVIC_SystemReset(); // Reset the STM32 Microcontroller
        }
    }
}

/* Thread function executes in constant intervals, this function can be called from FreeRTOS tasks or Timers ********/
void
CANopenNode_IRQ(CANopenNodeHandle* canopenSTM32) {
    CO_LOCK_OD(canopenSTM32->canOpenStack->CANmodule);
    if (!canopenSTM32->canOpenStack->nodeIdUnconfigured && canopenSTM32->canOpenStack->CANmodule->CANnormal) {
        bool_t syncWas = false;
        /* get time difference since last function call */
        uint32_t timeDifference_us = 1000; // 1ms second

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
        syncWas = CO_process_SYNC(canopenSTM32->canOpenStack, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
        CO_process_RPDO(canopenSTM32->canOpenStack, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
        CO_process_TPDO(canopenSTM32->canOpenStack, syncWas, timeDifference_us, NULL);
#endif

        /* Further I/O or nonblocking application code may go here. */
    }
    CO_UNLOCK_OD(canopenSTM32->canOpenStack->CANmodule);
}
