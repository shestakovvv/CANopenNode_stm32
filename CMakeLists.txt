set(MAIN_NODE_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/CANopenNode)
set(STM32_NODE_PATH ${CMAKE_CURRENT_LIST_DIR})

set(CAN_OPEN_NODE_SOURCES
        ${STM32_NODE_PATH}/CO_app_STM32.c
        ${STM32_NODE_PATH}/CO_driver_STM32.c

        ${MAIN_NODE_PATH}/CANopen.c
        ${MAIN_NODE_PATH}/301/CO_PDO.c
        ${MAIN_NODE_PATH}/301/CO_ODInterface.c
        ${MAIN_NODE_PATH}/301/CO_Emergency.c
        ${MAIN_NODE_PATH}/301/CO_NMT_Heartbeat.c
        ${MAIN_NODE_PATH}/301/CO_SDOclient.c
        ${MAIN_NODE_PATH}/301/CO_HBconsumer.c
        ${MAIN_NODE_PATH}/301/CO_SYNC.c
        ${MAIN_NODE_PATH}/301/CO_TIME.c

#        /* sdo download */
        ${MAIN_NODE_PATH}/301/CO_SDOserver.c
        ${MAIN_NODE_PATH}/301/CO_fifo.c
#        /* end sdo download */

        ${MAIN_NODE_PATH}/303/CO_LEDs.c

        ${MAIN_NODE_PATH}/305/CO_LSSslave.c

        PARENT_SCOPE
)

set(CAN_OPEN_NODE_INCLUDES
        ${STM32_NODE_PATH}
        ${MAIN_NODE_PATH}

        PARENT_SCOPE
)
