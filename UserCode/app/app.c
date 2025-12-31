/**
 * @file    app.h
 * @author  syhanjin
 * @date    2025-12-24
 */
#include "app.h"
#include "cmsis_os2.h"
#include "usart.h"

#include "drivers/ops.h"
#include "drivers/HWT101CT.h"

#define DEVICE_SENSOR_GRYO_YAW_HUART (&huart2)
HWT101CT_t sensor_gyro_yaw;

#define DEVICE_SENSOR_OPS_HUART (&huart6)
OPS_t sensor_ops;

void GYRO_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    HWT101CT_RxCallback(&sensor_gyro_yaw);
}
void GYRO_UART_RxErrorHandler(UART_HandleTypeDef* huart)
{
    HWT101CT_RxErrorHandler(&sensor_gyro_yaw);
}

void OPS_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    OPS_RxCpltCallback(&sensor_ops);
}
void OPS_UART_RxErrorHandler(UART_HandleTypeDef* huart)
{
    OPS_RxErrorHandler(&sensor_ops);
}

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
void Init(void* argument)
{
    /* 初始化代码 */
    HAL_UART_RegisterCallback(DEVICE_SENSOR_GRYO_YAW_HUART,
                              HAL_UART_RX_COMPLETE_CB_ID,
                              GYRO_UART_RxCpltCallback);
    HAL_UART_RegisterCallback(DEVICE_SENSOR_GRYO_YAW_HUART,
                              HAL_UART_ERROR_CB_ID,
                              GYRO_UART_RxErrorHandler);
    HWT101CT_Init(&sensor_gyro_yaw, DEVICE_SENSOR_GRYO_YAW_HUART);

    HAL_UART_RegisterCallback(DEVICE_SENSOR_OPS_HUART,
                              HAL_UART_RX_COMPLETE_CB_ID,
                              OPS_UART_RxCpltCallback);
    HAL_UART_RegisterCallback(DEVICE_SENSOR_OPS_HUART,
                              HAL_UART_ERROR_CB_ID,
                              OPS_UART_RxErrorHandler);
    OPS_Init(&sensor_ops,
             &(OPS_config_t) { .huart      = DEVICE_SENSOR_OPS_HUART,
                               .x_offset   = -290.0f,
                               .y_offset   = 10.0f,
                               .yaw_offset = 0.0f,
                               .yaw_car    = HWT101CT_GetYawPtr(&sensor_gyro_yaw) });

    osDelay(5000);

    OPS_WorldCoord_Reset(&sensor_ops);

    for (;;)
        osDelay(1);

    /* 初始化完成后退出线程 */
    osThreadExit();
}