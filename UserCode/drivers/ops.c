/**
 * @file    ops.c
 * @author  mogegee
 * @date    2025-12-24
 */
#include "ops.h"
#include <math.h>
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef DEBUG
#    define DEBUG_FRAME_COUNT(__p__) ((__p__)->frame_count++)
#    define DEBUG_FRAME_ERROR(__p__) ((__p__)->error_count++)
#    define DEBUG_SYNC_COUNT(__p__)  ((__p__)->sync_count++)
#else
#    define DEBUG_FRAME_COUNT(__p__) ((void*) __p__)
#    define DEBUG_FRAME_ERROR(__p__) ((void*) __p__)
#    define DEBUG_SYNC_COUNT(__p__)  ((void*) __p__)
#endif

/**
 * @brief  OPS模块初始化
 * @param  ops: OPS设备句柄
 * @param  ops_config: 配置
 */
void OPS_Init(OPS_t* ops, OPS_config_t* ops_config)
{
    ops->huart      = ops_config->huart;
    ops->sync_state = OPS_SYNC_WAIT_HEAD1;

    ops->lock = osMutexNew(NULL);

    ops->feedback.pos_x  = 0.0f;
    ops->feedback.pos_y  = 0.0f;
    ops->feedback.zangle = 0.0f;
    ops->feedback.xangle = 0.0f;
    ops->feedback.yangle = 0.0f;
    ops->feedback.w_z    = 0.0f;

    memset(ops->rx_buffer, 0, sizeof(ops->rx_buffer));

    ops->gyro_yaw                = ops_config->yaw_car;
    ops->gyro_yaw_body_zeropoint = 0.0f;
    ops->Cx                      = 0.0f;
    ops->Cy                      = 0.0f;
    ops->yaw_car                 = 0.0f;

    ops->x_offset       = ops_config->x_offset * 1e-3f; // mm to m
    ops->y_offset       = ops_config->y_offset * 1e-3f; // mm to m
    ops->cos_yaw_offset = cosf(DEG2RAD(ops_config->yaw_offset));
    ops->sin_yaw_offset = sinf(DEG2RAD(ops_config->yaw_offset));

    HAL_UART_Receive_IT(ops->huart, &ops->rx_buffer[0], 1);
}

/**
 * @brief  OPS串口发送数据
 * @attention 该函数为阻塞发送
 * @param  ops: OPS设备句柄指针
 * @param  data: 发送数据指针
 * @param  len: 发送数据长度（字节）
 */
static void OPS_SendData(const OPS_t* ops, const uint8_t* data, const uint16_t len)
{
    // 阻塞发送
    if (__get_IPSR() != 0)
        HAL_UART_Transmit(ops->huart, data, len, 100);
    else
    {
        osMutexAcquire(ops->lock, osWaitForever);
        HAL_UART_Transmit(ops->huart, data, len, 100);
        osMutexRelease(ops->lock);
    }
}

/**
 * @brief  OPS校准命令（发送"ACTR"）
 * @attention 该函数一般情况下不会被使用
 * @param  ops: OPS设备句柄指针
 */
void OPS_Calibration(OPS_t* ops)
{
    const uint8_t calib_cmd[4] = "ACTR";
    OPS_SendData(ops, calib_cmd, sizeof(calib_cmd));
}

/**
 * @brief  OPS清零命令（发送"ACT0"）
 * @param  ops: OPS设备句柄指针
 * @note   清零后角度/坐标重置为0
 */
void OPS_ZeroClearing(OPS_t* ops)
{
    const uint8_t zero_cmd[] = "ACT0"; // 清零命令
    OPS_SendData(ops, zero_cmd, sizeof(zero_cmd));
}

/**
 * @brief  OPS更新航向角（发送"ACTJ"+4字节float）
 * @param  ops: OPS设备句柄指针
 * @param  angle: 目标航向角（范围-180~180度，float类型）
 */
void OPS_UpdateYaw(const OPS_t* ops, const float angle)
{
    uint8_t cmd[8] = "ACTJ";
    memcpy(cmd + 4, &angle, 4);
    OPS_SendData(ops, cmd, sizeof(cmd));
}

/**
 * @brief  OPS更新X坐标（发送"ACTX"+4字节float）
 * @param  ops: OPS设备句柄指针
 * @param  posx: 目标X坐标（单位m，float类型）
 */
void OPS_UpdateX(const OPS_t* ops, const float posx)
{
    uint8_t     cmd[8] = "ACTX";
    const float value  = posx * 1000.0f;
    memcpy(cmd + 4, &value, 4);
    OPS_SendData(ops, cmd, sizeof(cmd));
}

/**
 * @brief  OPS更新Y坐标（发送"ACTY"+4字节float）
 * @param  ops: OPS设备句柄指针
 * @param  posy: 目标Y坐标（单位m，float类型）
 */
void OPS_UpdateY(const OPS_t* ops, const float posy)
{
    uint8_t     cmd[8] = "ACTY";
    const float value  = posy * 1000.0f;
    memcpy(cmd + 4, &value, 4);
    OPS_SendData(ops, cmd, sizeof(cmd));
}

/**
 * @brief  OPS更新XY坐标（发送"ACTD"+8字节float）
 * @param  ops: OPS设备句柄指针
 * @param  posx: 目标X坐标（单位m，float类型）
 * @param  posy: 目标Y坐标（单位m，float类型）
 */
void OPS_UpdateXY(OPS_t* ops, float posx, float posy)
{
    uint8_t     cmd[12] = "ACTD";
    const float value1  = posx * 1000.0f;
    const float value2  = posy * 1000.0f;
    memcpy(cmd + 4, &value1, 4);
    memcpy(cmd + 8, &value2, 4);
    OPS_SendData(ops, cmd, sizeof(cmd));
}

/**
 * @brief  解算车体中心位姿
 * @param  ops: OPS位姿数据指针
 */
static void CarCenterPose_Calc(OPS_t* ops)
{
    // 认为 OPS_World 相对于 World 的位姿即为 OPS 相对于 Body 的位姿
    // OPS 在 OPS_World 中的位置
    const float ops_x_ow = ops->feedback.pos_x;
    const float ops_y_ow = ops->feedback.pos_y;
    // Body 在 World 中的 yaw
    const float body_yaw_w = *ops->gyro_yaw - ops->gyro_yaw_body_zeropoint;

    // 角度范围修正
    // while (yaw_car_raw >= 180.0f)
    //     yaw_car_raw -= 360.0f;
    // while (yaw_car_raw < -180.0f)
    //     yaw_car_raw += 360.0f;

    // 计算OPS相对于世界坐标系的实际偏航角（车体角+安装角偏移）
    const float body_yaw_rad = DEG2RAD(body_yaw_w);

    const float cos_body_yaw = cosf(body_yaw_rad);
    const float sin_body_yaw = sinf(body_yaw_rad);

    // 解算车体中心的世界坐标（Cx, Cy）
    ops->Cx = ops->x_offset +                                               //
              ops->cos_yaw_offset * ops_x_ow - cos_body_yaw * ops->x_offset //
              - ops->sin_yaw_offset * ops_y_ow + sin_body_yaw * ops->y_offset;

    ops->Cy = ops->y_offset +                                               //
              ops->sin_yaw_offset * ops_x_ow - sin_body_yaw * ops->x_offset //
              + ops->cos_yaw_offset * ops_y_ow - cos_body_yaw * ops->y_offset;
    ops->yaw_car = body_yaw_w;
}

/**
 * @brief  OPS数据帧解析
 * @param  ops: OPS设备句柄指针
 * @frame_format: 0X0D(帧头1) + 0X0A(帧头2) + 24字节数据 + 0X0A(帧尾1) + 0X0D(帧尾2)
 */
static void OPS_ParseData(OPS_t* ops)
{
    float values[6];

    for (size_t i = 0; i < 6; i++)
        memcpy(&values[i], &ops->rx_buffer[2 + 4 * i], sizeof(float));

    // 解析24字节数据为6个float
    ops->feedback.zangle = values[0];
    ops->feedback.xangle = values[1];
    ops->feedback.yangle = values[2];
    ops->feedback.pos_x  = values[3] * 1e-3f;
    ops->feedback.pos_y  = values[4] * 1e-3f;
    ops->feedback.w_z    = values[5];

    // 解算车体位置
    CarCenterPose_Calc(ops);
}

void OPS_RxErrorHandler(OPS_t* ops)
{
    DEBUG_FRAME_ERROR(ops);

    // clear error flags
    __HAL_UART_CLEAR_PEFLAG(ops->huart);
    __HAL_UART_CLEAR_FEFLAG(ops->huart);
    __HAL_UART_CLEAR_NEFLAG(ops->huart);
    __HAL_UART_CLEAR_OREFLAG(ops->huart);

    // restart receive
    if (ops->huart->hdmarx->State == HAL_DMA_STATE_BUSY)
        HAL_UART_DMAStop(ops->huart);
    if (ops->sync_state != OPS_SYNC_WAIT_HEAD1)
        ops->sync_state = OPS_SYNC_WAIT_HEAD1;
    HAL_UART_Receive_IT(ops->huart, &ops->rx_buffer[0], 1);
}

/**
 * @brief  OPS USART接收完成回调（需在HAL_UART_RxCpltCallback中调用）
 * @param  ops: OPS设备句柄指针
 */
void OPS_RxCpltCallback(OPS_t* ops)
{
    if (ops->sync_state == OPS_SYNC_DMA_ACTIVE)
    {
        DEBUG_FRAME_COUNT(ops);
        // 校验帧头帧尾
        if (ops->rx_buffer[0] != OPS_HEAD1 || ops->rx_buffer[1] != OPS_HEAD2 ||
            ops->rx_buffer[OPS_FRAME_LEN - 2] != OPS_TAIL1 ||
            ops->rx_buffer[OPS_FRAME_LEN - 1] != OPS_TAIL2)
        {
            // 重新同步
            HAL_UART_DMAStop(ops->huart);
            ops->sync_state = OPS_SYNC_WAIT_HEAD1;
            HAL_UART_Receive_IT(ops->huart, &ops->rx_buffer[0], 1);
            DEBUG_FRAME_ERROR(ops);
            return;
        }
        OPS_ParseData(ops);
    }
    else if (ops->sync_state == OPS_SYNC_RECEIVING)
    {
        DEBUG_FRAME_COUNT(ops);
        DEBUG_SYNC_COUNT(ops);
        OPS_ParseData(ops);
        HAL_UART_Receive_DMA(ops->huart, ops->rx_buffer, OPS_FRAME_LEN);
        ops->sync_state = OPS_SYNC_DMA_ACTIVE;
    }
    else if (ops->sync_state == OPS_SYNC_WAIT_HEAD1)
    {
        if (ops->rx_buffer[0] == OPS_HEAD1)
        {
            HAL_UART_Receive_IT(ops->huart, &ops->rx_buffer[1], 1);
            ops->sync_state = OPS_SYNC_WAIT_HEAD2;
        }
        else
            HAL_UART_Receive_IT(ops->huart, &ops->rx_buffer[0], 1);
    }
    else if (ops->sync_state == OPS_SYNC_WAIT_HEAD2)
    {
        if (ops->rx_buffer[1] == OPS_HEAD2)
        {
            HAL_UART_Receive_IT(ops->huart, ops->rx_buffer + 2, OPS_FRAME_LEN - 2);
            ops->sync_state = OPS_SYNC_RECEIVING;
        }
        // 可能出现 HEAD1, HEAD1, HEAD2 的形式
        else if (ops->rx_buffer[1] == OPS_HEAD1)
        {
            HAL_UART_Receive_IT(ops->huart, &ops->rx_buffer[1], 1);
        }
        else
        {
            HAL_UART_Receive_IT(ops->huart, &ops->rx_buffer[0], 1);
            ops->sync_state = OPS_SYNC_WAIT_HEAD1;
        }
    }
}

/**
 * @brief  世界坐标系重置函数（清零pos_x、pos_y和陀螺仪偏航角）
 * @param  ops: OPS设备句柄指针
 */
void OPS_WorldCoord_Reset(OPS_t* ops)
{
    OPS_ZeroClearing(ops);
    ops->gyro_yaw_body_zeropoint = *ops->gyro_yaw;
}

#ifdef __cplusplus
}
#endif