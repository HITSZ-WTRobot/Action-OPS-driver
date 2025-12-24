/**
 * @file    ops.c
 * @author  mogegee
 * @date    2025-12-24
 */
#include "ops.h"
#include "math.h"
#include "string.h"

static void OPS_ParseData(OPS_t* ops, uint8_t data); // 数据帧解析
static void CarCenterPose_Calc(OPS_t* ops);

/**
 * @brief  OPS模块初始化
 * @param  ops: OPS设备句柄
 * @param  ops_config: 配置
 */
void OPS_Init(OPS_t* ops, OPS_config_t* ops_config)
{
    ops->pos_x      = 0.0f;
    ops->pos_y      = 0.0f;
    ops->zangle     = 0.0f;
    ops->xangle     = 0.0f;
    ops->yangle     = 0.0f;
    ops->w_z        = 0.0f;
    ops->tx_len     = 0;
    ops->recv_state = 0; // 初始状态：等待帧头1（0X0D）
    ops->parse_idx  = 0;
    memset(ops->rx_buf, 0, sizeof(ops->rx_buf));
    memset(ops->tx_buf, 0, sizeof(ops->tx_buf));
    memset(ops->posture.data, 0, sizeof(ops->posture.data));

    ops->gyro_yaw           = ops_config->yaw_car;
    ops->gyro_yaw_zeropoint = 0.0f;
    ops->Cx                 = 0.0f;
    ops->Cy                 = 0.0f;
    ops->yaw_car            = 0.0f;

    ops->x_offset   = ops_config->x_offset;
    ops->y_offset   = ops_config->y_offset;
    ops->yaw_offset = ops_config->yaw_offset;

    uint8_t dummy_buf = 0;
    HAL_UART_Receive_IT(ops->huart, &dummy_buf, 1);
}

/**
 * @brief  OPS串口发送数据
 * @param  ops: OPS设备句柄指针
 * @param  data: 发送数据指针
 * @param  len: 发送数据长度（字节）
 */
void OPS_SendData(OPS_t* ops, uint8_t* data, uint16_t len)
{
    // 检查USART是否忙
    if (ops->huart->gState == HAL_UART_STATE_READY)
    {
        // 复制数据到发送缓冲区
        if (len > sizeof(ops->tx_buf))
        {
            len = sizeof(ops->tx_buf);
        }
        memcpy(ops->tx_buf, data, len);
        ops->tx_len = len;
        HAL_UART_Transmit_IT(ops->huart, ops->tx_buf, len);
    }
}

/**
 * @brief  OPS校准命令（发送"ACTR"）
 * @param  ops: OPS设备句柄指针
 */
void OPS_Calibration(OPS_t* ops)
{
    char calib_cmd[4] = "ACTR";
    OPS_SendData(ops, calib_cmd, sizeof(calib_cmd));
}

/**
 * @brief  OPS清零命令（发送"ACT0"）
 * @param  ops: OPS设备句柄指针
 * @note   清零后角度/坐标重置为0
 */
void OPS_ZeroClearing(OPS_t* ops)
{
    uint8_t zero_cmd[] = "ACT0"; // 清零命令
    OPS_SendData(ops, zero_cmd, sizeof(zero_cmd));
}

/**
 * @brief  OPS更新航向角（发送"ACTJ"+4字节float）
 * @param  ops: OPS设备句柄指针
 * @param  angle: 目标航向角（范围-180~180度，float类型）
 */
void OPS_UpdateYaw(OPS_t* ops, float angle)
{
    uint8_t cmd[8] = "ACTJ";
    union
    {
        float   val;
        uint8_t data[4];
    } float_conv;

    float_conv.val = angle;
    memcpy(&cmd[4], float_conv.data, 4); // 拼接命令头和二进制数据
    OPS_SendData(ops, cmd, sizeof(cmd));
}

/**
 * @brief  OPS更新X坐标（发送"ACTX"+4字节float）
 * @param  ops: OPS设备句柄指针
 * @param  posx: 目标X坐标（单位m，float类型）
 */
void OPS_UpdateX(OPS_t* ops, float posx)
{
    uint8_t cmd[8] = "ACTX";
    union
    {
        float   val;
        uint8_t data[4];
    } float_conv;

    float_conv.val = posx * 1000.0f;
    memcpy(&cmd[4], float_conv.data, 4);
    OPS_SendData(ops, cmd, sizeof(cmd));
}

/**
 * @brief  OPS更新Y坐标（发送"ACTY"+4字节float）
 * @param  ops: OPS设备句柄指针
 * @param  posy: 目标Y坐标（单位m，float类型）
 */
void OPS_UpdateY(OPS_t* ops, float posy)
{
    uint8_t cmd[8] = "ACTY";
    union
    {
        float   val;
        uint8_t data[4];
    } float_conv;

    float_conv.val = posy * 1000.0f;
    memcpy(&cmd[4], float_conv.data, 4);
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
    uint8_t cmd[12] = "ACTD";
    union
    {
        float   val[2];
        uint8_t data[8];
    } float_conv;

    float_conv.val[0] = posx * 1000.0f;
    float_conv.val[1] = posy * 1000.0f;
    memcpy(&cmd[4], float_conv.data, 8);
    OPS_SendData(ops, cmd, sizeof(cmd));
}

/**
 * @brief  OPS数据帧解析
 * @param  ops: OPS设备句柄指针
 * @param  data: 单次接收的1字节数据
 * @frame_format: 0X0D(帧头1) + 0X0A(帧头2) + 24字节数据 + 0X0A(帧尾1) + 0X0D(帧尾2)
 */
void OPS_ParseData(OPS_t* ops, uint8_t data)
{
    switch (ops->recv_state)
    {
    case 0: // 等待帧头1（0X0D）
        if (data == 0X0D)
        {
            ops->recv_state = 1;
            ops->rx_buf[0]  = data;
        }
        break;

    case 1: // 等待帧头2（0X0A）
        if (data == 0X0A)
        {
            ops->recv_state = 2;
            ops->rx_buf[1]  = data;
            ops->parse_idx  = 0; // 重置数据解析索引
        }
        else
        {
            ops->recv_state = 0; // 帧错误，重置状态机
        }
        break;

    case 2: // 接收24字节数据段（角度+坐标+角速度）
        ops->posture.data[ops->parse_idx++] = data;
        ops->rx_buf[2 + ops->parse_idx - 1] = data;
        if (ops->parse_idx >= 24)
        {
            ops->recv_state = 3; // 数据段接收完成，等待帧尾1
        }
        break;

    case 3: // 等待帧尾1（0X0A）
        if (data == 0X0A)
        {
            ops->recv_state = 4;
            ops->rx_buf[26] = data;
        }
        else
        {
            ops->recv_state = 0; // 帧错误，重置
        }
        break;

    case 4: // 等待帧尾2（0X0D），帧解析完成
        if (data == 0X0D)
        {
            ops->recv_state = 0;
            ops->rx_buf[27] = data;
            // 解析24字节数据为6个float
            ops->zangle = ops->posture.val[0];
            ops->xangle = ops->posture.val[1];
            ops->yangle = ops->posture.val[2];
            ops->pos_y  = -ops->posture.val[3] * 1e-3f;
            ops->pos_x  = ops->posture.val[4] * 1e-3f;
            ops->w_z    = ops->posture.val[5];

            // 航向角范围修正
            if (ops->zangle < -135.0f)
            {
                ops->zangle += 360.0f;
            }

            CarCenterPose_Calc(ops);
        }
        else
        {
            ops->recv_state = 0; // 帧错误，重置
        }
        break;

    default:
        ops->recv_state = 0;
        break;
    }
}

/**
 * @brief  OPS USART接收完成回调（需在HAL_UART_RxCpltCallback中调用）
 * @param  ops: OPS设备句柄指针
 */
void OPS_HAL_UART_RxCpltCallback(OPS_t* ops)
{
    uint8_t recv_data = *(ops->huart->pRxBuffPtr - 1);
    OPS_ParseData(ops, recv_data);

    uint8_t dummy_buf = 0;
    HAL_UART_Receive_IT(ops->huart, &dummy_buf, 1);
}

/**
 * @brief  世界坐标系重置函数（清零pos_x、pos_y和陀螺仪偏航角）
 * @param  ops: OPS设备句柄指针
 */
void OPS_WorldCoord_Reset(OPS_t* ops)
{
    OPS_ZeroClearing(ops);
    ops->gyro_yaw_zeropoint = *ops->gyro_yaw - ops->gyro_yaw_zeropoint;
}

/**
 * @brief  解算车体中心位姿
 * @param  ops: OPS位姿数据指针
 */
void CarCenterPose_Calc(OPS_t* ops)
{
    float ops_x_world = ops->pos_x - ops->x_offset;
    float ops_y_world = ops->pos_y - ops->y_offset;
    float yaw_car_raw = *ops->gyro_yaw - ops->yaw_offset - ops->gyro_yaw_zeropoint;

    // 角度范围修正
    yaw_car_raw = fmodf(yaw_car_raw + 180.0f, 360.0f) - 180.0f;

    // 计算OPS相对于世界坐标系的实际偏航角（车体角+安装角偏移）
    float yaw_total = yaw_car_raw + ops->yaw_offset;
    float yaw_rad   = yaw_total * 3.1415926f / 180.0f;

    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    // 计算OPS相对于车体中心的世界坐标偏移（rx, ry）
    float rx = ops->x_offset * 1e-3f * cos_yaw + ops->y_offset * 1e-3f * sin_yaw;
    float ry = ops->x_offset * 1e-3f * sin_yaw - ops->y_offset * 1e-3f * cos_yaw;

    // 解算车体中心的世界坐标（Cx, Cy）
    ops->Cx      = ops_x_world - rx;
    ops->Cy      = ops_y_world - ry;
    ops->yaw_car = yaw_car_raw;
}
