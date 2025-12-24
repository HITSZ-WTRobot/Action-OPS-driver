/**
 * @file    ops.h
 * @author  mogegee
 * @date    2025-12-24
 * @brief   ops全方位平面定位系统
 */

#ifndef __OPS9_H
#define __OPS9_H
#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief OPS设备结构体
 */
typedef struct
{
    UART_HandleTypeDef* huart;

    float pos_x;  // X坐标（单位：m）
    float pos_y;  // Y坐标（单位：m）
    float zangle; // Z轴角度（航向角，单位：度）
    float xangle; // X轴角度（单位：度）
    float yangle; // Y轴角度（单位：度）
    float w_z;    // Z轴角速度（单位：dps）

    uint8_t  rx_buf[50];  // 接收缓冲区
    uint8_t  tx_buf[256]; // 发送缓冲区
    uint16_t tx_len;      // 待发送数据长度
    uint8_t  recv_state;  // 接收状态机（0-4）
    uint8_t  parse_idx;   // 数据解析索引

    float x_offset;   // X轴偏移（mm，OPS在车体中心前方为正）
    float y_offset;   // Y轴偏移（mm，OPS在车体中心左侧为正）
    float yaw_offset; // 初始角度偏移（度，逆时针为正）

    float* gyro_yaw;           // 车体中心偏航角（度，逆时针为正，由陀螺仪获得）
    float  gyro_yaw_zeropoint; // 记录零点

    float Cx;      // 车体相对世界坐标系位姿x（单位：m）
    float Cy;      // 车体相对世界坐标系位姿y（单位：m）
    float yaw_car; // 车体相对世界坐标系位姿yaw（单位：度）

    union
    {
        uint8_t data[24];
        float   val[6];
    } posture;
} OPS_t;

typedef struct
{
    float  x_offset;   // X轴偏移（mm，OPS在车体中心前方为正）
    float  y_offset;   // Y轴偏移（mm，OPS在车体中心左侧为正）
    float  yaw_offset; // 初始角度偏移（度，逆时针为正）
    float* yaw_car;    // 车体中心偏航角（度，逆时针为正，由陀螺仪获得）
} OPS_config_t;

void OPS_Init(OPS_t* ops, OPS_config_t* ops_config);   // 初始化
void OPS_Calibration(OPS_t* ops);                      // 校准（发送"ACTR"命令）
void OPS_ZeroClearing(OPS_t* ops);                     // 清零（发送"ACT0"命令）
void OPS_UpdateYaw(OPS_t* ops, float angle);           // 更新航向角（ACTJ命令）
void OPS_UpdateX(OPS_t* ops, float posx);              // 更新X坐标（ACTX命令）
void OPS_UpdateY(OPS_t* ops, float posy);              // 更新Y坐标（ACTY命令）
void OPS_UpdateXY(OPS_t* ops, float posx, float posy); // 更新XY坐标（ACTD命令）
void OPS_HAL_UART_RxCpltCallback(OPS_t* ops);          // 中断处理
void OPS_WorldCoord_Reset(OPS_t* ops);                 // 重置坐标系

#ifdef __cplusplus
}
#endif

#endif