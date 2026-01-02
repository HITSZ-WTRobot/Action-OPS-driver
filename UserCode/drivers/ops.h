/**
 * @file    ops.h
 * @authors mogegee syhanjin
 * @date    2025-12-24
 * @brief   ops全方位平面定位系统
 */

#ifndef __OPS9_H
#define __OPS9_H
#include "cmsis_os2.h"
#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define DEG2RAD(__DEG__) ((__DEG__) * 3.14159265358979323846f / 180.0f)

#define OPS_HEAD1 (0x0D)
#define OPS_HEAD2 (0x0A)

#define OPS_TAIL1 (0x0A)
#define OPS_TAIL2 (0x0D)

#define OPS_FRAME_LEN (28)

typedef enum
{
    OPS_SYNC_WAIT_HEAD1 = 0U,
    OPS_SYNC_WAIT_HEAD2,
    OPS_SYNC_RECEIVING,
    OPS_SYNC_DMA_ACTIVE,
} OPS_Sync_State_t;

/**
 * @brief OPS设备结构体
 *
 * @attention 需要开启 RX DMA 并设置为循环模式
 */
typedef struct
{
    // 发送锁
    osMutexId_t lock;

    UART_HandleTypeDef* huart;

    OPS_Sync_State_t sync_state; ///< 接收状态

    struct
    {
        float pos_x;  // X坐标（单位：m）
        float pos_y;  // Y坐标（单位：m）
        float zangle; // Z轴角度（航向角，单位：度）
        float xangle; // X轴角度（单位：度）
        float yangle; // Y轴角度（单位：度）
        float w_z;    // Z轴角速度（单位：dps）
    } feedback;

    float x_offset; // X轴偏移（m，OPS在车体中心前方为正）
    float y_offset; // Y轴偏移（m，OPS在车体中心左侧为正）
    float cos_yaw_offset;
    float sin_yaw_offset;

    float* gyro_yaw;                // 车体中心偏航角（度，逆时针为正，由陀螺仪获得）
    float  gyro_yaw_body_zeropoint; // Body yaw 零点

    float Cx;      // 车体相对世界坐标系位姿x（单位：m）
    float Cy;      // 车体相对世界坐标系位姿y（单位：m）
    float yaw_car; // 车体相对世界坐标系位姿yaw（单位：度）

    uint8_t rx_buffer[OPS_FRAME_LEN];

#ifdef DEBUG
    uint32_t sync_count;  ///< 同步计数
    uint32_t frame_count; ///< 帧数
    uint32_t error_count; ///< 错误帧数
#endif
} OPS_t;

typedef struct
{
    UART_HandleTypeDef* huart;
    float               x_offset;   /// X轴偏移（mm，OPS在车体中心前方为正）
    float               y_offset;   /// Y轴偏移（mm，OPS在车体中心左侧为正）
    float               yaw_offset; /// 初始角度偏移（度，逆时针为正）
    float*              yaw_car;    /// 车体中心偏航角（度，逆时针为正，由陀螺仪获得）
} OPS_config_t;

void OPS_Init(OPS_t* ops, OPS_config_t* ops_config); // 初始化
// void OPS_Calibration(OPS_t* ops);                      // 校准（发送"ACTR"命令）

void OPS_ZeroClearing(OPS_t* ops);                     // 清零（发送"ACT0"命令）
void OPS_UpdateYaw(const OPS_t* ops, float angle);     // 更新航向角（ACTJ命令）
void OPS_UpdateX(const OPS_t* ops, float posx);        // 更新X坐标（ACTX命令）
void OPS_UpdateY(const OPS_t* ops, float posy);        // 更新Y坐标（ACTY命令）
void OPS_UpdateXY(OPS_t* ops, float posx, float posy); // 更新XY坐标（ACTD命令）

void OPS_WorldCoord_Reset(OPS_t* ops); // 重置坐标系

void OPS_RxCpltCallback(OPS_t* ops); // 中断处理
void OPS_RxErrorHandler(OPS_t* ops); // 接收错误处理

static float* OPS_GetBodyXPtr(OPS_t* ops)
{
    return &ops->Cx;
}

static float* OPS_GetBodyYPtr(OPS_t* ops)
{
    return &ops->Cy;
}

static float* OPS_GetBodyYawPtr(OPS_t* ops)
{
    return &ops->yaw_car;
}

#ifdef __cplusplus
}
#endif

#endif