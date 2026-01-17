/**
 * @file    ops.h
 * @authors mogegee syhanjin
 * @date    2025-12-24
 * @brief   ops全方位平面定位系统
 *
 * 简述本驱动解算逻辑：
 *
 * 坐标系定义：
 *    - W   世界坐标系：本驱动认为的世界坐标系，同时也是返回值的参考坐标系
 *    - B   车身坐标系：固定在车体上的坐标系，原点在底盘集合中心，车体正前方为 x，左侧为 y
 *    - OW  码盘世界坐标系：Action 的码盘认为的世界坐标系
 *    - O   码盘坐标系：Action 码盘手册所述的坐标系
 *    - Now 重定位坐标系：重定位时刻的 B
 * 符号定义：
 *    T_{X}_{Y} 为坐标系 X 到坐标系 Y 的齐次变换矩阵.
 *    R 为 T = [R p; 0 1] 中的旋转矩阵，p 为平移向量.
 *
 * 初始化时我们会传入 O 与 B 的相对位置，由于码盘是固定在车体的，所以该位置不变，
 * 定义变换矩阵 T0 = T_O_B 为 O 到 B 的变换矩阵。
 *
 * 现在解释重置坐标系的变换原理（ W = B ）：
 * 向码盘发送重置命令，此时 OW 变为 O，同时 W 重置为 B，于是有变换关系 T_OW_W = T0,
 * 码盘将会返回数据 P 为 O 在 OW 中的位置，由陀螺仪可以得出 O 在 OW 时的 yaw，
 * 即：反馈量为 O 到 OW 的变换 Tf = T_O_OW，目标量为 B 到 W 的变换 T = T_B_W.
 * 所以有 T = T_OW_W * Tf * T_O_B^(-1) = T0 * Tf * T0^(-1).
 * 另外 T = [ R p ;
 *           0 1 ], 我们只需要 p
 * 拆分可得 p = R0 * pf + (I - R0 * Rf * R0^(-1)) * p0;
 * 由于二维情况下 RA * RB = RB * RA，所以可以化简为 p = R0 * pf + (I - Rf) * p0;
 * 得到最终结果：
 *
 *          p = p0 + R0 * pf - Rf * p0;
 *
 * 现在解释以当前位置为反馈进行重定位解算的原理 （ T_B_W = T_Now_W = Tn ）：
 * 本情况与上面的区别在于 T_OW_W != T0, 而是满足 T_OW_W = T_Now_W * T_OW_Now，
 * 由于 T_OW_Now = T0，有 T_OW_W = Tn * T0;
 * 此时带入到上面过程有 T = Tn * T0 * Tf * T0^(-1);
 * 所以得到这种情况下结果就是对上面的 p 再做一次 Tn 变换，
 * 可以得到 p = Rn * R0 * pf + Rn * (I - R0 * Rf * R0^(-1)) * p0 + pn
 *          = (Rn * p0 + pn) + (Rn * R0) * pf - Rn * Rf * p0
 *          = (Rn * p0 + pn) + (Rn * R0) * pf - Rf * Rn * p0
 *          = p_offset + R_base * pf - Rf * p_base;
 * 其中： p_base     = Rn * p0,
 *       R_base     = Rn * R0 = R(theta_n + theta_0), theta_base = theta_n + theta_0.
 *       p_offset   = p_base + pn,
 *
 * 所以最后的更新计算函数可以统一为
 *
 *          p   = p_offset + R_base * pf - Rf * p_base;
 *          yaw = yaw_f + theta_n
 *
 */

#ifndef __OPS9_H
#define __OPS9_H
#include "cmsis_os2.h"
#include "main.h"
#include <stdbool.h>

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

    struct
    {
        float x;
        float y;
    } p_base, p_offset;

    struct
    {
        float cos;
        float sin;
    } R_base;

    float theta_offset;

    struct
    {
        float x;
        float y;
        float yaw;
    } setup;

    float* gyro_yaw;    // 车体中心偏航角（度，逆时针为正，由陀螺仪获得）
    float  gyro_offset; // Body yaw 零点

    float Cx;   // 车体相对世界坐标系位姿x（单位：m）
    float Cy;   // 车体相对世界坐标系位姿y（单位：m）
    float Cyaw; // 车体相对世界坐标系位姿yaw（单位：度）

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
void OPS_ResetByWorldPose(OPS_t* ops, float x, float y, float yaw);

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
    return &ops->Cyaw;
}

static bool OPS_isConnected(OPS_t* ops)
{
    return ops->sync_state == OPS_SYNC_DMA_ACTIVE;
}

#ifdef __cplusplus
}
#endif

#endif