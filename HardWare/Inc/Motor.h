/**
 ******************************************************************************
 * @file    Motor.h
 * @brief   张大头42步进电机 DMA串口驱动
 * @note    基于 USART1 + DMA (IDLE中断) 实现一控四
 *          修改硬件接线只需修改下方"硬件配置宏"区域
 ******************************************************************************
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "dma.h"

/* ======================================================================== */
/*                       硬件配置宏 (修改接线只需改这里)                       */
/* ======================================================================== */

/** @brief 串口句柄 */
#define MOTOR_UART              (&huart1)

/** @brief DMA 句柄 */
#define MOTOR_DMA_TX            (&hdma_usart1_tx)
#define MOTOR_DMA_RX            (&hdma_usart1_rx)

/** @brief 是否使用 RS485 方向控制 (0=TTL直连, 1=RS485需DE引脚) */
#define MOTOR_USE_RS485_DE      0

#if MOTOR_USE_RS485_DE
/* RS485 方向控制引脚配置 */
#define MOTOR_RS485_DE_PORT     GPIOB
#define MOTOR_RS485_DE_PIN      GPIO_PIN_5
#define MOTOR_RS485_SET_TX()    HAL_GPIO_WritePin(MOTOR_RS485_DE_PORT, MOTOR_RS485_DE_PIN, GPIO_PIN_SET)
#define MOTOR_RS485_SET_RX()    HAL_GPIO_WritePin(MOTOR_RS485_DE_PORT, MOTOR_RS485_DE_PIN, GPIO_PIN_RESET)
#else
/* TTL直连,无需方向控制 */
#define MOTOR_RS485_SET_TX()    ((void)0)
#define MOTOR_RS485_SET_RX()    ((void)0)
#endif

/** @brief 校验模式: 0=固定0x6B, 1=XOR异或, 2=CRC-8 */
#define MOTOR_CHECKSUM_MODE     0

/* ======================================================================== */
/*                           电机地址与缓冲区配置                              */
/* ======================================================================== */

/** @brief 电机地址 */
#define MOTOR_ADDR_1            0x01
#define MOTOR_ADDR_2            0x02
#define MOTOR_ADDR_3            0x03
#define MOTOR_ADDR_4            0x04
#define MOTOR_ADDR_BROADCAST    0x00

/** @brief 电机数量 */
#define MOTOR_MAX_COUNT         4

/** @brief TX 最大帧长 (地址+功能码+数据+校验, 最坏情况位置模式=1+1+10+1=13) */
#define MOTOR_TX_FRAME_MAX      13

/** @brief RX DMA 接收缓冲区大小 */
#define MOTOR_RX_BUF_SIZE       32

/** @brief 电机响应超时(ms) */
#define MOTOR_TIMEOUT_MS        50

/* ======================================================================== */
/*                           电机ID索引                                       */
/* ======================================================================== */

#define MOTOR_ID_1              0
#define MOTOR_ID_2              1
#define MOTOR_ID_3              2
#define MOTOR_ID_4              3

/* ======================================================================== */
/*                           枚举类型定义                                     */
/* ======================================================================== */

/** @brief 电机状态 */
typedef enum {
    MOTOR_STATE_IDLE    = 0,    /* 空闲,可接收新命令 */
    MOTOR_STATE_BUSY    = 1,    /* 正在等待响应 */
    MOTOR_STATE_ERROR   = 2     /* 上次操作失败 */
} Motor_State_t;

/** @brief 错误码 */
typedef enum {
    MOTOR_OK                =  0,   /* 操作成功 */
    MOTOR_ERROR_TIMEOUT     = -1,   /* 响应超时 */
    MOTOR_ERROR_CHECKSUM    = -2,   /* 校验错误 */
    MOTOR_ERROR_MOTOR_FAULT = -3,   /* 电机返回 0xEE 错误 */
    MOTOR_ERROR_DMA_BUSY    = -4,   /* DMA 忙 */
    MOTOR_ERROR_INVALID_PARAM = -5, /* 参数无效 */
    MOTOR_ERROR_BUSY        = -6    /* 有其他命令正在执行 */
} Motor_Error_t;

/* ======================================================================== */
/*                           参数结构体                                       */
/* ======================================================================== */

/** @brief 速度模式参数 */
typedef struct {
    uint8_t  direction;     /* 方向: 0=CCW, 1=CW */
    uint16_t speed;         /* 速度值 (含义取决于微步细分, 典型为RPM*10或脉冲/秒) */
    uint8_t  acceleration;  /* 加速度档位 (1-10) */
    uint8_t  sync_flag;     /* 0=立即执行, 1=等待同步启动命令0xFF */
} Motor_SpeedParams_t;

/** @brief 位置模式参数 */
typedef struct {
    uint8_t  direction;     /* 方向: 0=CCW, 1=CW */
    uint16_t speed;         /* 速度值 */
    uint8_t  acceleration;  /* 加速度档位 (1-10) */
    uint32_t pulse_count;   /* 脉冲数 (4字节) */
    uint8_t  mode;          /* 模式: 0=相对位置, 1=绝对位置 */
    uint8_t  sync_flag;     /* 0=立即执行, 1=等待同步启动命令0xFF */
} Motor_PositionParams_t;

/* ======================================================================== */
/*                           电机上下文                                       */
/* ======================================================================== */

/** @brief 单电机上下文 */
typedef struct {
    uint8_t           address;            /* 电机地址 (1-4) */
    Motor_State_t     state;              /* 当前状态 */
    Motor_Error_t     last_error;         /* 最后一次错误码 */

    uint8_t           enabled;            /* 使能状态: 0=失能, 1=使能 */

    /* 保存上次设定的参数 (便于在急停后恢复) */
    Motor_SpeedParams_t    last_speed;
    Motor_PositionParams_t last_position;
} Motor_Context_t;

/** @brief 全局电机管理器 (单例) */
typedef struct {
    Motor_Context_t   motors[MOTOR_MAX_COUNT];  /* 4个电机上下文 */

    /* DMA RX 缓冲区 */
    uint8_t           rx_dma_buf[MOTOR_RX_BUF_SIZE];
    volatile uint16_t rx_length;          /* 本次DMA接收到的字节数 */

    /* 通信状态机 */
    volatile uint8_t  tx_busy;            /* 1=DMA TX正在进行中或等待响应 */
    int8_t            current_motor_id;    /* 当前命令对应的电机ID (-1=无) */
    uint8_t           is_broadcast;        /* 当前命令是否为广播 */
    uint32_t          operation_start_tick; /* 命令发送时刻的 tick */
} Motor_Manager_t;

/* ======================================================================== */
/*                           公开 API                                        */
/* ======================================================================== */

/** @brief 初始化电机驱动 (启动DMA-IDLE接收) */
void Motor_Init(void);

/** @brief 主循环轮询 (处理超时,需在while(1)中周期性调用) */
void Motor_Process(void);

/* ---- 电机使能 ---- */

/**
 * @brief 使能/失能电机
 * @param motor_id  电机ID (MOTOR_ID_1 ~ MOTOR_ID_4)
 * @param enable    1=使能(上电锁定), 0=失能(释放)
 * @return MOTOR_OK 或错误码
 */
Motor_Error_t Motor_Enable(uint8_t motor_id, uint8_t enable);

/* ---- 急停 ---- */

/** @brief 单个电机急停 */
Motor_Error_t Motor_EmergencyStop(uint8_t motor_id);

/** @brief 全部电机急停 (广播) */
Motor_Error_t Motor_EmergencyStopAll(void);

/* ---- 运动控制 ---- */

/** @brief 速度模式控制 */
Motor_Error_t Motor_SpeedControl(uint8_t motor_id, const Motor_SpeedParams_t *params);

/** @brief 位置(脉冲)模式控制 */
Motor_Error_t Motor_PositionControl(uint8_t motor_id, const Motor_PositionParams_t *params);

/* ---- 同步启动 ---- */

/** @brief 触发单个电机同步启动 (发送0xFF到指定地址) */
Motor_Error_t Motor_SyncStart(uint8_t motor_id);

/** @brief 触发所有电机同步启动 (广播0xFF) */
Motor_Error_t Motor_SyncStartAll(void);

/* ---- 状态查询 ---- */

/** @brief 查询电机是否忙 */
uint8_t Motor_IsBusy(uint8_t motor_id);

/** @brief 获取电机状态 */
Motor_State_t Motor_GetState(uint8_t motor_id);

/** @brief 获取最后的错误码 */
Motor_Error_t Motor_GetLastError(uint8_t motor_id);

/* ---- 测试函数 (由 test_config.h 宏控制是否编译, 不影响正式代码) ---- */

/**
 * @brief 电机自测函数 — 依次测试使能/速度/位置/急停
 * @note  由 test_runner.c 在 Test_Runner() 中调用
 *        受 test_config.h 中 TEST_MOTOR_ENABLE 宏控制
 *        关闭测试开关后, 此函数不参与编译, 不占 Flash
 */
void Motor_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
