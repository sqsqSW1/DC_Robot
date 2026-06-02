/**
 ******************************************************************************
 * @file    Motor.c
 * @brief   张大头42步进电机 DMA串口驱动实现
 * @note    使用 HAL_UARTEx_ReceiveToIdle_DMA 实现变长帧接收
 *          通过 weak 回调覆盖实现中断处理,不修改 CubeMX 生成的文件
 ******************************************************************************
 */

#include "Motor.h"
#include <string.h>
#include "test_config.h"   /* 测试开关 — 控制 Motor_Test() 是否参与编译 */

/* ======================================================================== */
/*                           模块内部宏定义                                   */
/* ======================================================================== */

/** @brief 急停命令的功能码和数据 */
#define CMD_FUNC_STOP           0xFE
#define CMD_DATA_STOP           0x98

/** @brief 同步启动的功能码 */
#define CMD_FUNC_SYNC_START     0xFF

/** @brief 电机响应成功码 */
#define MOTOR_RESP_OK           0x02

/** @brief 电机响应错误码 */
#define MOTOR_RESP_FAULT        0xEE

/** @brief 指令最短响应帧长度 (地址+响应码+校验 = 3字节) */
#define RESPONSE_MIN_LEN        3

/* ======================================================================== */
/*                           模块内部静态变量                                 */
/* ======================================================================== */

/** @brief 全局电机管理器单例 */
static Motor_Manager_t g_motor_mgr;

/** @brief 管理器简写指针 (内部使用) */
#define MGR  (&g_motor_mgr)

/* ======================================================================== */
/*                           内部函数声明                                     */
/* ======================================================================== */

static uint8_t       Motor_ComputeChecksum(const uint8_t *data, uint8_t len);
static Motor_Error_t Motor_SendFrame(uint8_t address, uint8_t func_code,
                                     const uint8_t *data, uint8_t data_len,
                                     uint8_t is_broadcast);
static void          Motor_ParseResponse(void);
static void          Motor_CompleteOperation(Motor_Error_t result);
static void          Motor_ReArmRxDma(void);

/* ======================================================================== */
/*                           校验计算                                         */
/* ======================================================================== */

/**
 * @brief 计算校验字节
 * @param data  数据帧 (包含地址和功能码, 不包含校验字节本身)
 * @param len   数据长度
 * @return 校验字节
 */
static uint8_t Motor_ComputeChecksum(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;

#if MOTOR_CHECKSUM_MODE == 0
    /* 固定 0x6B 校验 (出厂默认) */
    return 0x6B;
#elif MOTOR_CHECKSUM_MODE == 1
    /* XOR 异或校验 */
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
#elif MOTOR_CHECKSUM_MODE == 2
    /* CRC-8 校验 (多项式 0x07) */
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
#else
    #error "MOTOR_CHECKSUM_MODE must be 0, 1, or 2"
#endif
}

/* ======================================================================== */
/*                           DMA 重新武装                                    */
/* ======================================================================== */

/**
 * @brief 重新启动 DMA-IDLE 接收 (用于回调中重新武装)
 * @note  在 RX IDLE 事件后或接收异常中止后调用
 */
static void Motor_ReArmRxDma(void)
{
    HAL_UART_AbortReceive(MOTOR_UART);
    HAL_UARTEx_ReceiveToIdle_DMA(MOTOR_UART, MGR->rx_dma_buf, MOTOR_RX_BUF_SIZE);
    /* 注意: 这里忽略返回值,因为HAL_UART_AbortReceive可能返回HAL_ERROR如果DMA已经停止 */
}

/* ======================================================================== */
/*                           帧发送                                           */
/* ======================================================================== */

/**
 * @brief 组装帧并启动 DMA 发送
 * @param address      电机地址 (0x01-0x04 或 0x00广播)
 * @param func_code    功能码
 * @param data         数据区 (可为NULL, data_len=0)
 * @param data_len     数据长度
 * @param is_broadcast 是否为广播 (1=广播,无响应等待)
 * @return MOTOR_OK 或错误码
 */
static Motor_Error_t Motor_SendFrame(uint8_t address, uint8_t func_code,
                                     const uint8_t *data, uint8_t data_len,
                                     uint8_t is_broadcast)
{
    /* 检查 DMA 是否空闲 */
    if (MGR->tx_busy) {
        return MOTOR_ERROR_BUSY;
    }

    /* 组帧 */
    uint8_t frame[MOTOR_TX_FRAME_MAX];
    uint8_t frame_idx = 0;

    frame[frame_idx++] = address;
    frame[frame_idx++] = func_code;

    if (data != NULL && data_len > 0) {
        for (uint8_t i = 0; i < data_len; i++) {
            frame[frame_idx++] = data[i];
        }
    }

    /* 计算并附加校验字节 */
    frame[frame_idx] = Motor_ComputeChecksum(frame, frame_idx);
    uint8_t total_len = frame_idx + 1;

    /* 标记状态机 */
    MGR->tx_busy = 1;
    MGR->is_broadcast = is_broadcast;

    /* RS485 切换到发送模式 (TTL下为空操作) */
    MOTOR_RS485_SET_TX();

    /* 启动 DMA 发送 */
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(MOTOR_UART, frame, total_len);
    if (status != HAL_OK) {
        MGR->tx_busy = 0;
        MOTOR_RS485_SET_RX();
        return MOTOR_ERROR_DMA_BUSY;
    }

    return MOTOR_OK;
}

/* ======================================================================== */
/*                           响应解析                                         */
/* ======================================================================== */

/**
 * @brief 解析电机响应帧
 * @note  在 HAL_UARTEx_RxEventCallback 中调用 (中断上下文)
 *        保持轻量,只做校验和状态更新,不做耗时操作
 */
static void Motor_ParseResponse(void)
{
    int8_t motor_id = MGR->current_motor_id;

    /* 无等待中的命令则忽略 */
    if (motor_id < 0) {
        return;
    }

    /* 检查响应长度 */
    if (MGR->rx_length < RESPONSE_MIN_LEN) {
        Motor_CompleteOperation(MOTOR_ERROR_CHECKSUM);
        return;
    }

    uint8_t *buf = MGR->rx_dma_buf;

    /* 验证地址是否匹配 */
    uint8_t resp_addr = buf[0];
    Motor_Context_t *motor = &MGR->motors[motor_id];
    if (resp_addr != motor->address) {
        /* 地址不匹配,可能是噪声或其他电机的回复,忽略 */
        return;
    }

    /* 验证校验字节 */
    uint8_t resp_chk   = buf[MGR->rx_length - 1];
    uint8_t calc_chk   = Motor_ComputeChecksum(buf, MGR->rx_length - 1);

    if (resp_chk != calc_chk) {
        Motor_CompleteOperation(MOTOR_ERROR_CHECKSUM);
        return;
    }

    /* 解析响应码 */
    uint8_t resp_code = buf[1];

    if (resp_code == MOTOR_RESP_OK) {
        /* 操作成功 */
        Motor_CompleteOperation(MOTOR_OK);
    } else if (resp_code == MOTOR_RESP_FAULT) {
        /* 电机返回故障 */
        Motor_CompleteOperation(MOTOR_ERROR_MOTOR_FAULT);
    } else {
        /* 未知响应码,按故障处理 */
        Motor_CompleteOperation(MOTOR_ERROR_MOTOR_FAULT);
    }
}

/* ======================================================================== */
/*                           操作完成清理                                     */
/* ======================================================================== */

/**
 * @brief 完成当前操作,恢复状态机到空闲
 * @param result  操作结果
 */
static void Motor_CompleteOperation(Motor_Error_t result)
{
    int8_t motor_id = MGR->current_motor_id;

    if (motor_id >= 0) {
        Motor_Context_t *motor = &MGR->motors[motor_id];
        motor->state = (result == MOTOR_OK) ? MOTOR_STATE_IDLE : MOTOR_STATE_ERROR;
        if (result != MOTOR_OK) {
            motor->last_error = result;
        }
    }

    /* 清空状态机 */
    MGR->tx_busy = 0;
    MGR->current_motor_id = -1;
    MGR->is_broadcast = 0;
}

/* ======================================================================== */
/*                       HAL 回调函数 (覆盖 weak)                             */
/* ======================================================================== */

/**
 * @brief DMA 发送完成回调
 * @note  TX完成后: 1)切换RS485到接收模式 2)记录起始tick 3)广播命令直接完成
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != MOTOR_UART) {
        return;
    }

    /* 切回接收模式 */
    MOTOR_RS485_SET_RX();

    /* 记录操作起始时刻 (用于超时检测) */
    MGR->operation_start_tick = HAL_GetTick();

    /* 广播命令不等待响应,直接标记完成 */
    if (MGR->is_broadcast) {
        Motor_CompleteOperation(MOTOR_OK);
    }
    /* 单播命令: 不做额外操作,等待 RX IDLE 回调触发响应解析 */
}

/**
 * @brief UART 接收事件回调 (IDLE / 半传输 / 传输完成)
 * @param huart UART句柄
 * @param Size  接收到的字节数
 * @note  HAL 在检测到 IDLE 事件后调用此函数,Size 为实际接收字节数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != MOTOR_UART) {
        return;
    }

    /* 仅处理 IDLE 事件 (忽略 TC 和 HT 事件) */
    if (HAL_UARTEx_GetRxEventType(huart) != HAL_UART_RXEVENT_IDLE) {
        return;
    }

    /* 存储接收长度,交由响应解析 */
    MGR->rx_length = Size;

    /* 解析响应 */
    Motor_ParseResponse();

    /* 重新武装 DMA-IDLE 接收,准备接收下一次响应 */
    Motor_ReArmRxDma();
}

/**
 * @brief UART 错误回调
 * @note  发生噪声/帧错误/溢出等,中止当前操作并重新启动接收
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != MOTOR_UART) {
        return;
    }

    /* 终止当前操作 */
    if (MGR->tx_busy) {
        Motor_CompleteOperation(MOTOR_ERROR_TIMEOUT);
    }

    /* 重新启动 DMA-IDLE 接收 */
    Motor_ReArmRxDma();
}

/* ======================================================================== */
/*                            公开 API 实现                                  */
/* ======================================================================== */

/**
 * @brief 初始化电机驱动
 * @note  必须在 MX_USART1_UART_Init() 之后调用
 */
void Motor_Init(void)
{
    /* 清零管理器 */
    memset(MGR, 0, sizeof(Motor_Manager_t));

    MGR->current_motor_id = -1;

    /* 初始化每个电机的地址和状态 */
    for (uint8_t i = 0; i < MOTOR_MAX_COUNT; i++) {
        MGR->motors[i].address = MOTOR_ADDR_1 + i;  /* 0x01, 0x02, 0x03, 0x04 */
        MGR->motors[i].state   = MOTOR_STATE_IDLE;
        MGR->motors[i].last_error = MOTOR_OK;
        MGR->motors[i].enabled = 0;
    }

    /* RS485 默认为接收模式 */
    MOTOR_RS485_SET_RX();

    /* 启动首次 DMA-IDLE 接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(MOTOR_UART, MGR->rx_dma_buf, MOTOR_RX_BUF_SIZE);
}

/**
 * @brief 主循环轮询 (处理超时)
 * @note  需在 main() 的 while(1) 循环中周期性调用
 *        执行时间 O(1),不会阻塞
 */
void Motor_Process(void)
{
    /* 无进行中的操作则直接返回 */
    if (!MGR->tx_busy) {
        return;
    }

    /* 广播命令不需要超时检测 (TX完成时已直接完成) */
    if (MGR->is_broadcast) {
        return;
    }

    /* 检查超时 */
    if ((HAL_GetTick() - MGR->operation_start_tick) > MOTOR_TIMEOUT_MS) {
        /* 超时: 终止接收并标记错误 */
        Motor_CompleteOperation(MOTOR_ERROR_TIMEOUT);

        /* 重新启动 DMA-IDLE 接收 */
        Motor_ReArmRxDma();
    }
}

/* ---- 参数校验宏 ---- */

/** @brief 检查 motor_id 是否有效,无效则返回错误 */
#define CHECK_MOTOR_ID(id) \
    do { \
        if ((id) >= MOTOR_MAX_COUNT) return MOTOR_ERROR_INVALID_PARAM; \
    } while(0)

/** @brief 检查总线是否忙,忙则返回错误 */
#define CHECK_NOT_BUSY() \
    do { \
        if (MGR->tx_busy) return MOTOR_ERROR_BUSY; \
    } while(0)

/** @brief 开始新操作: 设置当前电机ID和状态 */
#define BEGIN_OPERATION(id) \
    do { \
        MGR->current_motor_id = (int8_t)(id); \
        MGR->motors[(id)].state = MOTOR_STATE_BUSY; \
    } while(0)

/* ---- 电机使能 ---- */

Motor_Error_t Motor_Enable(uint8_t motor_id, uint8_t enable)
{
    CHECK_MOTOR_ID(motor_id);
    CHECK_NOT_BUSY();

    Motor_Context_t *motor = &MGR->motors[motor_id];
    BEGIN_OPERATION(motor_id);

    uint8_t data = enable ? 0x01 : 0x00;
    Motor_Error_t ret = Motor_SendFrame(motor->address, 0xF3, &data, 1, 0);

    if (ret == MOTOR_OK) {
        motor->enabled = enable;
    } else {
        MGR->current_motor_id = -1;
        motor->state = MOTOR_STATE_ERROR;
    }
    return ret;
}

/* ---- 急停 ---- */

Motor_Error_t Motor_EmergencyStop(uint8_t motor_id)
{
    CHECK_MOTOR_ID(motor_id);
    CHECK_NOT_BUSY();

    Motor_Context_t *motor = &MGR->motors[motor_id];
    BEGIN_OPERATION(motor_id);

    uint8_t data = CMD_DATA_STOP;
    Motor_Error_t ret = Motor_SendFrame(motor->address, CMD_FUNC_STOP, &data, 1, 0);

    if (ret != MOTOR_OK) {
        MGR->current_motor_id = -1;
        motor->state = MOTOR_STATE_ERROR;
    }
    return ret;
}

Motor_Error_t Motor_EmergencyStopAll(void)
{
    CHECK_NOT_BUSY();

    /* 广播急停: 不指定单个电机ID,不等待响应 */
    MGR->current_motor_id = -1;  /* 无对应单电机 */

    uint8_t data = CMD_DATA_STOP;
    Motor_Error_t ret = Motor_SendFrame(MOTOR_ADDR_BROADCAST, CMD_FUNC_STOP, &data, 1, 1);

    if (ret != MOTOR_OK) {
        return ret;
    }

    /* 广播命令在TX完成回调中直接标记完成,这里已经成功了 */
    /* 更新所有电机的状态 */
    for (uint8_t i = 0; i < MOTOR_MAX_COUNT; i++) {
        MGR->motors[i].state = MOTOR_STATE_IDLE;
    }
    return MOTOR_OK;
}

/* ---- 速度模式控制 ---- */

Motor_Error_t Motor_SpeedControl(uint8_t motor_id, const Motor_SpeedParams_t *params)
{
    CHECK_MOTOR_ID(motor_id);
    if (params == NULL) return MOTOR_ERROR_INVALID_PARAM;
    CHECK_NOT_BUSY();

    Motor_Context_t *motor = &MGR->motors[motor_id];
    BEGIN_OPERATION(motor_id);

    /* 组数据区: [方向 1B] [速度高字节] [速度低字节] [加速度 1B] [同步标志 1B] */
    uint8_t data[5];
    data[0] = params->direction;
    data[1] = (uint8_t)((params->speed >> 8) & 0xFF);
    data[2] = (uint8_t)((params->speed >> 0) & 0xFF);
    data[3] = params->acceleration;
    data[4] = params->sync_flag;

    Motor_Error_t ret = Motor_SendFrame(motor->address, 0xF6, data, 5, 0);

    if (ret == MOTOR_OK) {
        motor->last_speed = *params;
    } else {
        MGR->current_motor_id = -1;
        motor->state = MOTOR_STATE_ERROR;
    }
    return ret;
}

/* ---- 位置模式控制 ---- */

Motor_Error_t Motor_PositionControl(uint8_t motor_id, const Motor_PositionParams_t *params)
{
    CHECK_MOTOR_ID(motor_id);
    if (params == NULL) return MOTOR_ERROR_INVALID_PARAM;
    CHECK_NOT_BUSY();

    Motor_Context_t *motor = &MGR->motors[motor_id];
    BEGIN_OPERATION(motor_id);

    /* 组数据区:
     * [方向 1B] [速度高] [速度低] [加速度 1B]
     * [脉冲3] [脉冲2] [脉冲1] [脉冲0] (MSB first)
     * [模式 1B] [同步标志 1B]
     */
    uint8_t data[10];
    data[0] = params->direction;
    data[1] = (uint8_t)((params->speed >> 8) & 0xFF);
    data[2] = (uint8_t)((params->speed >> 0) & 0xFF);
    data[3] = params->acceleration;
    data[4] = (uint8_t)((params->pulse_count >> 24) & 0xFF);
    data[5] = (uint8_t)((params->pulse_count >> 16) & 0xFF);
    data[6] = (uint8_t)((params->pulse_count >> 8) & 0xFF);
    data[7] = (uint8_t)((params->pulse_count >> 0) & 0xFF);
    data[8] = params->mode;
    data[9] = params->sync_flag;

    Motor_Error_t ret = Motor_SendFrame(motor->address, 0xFD, data, 10, 0);

    if (ret == MOTOR_OK) {
        motor->last_position = *params;
    } else {
        MGR->current_motor_id = -1;
        motor->state = MOTOR_STATE_ERROR;
    }
    return ret;
}

/* ---- 同步启动 ---- */

Motor_Error_t Motor_SyncStart(uint8_t motor_id)
{
    CHECK_MOTOR_ID(motor_id);
    CHECK_NOT_BUSY();

    Motor_Context_t *motor = &MGR->motors[motor_id];
    BEGIN_OPERATION(motor_id);

    /* 同步启动命令无数据区 */
    Motor_Error_t ret = Motor_SendFrame(motor->address, CMD_FUNC_SYNC_START, NULL, 0, 0);

    if (ret != MOTOR_OK) {
        MGR->current_motor_id = -1;
        motor->state = MOTOR_STATE_ERROR;
    }
    return ret;
}

Motor_Error_t Motor_SyncStartAll(void)
{
    CHECK_NOT_BUSY();

    /* 广播同步启动,无数据区 */
    MGR->current_motor_id = -1;
    Motor_Error_t ret = Motor_SendFrame(MOTOR_ADDR_BROADCAST, CMD_FUNC_SYNC_START, NULL, 0, 1);

    if (ret != MOTOR_OK) {
        return ret;
    }

    /* 更新所有电机状态 */
    for (uint8_t i = 0; i < MOTOR_MAX_COUNT; i++) {
        MGR->motors[i].state = MOTOR_STATE_IDLE;
    }
    return MOTOR_OK;
}

/* ---- 状态查询 ---- */

uint8_t Motor_IsBusy(uint8_t motor_id)
{
    if (motor_id >= MOTOR_MAX_COUNT) return 0xFF;  /* 无效ID返回非零 */
    return (MGR->motors[motor_id].state == MOTOR_STATE_BUSY) ? 1 : 0;
}

Motor_State_t Motor_GetState(uint8_t motor_id)
{
    if (motor_id >= MOTOR_MAX_COUNT) return MOTOR_STATE_ERROR;
    return MGR->motors[motor_id].state;
}

Motor_Error_t Motor_GetLastError(uint8_t motor_id)
{
    if (motor_id >= MOTOR_MAX_COUNT) return MOTOR_ERROR_INVALID_PARAM;
    return MGR->motors[motor_id].last_error;
}

/* ======================================================================== */
/*                    电机自测函数 (编译开关控制)                              */
/* ======================================================================== */

#if TEST_MOTOR_ENABLE

/* ---- 测试辅助: 等待指定电机完成当前操作 ---- */

/**
 * @brief 阻塞等待电机操作完成 (带超时保护)
 * @param motor_id  电机ID
 * @param timeout_ms 超时时间(ms), 超过后放弃等待
 * @return 0=操作完成, 1=超时
 */
static uint8_t Motor_Test_WaitReady(uint8_t motor_id, uint32_t timeout_ms)
{
    uint32_t tick_start = HAL_GetTick();
    while (Motor_IsBusy(motor_id)) {
        /* 在主循环未启动时手动调用 HAL_Delay 以保持 SysTick 运行 */
        HAL_Delay(1);
        if ((HAL_GetTick() - tick_start) > timeout_ms) {
            return 1;   /* 超时 */
        }
    }
    return 0;           /* 完成 */
}

/**
 * @brief 电机自测入口
 * @note  测试流程: 使能 → 速度模式 → 位置模式 → 急停
 *        每个步骤都有超时保护, 单步失败不阻塞后续测试
 *        测试完成后电机处于失能释放状态
 */
void Motor_Test(void)
{
    /* ================================================================ */
    /*  Step 1: 使能测试                                                 */
    /*  发送使能命令到电机1, 等待响应, 检查是否成功                       */
    /* ================================================================ */

    Motor_Error_t err = Motor_Enable(MOTOR_ID_1, 1);
    if (err != MOTOR_OK) {
        /* 发送失败 (DMA忙或参数错误) — 跳过后续测试 */
        goto test_done;
    }
    if (Motor_Test_WaitReady(MOTOR_ID_1, 1000)) {
        /* 1秒内未收到响应 — 检查电机连接和地址 */
        goto test_done;
    }
    if (Motor_GetLastError(MOTOR_ID_1) != MOTOR_OK) {
        /* 电机响应了但返回错误 — 检查校验模式 */
        goto test_done;
    }
    /* 使能成功 — 电机应上电锁定, 手动转不动轴 */

    HAL_Delay(200);  /* 等待电机稳定 */

    /* ================================================================ */
    /*  Step 2: 速度模式测试                                             */
    /*  使电机1以低速运行, 验证方向/速度控制正常                          */
    /* ================================================================ */

    {
        Motor_SpeedParams_t spd = {
            .direction    = 0,          /* CCW 逆时针 */
            .speed        = 200,        /* 低速 (RPM*10 或 脉冲/秒) */
            .acceleration = 3,          /* 加速度档位 */
            .sync_flag    = 0           /* 立即执行 */
        };
        err = Motor_SpeedControl(MOTOR_ID_1, &spd);
        if (err != MOTOR_OK) goto test_done;
        if (Motor_Test_WaitReady(MOTOR_ID_1, 1000)) goto test_done;
        if (Motor_GetLastError(MOTOR_ID_1) != MOTOR_OK) goto test_done;
    }

    HAL_Delay(1000);  /* 电机旋转1秒, 肉眼观察方向是否正确 */

    /* ================================================================ */
    /*  Step 2b: 反转速度测试                                            */
    /* ================================================================ */

    {
        Motor_SpeedParams_t spd = {
            .direction    = 1,          /* CW 顺时针 — 应反向旋转 */
            .speed        = 200,
            .acceleration = 3,
            .sync_flag    = 0
        };
        err = Motor_SpeedControl(MOTOR_ID_1, &spd);
        if (err != MOTOR_OK) goto test_done;
        if (Motor_Test_WaitReady(MOTOR_ID_1, 1000)) goto test_done;
        if (Motor_GetLastError(MOTOR_ID_1) != MOTOR_OK) goto test_done;
    }

    HAL_Delay(1000);  /* 电机反向旋转1秒 */

    /* 停止旋转 */
    {
        Motor_SpeedParams_t spd = {
            .direction    = 0,
            .speed        = 0,          /* 速度=0 = 停止 */
            .acceleration = 3,
            .sync_flag    = 0
        };
        err = Motor_SpeedControl(MOTOR_ID_1, &spd);
        if (err != MOTOR_OK) goto test_done;
        Motor_Test_WaitReady(MOTOR_ID_1, 500);
    }

    HAL_Delay(200);

    /* ================================================================ */
    /*  Step 3: 位置模式测试                                             */
    /*  移动3200脉冲 (通常为1/10圈, 取决于细分) 验证定位功能              */
    /* ================================================================ */

    {
        Motor_PositionParams_t pos = {
            .direction    = 0,          /* CCW */
            .speed        = 500,
            .acceleration = 5,
            .pulse_count  = 3200,       /* 脉冲数 (取决于细分, 调整此值改变行程) */
            .mode         = 0,          /* 相对位置 */
            .sync_flag    = 0           /* 立即执行 */
        };
        err = Motor_PositionControl(MOTOR_ID_1, &pos);
        if (err != MOTOR_OK) goto test_done;
        if (Motor_Test_WaitReady(MOTOR_ID_1, 2000)) goto test_done;
        if (Motor_GetLastError(MOTOR_ID_1) != MOTOR_OK) goto test_done;
    }

    HAL_Delay(200);

    /* 反向移动,回到原位附近 */
    {
        Motor_PositionParams_t pos = {
            .direction    = 1,          /* CW — 返回 */
            .speed        = 500,
            .acceleration = 5,
            .pulse_count  = 3200,
            .mode         = 0,          /* 相对位置 */
            .sync_flag    = 0
        };
        err = Motor_PositionControl(MOTOR_ID_1, &pos);
        if (err != MOTOR_OK) goto test_done;
        if (Motor_Test_WaitReady(MOTOR_ID_1, 2000)) goto test_done;
        if (Motor_GetLastError(MOTOR_ID_1) != MOTOR_OK) goto test_done;
    }

    HAL_Delay(200);

    /* ================================================================ */
    /*  Step 4: 急停测试                                                 */
    /* ================================================================ */

    err = Motor_EmergencyStop(MOTOR_ID_1);
    if (err != MOTOR_OK) goto test_done;
    Motor_Test_WaitReady(MOTOR_ID_1, 500);

    HAL_Delay(100);

    /* ================================================================ */
    /*  Step 5: 多电机同步启动测试 (电机2和电机3)                         */
    /*  如果只接了一个电机, 此步超时不会影响整体结果                       */
    /* ================================================================ */

    /* 先使能电机2和电机3 */
    Motor_Enable(MOTOR_ID_2, 1);
    Motor_Test_WaitReady(MOTOR_ID_2, 500);
    Motor_Enable(MOTOR_ID_3, 1);
    Motor_Test_WaitReady(MOTOR_ID_3, 500);

    /* 缓存速度命令到电机2和电机3 (sync_flag=1) */
    {
        Motor_SpeedParams_t spd = {
            .direction    = 0,
            .speed        = 300,
            .acceleration = 3,
            .sync_flag    = 1           /* 缓存,不立即执行 */
        };
        Motor_SpeedControl(MOTOR_ID_2, &spd);
        Motor_Test_WaitReady(MOTOR_ID_2, 500);
        Motor_SpeedControl(MOTOR_ID_3, &spd);
        Motor_Test_WaitReady(MOTOR_ID_3, 500);
    }

    /* 广播同步启动 — 两个电机应同时开始旋转 */
    Motor_SyncStartAll();
    HAL_Delay(2000);  /* 运转2秒 */

    /* 急停全部 */
    Motor_EmergencyStopAll();
    HAL_Delay(200);

    /* 失能全部电机 */
    Motor_Enable(MOTOR_ID_2, 0);
    Motor_Test_WaitReady(MOTOR_ID_2, 500);
    Motor_Enable(MOTOR_ID_3, 0);
    Motor_Test_WaitReady(MOTOR_ID_3, 500);

test_done:
    /* ================================================================ */
    /*  测试结束: 确保电机1处于安全状态 (失能释放)                        */
    /* ================================================================ */
    {
        Motor_Error_t final_err = Motor_Enable(MOTOR_ID_1, 0);
        if (final_err == MOTOR_OK) {
            Motor_Test_WaitReady(MOTOR_ID_1, 500);
        }
        /* 最终状态记录在 Motor_GetLastError(MOTOR_ID_1) 中,
         * 可通过调试器或逻辑分析仪查看 */
    }
    /* 测试结束 — 如果电机未响应, 用逻辑分析仪检查 USART1 TX/RX 波形 */
}

#endif /* TEST_MOTOR_ENABLE */
