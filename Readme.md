# 硬件相关

主控芯片：STM32F407ZGT6  
电机：张大头42步进电机

# 软件相关
## V1.0  
原始生成文件，配置USART1作为电机驱动通讯引脚，通过转接板实现一控四

## V1.1
新增电机驱动控制代码（`HardWare/Motor.c` `HardWare/Motor.h`），实现张大头42步进电机串口驱动

### 通讯方式
- USART1 + DMA（TX: DMA2_Stream7, RX: DMA2_Stream2）
- 利用 HAL 的 IDLE 中断 + DMA 实现变长帧接收，降低CPU占用
- 115200波特率，8N1，TTL直连转接板，一控四（地址0x01~0x04）

### 协议帧格式
`[地址 1B] [功能码 1B] [数据 N字节] [校验 1B]`，校验为固定 0x6B

### 主要API

#### 系统函数

**`void Motor_Init(void)`**
初始化驱动，启动 DMA-IDLE 接收监听。必须在 `MX_USART1_UART_Init()` 之后调用。

**`void Motor_Process(void)`**
超时检测，需在 `main()` 的 `while(1)` 循环中周期性调用。当命令等待响应超过 50ms 无回复时自动标记超时并重新武装 DMA。

---

#### 电机使能

**`Motor_Error_t Motor_Enable(uint8_t motor_id, uint8_t enable)`**
- `motor_id`: 电机ID（`MOTOR_ID_1` ~ `MOTOR_ID_4`）
- `enable`: `1` = 使能（上电锁定轴），`0` = 失能（释放轴可自由转动）
- 返回: `MOTOR_OK` 或错误码

---

#### 急停

**`Motor_Error_t Motor_EmergencyStop(uint8_t motor_id)`**
立即制动单个电机，发送 `0xFE 0x98` 命令。`motor_id` 指定电机ID。

**`Motor_Error_t Motor_EmergencyStopAll(void)`**
广播急停（地址 `0x00`），全部4个电机同时制动。无参数，发送后不等待响应直接返回。

---

#### 运动控制

**`Motor_Error_t Motor_SpeedControl(uint8_t motor_id, const Motor_SpeedParams_t *params)`**
速度模式控制，发送 `0xF6` 命令。

`Motor_SpeedParams_t` 结构体:
| 字段 | 类型 | 说明 |
|------|------|------|
| `direction` | `uint8_t` | `0`=CCW(逆时针), `1`=CW(顺时针) |
| `speed` | `uint16_t` | 速度值（单位取决于微步细分，典型为 RPM×10） |
| `acceleration` | `uint8_t` | 加速度档位（1~10） |
| `sync_flag` | `uint8_t` | `0`=立即执行, `1`=缓存等待同步启动 |

**`Motor_Error_t Motor_PositionControl(uint8_t motor_id, const Motor_PositionParams_t *params)`**
位置（脉冲）模式控制，发送 `0xFD` 命令。

`Motor_PositionParams_t` 结构体:
| 字段 | 类型 | 说明 |
|------|------|------|
| `direction` | `uint8_t` | `0`=CCW, `1`=CW |
| `speed` | `uint16_t` | 速度值 |
| `acceleration` | `uint8_t` | 加速度档位（1~10） |
| `pulse_count` | `uint32_t` | 脉冲数（4字节，如 `32000` = 一圈，取决于细分） |
| `mode` | `uint8_t` | `0`=相对位置（增量）, `1`=绝对位置 |
| `sync_flag` | `uint8_t` | `0`=立即执行, `1`=等待同步启动 |

---

#### 同步启动

**`Motor_Error_t Motor_SyncStart(uint8_t motor_id)`**
触发单个电机执行之前缓存的（`sync_flag=1`）命令，发送 `0xFF` 到指定地址。

**`Motor_Error_t Motor_SyncStartAll(void)`**
广播同步启动（地址 `0x00`），触发全部电机同时执行各自缓存的命令。用于需要多电机同步运动的场景。

---

#### 状态查询

**`uint8_t Motor_IsBusy(uint8_t motor_id)`** — 查询电机是否正在等待响应（`1`=忙）

**`Motor_State_t Motor_GetState(uint8_t motor_id)`** — 获取状态（`IDLE`/`BUSY`/`ERROR`）

**`Motor_Error_t Motor_GetLastError(uint8_t motor_id)`** — 获取最后一次错误码（`TIMEOUT`/`CHECKSUM`/`MOTOR_FAULT`/`DMA_BUSY`）

---

### 返回值

所有控制函数返回 `Motor_Error_t` 枚举:

| 返回值 | 值 | 含义 |
|--------|-----|------|
| `MOTOR_OK` | `0` | 命令已成功发送（异步，响应由中断处理） |
| `MOTOR_ERROR_BUSY` | `-6` | 当前有命令正等待响应，需等待或调用 `Motor_Process()` |
| `MOTOR_ERROR_INVALID_PARAM` | `-5` | 电机ID无效或参数指针为 NULL |
| `MOTOR_ERROR_DMA_BUSY` | `-4` | DMA 启动失败 |
| `MOTOR_ERROR_TIMEOUT` | `-1` | 响应超时（50ms未收到回复） |
| `MOTOR_ERROR_CHECKSUM` | `-2` | 响应校验错误 |
| `MOTOR_ERROR_MOTOR_FAULT` | `-3` | 电机返回 `0xEE` 故障码 |

---

### 使用示例

```c
#include "Motor.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();

    Motor_Init();                    // 初始化电机驱动

    /* 1. 使能电机1 */
    Motor_Enable(MOTOR_ID_1, 1);
    HAL_Delay(100);                  // 等待异步操作完成

    /* 2. 速度模式: 电机1 逆时针 500速 旋转 */
    Motor_SpeedParams_t spd = {
        .direction    = 0,           // CCW 逆时针
        .speed        = 500,         // 速度 500
        .acceleration = 5,           // 加速度档位 5
        .sync_flag    = 0            // 立即执行
    };
    Motor_SpeedControl(MOTOR_ID_1, &spd);
    HAL_Delay(100);

    /* 3. 位置模式: 电机2 顺时针 走 32000 脉冲 */
    Motor_PositionParams_t pos = {
        .direction    = 1,           // CW 顺时针
        .speed        = 800,
        .acceleration = 5,
        .pulse_count  = 32000,       // 脉冲数
        .mode         = 0,           // 相对位置
        .sync_flag    = 0
    };
    Motor_PositionControl(MOTOR_ID_2, &pos);
    HAL_Delay(100);

    /* 4. 多电机同步启动: 缓存命令 → 广播触发 */
    spd.sync_flag = 1;               // 缓存模式
    Motor_SpeedControl(MOTOR_ID_1, &spd);   // 电机1缓存速度命令
    HAL_Delay(10);
    Motor_SpeedControl(MOTOR_ID_3, &spd);   // 电机3缓存速度命令
    HAL_Delay(10);
    Motor_SyncStartAll();                    // 广播触发全部同步启动
    HAL_Delay(10);

    /* 5. 急停全部电机 */
    Motor_EmergencyStopAll();

    while (1) {
        Motor_Process();             // 必须周期性调用,处理超时
    }
}
```

### 硬件配置宏（Motor.h）

修改硬件接线只需改 `Motor.h` 顶部宏，无需改 `.c` 文件:

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `MOTOR_UART` | `(&huart1)` | 串口句柄，换串口改这里 |
| `MOTOR_USE_RS485_DE` | `0` | `0`=TTL直连, `1`=RS485需配置DE引脚 |
| `MOTOR_CHECKSUM_MODE` | `0` | `0`=固定0x6B, `1`=XOR异或, `2`=CRC-8 |
| `MOTOR_ADDR_1~4` | `0x01~0x04` | 电机地址，按实际修改 |


