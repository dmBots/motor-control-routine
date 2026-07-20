# DAMIAO | 达妙科技

DM-H7 FDCAN 电机控制 DEMO 示例



## 目录

[TOC]

<div style="page-break-after:always"></div>

## 1. demo 使用

demo 示例代码做了应用层、FDCAN 驱动、电机协议层分层，用户移植时按文件层级修改。

### 1.1 代码分层

| 文件 | 功能 |
| --- | --- |
| `Core/Src/main.c` | 工程入口、外设初始化、上电流程、TIM3 周期调度 |
| `User/bsp_fdcan.c` | FDCAN 滤波、启动、收发、波特率设置 |
| `User/bsp_fdcan.h` | CAN/FDCAN 模式宏、波特率宏、FDCAN 接口声明 |
| `User/dm_motor_ctrl.c` | 电机对象初始化、参数读取流程、FDCAN1 接收分发 |
| `User/dm_motor_ctrl.h` | 电机对象声明、应用层接口声明 |
| `User/dm_motor_drv.c` | 电机控制帧、反馈解析、寄存器读写保存 |
| `User/dm_motor_drv.h` | 电机枚举、控制模式、寄存器 RID、结构体定义 |

### 1.2 默认运行配置

当前 demo 控制 `motor[Motor1]`，默认参数由源码直接写入。

| 项目 | 当前值 | 源码位置 |
| --- | --- | --- |
| MCU | `STM32H723VGTx` | `MDK-ARM/CtrBoard.uvprojx` |
| Keil Target | `CtrBoard` | `MDK-ARM/CtrBoard.uvprojx` |
| 电机对象 | `motor[Motor1]` | `Core/Src/main.c` |
| 电机 CAN ID | `0x01` | `User/dm_motor_ctrl.c` |
| 电机反馈 ID | `0x00` | `User/dm_motor_ctrl.c` |
| 通信模式 | `CAN_CLASS` | `Core/Src/main.c` |
| 通信波特率 | `CAN_BR_1M` | `Core/Src/main.c` |
| 启动后控制模式 | `mit_mode` | `Core/Src/main.c` |
| 周期调度来源 | TIM3 中断 | `Core/Src/main.c` |
| 电源控制 IO | `PC14` | `Core/Inc/main.h` |
| FDCAN1 RX/TX | `PD0` / `PD1` | `Core/Src/fdcan.c` |

`Motor1` 默认控制参数：

| 字段 | 当前值 |
| --- | --- |
| `motor[Motor1].ctrl.pos_set` | `6.28f` |
| `motor[Motor1].ctrl.vel_set` | `1.0f` |
| `motor[Motor1].ctrl.tor_set` | `0.0f` |
| `motor[Motor1].ctrl.cur_set` | `0.02f` |
| `motor[Motor1].ctrl.kp_set` | `0.0f` |
| `motor[Motor1].ctrl.kd_set` | `0.0f` |
| `motor[Motor1].tmp.PMAX` | `12.5f` |
| `motor[Motor1].tmp.VMAX` | `30.0f` |
| `motor[Motor1].tmp.TMAX` | `10.0f` |

### 1.3 上电运行流程

`main.c` 启动后按以下顺序运行：

1. 初始化 GPIO、FDCAN1、TIM3、USART1、TIM4。
2. `power(1)` 将 `PC14` 输出高电平。
3. `bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M)` 设置经典 CAN 1 Mbps。
4. `bsp_can_init()` 配置 FDCAN 滤波器、启动 FDCAN1、打开 FIFO0 和错误中断。
5. `dm_motor_init()` 初始化 `motor[]`。
6. `motor[Motor1].ctrl.mode = mit_mode` 设置运行控制模式。
7. `write_motor_data(motor[Motor1].id, 10, mit_mode, 0, 0, 0)` 写控制模式寄存器。
8. `read_motor_data(motor[Motor1].id, RID_CAN_BR)` 读取 CAN 波特率寄存器。
9. `dm_motor_disable(&hfdcan1, &motor[Motor1])` 发送失能帧并清零控制给定。
10. `save_motor_data(motor[Motor1].id, 10)` 发送保存帧。
11. `dm_motor_enable(&hfdcan1, &motor[Motor1])` 发送使能帧。
12. `HAL_TIM_Base_Start_IT(&htim3)` 启动 TIM3 中断。

TIM3 中断回调执行：

```c
read_all_motor_data(&motor[Motor1]);

if(motor[Motor1].tmp.read_flag == 0)
    dm_motor_ctrl_send(&hfdcan1, &motor[Motor1]);
```

`read_flag` 从 `1` 递增到 `45` 时，TIM3 逐项读取电机寄存器。收到 `RID_X_OUT` 参数回复后，`receive_motor_data()` 将 `read_flag` 写为 `0`，之后 TIM3 开始周期发送 `Motor1` 控制帧。

### 1.4 FDCAN/CAN 设置

`bsp_fdcan.h` 定义了两类通信模式：

| 宏 | 数值 | 通信格式 |
| --- | ---: | --- |
| `CAN_CLASS` | `0` | 经典 CAN |
| `CAN_FD_BRS` | `1` | FDCAN BRS |

`bsp_fdcan.h` 定义了 10 个波特率代码：

| 宏 | 数值 |
| --- | ---: |
| `CAN_BR_125K` | `0` |
| `CAN_BR_200K` | `1` |
| `CAN_BR_250K` | `2` |
| `CAN_BR_500K` | `3` |
| `CAN_BR_1M` | `4` |
| `CAN_BR_2M` | `5` |
| `CAN_BR_2M5` | `6` |
| `CAN_BR_3M2` | `7` |
| `CAN_BR_4M` | `8` |
| `CAN_BR_5M` | `9` |

`bsp_fdcan_set_baud()` 的配置范围：

| `mode` | `baud` | `FrameFormat` | 
| --- | --- | --- |
| `CAN_CLASS` | `CAN_BR_125K` / `CAN_BR_200K` / `CAN_BR_250K` / `CAN_BR_500K` / `CAN_BR_1M` | `FDCAN_FRAME_CLASSIC` |
| `CAN_FD_BRS` | `CAN_BR_2M` / `CAN_BR_2M5` / `CAN_BR_3M2` / `CAN_BR_4M` / `CAN_BR_5M` | `FDCAN_FRAME_FD_BRS` | 

当前 demo 固定执行：

```c
bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);
```

电机端通信模式设置为经典 CAN，波特率设置为 1 Mbps。

### 1.5 电机 ID 设置

当前接收回调只处理标准 ID `0x00`：

```c
void fdcan1_rx_callback(void)
{
    uint16_t rec_id;
    uint8_t rx_data[8] = {0};
    fdcanx_receive(&hfdcan1, &rec_id, rx_data);
    switch (rec_id)
    {
        case 0x00:
            dm_motor_fbdata(&motor[Motor1], rx_data);
            receive_motor_data(&motor[Motor1], rx_data);
            break;
    }
}
```

当前 demo 电机 ID 设置：

| 电机 | CAN ID | Master ID / 反馈 ID |
| --- | --- | --- |
| `Motor1` | `0x01` | `0x00` |

新增电机时，新增电机的 Master ID / 反馈 ID 写入 `fdcan1_rx_callback()` 的新增 `case`。

### 1.6 电机控制模式

`mode_e` 控制模式定义：

| 枚举 | 数值 | 控制函数 |
| --- | ---: | --- |
| `mit_mode` | `1` | `mit_ctrl()` |
| `pos_mode` | `2` | `pos_ctrl()` |
| `spd_mode` | `3` | `spd_ctrl()` |
| `psi_mode` | `4` | `psi_ctrl()` |

模式 ID 偏移：

| 宏 | 数值 |
| --- | ---: |
| `MIT_MODE` | `0x000` |
| `POS_MODE` | `0x100` |
| `SPD_MODE` | `0x200` |
| `PSI_MODE` | `0x300` |

控制帧发送 ID 为 `motor_id + mode_id`。当前 `Motor1` 启动后使用 `mit_mode`，发送 ID 为 `0x01 + 0x000`。

### 1.7 参数读写和保存

读寄存器：

```c
read_motor_data(id, rid);
```

发送数据：

```text
data[0] = id & 0x0F
data[1] = (id >> 4) & 0x0F
data[2] = 0x33
data[3] = rid
```

写寄存器：

```c
write_motor_data(id, rid, d0, d1, d2, d3);
```

发送数据：

```text
data[0] = id & 0xFF
data[1] = (id >> 8) & 0x07
data[2] = 0x55
data[3] = rid
data[4] = d0
data[5] = d1
data[6] = d2
data[7] = d3
```

保存参数：

```c
save_motor_data(id, rid);
```

发送数据：

```text
data[0] = id & 0xFF
data[1] = (id >> 8) & 0x07
data[2] = 0xAA
data[3] = 0x01
```


## 2. 在 demo 中添加电机

添加电机控制时，修改 `dm_motor_ctrl.c` 和 `main.c`。

### 2.1 添加电机初始化

`dm_motor_drv.h` 中已经定义 `Motor1` 到 `Motor10`：

```c
typedef enum
{
    Motor1,
    Motor2,
    Motor3,
    Motor4,
    Motor5,
    Motor6,
    Motor7,
    Motor8,
    Motor9,
    Motor10,
    num
} motor_num;
```

添加 `Motor2` 时，如下在 `dm_motor_init()` 中写入初始化信息：

```c
memset(&motor[Motor2], 0, sizeof(motor[Motor2]));

motor[Motor2].id = 0x02;
motor[Motor2].mst_id = 0x12;
motor[Motor2].tmp.read_flag = 1;
motor[Motor2].ctrl.mode = mit_mode;
motor[Motor2].ctrl.vel_set = 0.0f;
motor[Motor2].ctrl.pos_set = 0.0f;
motor[Motor2].ctrl.tor_set = 0.0f;
motor[Motor2].ctrl.cur_set = 0.0f;
motor[Motor2].ctrl.kp_set = 0.0f;
motor[Motor2].ctrl.kd_set = 0.0f;
motor[Motor2].tmp.PMAX = 12.5f;
motor[Motor2].tmp.VMAX = 30.0f;
motor[Motor2].tmp.TMAX = 10.0f;
```

`PMAX`、`VMAX`、`TMAX` 同时用于 MIT 控制映射和反馈解析。

### 2.2 接收新电机数据

通过接收帧标准 ID 区分电机反馈数据。添加 `Motor2` 时，在 `fdcan1_rx_callback()` 中增加 `case 0x01`：

```c
void fdcan1_rx_callback(void)
{
    uint16_t rec_id;
    uint8_t rx_data[8] = {0};
    fdcanx_receive(&hfdcan1, &rec_id, rx_data);
    switch (rec_id)
    {
        case 0x01:
            dm_motor_fbdata(&motor[Motor1], rx_data);
            receive_motor_data(&motor[Motor1], rx_data);
            break;

        case 0x12:
            dm_motor_fbdata(&motor[Motor2], rx_data);
            receive_motor_data(&motor[Motor2], rx_data);
            break;
    }
}
```

`motor[].mst_id` 当前不参与接收分发，接收分发由 `switch (rec_id)` 决定。

### 2.3 添加新电机控制调度

在 `Core/Src/main.c` 的 `HAL_TIM_PeriodElapsedCallback()` 中增加新电机的参数读取和控制发送：

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        read_all_motor_data(&motor[Motor1]);
        if(motor[Motor1].tmp.read_flag == 0)
            dm_motor_ctrl_send(&hfdcan1, &motor[Motor1]);

        read_all_motor_data(&motor[Motor2]);
        if(motor[Motor2].tmp.read_flag == 0)
            dm_motor_ctrl_send(&hfdcan1, &motor[Motor2]);
    }
}
```

## 3. 目录结构

```bash
dm_ctrl(h7 fdcan) v1.1 裸机
├─ CtrBoard.ioc                     # STM32CubeMX 项目文件
├─ Core                             # HAL 外设配置和中断入口
│  ├─ Inc
│  │  ├─ fdcan.h
│  │  ├─ gpio.h
│  │  ├─ main.h
│  │  ├─ stm32h7xx_hal_conf.h
│  │  ├─ stm32h7xx_it.h
│  │  ├─ tim.h
│  │  └─ usart.h
│  └─ Src
│     ├─ fdcan.c                    # FDCAN1 外设初始化
│     ├─ gpio.c                     # GPIO 初始化
│     ├─ main.c                     # 主程序入口和 TIM3 调度
│     ├─ stm32h7xx_hal_msp.c
│     ├─ stm32h7xx_it.c             # FDCAN1 / TIM3 / TIM4 中断入口
│     ├─ system_stm32h7xx.c
│     ├─ tim.c                      # TIM3 / TIM4 初始化
│     └─ usart.c
├─ Drivers                          # STM32H7 HAL / CMSIS 驱动
├─ MDK-ARM                          # Keil 工程文件
│  ├─ CtrBoard.uvprojx
│  ├─ CtrBoard.uvoptx
│  └─ startup_stm32h723xx.s
├─ User                             # 用户代码
│  ├─ bsp_fdcan.c                   # FDCAN 滤波、启动、收发、波特率配置
│  ├─ bsp_fdcan.h
│  ├─ delay.c
│  ├─ delay.h
│  ├─ dm_motor_ctrl.c               # 电机初始化、参数读取、接收分发
│  ├─ dm_motor_ctrl.h
│  ├─ dm_motor_drv.c                # 电机协议帧、反馈解析、参数读写保存
│  └─ dm_motor_drv.h
└─ USAGE.md                         # 使用说明
```
