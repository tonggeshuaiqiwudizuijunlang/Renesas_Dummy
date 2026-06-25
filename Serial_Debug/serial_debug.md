# 串口调试与波形观察模块 (Serial Debug)

本模块基于 VOFA+ 的 JustFloat 协议，用于通过串口发送多通道 `float` 数据到上位机显示波形，也可以接收上位机下发的浮点参数。

## 初始化示例

```c
#include "serial_debug.h"

static SerialDebug_Instance_s *vofa_debug;

static void Debug_Cmd_Callback(float *cmd_data, uint8_t cmd_num)
{
    if (cmd_num >= 2)
    {
        // cmd_data[0], cmd_data[1] ... 可用于在线调参
    }
}

void Debug_Init(void)
{
    SerialDebug_Init_Config_s debug_conf = {0};

    debug_conf.usart_config.usart_handle = UART_1;
    debug_conf.usart_config.tx_pin = UART1_TX_P04_1;
    debug_conf.usart_config.rx_pin = UART1_RX_P04_0;
    debug_conf.usart_config.uart_buadrate = BUAD_115200;
    debug_conf.cmd_callback = Debug_Cmd_Callback;

    vofa_debug = SerialDebug_Init(&debug_conf);
}
```

## 发送波形数据

```c
float wave_data[4];
wave_data[0] = vx;
wave_data[1] = vy;
wave_data[2] = yaw;
wave_data[3] = height;

SerialDebug_Send_JustFloat(vofa_debug, wave_data, 4);
```

## 注意事项

- 当前工程的底层串口接口是 `bsp_uart.h`，发送函数为 `USARTSend(instance, buf, len)`。
- 一个串口只能注册一次，调试串口不要和遥控器、底盘通信串口重复。
- VOFA+ 中选择 JustFloat 协议，帧尾为 `00 00 80 7F`。
