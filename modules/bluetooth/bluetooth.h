#ifndef BLUETOOTH_H
#define BLUETOOTH_H


#include "bsp_uart.h"
#include "bsp_gpio.h"

// 保持1字节对齐，防止编译器自动插入填充字节导致结构体大小不匹配
#pragma pack(1)
typedef struct {
    uint8_t header; // 固定为0xAA

    float joint1;
    float joint2;
    float joint3;
    float joint4;
    float joint5;
    float joint6;

    uint16_t tailer; // 固定为0xFFFB   
} RX_BT_Data_s;

typedef struct {
    uint8_t header; // 固定为0x5A

    float joint1;
    float joint2;
    float joint3;
    float joint4;
    float joint5;
    float joint6;
    uint8_t is_finished; // 0:未完成 1:已完成

    uint16_t tailer; // 固定为0xFFFB   
} TX_BT_Data_s;

#pragma pack()


/* --- 蓝牙状态机枚举 --- */
typedef enum {
    BT_STEP_IDLE,           // 空闲状态
    BT_STEP_HARD_RESET,     // 硬件复位
    BT_STEP_WAIT_READY,     // 等待启动 Ready
    BT_STEP_SEND_AT,        // 发送 AT 测试
    BT_STEP_SEND_ECHO,      // 关闭回显 ATE0
    BT_STEP_BLE_INIT,       // 蓝牙初始化 (配置为从机 Server)
    BT_STEP_SET_NAME,       // 设置蓝牙名称
    BT_STEP_ADV_START,      // 开启蓝牙广播
    BT_STEP_WAIT_CONN,      // 等待手机或其他主机连接 (+BLECONN)
    BT_STEP_ENTER_SPP,      // 开启透传模式
    BT_STEP_DATA_TRANS,     // 透传数据收发阶段
    BT_STEP_ERROR           // 出错状态
} bt_step_t;




/* --- API 接口 --- */
RX_BT_Data_s* BT_Init(uart_ctrl_t *p_ctrl, uart_cfg_t const *p_cfg);
void BT_Task_Entry(void);                           // 放在操作系统的任务 while(1) 中轮询
bool BT_SendData(uint8_t* tx_data, uint16_t len);   // 发送透传数据
bool BT_IsConnected(void);                          // 检查是否在透传状态

#endif /* BLUETOOTH_H */