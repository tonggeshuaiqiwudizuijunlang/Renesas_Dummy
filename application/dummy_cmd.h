#ifndef DUMMY_CMD_H
#define DUMMY_CMD_H

#include "motor_def.h"
#include "robot_types.h"

#define DUMMY_CMD_UART_MODE_BLUETOOTH      0
#define DUMMY_CMD_UART_MODE_SERIAL_DEBUG   1

#ifndef DUMMY_CMD_UART_MODE
#define DUMMY_CMD_UART_MODE DUMMY_CMD_UART_MODE_SERIAL_DEBUG
#endif

#ifndef DUMMY_CMD_FKIK_TEST_ENABLE
#define DUMMY_CMD_FKIK_TEST_ENABLE 0
#endif

#ifndef SERIAL_DEBUG_SEND_DIV
#define SERIAL_DEBUG_SEND_DIV 4U
#endif

#ifndef DUMMY_CMD_FKIK_TEST_SEND_DIV
#define DUMMY_CMD_FKIK_TEST_SEND_DIV SERIAL_DEBUG_SEND_DIV
#endif

#ifndef DUMMY_CMD_REMOTE_POSE_TEST_ENABLE
#define DUMMY_CMD_REMOTE_POSE_TEST_ENABLE 1
#endif

#ifndef REMOTE_POSE_POS_GAIN
#define REMOTE_POSE_POS_GAIN 0.001f
#endif

#ifndef REMOTE_POSE_ATT_GAIN
#define REMOTE_POSE_ATT_GAIN 0.00005f
#endif




void DummyCmd_Init(void);
void DummyCmd_Task(void);


#endif
