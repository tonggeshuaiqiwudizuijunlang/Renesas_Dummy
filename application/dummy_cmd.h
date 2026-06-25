#ifndef DUMMY_CMD_H
#define DUMMY_CMD_H

#include "motor_def.h"
#include "robot_types.h"

#define DUMMY_CMD_UART_MODE_BLUETOOTH      0
#define DUMMY_CMD_UART_MODE_SERIAL_DEBUG   1

#ifndef DUMMY_CMD_UART_MODE
#define DUMMY_CMD_UART_MODE DUMMY_CMD_UART_MODE_SERIAL_DEBUG
#endif

#ifndef SERIAL_DEBUG_SEND_DIV
#define SERIAL_DEBUG_SEND_DIV 4U
#endif

#ifndef DUMMY_CMD_FKIK_TEST_ENABLE
#define DUMMY_CMD_FKIK_TEST_ENABLE 0
#endif

#ifndef DUMMY_CMD_FKIK_TEST_SEND_DIV
#define DUMMY_CMD_FKIK_TEST_SEND_DIV SERIAL_DEBUG_SEND_DIV
#endif




void DummyCmd_Init(void);
void DummyCmd_Task(void);


#endif
