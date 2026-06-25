#ifndef SERIAL_DEBUG_H
#define SERIAL_DEBUG_H

#include <stdint.h>
#include <stdbool.h>

#include "bsp_uart.h"

#define SERIAL_DEBUG_JUSTFLOAT_CHANNELS     12U

/*
 * Serial_Debug only sends a raw float array.
 * The upper layer decides channel count and channel mapping.
 */
void Serial_Debug_Init(uart_ctrl_t *p_ctrl, uart_cfg_t const *p_cfg);
bool Serial_Debug_SendFloatArray(const float *data, uint16_t channel_count);

#endif // SERIAL_DEBUG_H
