#include "serial_debug.h"

static USARTInstance *serial_debug_uart_instance = NULL;

static void Serial_Debug_RxCallback(void)
{
    /* Serial_Debug only uses TX. RX callback is kept empty for USARTRegister. */
}

void Serial_Debug_Init(uart_ctrl_t *p_ctrl, uart_cfg_t const *p_cfg)
{
    USART_Init_Config_s config;

    config.recv_buff_size = 1U;
    config.p_uart_ctrl = p_ctrl;
    config.p_uart_cfg = p_cfg;
    config.module_callback = Serial_Debug_RxCallback;

    serial_debug_uart_instance = USARTRegister(&config);
}

bool Serial_Debug_SendFloatArray(const float *data, uint16_t channel_count)
{
    if (serial_debug_uart_instance == NULL || data == NULL || channel_count == 0U)
        return false;

    if (serial_debug_uart_instance->tx_busy)
        return false;

    serial_debug_uart_instance->tx_busy = true;
    fsp_err_t err = R_SCI_UART_Write(serial_debug_uart_instance->p_ctrl,
                                     (uint8_t *)data,
                                     (uint32_t)(channel_count * sizeof(float)));
    if (err != FSP_SUCCESS)
    {
        serial_debug_uart_instance->tx_busy = false;
        return false;
    }

    return true;
}
