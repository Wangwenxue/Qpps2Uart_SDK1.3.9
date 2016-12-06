/**
 ****************************************************************************************
 *
 * @file app_sys.h
 *
 * @brief Application System Functioins
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef _APP_SYS_H_
#define _APP_SYS_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (QN_DEMO_MENU || QN_EACI || QN_PT) // wenxue add QN_PT
#if (QN_DEMO_MENU)

#define QN_UART_RX_LEN      0x10

/// Application UART environment context structure
struct app_uart_env_tag
{
    uint8_t len;
    uint8_t buf_rx[QN_UART_RX_LEN];
};
#endif

/// Message of UART data request send to application
struct app_uart_data_req
{
    uint8_t len;
    uint8_t data[1];
};

/// Application handle message of UART received data
struct app_uart_data_ind
{
    uint8_t len;
    uint8_t data[1];
};

#endif

#if QN_DBG_PRINT

/*
 ****************************************************************************************
 * @brief Uart initialization.
 *
 ****************************************************************************************
 */
void app_uart_init(void);

#endif

#define DATA_BUF_LEN 1000

struct app_pass_env_tag
{
	uint8_t head;
    uint8_t len_H;
    uint8_t len_L;
    uint8_t uart_flag;
    uint16_t length;			//uart rx left data
    uint16_t length_backup;		//ble tx left data
    uint16_t ble_tx_pack_max;
    uint16_t ble_tx_pack_ct;
	uint8_t *p_uart_rx;			// uart rx data pointer
	uint8_t *p_ble_send;		// ble tx data pointer
    uint8_t buf_rx[DATA_BUF_LEN];
};

#endif // _APP_SYS_H_

