/**
 ****************************************************************************************
 *
 * @file app_eaci_uart.h
 *
 * @brief UART transport module functions for Easy Application Controller Interface.
 *
 * Copyright (C) Quintic 2012-2013
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef APP_PT_H_
#define APP_PT_H_

#include "app_env.h"

#define UART_RX_TIMER		QN_TIMER3
#define PT_UART_PORT		QN_UART0

#define PT_FIFO_X_SIZE        200//50
#define PT_FIFO_Y_SIZE        20
#define FIFO_SIZE			  1000

#define PT_RX_RAW_FRAME_EVENT_ID 4



enum pt_st
{
    PT_DEEPSLEEP	= 0,
    PT_ADV			= 1,
    PT_CONN_EMPTY   = 2,
    PT_CONN_FULL	= 3,

};
enum ble_tx_st
{
    BLE_TX_STOP	    = 0,
    BLE_TX_DELAY	= 1,
    BLE_TX_GOING    = 2,
    BLE_TX_NOBUF	= 4,
};

enum uart_rx_st
{
    UART_RX_STOP	= 0,
//    UART_RX_DELAY	= 1,
    UART_RX_GOING   = 2,
//    UART_RX_NOBUF	= 4,
};

enum pt_fifo_st
{
    PT_FIFO_EMPT	= 0,    // empty
    PT_FIFO_NOFU	= 1,    // not full 
    PT_FIFO_FULL    = 2,    // FIFO full
 //   PT_FIFO_SUCC    = 3,    // operation success
 //   PT_FIFO_FAIL    = 4,    // operation fail
};

struct woke_param
{
	uint16_t adv_intv_min; 		//adv_interval
	uint16_t adv_intv_max;
	uint16_t conn_intv_min;   	//conn_interval
	uint16_t conn_intv_max;
	uint8_t ble_name_len;
	uint8_t ble_name[17];
	uint8_t baudrate;
	uint8_t wakeup_exmcu_timer;
	uint16_t raw_frame_timer;
	uint8_t adv_user_data_len;
	uint8_t adv_user_data[30];
};

struct PT_FIFO_ELEMENT
{
	uint8_t len; 			///UART RX Raw data
	uint8_t buf[PT_FIFO_Y_SIZE]; 			///UART RX Raw data
};

struct PT_FIFO
{
	uint8_t in; 			///UART RX Raw data paramete
	uint8_t out;
    uint8_t state;          //  fifo state: empty, not full, full
	uint8_t full_ct;		// debug information
	uint8_t empty_ct;		// debug information
	struct PT_FIFO_ELEMENT elemt[PT_FIFO_X_SIZE];
};

struct pt_env_tag
{
    uint8_t state;		                //deepsleep, advertising, connect full, connect empty
    uint8_t ble_tx_state;		        //stop, tx delay, tx going
    uint8_t uart_rx_state;		        //stop, tx delay, tx going
    struct woke_param user_param;
    
 //	uint8_t rx_raw_len;
//	uint8_t rx_raw_buf[PT_FIFO_Y_SIZE]; 			///UART RX Raw data parameter

};


extern uint8_t pt_fifo_get(uint8_t *pBuf);
extern uint8_t pt_fifo_put(uint8_t *pPutDat, uint8_t PutLen);
extern void pt_fifo_init(void);
extern void pt_rx_raw_start(void);
extern void ble_data_tx(void);
extern void pt_init(void);
extern void pt_enable(void);
extern void pt_disable(void);
extern void pt_rx_raw_done(void);

extern struct pt_env_tag  pt_env;
extern struct PT_FIFO pt_data_fifo;	//data fifo

#endif 
