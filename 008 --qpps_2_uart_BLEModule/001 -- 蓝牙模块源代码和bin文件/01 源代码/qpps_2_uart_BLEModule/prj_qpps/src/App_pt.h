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


#define TX_WAKEUP_TIMER		QN_TIMER0

#define PT_FIFO_X_SIZE        200//50
#define PT_FIFO_Y_SIZE        20
#define FIFO_SIZE			  1000

#define PT_RX_RAW_FRAME_EVENT_ID 4

#define EVENT_UART_TX_ID 10
#define EVENT_GPIO_UART_RX_WAKEUP_ID     6
#define EVENT_GPIO_STCHANGE_ID     0

// wenxue used for contec
#define GAP_ADV_100MS_INTV                               0x00A0
#define GAP_CONN_100MS_INTV                              0x0050
#define BLE_NAME_LEN_MAX 20  // wenxue used for contec
#define ADV_USER_DATA_LEN_MAX 16

#define MAC_ADRRESS_DEFAULT            "\x01\x00\x00\xBE\x7C\x08"



#define PT_TX_FRAME_MAX			    256 // wenxue  used for QN9020 BLE Module TX buffer

#define PT_RX_PDU_BUF_DEEP 6

enum pt_st
{
    PT_DEEPSLEEP	= 0,
    PT_ADV			= 1,
    PT_CONN_EMPTY   = 2,
    PT_CONN_FULL	= 3,
	  PT_BETWEEM_ADV_CONN	= 4, // 连接后断开广播

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

enum 
{
	///PT UART TX Done state
    PT_UART_TX_IDLE = 0,
	///PT UART TX Start state
    PT_UART_TX_ONGOING,
	///PT UART RX PDU Start state --- message type
	PT_RX_PDU_STATR,
	///PT UART RX PDU Header State --- message header
	PT_RX_PDU_HDR,
    ///PT UART RX PDU Header State --- message payload
    PT_RX_PDU_PAYL,
	///PT_UART_RX PDU IDLE
	PT_RX_PDU_IDLE,
	///PT_UART RX RAW START
	PT_RX_RAW_START,
	/// PT UART RX RAW ONGOING
	PT_RX_RAW_ONGOING,
	///PT_UART RX RAW IDLE
	PT_RX_RAW_IDLE,
    ///PT UART RX PDU Header state --- message error	
    PT_STATE_RX_ERR,
    ///Number of states
    PT_STATE_MAX
};

struct woke_param
{
	uint16_t adv_intv_min; 		//adv_interval
	uint16_t adv_intv_max;
	uint16_t conn_intv_min;   	//conn_interval
	uint16_t conn_intv_max;
	uint8_t ble_name_len;
	uint8_t ble_name[21];
	uint8_t baudrate;
	uint8_t wakeup_exmcu_timer;
	uint16_t raw_frame_timer;
	uint8_t adv_user_data_len;
	uint8_t mac_address[6];
	uint8_t adv_user_data[23];
};

struct PT_FIFO_ELEMENT
{
	uint8_t len; 			                ///UART RX Raw data lengh
	uint8_t buf[PT_FIFO_Y_SIZE]; 			///UART RX Raw data
};

struct PT_FIFO
{
	uint8_t in; 			  // UART RX Raw data paramete
	uint8_t out;
  uint8_t state;      //  fifo state: empty, not full, full
	uint8_t full_ct;		// debug information
	uint8_t empty_ct;		// debug information
	struct PT_FIFO_ELEMENT elemt[PT_FIFO_X_SIZE];
	

};

struct pdu_frame
{ 
	  struct co_list_hdr hdr;     ///< List header for chaining
    
    uint8_t         hci_type;   ///< Type of HCI data (used by the HCI only)
    int8_t          hci_off;    ///< Offset of the HCI data in the message (used by the HCI only)
    uint16_t        hci_len;    ///< Length of the HCI traffic (used by the HCI only)
    
    ke_msg_id_t     id;         ///< Message id.
    ke_task_id_t    dest_id;    ///< Destination kernel identifier.
    ke_task_id_t    src_id;     ///< Source kernel identifier.
    uint16_t        param_len;  ///< Parameter embedded struct length.
    uint8_t        param[1];   ///< Parameter embedded struct. Must be word-aligned.
};

struct pt_env_tag
{
    uint8_t state;		                //deepsleep, advertising, connect full, connect empty
    uint8_t ble_tx_state;		          //stop, tx delay, tx going
    uint8_t uart_rx_state;		        //stop, tx delay, tx going
    struct woke_param user_param;
	  
	  ///UART RX parameter
		volatile uint8_t rx_state;
    uint8_t rx_pdu_error;	
  	struct co_list queue_tx;	//Queue of kernel messages corresponding to packets sent through HCI
	 ///UART TX parameter 
    uint8_t tx_state;       	//either transmitting or done.
	
	
	struct pdu_frame rx_pdu_buf[PT_RX_PDU_BUF_DEEP];
	uint8_t rx_pdu_type;																///UART RX PDU data parameter
	uint8_t rx_pdu_id;																	///UART RX PDU data parameter
	uint8_t rx_pdu_len;
	uint8_t rx_pdu_wridex;
	
};

enum
{
    ///Reserved
    PT_PDU_TYPE_RSV = 0x0,
    ///Command
    PT_PDU_TYPE_CMD = 0xEA,
    ///Data Request
    PT_PDU_TYPE_DATA_REQ,
    ///Data Indication
    PT_PDU_TYPE_DATA_IND,
    ///Event
    PT_PDU_TYPE_EVT,
    ///Number of Message Type
    PT_PDU_TYPE_MAX
};

///pt pdu rx Error Reason
enum
{
    /// Type error
    PT_TYPE_ERROR,
    /// MSG Out of Range
    PT_PDU_OOR_ERROR,
    ///Number of Error
    PT_ERROR_MAX
};

///pt pdu Event
enum
{
    ///Reserved
    PT_PDU_EVT_RSV = 0x0,
	
	  ///uart baud wenxue used for contec
  	PT_PDU_EVT_UART_BAUD = 0x01,
	
	  ///Peripheral Device Name
    PT_PDU_EVT_SET_DEVNAME = 0x02,
	
	  ///Peripheral update adv param
	  PT_PDU_EVT_PER_UPDATE_ADV_PARAM = 0x03,
	
	  ///Peripheral Upadate param state
	  PT_PDU_EVT_PER_UPDATE_CONN_PARAM = 0x04,
	
		///ble Address
	  PT_PDU_EVT_ADD = 0x05,	
	
	  /// TX POWER
	  PT_PDU_EVT_TX_POWER = 0x06,
	
	 	///adv user data
	  PT_PDU_EVT_ADV_USER_DATA = 0x07,
	
	  /// Wakeup exmcu timer
	  PT_PDU_EVT_WAKEUP_EXMCU_TIMER = 0x08,
		
	 ///set default paramter
	 PT_PDU_EVT_SET_DEFAULT_PARAMETER = 0x09, 
	 
	 	///Data Error
   PT_PDU_EVT_ERROR = 0xFA,
		
    ///Number of Event
    EACI_MSG_EVT_MAX
};


/// pt pdu COMMAND
enum
{ 
    ///Reserved
    PT_PDU_CMD_RSV = 0x0,
     
    ///set uart baud
	  PT_PDU_CMD_SET_UART_BAUD = 0x01, 

    ///Peripheral Set Device Name
    PT_PDU_CMD_SET_DEVNAME = 0x02,	
	
	 /// Peripheral updata adv param
	  PT_PDU_CMD_PER_UPDATE_ADV_PARAM = 0x03,
	
	 ///Peripheral Update Param
   PT_PDU_CMD_PER_UPDATE_CONN_PARAM = 0x04,
	
		///read chip add
	 PT_PDU_CMD_RD_ADD = 0x05,
	
	/// TX POWER
	PT_PDU_CMD_SET_TX_POWER = 0x06,	
	
	///set adv user data
	PT_PDU_CMD_SET_ADV_USER_DATA = 0x07,
	
		///set wakeup extern MCU timer
	PT_PDU_CMD_SET_WAKEUP_MCU_TIMER = 0x08,
	
	// wenxue used for contec
	///set default paramter
	PT_PDU_CMD_SET_DEFAULT_PARAMETER = 0x09, 
	
   ///Number of Command
   PT_PDU_CMD_MAX
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
extern void pt_rx_pdu_payl(void);



extern void pt_rx_pdu_start(void);
extern void pt_rx_pdu_done(void);
extern void pt_pdu_send(uint8_t len, uint8_t *par);
extern void pt_uart_write(struct ke_msg *msg);

extern void pt_pdu_send_error(uint8_t reason);

extern struct pt_env_tag  pt_env;
extern struct PT_FIFO pt_data_fifo;	//data fifo
extern void module_init(void);
extern void pt_gpio_init();
extern void pt_tx_done(void);
extern void app_event_gpio_stchange_handler(void);
extern void pt_state_set(enum pt_st state);
extern void app_pt_task_msg_hdl(ke_msg_id_t const msgid, void const *param);
extern void timer0_callback(void);

extern void pt_pdu_hdl(uint8_t pdu_type, uint8_t pdu_id, uint8_t param_len, uint8_t const *param);
extern void pt_pdu_cmd_hdl(uint8_t pdu_id, uint8_t param_len, uint8_t const *param);
extern void pt_pdu_cmd_set_tx_power(uint8_t param_len,uint8_t const *param);
extern void pt_pdu_cmd_per_update_adv_param (uint8_t param_len,uint8_t const *param);
extern void pt_pdu_cmd_per_update_conn_param(uint8_t param_len,uint8_t const *param);
extern void pt_pdu_cmd_set_devname_hdl(uint8_t param_len, uint8_t const *param);
extern void pt_pdu_cmd_rd_add_hdl(uint8_t param_len, uint8_t const *param);
extern void pt_pdu_cmd_set_adv_user_data(uint8_t param_len, uint8_t const *param);
extern void pt_pdu_cmd_set_default_paramter(uint8_t param_len, uint8_t const *param); 
extern void pt_pdu_cmd_set_uart_baud(uint8_t param_len, uint8_t const *param); 
extern void pt_pdu_cmd_set_wakeup_exmcu_timer(uint8_t param_len, uint8_t const *param);
extern int  app_pt_uartbaud_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);


#define UART_CTS               (GPIO_P27)   // wenxue user for contec
#define CONN_STAT              (GPIO_P30)   // wenxue user for contec 
#define UART_RTS				       (GPIO_P26)   // wenxue user for contec 
#define PT_STATE_CHANGE        (GPIO_P11)		//Extern MCU change ble state . wenxue contec 0-start advertising
#define PT_UART_RX_WAKEUP      (GPIO_P06)		//Extern MCU wakeup module when it tramsmit data with uart to module. wenxue contec
#define PT_UART_TX_WAKEUP      (GPIO_P13)		//Module wakeup extern MCU when the module's uart prepare to transmit data with uart.
//#define PT_STATE1				       (GPIO_P23)   
//#define PT_STATE0				       (GPIO_P24)

#endif 
