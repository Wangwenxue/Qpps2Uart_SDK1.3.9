
#include "App_pt.h"
#include "uart.h"
#include "timer.h"
#include "lib.h"
#include "timer.h"
#include "nvds.h"

struct pt_env_tag  pt_env;
struct PT_FIFO pt_data_fifo;	//data fifo


// wenxue contec
uint8_t tx_raw_tempbuf[PT_TX_FRAME_MAX];
uint8_t *tx_bufptr=tx_raw_tempbuf ;
uint8_t tx_length = 0;
bool cmd_set_default_parameter_flag=false;


void pt_rx_raw_done(void);
uint8_t pt_fifo_get(uint8_t *pBuf);
uint8_t pt_fifo_put(uint8_t *pPutDat, uint8_t PutLen);
void pt_fifo_init(void);
void ble_data_tx(void);

void pt_init(void);
void pt_rx_raw_frame_hdl(void);
void pt_enable(void);
void pt_disable(void);

/*      GPIO */
/**
 ****************************************************************************************
 * @brief Handles button press before key debounce.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
void app_event_gpio_rxwakeup_handler(void)
{
    // delay 20ms to debounce
    ke_evt_clear(1UL << EVENT_GPIO_UART_RX_WAKEUP_ID);

}

/**
 ****************************************************************************************
 * @brief   module init
 *
 *  Initialize the module related parameters.
 *****************************************************************************************
 */
void module_init(void)
{
	  //1:uart bautrate. if NVDS empty,used local uart bautrate,otherwise use NVDS uart bautrate.
    nvds_tag_len_t uart_baud_len= 1;
	  uint8_t uart_baud = 0;
	  if(NVDS_OK != nvds_get(NVDS_TAG_UART_BAUD,&uart_baud_len,(uint8_t *)&uart_baud))
	  {
		    pt_env.user_param.baudrate = UART_115200; // wenxue 115200
			  pt_env.user_param.raw_frame_timer = 5; //5ms
	  }
	  else
		{		
			if     (uart_baud == 0x00) pt_env.user_param.baudrate = UART_2400;
			else if(uart_baud == 0x01) pt_env.user_param.baudrate = UART_4800;
			else if(uart_baud == 0x02) pt_env.user_param.baudrate = UART_9600;
			else if(uart_baud == 0x03) pt_env.user_param.baudrate = UART_19200;
			else if(uart_baud == 0x04) pt_env.user_param.baudrate = UART_38400;
			else if(uart_baud == 0x05) pt_env.user_param.baudrate = UART_57600;
			else if(uart_baud == 0x06) pt_env.user_param.baudrate = UART_115200;
			
			
		  if     (pt_env.user_param.baudrate == UART_115200) pt_env.user_param.raw_frame_timer = 5; //5ms
			else if(pt_env.user_param.baudrate == UART_57600)  pt_env.user_param.raw_frame_timer = 6; //6ms
			else if(pt_env.user_param.baudrate == UART_38400)  pt_env.user_param.raw_frame_timer = 7; //7ms
			else if(pt_env.user_param.baudrate == UART_19200)  pt_env.user_param.raw_frame_timer = 8; //8ms
			else if(pt_env.user_param.baudrate == UART_9600)   pt_env.user_param.raw_frame_timer = 9; //9ms
			else if(pt_env.user_param.baudrate == UART_4800)   pt_env.user_param.raw_frame_timer = 10; //10ms
			else if(pt_env.user_param.baudrate == UART_2400)   pt_env.user_param.raw_frame_timer = 11; //11ms
		
		}

	  // uart0 passthrough init wenxue 
    uart_init(PT_UART_PORT, __USART_CLK, (enum UART_BAUDRATE)pt_env.user_param.baudrate);
    uart_tx_enable(PT_UART_PORT, MASK_ENABLE);
    uart_rx_enable(PT_UART_PORT, MASK_ENABLE);
	
		//2:ble name. if NVDS empty,used local name,otherwise use NVDS name.
		nvds_tag_len_t read_len = BLE_NAME_LEN_MAX+1;
		if(NVDS_OK != nvds_get(NVDS_TAG_DEVICE_NAME,&read_len,&pt_env.user_param.ble_name[0]))
		{
			pt_env.user_param.ble_name_len = strlen(QN_LOCAL_NAME);
			strcpy((char *)&pt_env.user_param.ble_name[0], QN_LOCAL_NAME);
		}
		else
		{
			pt_env.user_param.ble_name_len = read_len - 1 ;
		}
	
	 //3:adv interval. if NVDS empty,used local adv interval,otherwise use NVDS adv interval.
	  nvds_tag_len_t adv_interval_len= 2;
	  uint8_t adv_interval[2] = {0x00,0x00};
	  if(NVDS_OK != nvds_get(NVDS_TAG_ADV_INTERVAL,&adv_interval_len,(uint8_t *)&adv_interval[0]))
	  {
		   pt_env.user_param.adv_intv_min= GAP_ADV_100MS_INTV; //  wenxue contec 广播间隔 初始化 100ms(0x00A0 *0.625=100)
	     pt_env.user_param.adv_intv_max= GAP_ADV_100MS_INTV;
	  }
	  else
		{		 
			pt_env.user_param.adv_intv_min= (uint16_t)(adv_interval[0]+(adv_interval[1]<<8));
			pt_env.user_param.adv_intv_max= (uint16_t)(adv_interval[0]+(adv_interval[1]<<8));	
		}
	
	 //4:connect interval.if NVDS empty,used local connect interval,otherwise use NVDS connect interval.
		nvds_tag_len_t connect_interval_len= 2;
	  uint8_t connect_interval[2] = {0x00,0x00};
	  if(NVDS_OK != nvds_get(NVDS_TAG_CONN_INTERVAL,&connect_interval_len,(uint8_t *)&connect_interval[0]))
	  {
		   pt_env.user_param.conn_intv_min= GAP_CONN_100MS_INTV; //  wenxue contec 广播间隔 初始化 100ms(0x00A0 *0.625=100)
	     pt_env.user_param.conn_intv_max= GAP_CONN_100MS_INTV;
	  }
	  else
		{		 
			pt_env.user_param.conn_intv_min= (uint16_t)(connect_interval[0]+(connect_interval[1]<<8));
			pt_env.user_param.conn_intv_max= (uint16_t)(connect_interval[0]+(connect_interval[1]<<8));	
		}
	
	 //5:use NVDS Mac Address
	 nvds_tag_len_t len_temp = 0x06;
	 if (NVDS_OK != nvds_get(NVDS_TAG_BD_ADDRESS, &len_temp, &pt_env.user_param.mac_address[0]))
   {
		  strcpy((char *)&pt_env.user_param.mac_address[0], MAC_ADRRESS_DEFAULT);
   }
   else
   {
		 ;
   }
		
	 //6:Tx power. if NVDS empty,used local Tx power,otherwise use NVDS Tx power.	
	  nvds_tag_len_t tx_power_len= 1;
	  uint8_t tx_power[1] = {0x00};
	  if(NVDS_OK != nvds_get(NVDS_TAG_TX_POWER,&tx_power_len,(uint8_t *)&tx_power[0]))
	  {
	   	rf_tx_power_level_set((enum TX_POWER)TX_GAIN_LEVEL10); // default Tx Power 0dbm				
	  }
	  else
		{	
      rf_tx_power_level_set((enum TX_POWER)tx_power[0]);				
		}
			
  //7:adv user data wenxue 全局变量默认就是0
	 pt_env.user_param.adv_user_data_len=0; 
	 memset(pt_env.user_param.adv_user_data,0,sizeof(pt_env.user_param.adv_user_data));
		
  //8:wakeup exmcu timer
	 pt_env.user_param.wakeup_exmcu_timer = 0;
		
	 // GPIO Init
   pt_gpio_init();
	 
	 // Passthrogh Init
	 pt_init();
}

/**
 ****************************************************************************************
 * @brief   pt_swfc_gpio_init initilization
 * @description
 *  This function init gpio for software flow control mode
 ****************************************************************************************
 */
void pt_gpio_init()
{
	// wenxue contec uart_cts setting
  gpio_pull_set(UART_CTS, GPIO_PULL_UP);
  gpio_set_direction_field(UART_CTS, (uint32_t)GPIO_INPUT);
 
 // PT_UART_RX_WAKEUP
	gpio_wakeup_config(PT_UART_RX_WAKEUP, GPIO_WKUP_BY_LOW); //extern MCU wakup module 
	gpio_enable_interrupt(PT_UART_RX_WAKEUP);
	
	// PT_UART_TX_WAKEUP
	gpio_set_direction_field(PT_UART_TX_WAKEUP,(uint32_t)GPIO_OUTPUT);//module wakeup extern MCU
	gpio_write_pin(PT_UART_TX_WAKEUP,GPIO_HIGH);
	
	// PT_STATE1,PT_STATE0
//	gpio_set_direction_field(PT_STATE1|PT_STATE0,(uint32_t)GPIO_OUTPUT);
	
	
	// wenxue  CONN_STAT used for contec
	gpio_set_direction_field(CONN_STAT,(uint32_t)GPIO_OUTPUT);
	gpio_write_pin(CONN_STAT,GPIO_LOW);
	
		// wenxue  UART_RTS used for contec 
	gpio_set_direction_field(UART_RTS,(uint32_t)GPIO_OUTPUT);
		
  gpio_wakeup_config(PT_STATE_CHANGE, GPIO_WKUP_BY_LOW);// user for start advertising
  gpio_enable_interrupt(PT_STATE_CHANGE);
	
}

void pt_init()
{
	  pt_env.state = PT_DEEPSLEEP;
	  pt_state_set(PT_DEEPSLEEP);
	
	  //UART TX list init
    pt_env.tx_state = PT_UART_TX_IDLE;	//initialize tx state
    co_list_init(&pt_env.queue_tx);			//init TX queue
	
		// init timer0 for module wakeup exmcu.
		timer_init(TX_WAKEUP_TIMER, timer0_callback);
	
	  //Set call back func for GPIO (PT_UART_RX_WAKEUP)
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_GPIO_UART_RX_WAKEUP_ID,
										  app_event_gpio_rxwakeup_handler)) 
    ASSERT_ERR(0);
		
		
		if(KE_EVENT_OK != ke_evt_callback_set(EVENT_UART_TX_ID, 
										  pt_tx_done))
    ASSERT_ERR(0);
		
	
	
	if(KE_EVENT_OK != ke_evt_callback_set(PT_RX_RAW_FRAME_EVENT_ID, 
										  pt_rx_raw_frame_hdl))
	ASSERT_ERR(0);
	
	
	if(KE_EVENT_OK != ke_evt_callback_set(EVENT_GPIO_STCHANGE_ID,
													app_event_gpio_stchange_handler)) 
	ASSERT_ERR(0);

	if(KE_EVENT_OK != ke_evt_callback_set(PT_RX_RAW_FRAME_EVENT_ID, 
													pt_rx_raw_frame_hdl))
	ASSERT_ERR(0);
	
	
	//Start uart to receive command wenxue 开始接收命令集
	pt_rx_pdu_start();
	
}

void pt_enable(void)
{
  	pt_rx_raw_start();
}
void pt_disable(void)
{
  	uart_rx_int_enable(QN_UART0, MASK_DISABLE);
}

uint32_t timer3_CT = 0;
void timer3_callback(void)
{
	timer3_CT++;

	//receive one frame data,disable uart and timer.
//	uart_rx_int_enable(QN_UART0, MASK_DISABLE);
	timer_enable(QN_TIMER3, MASK_DISABLE);
	timer_timer_ClrIntFlag(QN_TIMER3, TIMER_MASK_TOVF|TIMER_MASK_OCF|TIMER_MASK_ICF);
	
	if(pt_data_fifo.elemt[pt_data_fifo.in].len) // 判断是否为0，代表是否有数据，有可能是20整字节后进入的这里，那就为0
	{
		pt_data_fifo.in++;	//in this place should pt_data_fifo.in >= pt_data_fifo.out  加1的原因是 这一次会把小于20字节的数据都发走，下一次得存放到下一个数组内
		if(pt_data_fifo.in >= PT_FIFO_X_SIZE)
		{
			pt_data_fifo.in = 0;
		}
		if(pt_data_fifo.in == pt_data_fifo.out) // 饶了一圈又回来了
		{
			pt_data_fifo.full_ct++;
			pt_data_fifo.state = PT_FIFO_FULL;
		}
		else
		{
			pt_data_fifo.state = PT_FIFO_NOFU;
		}
		pt_data_fifo.elemt[pt_data_fifo.in].len = 0; // wenxue len=0 下一个结构体的起始位置
		if((pt_env.ble_tx_state != BLE_TX_GOING)&&(pt_data_fifo.state != PT_FIFO_EMPT))
		{
			// wenxue 发送数据
			ke_evt_set(1UL << PT_RX_RAW_FRAME_EVENT_ID);	//don't send ble data in uart interrupt, set an env to kernel
			pt_env.ble_tx_state = BLE_TX_GOING;
		}

	}
	
	if(pt_env.uart_rx_state != UART_RX_STOP)	// 等于UART_RX_STOP 不再接收数据
	    uart_read(PT_UART_PORT, pt_data_fifo.elemt[pt_data_fifo.in].buf+pt_data_fifo.elemt[pt_data_fifo.in].len, 1,  pt_rx_raw_done);

}

void pt_fifo_init(void)
{
  pt_data_fifo.elemt[0].len  = 0;
	pt_data_fifo.in = 0;
	pt_data_fifo.out = 0;
	pt_data_fifo.state = PT_FIFO_EMPT;
}
void pt_rx_raw_start(void)
{
	pt_env.rx_state = PT_RX_RAW_START;
	
	pt_fifo_init();
  pt_env.ble_tx_state = BLE_TX_STOP;
	pt_env.uart_rx_state = UART_RX_GOING;

	
	uart_uart_GetRXD(QN_UART0);//before read start,clear fifo data.
	uart_uart_ClrIntFlag(QN_UART0, 0x0000003d);//before read start,clear all the interrupt flag
	
  uart_read(PT_UART_PORT, pt_data_fifo.elemt[0].buf, 1, pt_rx_raw_done);
	uart_rx_int_enable(PT_UART_PORT, MASK_ENABLE);
	
	// Rx timer config
	timer_config(QN_TIMER3, TIMER_PSCAL_DIV, TIMER_COUNT_MS(pt_env.user_param.raw_frame_timer , TIMER_PSCAL_DIV));
	timer_init(QN_TIMER3, timer3_callback);

}

/*
* note: when UART baud rade is 115200, UART speed is higher than ble.
*          baud rade is 2400, 9600, 38400, BLE speed is higher than UART. 
* data fifo should deal with it
*/
void pt_rx_raw_done(void)	
{
	pt_env.rx_state = PT_RX_RAW_ONGOING;  // wenxue 
	
	pt_data_fifo.elemt[pt_data_fifo.in].len++; // 长度加 1 单个数据包结构体中的长度
	if(pt_data_fifo.elemt[pt_data_fifo.in].len >= PT_FIFO_Y_SIZE) // wenxue 当长度达到20字节时
	{
		pt_data_fifo.in++;	      //in this place should pt_data_fifo.in >= pt_data_fifo.out  // wenxue 20字节数据包 计数加1
		if(pt_data_fifo.in >= PT_FIFO_X_SIZE) // 长度大于定义的最大长度200组时,缓冲区溢出
		{
			pt_data_fifo.in = 0;
		}
		if(pt_data_fifo.in == pt_data_fifo.out) // 什么时候会相等？？？ 接收饶了一圈又追上发送了
		{
			pt_data_fifo.full_ct++;
			pt_data_fifo.state = PT_FIFO_FULL;
			pt_env.uart_rx_state = UART_RX_STOP;
			pt_env.state = PT_CONN_FULL;
			pt_state_set(PT_CONN_FULL);// Connected and busy，设置UART_RTS,保证串口不再接收数据
			
		}
		else
		{
			pt_data_fifo.state = PT_FIFO_NOFU;
		}
		pt_data_fifo.elemt[pt_data_fifo.in].len = 0;// len 重头开始计数
		
		// wenxue 只有在pt_env.ble_tx_state != BLE_TX_GOING 才置event,即发送数据。 否则只需要收数据即可，因为BLE这时已经在发送着数据
		if((pt_env.ble_tx_state != BLE_TX_GOING)&&(pt_data_fifo.state != PT_FIFO_EMPT))//wenxue pt_data_fifo.state != PT_FIFO_EMPT 这是一定的
		{
			// wenxue 开始发送数据
			ke_evt_set(1UL << PT_RX_RAW_FRAME_EVENT_ID);	//don't send ble data in uart interrupt, set an env to kernel
			pt_env.ble_tx_state = BLE_TX_GOING; // 表示正在发送数据
		}
	}// end   if(pt_data_fifo.elemt[pt_data_fifo.in].len >= PT_FIFO_Y_SIZE) // wenxue 当长度达到20字节时

	
	
	// 继续接收下一字节
	if(pt_env.uart_rx_state != UART_RX_STOP)	 // 若果等于UART_RX_STOP,串口就不再接收数据了
	{
		//stop timer, prepare next byte wenxue 一收到数据就关闭定时器
		do{timer_timer_SetCRWithMask(QN_TIMER3, TIMER_MASK_TEN, MASK_DISABLE);}
		while((UART_RX_TIMER->CR & 0x00000001) == 1);

		// 继续接收下一字节
	  uart_read(PT_UART_PORT, pt_data_fifo.elemt[pt_data_fifo.in].buf+pt_data_fifo.elemt[pt_data_fifo.in].len, 1,  pt_rx_raw_done);

	    //start timer // wenxue  在revice next byte之后就打开定时器,如果定时时间到，表示这段时间没有接收到数据。
	    do{timer_enable(QN_TIMER3, MASK_ENABLE);}
	    while((UART_RX_TIMER->CR & 0x00000001) == 0);
	}
        
}


void pt_rx_pdu_payl(void)
{
	pt_env.rx_state = PT_RX_PDU_PAYL;
	uart_read(PT_UART_PORT, &pt_env.rx_pdu_buf[pt_env.rx_pdu_wridex].param[1], pt_env.rx_pdu_len, pt_rx_pdu_done);
}


void pt_rx_raw_frame_hdl(void)
{
	ke_evt_clear(1UL << PT_RX_RAW_FRAME_EVENT_ID);
	ble_data_tx(); // wenxue 调用发送函数
	
}
void ble_data_tx()
{
	app_qpps_env->tx_buffer_available--;
	// wenxue ! BLE 发送的顺序是从前往后依次发送
	app_qpps_data_send(app_qpps_env->conhdl, 0, pt_data_fifo.elemt[pt_data_fifo.out].len, pt_data_fifo.elemt[pt_data_fifo.out].buf);
	pt_data_fifo.out ++;
	if(pt_data_fifo.out == PT_FIFO_X_SIZE)
	{
		pt_data_fifo.out = 0;// 发到最大值置0,重头发
	}
	if(pt_data_fifo.out == pt_data_fifo.in) // 发送的赶上接收的了
	{
		pt_data_fifo.state = PT_FIFO_EMPT; // wenxue ! 
		pt_data_fifo.empty_ct++;
		
		// wenxue pt_env.uart_rx_state == UART_RX_STOP 是之前收的太快，不再接收，这时发送发到了接收的位置
		if(pt_env.uart_rx_state == UART_RX_STOP)	
		{
			pt_rx_raw_start(); //重新初始化 重头开始
			pt_env.state = PT_CONN_EMPTY;// notification 已经打开串口可以接收数据
			pt_state_set(PT_CONN_EMPTY);// Connected and idle		
		}
	}
		
}



void pt_rx_pdu_start(void)
{
	uart_uart_GetRXD(PT_UART_PORT);//before read start,clear fifo data.
	uart_uart_ClrIntFlag(PT_UART_PORT, 0x0000003d);//before read start,clear all the interrupt flag

	pt_env.rx_state = PT_RX_PDU_STATR;
	uart_read(PT_UART_PORT, (uint8_t *)&pt_env.rx_pdu_type, 1, pt_rx_pdu_done); 
	uart_rx_int_enable(PT_UART_PORT, MASK_ENABLE);
	
	
}


void pt_rx_pdu_hdr(void)
{
	pt_env.rx_state = PT_RX_PDU_HDR;
	uart_read(PT_UART_PORT, (uint8_t *)&pt_env.rx_pdu_id, 2, pt_rx_pdu_done);
}

void pt_rx_pdu_done(void) 
{
	switch(pt_env.rx_state)
    {
        // Message Type received
        case PT_RX_PDU_STATR:  // initial state
				{
            if (pt_env.rx_pdu_type == PT_PDU_TYPE_CMD )
            {
                pt_env.rx_pdu_error = TRUE;
                pt_rx_pdu_hdr();
            }
            else
            {
                if (pt_env.rx_pdu_error == TRUE)
                {
                    pt_env.rx_pdu_error = FALSE;
                    pt_pdu_send_error(PT_TYPE_ERROR);
                }
                pt_rx_pdu_start();
            }
				} break;
				
				
		         // Message ID and Parameter Length received
    case PT_RX_PDU_HDR:
		{
			//error process
			if (pt_env.rx_pdu_id > PT_PDU_CMD_MAX)
			{
					pt_pdu_send_error(PT_PDU_OOR_ERROR);
					pt_rx_pdu_start();
					break;
			}
			
			pt_env.rx_pdu_buf[pt_env.rx_pdu_wridex].id = APP_SYS_UART_DATA_IND;
			pt_env.rx_pdu_buf[pt_env.rx_pdu_wridex].dest_id = TASK_APP;
			pt_env.rx_pdu_buf[pt_env.rx_pdu_wridex].src_id = (uint16_t)(pt_env.rx_pdu_type << 8 | pt_env.rx_pdu_id);
			pt_env.rx_pdu_buf[pt_env.rx_pdu_wridex].param_len = pt_env.rx_pdu_len + 1;
			pt_env.rx_pdu_buf[pt_env.rx_pdu_wridex].param[0] = pt_env.rx_pdu_len;
			
			if(pt_env.rx_pdu_len == 0)
			{
				pt_env.rx_state = PT_RX_PDU_IDLE;
				
				ke_msg_send(pt_env.rx_pdu_buf[pt_env.rx_pdu_wridex].param);
				
				// 这一段没看懂？？？
				pt_env.rx_pdu_wridex++;
				if(pt_env.rx_pdu_wridex == PT_RX_PDU_BUF_DEEP)
				pt_env.rx_pdu_wridex = 0;
				
				pt_rx_pdu_start();
			}
			else
			{
				pt_rx_pdu_payl();
			}
			
		}break;		
			
       // Parameter received
    case PT_RX_PDU_PAYL:
		{	
			pt_env.rx_state = PT_RX_PDU_IDLE;
		
			ke_msg_send(pt_env.rx_pdu_buf[pt_env.rx_pdu_wridex].param);
			
			pt_env.rx_pdu_wridex++;
			if(pt_env.rx_pdu_wridex == PT_RX_PDU_BUF_DEEP)
			pt_env.rx_pdu_wridex = 0;
		
			pt_rx_pdu_start();

		}break;

		
			
    default:
        break;
    }
}

void pt_pdu_send_error(uint8_t reason)
{
    uint8_t pdu[] = {PT_PDU_TYPE_EVT, 0, 1, 0};
    pdu[1] = PT_PDU_EVT_ERROR;
    pdu[3] = reason;
    pt_pdu_send(sizeof(pdu), pdu);
}

void app_event_pt_tx_handler(void)
{
	gpio_write_pin(PT_UART_TX_WAKEUP,GPIO_HIGH);
	
	ke_evt_set(1UL<<EVENT_UART_TX_ID);
}

void pt_uart_write(struct ke_msg *msg)
{
    //go to start tx state
    pt_env.tx_state = PT_UART_TX_ONGOING;
	
	
	if(pt_env.user_param.wakeup_exmcu_timer == 0)
	{
		gpio_write_pin(PT_UART_TX_WAKEUP,GPIO_LOW);
		uart_write(PT_UART_PORT, ((uint8_t *)&msg->param), msg->param_len, app_event_pt_tx_handler); // wenxue  中断发送 完了才会调用回调函数
	}
	else
	{
		gpio_write_pin(PT_UART_TX_WAKEUP,GPIO_LOW);
		timer_config(TX_WAKEUP_TIMER, TIMER_PSCAL_DIV, TIMER_COUNT_MS(pt_env.user_param.wakeup_exmcu_timer, TIMER_PSCAL_DIV));
		timer_enable(TX_WAKEUP_TIMER, MASK_ENABLE);
		
	}
}

// Push msg into eaci tx queue
static void pt_push(struct ke_msg *msg)
{
    // Push the message into the list of messages pending for transmission
	   co_list_push_back(&pt_env.queue_tx, &msg->hdr);

    // Check if there is no transmission ongoing
    if (pt_env.tx_state == PT_UART_TX_IDLE)
        // Forward the message to the HCI UART for immediate transmission
        pt_uart_write(msg);
}

 /****************************************************************************************
 * @brief EACI send PDU
 *
 ****************************************************************************************
 */
void pt_pdu_send(uint8_t len, uint8_t *par)
{
    // Allocate one msg for EACI tx
    uint8_t *msg_param = (uint8_t*)ke_msg_alloc(0, 0, 0, len);

    // Save the PDU in the MSG
    memcpy(msg_param, par, len);

    //extract the ke_msg pointer from the param passed and push it in HCI queue
    pt_push(ke_param2msg(msg_param));
}


/**
 ****************************************************************************************
 * @brief After-process when one PDU has been sent.
 *
 ****************************************************************************************
 */
void pt_tx_done(void)
{	
    struct ke_msg * msg;
    // Clear the event
    ke_evt_clear(1<<EVENT_UART_TX_ID);
    // Go back to IDLE state
    pt_env.tx_state = PT_UART_TX_IDLE;
    //release current message (which was just sent)
    msg = (struct ke_msg *)co_list_pop_front(&pt_env.queue_tx);
    // Free the kernel message space
    ke_msg_free(msg);
    // Check if there is a new message pending for transmission
    if ((msg = (struct ke_msg *)co_list_pick(&pt_env.queue_tx)) != NULL)
    {
        // Forward the message to the HCI UART for immediate transmission
        pt_uart_write(msg);
    }

  #if 0
	if((set_uart_baud_flag == true)&& (msg = (struct ke_msg *)co_list_pick(&pt_env.queue_tx)) == NULL) // wenxue used for contec
	{
	  uart_init(PT_UART_PORT, USARTx_CLK(0), (enum UART_BAUDRATE)pt_env.user_param.baudrate); // wenxue test
	  uart_tx_enable(PT_UART_PORT, MASK_ENABLE);
	  uart_rx_enable(PT_UART_PORT, MASK_ENABLE);
	  pt_rx_pdu_start(); 
	  set_uart_baud_flag = false;
	}
	#endif
}


void app_pt_gpio_stchange_process(void)
{
   if(gpio_read_pin(PT_STATE_CHANGE) == GPIO_LOW)
	{
		switch(pt_env.state)
		{
			case PT_DEEPSLEEP:
			{
				if(APP_IDLE == ke_state_get(TASK_APP))
				{
					if(!app_qpps_env->enabled)
					{
						// start adv
						app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
								app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
								app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
								pt_env.user_param.adv_intv_min, pt_env.user_param.adv_intv_max);
#if (QN_DEEP_SLEEP_EN)
                        // prevent entering into deep sleep mode
                        sleep_set_pm(PM_SLEEP);
#endif
					}
				}
			}break;
			case PT_ADV:
			{
				if(APP_ADV == ke_state_get(TASK_APP))
				{
					// stop adv
					app_gap_adv_stop_req();

#if (QN_DEEP_SLEEP_EN)
                    // allow entering into deep sleep mode
                    sleep_set_pm(PM_DEEP_SLEEP);
#endif
				}
			}break;
			case PT_CONN_EMPTY:
			{
				app_gap_discon_req(0);
			}break;
			case PT_CONN_FULL:
			{
			   app_gap_discon_req(0);
			}break;
			
			default :
			{
				ASSERT_ERR(0);
			}break;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Handles button press before key debounce.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
void app_event_gpio_stchange_handler(void)
{
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
    if (sleep_env.deep_sleep)
    {
        sleep_env.deep_sleep = false;
        // start 32k xtal wakeup timer
        wakeup_32k_xtal_start_timer();
    }
#endif

  
    ke_evt_clear(1UL << EVENT_GPIO_STCHANGE_ID);
		
		app_pt_gpio_stchange_process();
}



/**
 ****************************************************************************************
 * @brief   pt_state_set
 * @param[in]    
 * @param[in]    
 * @description
 *  This function set the pt state
 ****************************************************************************************
 */
void pt_state_set(enum pt_st state)
{
	switch(state)
	{
		case PT_DEEPSLEEP:// deep sleep 状态
		{
			// wenxue contec
			gpio_write_pin(CONN_STAT, GPIO_LOW);
			// wenxue contec
			gpio_write_pin(UART_RTS, GPIO_HIGH); 
			
		}break;
		case PT_ADV: // 广播状态
		{		
			// wenxue contec
			gpio_write_pin(CONN_STAT, GPIO_LOW);
			// wenxue contec
			gpio_write_pin(UART_RTS, GPIO_HIGH); 
		}break;
		case PT_CONN_EMPTY:// 连接状态,可以接收串口数据
		{
			// wenxue contec
			gpio_write_pin(CONN_STAT, GPIO_HIGH); 
			// wenxue contec
			gpio_write_pin(UART_RTS, GPIO_LOW); 
		}break;
		case PT_CONN_FULL:// 连接状态,不可以接收串口数据
		{	
			// wenxue contec
			gpio_write_pin(CONN_STAT, GPIO_HIGH);
			// wenxue contec
			gpio_write_pin(UART_RTS, GPIO_HIGH); 
		}break;
		
		default :break;
	}
}



void app_pt_task_msg_hdl(ke_msg_id_t const msgid, void const *param)
{
    switch(msgid)
    {
		//start adv complete
		case GAP_SET_MODE_REQ_CMP_EVT:
		{
				if(pt_env.state == PT_BETWEEM_ADV_CONN)	// if this adv is after disconnect, begin pdu rx.
				{
					pt_rx_pdu_start(); 			//restart uart receive pdu data
				}
			pt_env.state = PT_ADV;
			pt_state_set(PT_ADV);
		}break;
		
		
	  //stop adv complete
		case GAP_ADV_REQ_CMP_EVT:
		{
			pt_env.state = PT_DEEPSLEEP;
			pt_state_set(PT_DEEPSLEEP);
		}break;	
		
				// connect to client complete
		case GAP_LE_CREATE_CONN_REQ_CMP_EVT://The module has been connected with the extern machine,but the notifications aren't configed.
		{
			pt_env.state = PT_CONN_FULL; // 已经连接但是Notification还没有打开
			pt_state_set(PT_CONN_FULL);
			
		}break;
		
				//disconnect complete
		case GAP_DISCON_CMP_EVT://after disconnect with the equipment,the module will advertising auto.
		{

				//disable uart rx interrupt and timer interrupt
				uart_rx_int_enable(PT_UART_PORT, MASK_DISABLE);
				timer_enable(UART_RX_TIMER, MASK_DISABLE);
				ke_evt_clear(1UL << PT_RX_RAW_FRAME_EVENT_ID);
			
			//clear all fifo data
			//	pt_env.rx_raw_rdidex = 0;
			//	pt_env.rx_raw_wridex = 0;
				
				pt_env.state = PT_BETWEEM_ADV_CONN;
			
			// start adv
			app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
							app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
							app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
							pt_env.user_param.adv_intv_min, pt_env.user_param.adv_intv_max);
		}break;
		
		case QPPS_CFG_INDNTF_IND:
		{		
			pt_rx_raw_start(); //stop uart receive pdu data and start raw data receive. wenxue
			pt_env.state = PT_CONN_EMPTY;// notification 已经打开串口可以接收数据
			pt_state_set(PT_CONN_EMPTY);// Connected and idle		
			
		}
            break;
		
		//sent ble data to ext MCU
		case QPPS_DAVA_VAL_IND: // BLE接收到数据
		{
			struct qpps_data_val_ind* par = (struct qpps_data_val_ind*)param;

			// wenxue contec
			memcpy(tx_bufptr, &(par->data[0]), par->length);
			tx_bufptr = tx_bufptr+par->length;
			tx_length = tx_length + par->length;
			

		    if(gpio_read_pin(UART_CTS)== GPIO_LOW) // 立刻发出去一帧数据
		    {
			  pt_pdu_send(tx_length, tx_raw_tempbuf);
			  memset(tx_raw_tempbuf, 0, sizeof(tx_raw_tempbuf)); // clear buffer  
				tx_length = 0;
				tx_bufptr = tx_raw_tempbuf;
		    }
			else   // don't send data out 
			{
			  if(tx_length >= PT_TX_FRAME_MAX-20)// 当超过256-20 字节时就发送出去
				{
			  	 pt_pdu_send(tx_length, tx_raw_tempbuf);
					 memset(tx_raw_tempbuf, 0, sizeof(tx_raw_tempbuf)); // clear buffer  
           tx_length = 0;			
           tx_bufptr = tx_raw_tempbuf;				
				}
			}
		 

		}break;
		
		case GAP_SET_DEVNAME_REQ_CMP_EVT:
		{
				QPRINTF("save device name sucesful \r\n"); // wenxue
			
			  if(cmd_set_default_parameter_flag!= true)
				{
					//respone device name to exturn mcu. wenxue used for contec
					uint8_t pdu[4] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_SET_DEVNAME, 0x01,0x00}; 
					pt_pdu_send(0x04, pdu);	
			  } 
				else
					cmd_set_default_parameter_flag=false;
	
		}break;
		
		
		   default :break;
	}
		
	}

	
	void timer0_callback(void)
{
	timer_enable(TX_WAKEUP_TIMER, MASK_DISABLE);
	timer_timer_ClrIntFlag(TX_WAKEUP_TIMER, TIMER_MASK_TOVF|TIMER_MASK_OCF|TIMER_MASK_ICF);
	
	struct ke_msg * msg;
	if ((msg = (struct ke_msg *)co_list_pick(&pt_env.queue_tx)) != NULL)
	uart_write(PT_UART_PORT, ((uint8_t *)&msg->param), msg->param_len, app_event_pt_tx_handler);
}



void pt_pdu_hdl(uint8_t pdu_type, uint8_t pdu_id, uint8_t param_len, uint8_t const *param)
{
    switch (pdu_type)
    {
        case PT_PDU_TYPE_CMD:
				{
						pt_pdu_cmd_hdl(pdu_id, param_len, param);
					
				}break;

        default:
            break;
    }
}
 

void pt_pdu_cmd_hdl(uint8_t pdu_id, uint8_t param_len, uint8_t const *param)
{
	switch(pdu_id)
	{
					
		case PT_PDU_CMD_SET_UART_BAUD: // wenxue used for contec
			pt_pdu_cmd_set_uart_baud(param_len, param);
	  	break;
		
		case PT_PDU_CMD_SET_DEVNAME:
	   	pt_pdu_cmd_set_devname_hdl(param_len, param);
		break;
		
			case PT_PDU_CMD_PER_UPDATE_ADV_PARAM:
			pt_pdu_cmd_per_update_adv_param(param_len, param);
			break;
		
		case PT_PDU_CMD_PER_UPDATE_CONN_PARAM:
			pt_pdu_cmd_per_update_conn_param(param_len, param);
			break;
				
		
		case PT_PDU_CMD_RD_ADD:
			pt_pdu_cmd_rd_add_hdl(param_len ,param);
			break;
		
		
		case PT_PDU_CMD_SET_TX_POWER:
			pt_pdu_cmd_set_tx_power(param_len, param);
			break;
		
				
		case PT_PDU_CMD_SET_ADV_USER_DATA:
			pt_pdu_cmd_set_adv_user_data(param_len, param);
	  	break;
		
		case PT_PDU_CMD_SET_WAKEUP_MCU_TIMER:
			pt_pdu_cmd_set_wakeup_exmcu_timer(param_len, param);
			break;
		
		case PT_PDU_CMD_SET_DEFAULT_PARAMETER: // wenxue used for contec
			pt_pdu_cmd_set_default_paramter(param_len, param);
			break;
		
		default :
			break;
	}
}


/* 查询 设置串口波特率 */
void pt_pdu_cmd_set_uart_baud(uint8_t param_len,uint8_t const *param)
{
	uint8_t uart_baud;
	
	if(param_len == 0) // 查询串口波特率
	{
		  if     (pt_env.user_param.baudrate == UART_2400)   uart_baud = 0x00;
	    else if(pt_env.user_param.baudrate == UART_4800)   uart_baud = 0x01;
	    else if(pt_env.user_param.baudrate == UART_9600)   uart_baud = 0x02;
	    else if(pt_env.user_param.baudrate == UART_19200)  uart_baud = 0x03; 
		  else if(pt_env.user_param.baudrate == UART_38400)  uart_baud = 0x04;
	    else if(pt_env.user_param.baudrate == UART_57600)  uart_baud = 0x05;
	    else if(pt_env.user_param.baudrate == UART_115200) uart_baud = 0x06;
		
	  	uint8_t pdu[] = {PT_PDU_TYPE_EVT, PT_PDU_EVT_UART_BAUD, 1,uart_baud};
	    pt_pdu_send(sizeof(pdu), pdu);
	}
		
	else if(param_len == 1) //设置串口波特率
	{
		
		if     (param[0] == 0x00) {pt_env.user_param.baudrate = UART_2400;  pt_env.user_param.raw_frame_timer = 11;}
		else if(param[0] == 0x01) {pt_env.user_param.baudrate = UART_4800;  pt_env.user_param.raw_frame_timer = 10;}
		else if(param[0] == 0x02) {pt_env.user_param.baudrate = UART_9600;  pt_env.user_param.raw_frame_timer = 9;}
		else if(param[0] == 0x03) {pt_env.user_param.baudrate = UART_19200; pt_env.user_param.raw_frame_timer = 8;}
		else if(param[0] == 0x04) {pt_env.user_param.baudrate = UART_38400; pt_env.user_param.raw_frame_timer = 7;}
		else if(param[0] == 0x05) {pt_env.user_param.baudrate = UART_57600; pt_env.user_param.raw_frame_timer = 6;}
		else if(param[0] == 0x06) {pt_env.user_param.baudrate = UART_115200;pt_env.user_param.raw_frame_timer = 5;}
		else 
		{
			pt_pdu_send_error(PT_PDU_OOR_ERROR);
			return ;
		}
		
		if(NVDS_OK != nvds_put(NVDS_TAG_UART_BAUD, param_len, (uint8_t*)param))
		{
			  //respone uart baud to exturn mcu. wenxue used for contec
				uint8_t pdu[4] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_UART_BAUD, 0x01,0x01}; 
				QPRINTF("set uart baud failed \r\n"); // wenxue
				pt_pdu_send(0x04, pdu);
		}
		else
		{
			  //respone uart baud to exturn mcu. wenxue used for contec
				uint8_t pdu[4] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_UART_BAUD,0x01,0x00}; 
				QPRINTF("set uart baud succeed \r\n"); // wenxue
				pt_pdu_send(0x04, pdu);		
		}
		
		 ke_timer_set(APP_SYS_UART_BAUD_TIMER, TASK_APP, 1);	    	
	}
	else // param_len >1   error
	{
		pt_pdu_send_error(PT_PDU_OOR_ERROR);
	}
	
}



/* 查询 设置设备名称 */
void pt_pdu_cmd_set_devname_hdl(uint8_t param_len, uint8_t const *param)
{
	uint8_t changed_temp = 0;
	
	
	if(param_len == 0) // 查询设备名称
	{
		//respone device name to exturn mcu.
		uint8_t pdu[BLE_NAME_LEN_MAX+3] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_SET_DEVNAME, pt_env.user_param.ble_name_len};
		memcpy(&pdu[3], pt_env.user_param.ble_name, pt_env.user_param.ble_name_len);
		pt_pdu_send(pt_env.user_param.ble_name_len+3, pdu);
		  
	}
		

	//receive device name
	else if ((param_len>0) && (param_len <= BLE_NAME_LEN_MAX)) 
	{
		//(1)compare current devide name with uart receive device name
		changed_temp = memcmp(param,pt_env.user_param.ble_name,(param_len>=pt_env.user_param.ble_name_len ? param_len : pt_env.user_param.ble_name_len));
		//(2)name need change ,save device name to NVDS
		if(changed_temp!=0) //name not equal
		{
			//save failed
			if(NVDS_OK != nvds_put(NVDS_TAG_DEVICE_NAME, param_len+1, (uint8_t*)param))
			{		
				//respone device name to exturn mcu. wenxue used for contec
				uint8_t pdu[4] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_SET_DEVNAME, 0x01,0x01}; 
				QPRINTF("save device name failed \r\n"); // wenxue
				pt_pdu_send(0x04, pdu);
			}
			else //save sucessful
			{
				//save local name, adv will use this new name.
				pt_env.user_param.ble_name_len = param_len;
				memcpy(pt_env.user_param.ble_name, param, pt_env.user_param.ble_name_len);
				//set the device name to gap.
				app_gap_set_devname_req(pt_env.user_param.ble_name, pt_env.user_param.ble_name_len);
			}
		}
		//uart receive device name is equal to current name, only response
		else
		{
			//respone device name to exturn mcu.
			uint8_t pdu[BLE_NAME_LEN_MAX+3] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_SET_DEVNAME, pt_env.user_param.ble_name_len};
			memcpy(&pdu[3], pt_env.user_param.ble_name, pt_env.user_param.ble_name_len);
			pt_pdu_send(pt_env.user_param.ble_name_len+3, pdu);
		}

	}
	
	//read device name or lenth is out of range
	else if(param_len>BLE_NAME_LEN_MAX) 
	{
		pt_pdu_send_error(PT_PDU_OOR_ERROR);
	}

}


/* 查询 设置广播间隔*/
void pt_pdu_cmd_per_update_adv_param (uint8_t param_len,uint8_t const *param)
{
	uint16_t min_temp = (uint16_t)(param[0]+(param[1]<<8));
	uint16_t max_temp = (uint16_t)(param[0]+(param[1]<<8)); // wenxue used for contec
	
	if(param_len == 0)// wenxue contec param_len == 0 ---- 查询
	{
	  //respone adv Interval to exturn mcu.
	  uint8_t pdu[] = {PT_PDU_TYPE_EVT, PT_PDU_EVT_PER_UPDATE_ADV_PARAM, 0x02,
	 	(uint8_t)pt_env.user_param.adv_intv_max,(uint8_t)(pt_env.user_param.adv_intv_max>>8)};
	  pt_pdu_send(sizeof(pdu), pdu);	
	}
	
	else if((min_temp>=0x0006)&&(max_temp<=0x4040)&&(max_temp>=min_temp)) //  min:3.75ms max:10.28s
	{
	  if(NVDS_OK != nvds_put(NVDS_TAG_ADV_INTERVAL, param_len, (uint8_t*)param))
	  {
	     //respone device name to exturn mcu. wenxue used for contec
				uint8_t pdu[4] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_PER_UPDATE_ADV_PARAM, 0x01,0x01}; 
				QPRINTF("save adv interval failed \r\n"); // wenxue
				pt_pdu_send(0x04, pdu);

	  }
	  else
	  {
		pt_env.user_param.adv_intv_min = min_temp;
		pt_env.user_param.adv_intv_max = max_temp;
		QPRINTF("save adv interval succeed \r\n"); // wenxue
		if(pt_env.state == PT_ADV)
		{
			app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
							  app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
							  app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
							  pt_env.user_param.adv_intv_min, pt_env.user_param.adv_intv_max);
		}

     // wenxue used for contec	
	   uint8_t pdu[4] = {PT_PDU_TYPE_EVT, PT_PDU_EVT_PER_UPDATE_ADV_PARAM, 0x01,0x00};
	   pt_pdu_send(4, pdu);	
	  }
	}
}



/* 查询 设置连接间隔 */
void pt_pdu_cmd_per_update_conn_param(uint8_t param_len,uint8_t const *param)
{
	uint16_t min_temp = (uint16_t)(param[0]+(param[1]<<8));
	uint16_t max_temp = (uint16_t)(param[0]+(param[1]<<8)); // wenxue used for contec

	if(param_len == 0)// wenxue contec param_len == 0 ---- 查询
	{
	  //respone adv Interval to exturn mcu.
	  uint8_t pdu[] = {PT_PDU_TYPE_EVT, PT_PDU_EVT_PER_UPDATE_CONN_PARAM, 0x02,
	 	(uint8_t)pt_env.user_param.conn_intv_max,(uint8_t)(pt_env.user_param.conn_intv_max>>8)};
	  pt_pdu_send(sizeof(pdu), pdu);	
	}

	
	else if((min_temp>=0x0006)&&(max_temp<=0x0c80)&&(max_temp>=min_temp))
	{
   if(NVDS_OK != nvds_put(NVDS_TAG_CONN_INTERVAL, param_len, (uint8_t*)param))
	 {
    //respone connect interval to exturn mcu. wenxue used for contec
		uint8_t pdu[4] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_PER_UPDATE_CONN_PARAM, 0x01,0x01}; 
		QPRINTF("save connect interval failed \r\n"); // wenxue
		pt_pdu_send(0x04, pdu);
	  }
	 else //save sucessful
	 {
		QPRINTF("save connect interval sucesful \r\n"); // wenxue
		//updata the new interval value 
		pt_env.user_param.conn_intv_min = min_temp;
		pt_env.user_param.conn_intv_max = max_temp;
	 //	pt_env.user_param.latency = latency_temp;
		 
		// if module's state is connect,immediately updated. Otherwise, updata after next connect.
		if((pt_env.state == PT_CONN_EMPTY) ||(pt_env.state == PT_CONN_FULL))
		{
			struct gap_conn_param_update conn_par;
			conn_par.intv_min = pt_env.user_param.conn_intv_min;
			conn_par.intv_max = pt_env.user_param.conn_intv_max;
			conn_par.latency = 0x0000;
			conn_par.time_out = GAP_PPCP_STO_MULT;
			app_gap_param_update_req(app_env.dev_rec[0].conhdl, &conn_par);
		} 
	
	 // wenxue used for contec	
    uint8_t pdu[4] = {PT_PDU_TYPE_EVT, PT_PDU_EVT_PER_UPDATE_CONN_PARAM, 0x01,0x00};
    pt_pdu_send(4, pdu); 
        
	}			
 }
}

/* 查询 BLE MAC 地址 */
void pt_pdu_cmd_rd_add_hdl(uint8_t param_len, uint8_t const *param)
{
	uint8_t pdu[] = {PT_PDU_TYPE_EVT, PT_PDU_EVT_ADD, 6,0x01,0x00,0x00,0xbe,0x7c,0x08};
	nvds_tag_len_t len_temp = 0x06;
	if (NVDS_OK != nvds_get(NVDS_TAG_BD_ADDRESS, &len_temp, &pdu[3]))
    {
		pt_pdu_send(sizeof(pdu), pdu);
    }
    else
    {
		pt_pdu_send(sizeof(pdu), pdu);
    }
}

/* 查询 设置发射功率 */
void pt_pdu_cmd_set_tx_power(uint8_t param_len,uint8_t const *param)
{
	if(param_len == 0)// wenxue contec param_len == 0 ---- 查询
	{
	  //respone adv Interval to exturn mcu.
	  uint8_t pdu[4] = {PT_PDU_TYPE_EVT, PT_PDU_EVT_TX_POWER, 0x01};
		pdu[3]=rf_tx_power_level_get(); ;
	  pt_pdu_send(sizeof(pdu), pdu);	
	}
	
	else if(param[0] < TX_GAIN_LEVEL12)
	{
      if(NVDS_OK != nvds_put(NVDS_TAG_TX_POWER, param_len, (uint8_t*)param))
	    {
        //respone tx power to exturn mcu. wenxue used for contec
				uint8_t pdu[4] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_TX_POWER, 0x01,0x01}; 
				QPRINTF("tx power save failed \r\n"); // wenxue
				pt_pdu_send(0x04, pdu);
	     }
	    else //save sucessful
	    {
				QPRINTF("save device name sucesful \r\n"); // wenxue
				rf_tx_power_level_set((enum TX_POWER)param[0]);

				//response current power level 
	      uint8_t pdu[4] = {PT_PDU_TYPE_EVT, PT_PDU_EVT_TX_POWER, 0x01,0x00};
	      pt_pdu_send(4, pdu);	
	    } 
	}
	else if(param[0] == TX_GAIN_LEVEL12)
	{
		//Chip need calibration after entering or exiting this level
	}
	else
	{
		;
	}

}


/* 查询 设置自定义广播数据 */
void pt_pdu_cmd_set_adv_user_data(uint8_t param_len, uint8_t const *param)
{
  if(param_len == 0)  // wenxue 查询
	{
		  uint8_t pdu[ADV_USER_DATA_LEN_MAX+3] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_ADV_USER_DATA, pt_env.user_param.adv_user_data_len};
	    memcpy(&pdu[3], pt_env.user_param.adv_user_data, pt_env.user_param.adv_user_data_len);
	    pt_pdu_send(pt_env.user_param.adv_user_data_len+3, pdu);
	}

	else if((param_len >=1)&&(param_len<=23)) // wenxue used for contec
	{
		pt_env.user_param.adv_user_data_len = param_len ;
		memcpy(pt_env.user_param.adv_user_data, param, pt_env.user_param.adv_user_data_len);

		// wenxue used for contec	
	   uint8_t pdu[4] = {PT_PDU_TYPE_EVT, PT_PDU_EVT_ADV_USER_DATA, 0x01,0x00};
	   pt_pdu_send(4, pdu);	
	}	
	
	else
	{
		 // wenxue used for contec	
	   uint8_t pdu[4] = {PT_PDU_TYPE_EVT, PT_PDU_EVT_ADV_USER_DATA, 0x01,0x01};
	   pt_pdu_send(4, pdu);	
	}
	

	
	if(pt_env.state == PT_ADV)
	{
		app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
						  app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
						  app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
						  pt_env.user_param.adv_intv_min, pt_env.user_param.adv_intv_max);
	}
	
}


/* 查询 设置串口延时*/
void pt_pdu_cmd_set_wakeup_exmcu_timer(uint8_t param_len, uint8_t const *param)
{
	if(param_len == 1)
	{
		pt_env.user_param.wakeup_exmcu_timer = param[0];
	}
	uint8_t pdu[4] = {PT_PDU_TYPE_EVT, PT_PDU_EVT_WAKEUP_EXMCU_TIMER, 0x01,0x00};
	pt_pdu_send(4, pdu);
	
}


/* 恢复出厂设置 */
// wenxue used for contec 将NVDS中的数据恢复为默认值
void pt_pdu_cmd_set_default_paramter(uint8_t param_len,uint8_t const *param)
{
  	cmd_set_default_parameter_flag=true;
	 // 1. restore the default uart baud
	  nvds_tag_len_t uart_baud_len= 1;
	  uint8_t uart_baud = 0x06;
	 if(NVDS_OK != nvds_put(NVDS_TAG_UART_BAUD,uart_baud_len,(uint8_t *)&uart_baud))
	 {
			  //respone information to exturn mcu. 
				uint8_t pdu[4] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_SET_DEFAULT_PARAMETER, 0x01,0x01}; 
				QPRINTF("restore default settings failed \r\n");
				pt_pdu_send(0x04, pdu);
				return ;
	 }
	 else
	 {	
		pt_env.user_param.baudrate = UART_115200;
		pt_env.user_param.raw_frame_timer = 5;	
	 }
		

  //2. restore the default device name 
	//save failed
	if(NVDS_OK != nvds_put(NVDS_TAG_DEVICE_NAME, strlen(QN_LOCAL_NAME)+1, (uint8_t *)QN_LOCAL_NAME))
	{
			  //respone information to exturn mcu. 
				uint8_t pdu[4] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_SET_DEFAULT_PARAMETER, 0x01,0x01}; 
				QPRINTF("restore default settings failed \r\n");
				pt_pdu_send(0x04, pdu);
				return ;    
	}
	else
	{   
		   pt_env.user_param.ble_name_len = strlen(QN_LOCAL_NAME);	
		   memcpy(pt_env.user_param.ble_name, QN_LOCAL_NAME,pt_env.user_param.ble_name_len);
		   //set the device name to gap.
		   app_gap_set_devname_req(pt_env.user_param.ble_name, pt_env.user_param.ble_name_len);
	}
	
	
	//3. restore the default adv Interval 100ms
	nvds_tag_len_t adv_interval_len= 2;
	uint8_t adv_interval[2] = {0x00,0xA0};
	if(NVDS_OK != nvds_put(NVDS_TAG_ADV_INTERVAL,adv_interval_len,(uint8_t *)&adv_interval))
	{
		  //respone information to exturn mcu. 
				uint8_t pdu[4] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_SET_DEFAULT_PARAMETER, 0x01,0x01}; 
				QPRINTF("restore default settings failed \r\n");
				pt_pdu_send(0x04, pdu);
				return ;
		 
	}
	else
	{
    	pt_env.user_param.adv_intv_min = GAP_ADV_100MS_INTV;
	    pt_env.user_param.adv_intv_max = GAP_ADV_100MS_INTV;
		
		if(pt_env.state == PT_ADV)
		{
			app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
							  app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
							  app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
							  pt_env.user_param.adv_intv_min, pt_env.user_param.adv_intv_max);
		}
	}
	

	//4. set the default connect Interval 
		nvds_tag_len_t connect_interval_len= 2;
	  uint8_t connect_interval[2] = {0x00,0x50};
	  if(NVDS_OK != nvds_put(NVDS_TAG_CONN_INTERVAL,connect_interval_len,(uint8_t *)&connect_interval[0]))
	  {
		    //respone information to exturn mcu. 
				uint8_t pdu[4] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_SET_DEFAULT_PARAMETER, 0x01,0x01}; 
				QPRINTF("restore default settings failed \r\n");
				pt_pdu_send(0x04, pdu);
				return ;
	  }
	  else
		{	
      	pt_env.user_param.conn_intv_min = GAP_CONN_100MS_INTV;
      	pt_env.user_param.conn_intv_max = GAP_CONN_100MS_INTV;			
			// if module's state is connect,immediately updated. Otherwise, updata after next connect.
			if((pt_env.state == PT_CONN_EMPTY) ||(pt_env.state == PT_CONN_FULL))
			{
				struct gap_conn_param_update conn_par;
				conn_par.intv_min = pt_env.user_param.conn_intv_min;
				conn_par.intv_max = pt_env.user_param.conn_intv_max;
				conn_par.latency = 0x0000;
				conn_par.time_out = GAP_PPCP_STO_MULT;
				app_gap_param_update_req(app_env.dev_rec[0].conhdl, &conn_par);
			} 
		}
	

	  // 6. restore default tx power
		nvds_tag_len_t tx_power_len= 1;
	  uint8_t tx_power[1] = {0x0A};
	  if(NVDS_OK != nvds_put(NVDS_TAG_TX_POWER,tx_power_len,(uint8_t *)&tx_power[0]))
	  {
			 //respone information to exturn mcu. 
				uint8_t pdu[4] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_SET_DEFAULT_PARAMETER, 0x01,0x01}; 
				QPRINTF("restore default settings failed \r\n");
				pt_pdu_send(0x04, pdu);
				return ;  
	  }
	  else
		{	
			rf_tx_power_level_set((enum TX_POWER)TX_GAIN_LEVEL10);// default Tx Power 0dbm				
		}
		
		
		// 7:adv user data
		pt_env.user_param.adv_user_data_len=0; 
		memset(pt_env.user_param.adv_user_data,0,sizeof(pt_env.user_param.adv_user_data));
		
		
    //8:wakeup exmcu timer
	  pt_env.user_param.wakeup_exmcu_timer = 0;
		
		//respone information to exturn mcu. 
		uint8_t pdu[4] = {PT_PDU_TYPE_EVT,PT_PDU_EVT_SET_DEFAULT_PARAMETER, 0x01,0x00}; 
		QPRINTF("restore default settings succeed \r\n");
		pt_pdu_send(0x04, pdu);
		ke_timer_set(APP_SYS_UART_BAUD_TIMER, TASK_APP, 1);
	
}



int app_pt_uartbaud_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case APP_SYS_UART_BAUD_TIMER:
        {
			//app_pt_gpio_stchange_process();
		ke_timer_clear(APP_SYS_UART_BAUD_TIMER, TASK_APP);
		uart_init(PT_UART_PORT, USARTx_CLK(0), (enum UART_BAUDRATE)pt_env.user_param.baudrate); // wenxue test
	  uart_tx_enable(PT_UART_PORT, MASK_ENABLE);
	  uart_rx_enable(PT_UART_PORT, MASK_ENABLE);
	  pt_rx_pdu_start(); 
			
        }break;

        default:
		ASSERT_ERR(0);
		break;
    }

    return (KE_MSG_CONSUMED);
}






