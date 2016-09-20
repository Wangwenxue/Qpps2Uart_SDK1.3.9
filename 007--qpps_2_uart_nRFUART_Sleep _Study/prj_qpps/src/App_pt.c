
#include "app_pt.h"
#include "uart.h"
#include "timer.h"
#include "lib.h"


struct pt_env_tag  pt_env;
struct PT_FIFO pt_data_fifo;	//data fifo

void pt_rx_raw_done(void);
uint8_t pt_fifo_get(uint8_t *pBuf);
uint8_t pt_fifo_put(uint8_t *pPutDat, uint8_t PutLen);
void pt_fifo_init(void);
void ble_data_tx(void);

void pt_init(void);
void pt_rx_raw_frame_hdl(void);
void pt_enable(void);
void pt_disable(void);

void pt_init()
{
	if(KE_EVENT_OK != ke_evt_callback_set(PT_RX_RAW_FRAME_EVENT_ID, 
										  pt_rx_raw_frame_hdl))
	ASSERT_ERR(0);
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
	pt_fifo_init();
  pt_env.ble_tx_state = BLE_TX_STOP;
	pt_env.uart_rx_state = UART_RX_GOING;
	pt_env.state = PT_CONN_EMPTY; // wenxue ??? 除此之外没有再设置了？

	uart_uart_GetRXD(QN_UART0);//before read start,clear fifo data.
	uart_uart_ClrIntFlag(QN_UART0, 0x0000003d);//before read start,clear all the interrupt flag
	
  uart_read(PT_UART_PORT, pt_data_fifo.elemt[0].buf, 1, pt_rx_raw_done);
	uart_rx_int_enable(PT_UART_PORT, MASK_ENABLE);
	
	// Rx timer config
	timer_config(QN_TIMER3, TIMER_PSCAL_DIV, TIMER_COUNT_MS(/*pt_env.user_param.raw_frame_timer*/5 , TIMER_PSCAL_DIV));
	timer_init(QN_TIMER3, timer3_callback);
}

/*
* note: when UART baud rade is 115200, UART speed is higher than ble.
*          baud rade is 2400, 9600, 38400, BLE speed is higher than UART. 
* data fifo should deal with it
*/
void pt_rx_raw_done(void)	
{
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
	}

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
			//pt_rx_raw_start();
			pt_enable(); // 重新初始化 重头开始
		}
	}
		
}
