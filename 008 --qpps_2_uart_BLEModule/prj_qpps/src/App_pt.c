
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
	
	if(pt_data_fifo.elemt[pt_data_fifo.in].len) // �ж��Ƿ�Ϊ0�������Ƿ������ݣ��п�����20���ֽں���������Ǿ�Ϊ0
	{
		pt_data_fifo.in++;	//in this place should pt_data_fifo.in >= pt_data_fifo.out  ��1��ԭ���� ��һ�λ��С��20�ֽڵ����ݶ����ߣ���һ�εô�ŵ���һ��������
		if(pt_data_fifo.in >= PT_FIFO_X_SIZE)
		{
			pt_data_fifo.in = 0;
		}
		if(pt_data_fifo.in == pt_data_fifo.out) // ����һȦ�ֻ�����
		{
			pt_data_fifo.full_ct++;
			pt_data_fifo.state = PT_FIFO_FULL;
		}
		else
		{
			pt_data_fifo.state = PT_FIFO_NOFU;
		}
		pt_data_fifo.elemt[pt_data_fifo.in].len = 0; // wenxue len=0 ��һ���ṹ�����ʼλ��
		if((pt_env.ble_tx_state != BLE_TX_GOING)&&(pt_data_fifo.state != PT_FIFO_EMPT))
		{
			// wenxue ��������
			ke_evt_set(1UL << PT_RX_RAW_FRAME_EVENT_ID);	//don't send ble data in uart interrupt, set an env to kernel
			pt_env.ble_tx_state = BLE_TX_GOING;
		}

	}
	
	if(pt_env.uart_rx_state != UART_RX_STOP)	// ����UART_RX_STOP ���ٽ�������
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
	pt_env.state = PT_CONN_EMPTY; // wenxue ??? ����֮��û���������ˣ�

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
	pt_data_fifo.elemt[pt_data_fifo.in].len++; // ���ȼ� 1 �������ݰ��ṹ���еĳ���
	if(pt_data_fifo.elemt[pt_data_fifo.in].len >= PT_FIFO_Y_SIZE) // wenxue �����ȴﵽ20�ֽ�ʱ
	{
		pt_data_fifo.in++;	      //in this place should pt_data_fifo.in >= pt_data_fifo.out  // wenxue 20�ֽ����ݰ� ������1
		if(pt_data_fifo.in >= PT_FIFO_X_SIZE) // ���ȴ��ڶ������󳤶�200��ʱ,���������
		{
			pt_data_fifo.in = 0;
		}
		if(pt_data_fifo.in == pt_data_fifo.out) // ʲôʱ�����ȣ����� ��������һȦ��׷�Ϸ�����
		{
			pt_data_fifo.full_ct++;
			pt_data_fifo.state = PT_FIFO_FULL;
			pt_env.uart_rx_state = UART_RX_STOP;
		}
		else
		{
			pt_data_fifo.state = PT_FIFO_NOFU;
		}
		pt_data_fifo.elemt[pt_data_fifo.in].len = 0;// len ��ͷ��ʼ����
		
		// wenxue ֻ����pt_env.ble_tx_state != BLE_TX_GOING ����event,���������ݡ� ����ֻ��Ҫ�����ݼ��ɣ���ΪBLE��ʱ�Ѿ��ڷ���������
		if((pt_env.ble_tx_state != BLE_TX_GOING)&&(pt_data_fifo.state != PT_FIFO_EMPT))//wenxue pt_data_fifo.state != PT_FIFO_EMPT ����һ����
		{
			// wenxue ��ʼ��������
			ke_evt_set(1UL << PT_RX_RAW_FRAME_EVENT_ID);	//don't send ble data in uart interrupt, set an env to kernel
			pt_env.ble_tx_state = BLE_TX_GOING; // ��ʾ���ڷ�������
		}
	}

	// ����������һ�ֽ�
	if(pt_env.uart_rx_state != UART_RX_STOP)	 // ��������UART_RX_STOP,���ھͲ��ٽ���������
	{
		//stop timer, prepare next byte wenxue һ�յ����ݾ͹رն�ʱ��
		do{timer_timer_SetCRWithMask(QN_TIMER3, TIMER_MASK_TEN, MASK_DISABLE);}
		while((UART_RX_TIMER->CR & 0x00000001) == 1);

		// ����������һ�ֽ�
	  uart_read(PT_UART_PORT, pt_data_fifo.elemt[pt_data_fifo.in].buf+pt_data_fifo.elemt[pt_data_fifo.in].len, 1,  pt_rx_raw_done);

	    //start timer // wenxue  ��revice next byte֮��ʹ򿪶�ʱ��,�����ʱʱ�䵽����ʾ���ʱ��û�н��յ����ݡ�
	    do{timer_enable(QN_TIMER3, MASK_ENABLE);}
	    while((UART_RX_TIMER->CR & 0x00000001) == 0);
	}
        
}


void pt_rx_raw_frame_hdl(void)
{
	ke_evt_clear(1UL << PT_RX_RAW_FRAME_EVENT_ID);
	ble_data_tx(); // wenxue ���÷��ͺ���
	
}
void ble_data_tx()
{
	app_qpps_env->tx_buffer_available--;
	// wenxue ! BLE ���͵�˳���Ǵ�ǰ�������η���
	app_qpps_data_send(app_qpps_env->conhdl, 0, pt_data_fifo.elemt[pt_data_fifo.out].len, pt_data_fifo.elemt[pt_data_fifo.out].buf);
	pt_data_fifo.out ++;
	if(pt_data_fifo.out == PT_FIFO_X_SIZE)
	{
		pt_data_fifo.out = 0;// �������ֵ��0,��ͷ��
	}
	if(pt_data_fifo.out == pt_data_fifo.in) // ���͵ĸ��Ͻ��յ���
	{
		pt_data_fifo.state = PT_FIFO_EMPT; // wenxue ! 
		pt_data_fifo.empty_ct++;
		
		// wenxue pt_env.uart_rx_state == UART_RX_STOP ��֮ǰ�յ�̫�죬���ٽ��գ���ʱ���ͷ����˽��յ�λ��
		if(pt_env.uart_rx_state == UART_RX_STOP)	
		{
			//pt_rx_raw_start();
			pt_enable(); // ���³�ʼ�� ��ͷ��ʼ
		}
	}
		
}
