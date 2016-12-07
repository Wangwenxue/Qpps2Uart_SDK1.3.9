#include "gpio_uart_driver.h"


//Can be porting if you need
//Porting end


// 配置参数
//gpio_uart_config_t g_gpio_uart_config;


//接收相关变量
static int8_t  recv_state= UART_IDLE;   // rx state
static uint8_t bit_recv = 0;            // rx bit value
static uint8_t bit_recv_index = 0;      // rx bit index
static uint8_t data_recv = 0;           // rx revived data
static int8_t  parity_r = 0;            // rx parity


//发送相关变量
static int8_t   send_state = UART_IDLE; // tx state
static uint8_t  bit_send = 0;           // tx bit value
static uint8_t  bit_send_index = 0;     // tx bit index
static int8_t   parity_t = 0;           // tx parity

volatile bool g_parity_error ; // wenxue

static gpio_uart_config_t gpio_uart_demo; // wenxue


///UART0 environment variable
static struct gpio_uart_env_tag uart_env;

/**
 ****************************************************************************************
 * @brief Initialize UART Rx related variables               
 * @description
 *  This function is used to initialize Rx related variables
 *****************************************************************************************
 */
static void gpio_uart_rx_clear()
{
  recv_state= UART_IDLE; //  recv state
  bit_recv = 0;          //  bit value
  bit_recv_index = 0;    // bit index
  data_recv = 0;         // data value
  parity_r = 0;         // parity parameter
}


/**
 ****************************************************************************************
 * @brief Initialize UART Tx related variables               
 * @description
 *  This function is used to initialize Tx related variables
 *****************************************************************************************
 */
static void gpio_uart_tx_clear()
{
  send_state = UART_IDLE;
  bit_send = 0;
  bit_send_index = 0;
  parity_t = 0;
}




/**
 **********************************************************************************************
 * @brief gpio_uart initialization function           
 * @description
 *  This function is used to initialize structure gpio_uart_demo and call funciton gpio_uart_init
 ***********************************************************************************************
 */
void gpio_uart_demo_init()
{
    gpio_uart_rx_clear();
    gpio_uart_tx_clear();
    
    gpio_uart_demo.baudrate = 9600;
    gpio_uart_demo.databit = 8;
    gpio_uart_demo.parity = GPIO_UART_PARITY_MODE_NONE;
    gpio_uart_demo.stop = 1;
	 
    gpio_uart_demo.tx_timer = QN_TIMER0;
    gpio_uart_demo.rx_timer = QN_TIMER1;
     
	
    gpio_uart_demo.tx_pin_num = GPIO_P03;
    gpio_uart_demo.rx_pin_num = GPIO_P31;

    
    gpio_uart_init(&gpio_uart_demo);
		

}


/**
 ****************************************************************************************
 * @brief Initialize the UART to default values.
 * @param[in]       config          
 * @description
 *  This function is used to initialize UART, it consists of baud-rate, GPIO assignment, timer config
 *  enable NVIC IRQ.
 *****************************************************************************************
 */
void gpio_uart_init(const gpio_uart_config_t *config)
{
		
	/* Initialize GPIO */
 //  gpio_init(BOARD_GPIO_UART_RXPIN_HANDLER);
	
	gpio_set_direction(config->tx_pin_num, GPIO_OUTPUT);
	
	gpio_pull_set(config->rx_pin_num, GPIO_PULL_UP);
	gpio_set_direction(config->rx_pin_num, GPIO_INPUT);
	gpio_set_interrupt(config->rx_pin_num, GPIO_INT_FALLING_EDGE);
	gpio_enable_interrupt(config->rx_pin_num);
	
	/* Initialize Timer */
	/*
  * Do not start PIT timer here, it start when start bit arrive
  */
	timer_init(config->rx_timer, BOARD_GPIO_UART_TIMER_RX_HANDLER);
	timer_config(config->rx_timer, TIMER_PSCAL_DIV, TIMER_COUNT_US((1000000U/config->baudrate)/2, TIMER_PSCAL_DIV));

	timer_init(config->tx_timer, BOARD_GPIO_UART_TIMER_TX_HANDLER);
	timer_config(config->tx_timer, TIMER_PSCAL_DIV, TIMER_COUNT_US(1000000U/config->baudrate, TIMER_PSCAL_DIV));
	


}


/**
 ****************************************************************************************
 * @brief Start a data transmission.
 * @param[in]  bufptr          Pointer to the TX data buffer
 * @param[in]  size            Size of the transmission
 * @param[in]  tx_callback     Callback for end of transmission
 * @description
 * This function is used to write data into TX buffer to transmit data.
 *
 *****************************************************************************************
 */
void gpio_uart_write(uint8_t *databuf, uint32_t num,void (*tx_callback)(void))
{
	
  gpio_uart_tx_clear();
  
  //Store environment parameters
  uart_env.tx.size = num;
  uart_env.tx.bufptr = databuf;
  #if UART_CALLBACK_EN==TRUE
  uart_env.tx.callback = tx_callback;
  #endif
	
	
  //start timer to send
	timer_enable(gpio_uart_demo.tx_timer, MASK_ENABLE);
	
}



/**
 ****************************************************************************************
 * @brief Start a data reception.
 * @param[in,out]  bufptr         Pointer to the RX buffer
 * @param[in]      size           Size of the expected reception
 * @param[in]      rx_callback    Callback for end of reception
 * @description
 * This function is used to read Rx data from RX FIFO and the data will be stored in bufptr.
 * As soon as the end of the data transfer is detected, the callback function is executed.
 *
 *****************************************************************************************
 */
void gpio_uart_read(uint8_t *bufptr, uint32_t size, void (*rx_callback)(void))
{   
 
   //Store environment parameters
    uart_env.rx.size = size;
    uart_env.rx.bufptr = bufptr;
    #if UART_CALLBACK_EN==TRUE
    uart_env.rx.callback = rx_callback;
    #endif
	
	
    // Start data transmission   
	  gpio_set_interrupt(gpio_uart_demo.rx_pin_num, GPIO_INT_FALLING_EDGE);//下降沿做起始位判断
  	gpio_enable_interrupt(gpio_uart_demo.rx_pin_num);// enable gpio interrupt
	  NVIC_EnableIRQ(GPIO_IRQn);// 打开GPIO中断，判断是否有起始位开始接收数据
}

/**
 ****************************************************************************************
 * @brief UART RX start bit falling ege interrupt handler.
 * @description
 * Disble GPIO interrupt and start RX timer
 ****************************************************************************************
 */
void BOARD_GPIO_UART_RXPIN_HANDLER(enum gpio_pin pin)
{
	//clear int
	gpio_gpio_IntClear(QN_GPIO, pin);
	//disable int
  gpio_disable_interrupt(gpio_uart_demo.rx_pin_num);
	
	timer_init(gpio_uart_demo.rx_timer, BOARD_GPIO_UART_TIMER_RX_HANDLER);
	timer_config(gpio_uart_demo.rx_timer, TIMER_PSCAL_DIV, TIMER_COUNT_US((1000000U/gpio_uart_demo.baudrate)/2, TIMER_PSCAL_DIV));
	timer_enable(gpio_uart_demo.rx_timer, MASK_ENABLE);
	
}

/**
 ****************************************************************************************
 * @brief UART RX interrupt handler.
 * @description
 *  Receive data from Rx pin until expected receiving data size is decreased to zero.
 ****************************************************************************************
 */
void BOARD_GPIO_UART_TIMER_RX_HANDLER(void) 
{
  {
    //Recv state machine
    switch(recv_state)
    {
    case UART_TRANSFERRING:    
      //Get one bit by checking the RX GPIO input level  
		//bit_recv = gpio_read_pin(gpio_uart_demo.rx_pin_num);   
      if(gpio_read_pin(gpio_uart_demo.rx_pin_num)== GPIO_LOW) 
				 bit_recv = 0x00;   
			else
				 bit_recv = 0x01;   
		
      data_recv |= (bit_recv << bit_recv_index);
      parity_r ^= bit_recv;
      //yangliang change
      //parity_r = (bit_recv + parity_r)&0x01;
      bit_recv_index ++;
      
      if(bit_recv_index == 8)
      {
        if(gpio_uart_demo.parity == GPIO_UART_PARITY_MODE_NONE)
        {
            recv_state = UART_STOP;
        }
        else
        {
            recv_state = UART_PARITY;
        }

      }
      break;
      
    case UART_PARITY:
		 if(gpio_read_pin(gpio_uart_demo.rx_pin_num)== GPIO_LOW) 
				 bit_recv = 0x00;   
			else
				 bit_recv = 0x01;   
		
		
      parity_r  ^= bit_recv;
      //yangliang change
      //parity_r = (bit_recv + parity_r)&0x01;

      recv_state = UART_STOP;
      break;
      
    case UART_STOP:
         
      // discard the character if parity check fail.
      if (gpio_uart_demo.parity == GPIO_UART_PARITY_MODE_ODD)
      {
       // if(parity_r == 1)
        if(parity_r == 0) // wenxue
        {
            data_recv = 0;
            g_parity_error = true;
            
        }
      }
      else if(gpio_uart_demo.parity == GPIO_UART_PARITY_MODE_EVEN)
      {
       // if(parity_r == 0)
          if(parity_r == 1) // wenxue
        {
            data_recv = 0;
            g_parity_error = true;
        }
      }
      
      
      
      if (uart_env.rx.size > 0) {
           *uart_env.rx.bufptr++ = data_recv;
           uart_env.rx.size--;
           
            if (uart_env.rx.size == 0) {
              
              // Disable UART all rx int,stop timer
              gpio_disable_interrupt(gpio_uart_demo.rx_pin_num);
						//	timer_enable(gpio_uart_demo.rx_timer, MASK_DISABLE);
							timer_reset(gpio_uart_demo.rx_timer);
				
				      gpio_uart_rx_clear();
							
              #if UART_CALLBACK_EN==TRUE
              // Call end of reception callback        
               uart_env.rx.callback();
              #endif
             
            
            }
            else
            {         
              gpio_uart_rx_clear();
              
              /* Stop Timer and Enable RXD gpio interrupt */
							//timer_enable(gpio_uart_demo.rx_timer, MASK_DISABLE);
							timer_reset(gpio_uart_demo.rx_timer);
						  gpio_gpio_IntClear(QN_GPIO, gpio_uart_demo.rx_pin_num);
              gpio_enable_interrupt(gpio_uart_demo.rx_pin_num);
             }
        }
      
      break;
      
    case UART_IDLE:
      /* Reset recieved data */
      data_recv      = 0;
      bit_recv_index = 0;
      parity_r       = 0;
	    recv_state = UART_TRANSFERRING;
		  timer_timer_SetTOPR(gpio_uart_demo.rx_timer, (1000000U/gpio_uart_demo.baudrate));
//      if( gpio_read_pin(gpio_uart_demo.rx_pin_num)==GPIO_LOW) // start bit
//      {
//        recv_state = UART_TRANSFERRING;
//        // modify timer period to normal, not the half frequency
//		//		timer_reset(gpio_uart_demo.rx_timer);
//			//	timer_init(gpio_uart_demo.rx_timer, BOARD_GPIO_UART_TIMER_RX_HANDLER);
//     //   timer_config(gpio_uart_demo.rx_timer, TIMER_PSCAL_DIV, TIMER_COUNT_US((1000000U/gpio_uart_demo.baudrate), TIMER_PSCAL_DIV));
//     //   timer_enable(gpio_uart_demo.rx_timer, MASK_ENABLE);
//				timer_timer_SetTOPR(gpio_uart_demo.rx_timer, (1000000U/gpio_uart_demo.baudrate));
//			}
//      else  // not start bit
//      {
//        gpio_uart_rx_clear();
//        
//        timer_enable(gpio_uart_demo.rx_timer, MASK_DISABLE);
//				/* Enable RXD gpio interrupt */		
//        gpio_gpio_IntClear(QN_GPIO, gpio_uart_demo.rx_pin_num);
//        gpio_enable_interrupt(gpio_uart_demo.rx_pin_num);
//      
//      }
      break;
      
    default:
      break;
    }        
  }	
}


/**
 ****************************************************************************************
 * @brief UART TX interrupt handler.
 * @description
 *  Transmit data to Tx pin until expected tramsmitting data size is decreased to zero.
 ****************************************************************************************
 */
void BOARD_GPIO_UART_TIMER_TX_HANDLER(void)
{
	
  //发送
  {
    switch(send_state)
    {
    case UART_START:
      //Put GPIO to logic 0 to indicate the start
	  	gpio_write_pin(gpio_uart_demo.tx_pin_num, GPIO_LOW);
      bit_send_index = 0;
      parity_t   = 0;
      send_state = UART_TRANSFERRING;
      break;
      
    case UART_TRANSFERRING:
      bit_send = (*uart_env.tx.bufptr >> bit_send_index) & 0x1;
      //Put GPIO to the logic level according to the bit value. 0 for low, and 1 for high;
      if(bit_send)
				gpio_write_pin(gpio_uart_demo.tx_pin_num, GPIO_HIGH);       
      else
				gpio_write_pin(gpio_uart_demo.tx_pin_num, GPIO_LOW);
       
      parity_t ^= bit_send;
      bit_send_index++;
      if( bit_send_index == 8)
      {
        if(gpio_uart_demo.parity == GPIO_UART_PARITY_MODE_NONE)
        {
          send_state = UART_STOP;
        }
        else
        {
          send_state = UART_PARITY;
        }
      }
      break;
      
    case UART_PARITY:
      if(gpio_uart_demo.parity == GPIO_UART_PARITY_MODE_ODD)
        bit_send = (~parity_t)&1; 
      else
        bit_send = parity_t; // even
      
      if(bit_send)
				gpio_write_pin(gpio_uart_demo.tx_pin_num, GPIO_HIGH);
      else
        gpio_write_pin(gpio_uart_demo.tx_pin_num, GPIO_LOW);
      
      send_state = UART_STOP;
      break;
      
    case UART_STOP:
      //Put GPIO to logic 1 to stop the transferring
      gpio_write_pin(gpio_uart_demo.tx_pin_num, GPIO_HIGH);
      uart_env.tx.size--;
      
      if(uart_env.tx.size)
      {    /* Continue to send the next data */
        send_state = UART_START ;
        uart_env.tx.bufptr++;
      }
      else
      {   /* Finish data sending */
        send_state = UART_IDLE;
				timer_enable(gpio_uart_demo.tx_timer, MASK_DISABLE);
        
        // Tx callback
        #if UART_CALLBACK_EN==TRUE
        // Call end of transmission callback
        if(uart_env.tx.callback != NULL)
        {
            uart_env.tx.callback();
        }
        #endif
        
      }
      break;
      
    case UART_IDLE:
      send_state = UART_START;
      break;
      
    default:
      break;
    }  
    
  }
	
}


