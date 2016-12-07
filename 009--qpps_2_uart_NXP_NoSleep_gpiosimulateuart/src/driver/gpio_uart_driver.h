#ifndef _GPIO_UART_DRIVER_H_
#define _GPIO_UART_DRIVER_H_

//#include "fsl_common.h"
//#include "fsl_pit.h"
//#include "board.h"
//#include "fsl_gpio.h"
//#include "fsl_port.h"
#include "gpio.h"
#include "timer.h"

#define UART_CALLBACK_EN                 TRUE        /*!< Enable/Disable UART Driver Callback */

enum UART_STATE
{
   UART_IDLE = 0,
   UART_START,
   UART_TRANSFERRING,
   UART_PARITY,
   UART_STOP
};


/// parity mode
enum GPIO_UART_PARITY_MODE
{
    GPIO_UART_PARITY_MODE_NONE = 0,        
    GPIO_UART_PARITY_MODE_ODD,            
    GPIO_UART_PARITY_MODE_EVEN
};

///Instance structure for gpio_uart configuration
typedef struct 
{
//    GPIO_Type *tx_gpio_base;    //
//    GPIO_Type *rx_gpio_base;
//    PORT_Type *tx_port_base;
//    PORT_Type *rx_port_base;
//    uint32_t tx_pin_num; 
//    uint32_t rx_pin_num;
	
	  enum gpio_pin tx_pin_num;
	  enum gpio_pin rx_pin_num;
    
    uint8_t databit;   //数据位长度，通常是8
    uint8_t stop;      //停止位长度
    uint8_t parity;    //校验位，0表示不校验，1表示奇校验，2表示偶校验  
	  uint32_t baudrate;  // 波特率
	
	  QN_TIMER_TypeDef *tx_timer;
	  QN_TIMER_TypeDef *rx_timer;
	
  
 
} gpio_uart_config_t;




///UART TX/RX parameters
struct uart_txrxchannel
{
    uint8_t  *bufptr;
    int32_t  size;
    #if UART_CALLBACK_EN==TRUE
    void     (*callback)(void);
    #endif
};

///UART environment parameters
struct gpio_uart_env_tag
{
    struct uart_txrxchannel tx;
    struct uart_txrxchannel rx;
};


extern void gpio_uart_init(const gpio_uart_config_t *config);
extern void gpio_uart_demo_init();
extern void gpio_uart_write(uint8_t *databuf, uint32_t num,void (*tx_callback)(void));
extern void gpio_uart_read(uint8_t *bufptr, uint32_t size, void (*rx_callback)(void));
extern void BOARD_GPIO_UART_RXPIN_HANDLER(enum gpio_pin pin);
extern void BOARD_GPIO_UART_TIMER_RX_HANDLER(void); 
extern  void BOARD_GPIO_UART_TIMER_TX_HANDLER(void);


#endif