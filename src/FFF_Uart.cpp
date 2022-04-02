#include <Arduino.h>
#include <FFF_Settings.h>
#include <driver/uart.h>

/**********/
/* UART 2 */
/**********/
static QueueHandle_t uart2_queue;
volatile bool uart2_rx_complete;

static void IRAM_ATTR uart2_isr_handler(void *arg)
{
  uint16_t rx_fifo_len, status;
  uint16_t index = 0;

  status = UART2.int_st.val;
  rx_fifo_len = UART2.status.rxfifo_cnt;
  uart2_rx_complete = false;

  while(rx_fifo_len)
  {
    
  }
}


void FFF_Uart_init()
{
  Serial.begin(SERIAL0_BAUDRATE);

  // Serial 2 configuration 
  uart_config_t uart2_config = 
  {
      .baud_rate = 500000,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  }; 
  
  uart_param_config(UART_NUM_2, &uart2_config);  
  uart_set_pin(UART_NUM_2, TX_TO_FPGA_PIN, RX_TO_FPGA_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_2, MEASUREMENT_LENGTH, 0, 8, &uart2_queue, 0);
    
}