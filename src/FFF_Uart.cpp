#include <Arduino.h>
#include <FFF_Settings.h>
#include <FFF_Rtos.h>
#include <FFF_DiaAnalyzer.h>
#include <driver/uart.h>

/**********/
/* UART 2 */
/**********/
static void IRAM_ATTR uart2_isr_handler(void *arg);

static intr_handle_t handle_uart2_isr;
static QueueHandle_t uart2_queue;

volatile bool uart2_rx_complete;
uint16_t u2rxLen = 0;

TaskHandle_t UartFpgaTaskHandle;

uint8_t uart2Buf0[MEASUREMENT_LENGTH];
uint8_t uart2Buf1[MEASUREMENT_LENGTH];

FFF_Buffer FFF_Buf_uart2_data0 = 
{
  .len = 0,
  .dataPtr = uart2Buf0
};


FFF_Buffer FFF_Buf_uart2_data1 = 
{
  .len = 0,
  .dataPtr = uart2Buf1
};


FFF_Buffer* activeBufPtr = &FFF_Buf_uart2_data0;


bool sByte = false;
uint16_t sInd = 0;
bool yByte = false;
uint16_t yInd = 0;
bool nByte = false;
uint16_t nInd = 0;
bool cByte = false;
uint16_t cInd = 0;
bool syncDetected = false;



void FFF_Uart_init()
{
  // Serial 0 configuration with Arduino
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

  // release the pre registered UART handler/subroutine
  ESP_ERROR_CHECK(uart_isr_free(UART_NUM_2));       

  // register new UART ISR subroutine
  ESP_ERROR_CHECK(uart_isr_register(UART_NUM_2, uart2_isr_handler, NULL, ESP_INTR_FLAG_IRAM, &handle_uart2_isr));

  uart_driver_install(UART_NUM_2, MEASUREMENT_LENGTH, 0, 8, &uart2_queue, 0);
  uart_param_config(UART_NUM_2, &uart2_config);  
  uart_set_pin(UART_NUM_2, TX_TO_FPGA_PIN, RX_TO_FPGA_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_enable_rx_intr(UART_NUM_2);
}


static void IRAM_ATTR uart2_isr_handler(void *arg)
{
  uint16_t rx_fifo_len, status;
  uint16_t i = 0;
  uint16_t cnt = 0;

  bool invalidSync = false;

  status = UART2.int_st.val;
  rx_fifo_len = UART2.status.rxfifo_cnt;

  while(rx_fifo_len)
  {
    uint8_t tempByte = UART2.fifo.rw_byte;

    if (tempByte == 'S')
    {
      sByte = true;
      sInd = cnt;
    }
    else if (tempByte == 'Y')
    {
      yByte = true;
      yInd = cnt;
      
      if (yInd - sInd != 1)
      {
        invalidSync = true;
      }
    } 
    else if (tempByte == 'N')
    {
      nByte = true;
      nInd = cnt;
      
      if (nInd - yInd != 1)
      {
        invalidSync = true;
      }
    } 
    else if (tempByte == 'C')
    {
      cByte = true;
      cInd = cnt;
      
      if (cInd - nInd != 1)
      {
        invalidSync = true;
      }
    }


    if (invalidSync)
    {
      sByte = false;
      yByte = false;
      nByte = false;
      cByte = false;
    }


    rx_fifo_len--;
    cnt++;
  }

  if (sByte && yByte && nByte && cByte)
  {
    syncDetected = true;
    sByte = false;
    yByte = false;
    nByte = false;
    cByte = false;

    BaseType_t xYieldRequired;

    // Resume the suspended task.
    xYieldRequired = xTaskResumeFromISR(FFF_DiaAn_getTaskHandle());
  }
  // after reading bytes from buffer clear UART interrupt status
  uart_clear_intr_status(UART_NUM_2, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);
}


TaskHandle_t* FFF_Uart_getTaskHandleUart2()
{
  return &UartFpgaTaskHandle;
}



QueueHandle_t* FFF_Uart_getQueueHandleUart2()
{
  return &uart2_queue;
}


FFF_Buffer* FFF_Uart_getCurrentBufferUart2()
{
  return activeBufPtr;
}