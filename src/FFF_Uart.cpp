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

uint8_t uart2Buf0[MEASUREMENT_LENGTH + SYNC_LENGTH];
uint8_t uart2Buf1[MEASUREMENT_LENGTH + SYNC_LENGTH];

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
bool bufferWriteActive;

uint8_t currByte = 0;
uint32_t cnt = 0;



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

  // // release the pre registered UART handler/subroutine
  // ESP_ERROR_CHECK(uart_isr_free(UART_NUM_2));       

  // register new UART ISR subroutine
  ESP_ERROR_CHECK(uart_isr_register(UART_NUM_2, uart2_isr_handler, NULL, ESP_INTR_FLAG_IRAM, &handle_uart2_isr));

  uart_driver_install(UART_NUM_2, MEASUREMENT_LENGTH, 0, 8, &uart2_queue, 0);
  uart_param_config(UART_NUM_2, &uart2_config);  
  uart_set_pin(UART_NUM_2, TX_TO_FPGA_PIN, RX_TO_FPGA_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}



void IRAM_ATTR discardSyncDetect()
{
  sByte = false;
  sInd = 0;
  yByte = false;
  yInd = 0;
  nByte = false;
  nInd = 0;
  cByte = false;
  cInd = 0;
}


bool IRAM_ATTR checkSync()
{
  if (currByte == 'S')
  {
    sInd = cnt;
    sByte = true;
  }
  else if (currByte == 'Y')
  {
    yInd = cnt;
    if (yInd - sInd == 1 && sByte)
    {
      yByte = true;
    } 
    else 
    {
      discardSyncDetect();
    }
  } 
  else if (currByte == 'N')
  {
    nInd = cnt;
    if (nInd - yInd == 1 && yByte)
    {
      nByte = true;
    }
    else 
    {
      discardSyncDetect();
    }
  } 
  else if (currByte == 'C')
  {
    cInd = cnt;
    if (cInd - nInd == 1 && cByte)
    {
      cByte = true;
    }
    else 
    {
      discardSyncDetect();
    }
  }

  
  if (sByte && yByte && nByte && cByte)
  {
    cnt = 0;
    return true;
  }

  return false;
}


void IRAM_ATTR switchActiveBuffer()
{
  if (activeBufPtr == &FFF_Buf_uart2_data0)
  {
    activeBufPtr = &FFF_Buf_uart2_data1;
  }
  else 
  {
    activeBufPtr = &FFF_Buf_uart2_data0;
  }
}

/*************************************************/
/* INTERRUPT HANDLER */
/*************************************************/
static void IRAM_ATTR uart2_isr_handler(void *arg)
{
  uint16_t rx_fifo_len, status;
  uint16_t i = 0;

  status = UART2.int_st.val;
  rx_fifo_len = UART2.status.rxfifo_cnt;

  while(rx_fifo_len)
  { // FIFO WHILE
    currByte = UART2.fifo.rw_byte;

    if (bufferWriteActive && !activeBufPtr->protect)
    { // WRITING TO BUFFER
      // Normal operation, we just write to the buffer
      activeBufPtr->dataPtr[activeBufPtr->len] = currByte;
      // Prevent out of bounds issues
      if (activeBufPtr->len < MEASUREMENT_LENGTH)
      {
        activeBufPtr->len++;
      }
      else 
      {
        // We would write out of bounds 
        // so reset the length and don't write to a buffer anymore
        bufferWriteActive = false;
        activeBufPtr->len = 0;
      }

      if (checkSync())
      {
        // Prevent out of bounds issues
        if (activeBufPtr->len > SYNC_LENGTH)
        {
          activeBufPtr->len -= SYNC_LENGTH;
          // Task will retrieve the filled buffer via a getter
          xTaskResumeFromISR(FFF_DiaAn_getTaskHandle());
          switchActiveBuffer();
        }
        else 
        {
          // Some error occured so go into listen-only mode again
          activeBufPtr->len = 0;
          bufferWriteActive = false;
        }
      }
    } 
    else 
    { // NOT WRITING TO BUFFER (LISTEN-ONLY MODE)
      // While we are not writing to a buffer we must detect sync
      if (checkSync())
      {
        bufferWriteActive = true;
      }
    }

    rx_fifo_len--;
    cnt++;    
  } // END FIFO WHILE

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


FFF_Buffer* FFF_Uart_getFilledBufferUart2()
{
  FFF_Buffer* retBufPtr;
  if (activeBufPtr == &FFF_Buf_uart2_data0)
  {
    retBufPtr = &FFF_Buf_uart2_data1;
  }

  retBufPtr = &FFF_Buf_uart2_data0;


  if (retBufPtr->protect)
  {
    return NULL;
  }
  else 
  {
    return retBufPtr;
  }
}


// Protects the buffer that is currently not active
void FFF_Uart_protectBufferUart2(FFF_Buffer* bufPtr)
{
  if (bufPtr)
  {
    bufPtr->protect = true;
  }
}

void FFF_Uart_unprotectBufferUart2(FFF_Buffer* bufPtr)
{
  if (bufPtr)
  {
    bufPtr->protect = false;
  }
}


void FFF_Uart_activateInterruptRX_Uart2()
{
  uart_enable_rx_intr(UART_NUM_2);
}


void FFF_Uart_deactivateInterruptRX_Uart2()
{
  uart_disable_rx_intr(UART_NUM_2);
}