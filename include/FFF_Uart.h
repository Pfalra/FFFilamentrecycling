#ifndef FFF_UART_H
#define FFF_UART_H

void FFF_Uart_init();
uint8_t* FFF_Uart_getCurrentBufferUart2();
FFF_Buffer* FFF_Uart_getFilledBufferUart2();
void FFF_Uart_protectBufferUart2(FFF_Buffer* bufPtr);
void FFF_Uart_unprotectBufferUart2(FFF_Buffer* bufPtr);
void FFF_Uart_activateInterruptRX_Uart2();
void FFF_Uart_deactivateInterruptRX_Uart2();
#endif 