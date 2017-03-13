#ifndef UART_H
#define UART_H

#define BT_GPIO_PERIPH			SYSCTL_PERIPH_GPIOE
#define BT_GPIO_BASE			GPIO_PORTE_BASE
#define BT_UART_PERIPH			SYSCTL_PERIPH_UART5
#define BT_UART_BASE			UART5_BASE
#define BT_UART_INT			INT_UART5
#define BT_RX_GPIO_MAP			GPIO_PE4_U5RX
#define BT_RX_GPIO_PIN			GPIO_PIN_4
#define BT_TX_GPIO_MAP			GPIO_PE5_U5TX
#define BT_TX_GPIO_PIN			GPIO_PIN_5

extern void UART0_IntHandler(void);
extern void BT_UART_IntHandler(void);

void ConfigureUART0(void);
void bluetooth_init(void);
void bluetooth_configure(void);
void BTUARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
int UARTwrite(const char *pcBuf, uint32_t ui32Len);

#endif
