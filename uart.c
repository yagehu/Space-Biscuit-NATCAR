#include <stdbool.h>
#include <stdint.h>

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "utils/uartstdio.h"

#include "UART.h"

//*****************************************************************************
// Configure the UART and its pins.  This must be called before UARTprintf().
//*****************************************************************************
void ConfigureUART0(void)
{
	// Enable the GPIO Peripheral used by the UART.
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Enable UART0
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	// Configure GPIO Pins for UART mode.
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Use the internal 16MHz oscillator as the UART clock source.
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	// Initialize the UART for console I/O.
	UARTStdioConfig(0, 115200, 16000000);

	UARTIntRegister(UART0_BASE, UART0_IntHandler);
	IntEnable(INT_UART0);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

void bluetooth_init(void)
{
	ROM_SysCtlPeripheralEnable(BT_GPIO_PERIPH);
	ROM_SysCtlPeripheralEnable(BT_UART_PERIPH);
	ROM_GPIOPinConfigure(BT_RX_GPIO_MAP);
	ROM_GPIOPinConfigure(BT_TX_GPIO_MAP);
	ROM_GPIOPinTypeUART(BT_GPIO_BASE, BT_RX_GPIO_PIN | BT_TX_GPIO_PIN);
	//UARTClockSourceGet(BT_UART_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(5, 115200, ROM_SysCtlClockGet());
//	ROM_UARTConfigSetExpClk(
//		BT_UART_BASE,
//		ROM_SysCtlClockGet(),
//		115200,
//		(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTIntRegister(BT_UART_BASE, BT_UART_IntHandler);
	IntEnable(BT_UART_INT);
	UARTIntEnable(BT_UART_BASE, UART_INT_RX | UART_INT_RT);
}

/* Only for setting up a new HC-06
 * Must send with 9600 baud rate
 */
void bluetooth_configure(void)
{
	/* Configure Bluetooth module */
	BTUARTSend((uint8_t *)"AT", 2);
	ROM_SysCtlDelay(20000000);
	BTUARTSend((uint8_t *)"AT+NAMESB_HC06", 14);
	ROM_SysCtlDelay(20000000);
	BTUARTSend((uint8_t *)"AT+PIN2587", 10);
	ROM_SysCtlDelay(20000000);
	BTUARTSend((uint8_t *)"AT+VERSION", 10);
	ROM_SysCtlDelay(30000000);
	BTUARTSend((uint8_t *)"AT+BAUD8", 8);
}

void BTUARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
	while (ui32Count--) {
		ROM_UARTCharPut(BT_UART_BASE, *pui8Buffer++);
	}
}
