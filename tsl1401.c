#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "utils/uartstdio.h"

#include "motor.h"
#include "servo.h"
#include "tsl1401.h"

buffer_t *current_buffer;
uint8_t current_buffer_count;

void TSL1401Config(void)
{
	ROM_SysCtlPeripheralEnable(CLK_GPIO_PERIPH);
	ROM_SysCtlPeripheralEnable(SI_GPIO_PERIPH);
	ROM_SysCtlPeripheralEnable(AOUT_GPIO_PERIPH);
	ROM_SysCtlPeripheralEnable(AOUT_ADC_PERIPH);

	/* CLK */
	ROM_GPIODirModeSet(CLK_BASE, CLK_PIN, GPIO_DIR_MODE_OUT);
	ROM_GPIOPadConfigSet(
		CLK_BASE,
		CLK_PIN,
		GPIO_STRENGTH_4MA,
		GPIO_PIN_TYPE_STD_WPU
	);

	/* SI */
	ROM_GPIODirModeSet(SI_BASE, SI_PIN, GPIO_DIR_MODE_OUT);
	ROM_GPIOPadConfigSet(
		SI_BASE,
		SI_PIN,
		GPIO_STRENGTH_4MA,
		GPIO_PIN_TYPE_STD_WPU
	);

	/* AOUT */
	GPIOPinTypeADC(AOUT_BASE, AOUT_PIN);
	ADCSequenceConfigure(AOUT_ADC_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ROM_ADCSequenceStepConfigure(
		AOUT_ADC_BASE,
		3,
		0,
		ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END
	);
	ROM_ADCSequenceEnable(AOUT_ADC_BASE, 3);
	ADCIntRegister(AOUT_ADC_BASE, 3, TSL1401ADC_IntHandler);
	ROM_ADCIntClear(AOUT_ADC_BASE, 3);
	ADCIntEnable(AOUT_ADC_BASE,3);
	IntEnable(INT_ADC0SS3);

	/* Timer */
	ROM_SysCtlPeripheralEnable(TSL1401_TIMER_PERIPH);
	ROM_TimerConfigure(TSL1401_TIMER_BASE, TIMER_CFG_PERIODIC);
	ROM_TimerLoadSet(
		TSL1401_TIMER_BASE,
		TIMER_A,
		ROM_SysCtlClockGet() / TSL1401_TIMER_FREQ);
	TimerIntRegister(TSL1401_TIMER_BASE, TIMER_A, TSL1401Timer_IntHandler);
	ROM_IntEnable(TSL1401_TIMER_INT);
	ROM_TimerIntEnable(TSL1401_TIMER_BASE, TIMER_TIMA_TIMEOUT);
	ROM_TimerEnable(TSL1401_TIMER_BASE, TIMER_A);

	/* Initialize the linked list */
	head = &frame4;
	tail = &frame0;
	frame0.next = &frame1;
	frame1.next = &frame2;
	frame2.next = &frame3;
	frame3.next = &frame4;
	frame4.next = NULL;
	frame0.frame_state = INITIAL;
	frame1.frame_state = INITIAL;
	frame2.frame_state = INITIAL;
	frame3.frame_state = INITIAL;
	frame4.frame_state = INITIAL;

	IntMasterEnable();
}

void TSL1401Timer_IntHandler(void)
{
	TimerIntClear(TSL1401_TIMER_BASE, TIMER_TIMA_TIMEOUT);
	ROM_TimerDisable(TSL1401_TIMER_BASE, TIMER_A);
	ROM_ADCIntEnable(ADC0_BASE, 3);

	TriggerADC();
	TimerLoadSet(
		TSL1401_TIMER_BASE,
		TIMER_A,
		ROM_SysCtlClockGet() / TSL1401_TIMER_FREQ);
	ROM_TimerEnable(TSL1401_TIMER_BASE, TIMER_A);
}

void TSL1401ADC_IntHandler(void)
{
	ReadFrame();
}

void ReadFrame(void)
{
	uint32_t val;

	/* CLK high */
	ROM_GPIOPinWrite(CLK_BASE, CLK_PIN, CLK_PIN);
	/* Clear the ADC interrupt flag */
	ROM_ADCIntClear(ADC0_BASE, 3);

	/* Read ADC value */
	ROM_ADCSequenceDataGet(ADC0_BASE, 3, &val);
	current_buffer->data[current_buffer_count] = val >> 4;
	current_buffer->histogram[current_buffer->data[current_buffer_count]] += 1;

	if (current_buffer_count < 127)
	{
		current_buffer_count++;
		/* Trigger ADC conversion */
		ROM_ADCProcessorTrigger(ADC0_BASE, 3);
	} else if (current_buffer_count == 127) {
		ROM_ADCIntDisable(ADC0_BASE, 3);
		current_buffer_count = 0;
		isBuffer1 = !isBuffer1;
		current_buffer->isDone = true;
	}

	/* CLK low */
	ROM_GPIOPinWrite(CLK_BASE, CLK_PIN, 0);
}

void TriggerADC(void)
{
	/* SI high */
	ROM_GPIOPinWrite(SI_BASE, SI_PIN, SI_PIN);
	/* Setup time t_SI = 20 ns */
	ROM_SysCtlDelay(1);
	/* CLK high */
	ROM_GPIOPinWrite(CLK_BASE, CLK_PIN, CLK_PIN);
	/* Analog output settling time t_s = 120 ns */
	ROM_SysCtlDelay(6);

	/* Check to see which buffer to write to */
	if (isBuffer1)
		current_buffer = &buffer1;
	else
		current_buffer = &buffer2;

	/* SI low */
	ROM_GPIOPinWrite(SI_BASE, SI_PIN, 0);
	/* Trigger ADC conversion */
	ROM_ADCProcessorTrigger(ADC0_BASE, 3);
	/* CLK low */
	ROM_GPIOPinWrite(CLK_BASE, CLK_PIN, 0);
}
