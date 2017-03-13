#include <stdbool.h>
#include <stdint.h>

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"

#include "motor.h"

void motor_init(void)
{
	SysCtlPWMClockSet(MOTOR_PWM_CLOCK_DIV);
	SysCtlPeripheralEnable(MOTOR_PWM_PERIPH);
	SysCtlPeripheralEnable(MOTOR_GPIO_PERIPH);
	GPIOPinConfigure(MOTOR_GPIO_PIN_MAP);
	GPIOPinTypePWM(MOTOR_GPIO_BASE, MOTOR_GPIO_PIN);
	PWMGenConfigure(
		MOTOR_PWM_BASE,
		MOTOR_PWM_GEN,
		PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(MOTOR_PWM_BASE, MOTOR_PWM_GEN, MOTOR_PWM_PERIOD);
	PWMPulseWidthSet(MOTOR_PWM_BASE, MOTOR_PWM_OUT, 0 );
	PWMOutputState(MOTOR_PWM_BASE, MOTOR_PWM_OUT_BIT, true);
	PWMGenEnable(MOTOR_PWM_BASE, MOTOR_PWM_GEN);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);

	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);

	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5);
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0);
}

/* duty_cycle 0-100 */
void motor_update(uint8_t duty_cycle, bool is_forward)
{
	PWMPulseWidthSet(MOTOR_PWM_BASE, MOTOR_PWM_OUT, MOTOR_PWM_CYCLES(duty_cycle));

	if (is_forward) {
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0);
	} else if (!is_forward) {
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0);
	}
}
