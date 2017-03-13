#include <stdbool.h>
#include <stdint.h>

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"

#include "servo.h"

void servo_init(void)
{
	SysCtlPeripheralEnable(SERVO_PWM_PERIPH);
	SysCtlPeripheralEnable(SERVO_GPIO_PERIPH);
	SysCtlPWMClockSet(SERVO_PWM_CLOCK_DIV);
	GPIOPinConfigure(SERVO_GPIO_PIN_MAP);
	GPIOPinTypePWM(SERVO_GPIO_BASE, SERVO_GPIO_PIN);
	PWMGenConfigure(
		SERVO_PWM_BASE,
		SERVO_PWM_GEN,
		PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(SERVO_PWM_BASE, SERVO_PWM_GEN, SERVO_PWM_PERIOD);
	PWMPulseWidthSet(SERVO_PWM_BASE, SERVO_PWM_OUT, SERVO_PWM_CYCLES(150));
	PWMOutputState(SERVO_PWM_BASE, SERVO_PWM_OUT_BIT, true);
	PWMGenEnable(SERVO_PWM_BASE, SERVO_PWM_GEN);
}

void servo_update(uint16_t pulse_width)
{
	PWMPulseWidthSet(
		SERVO_PWM_BASE,
		SERVO_PWM_OUT,
		SERVO_PWM_CYCLES(pulse_width));
}
