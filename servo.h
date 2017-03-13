#ifndef SERVO_H
#define SERVO_H

#define SERVO_PWM_CLOCK_DIV	SYSCTL_PWMDIV_16
#define SERVO_PWM_PERIPH	SYSCTL_PERIPH_PWM0
#define SERVO_PWM_BASE		PWM0_BASE
#define SERVO_PWM_GEN		PWM_GEN_0
#define SERVO_PWM_OUT		PWM_OUT_0
#define SERVO_PWM_OUT_BIT	PWM_OUT_0_BIT
#define SERVO_GPIO_PERIPH	SYSCTL_PERIPH_GPIOB
#define SERVO_GPIO_BASE		GPIO_PORTB_BASE
#define SERVO_GPIO_PIN		GPIO_PIN_6
#define SERVO_GPIO_PIN_MAP	GPIO_PB6_M0PWM0

#define SERVO_PWM_PERIOD	(SysCtlClockGet() / 16 / 200)
/* Convert time in 0.01 ms to clock cycles */
#define SERVO_PWM_CYCLES(x)	(SysCtlClockGet() / 16 / 100000 * x)

void servo_init(void);
void servo_update(uint16_t pulse_width);

#endif
