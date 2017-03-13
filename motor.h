#ifndef MOTOR_H
#define MOTOR_H

#define MOTOR_PWM_CLOCK_DIV	SYSCTL_PWMDIV_16
#define MOTOR_PWM_PERIPH	SYSCTL_PERIPH_PWM0
#define MOTOR_PWM_BASE		PWM0_BASE
#define MOTOR_PWM_GEN		PWM_GEN_0
#define MOTOR_PWM_OUT		PWM_OUT_1
#define MOTOR_PWM_OUT_BIT	PWM_OUT_1_BIT
#define MOTOR_GPIO_PERIPH	SYSCTL_PERIPH_GPIOB
#define MOTOR_GPIO_BASE		GPIO_PORTB_BASE
#define MOTOR_GPIO_PIN		GPIO_PIN_7
#define MOTOR_GPIO_PIN_MAP	GPIO_PB7_M0PWM1

#define MOTOR_PWM_PERIOD	(SysCtlClockGet() / 16 / 200)
#define MOTOR_PWM_CYCLES(x)	(MOTOR_PWM_PERIOD * x / 100)

void motor_init(void);
void motor_update(uint8_t duty_cycle, bool is_forward);

#endif
