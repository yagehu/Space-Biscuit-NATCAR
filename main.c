#include <float.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "utils/uartstdio.h"

#include "motor.h"
#include "servo.h"
#include "tsl1401.h"
#include "uart.h"

/*****************************************************************************/
/* Global Variables */
/*****************************************************************************/
bool isBuffer1;
/* Ping pong buffers */
buffer_t buffer1;
buffer_t buffer2;
buffer_t *buffer;
/* Binarized camera output */
uint8_t binarized[128];
/* Number of tracks seen by camera */
int line_count;

/*****************************************************************************/
/* Function Prototypes */
/*****************************************************************************/
/* Binarization */
void gray2bw(void);
/* Control */
void process_frame(void);
/* Functions for computing PID */
int8_t get_proportional(uint8_t center);
int16_t get_integral(void);
int8_t get_derivative(void);
/* Deciding current state of car */
void set_current_state(void);
/* Setting servo output */
uint16_t set_steer(void);

/*****************************************************************************/
/* Macros */
/*****************************************************************************/
#define SPEED_MAX		30
#define SPEED_LOW		20
/* PID constants */
#define Kp 0.030f
#define Ki 0.100f
#define Kd 3.000f

int main(void)
{
	ROM_FPUEnable();
	ROM_FPULazyStackingEnable();
	/* Set system clock to 80 MHz */
	ROM_SysCtlClockSet(
		SYSCTL_SYSDIV_2_5 |
		SYSCTL_USE_PLL |
		SYSCTL_XTAL_16MHZ |
		SYSCTL_OSC_MAIN
	);

	/* Choose one: UART0 for USB UART or Bluetooth */
	//ConfigureUART0();
	bluetooth_init();
	/* Configure camera interface */
	TSL1401Config();
	/* Configure servo interface */
	servo_init();
	/* Configure motor interface */
	motor_init();

	/* Process ping-pong buffers */
	while (1) {
		if (buffer1.isDone) {
			UARTprintf("1 ");
			buffer = &buffer1;
			gray2bw();
			process_frame();
			buffer->isDone = false;
		} else if (buffer2.isDone) {
			UARTprintf("2 ");
			buffer = &buffer2;
			gray2bw();
			process_frame();
			buffer->isDone = false;
		}
	}
}

void process_frame(void)
{
	int i;
	uint8_t left;
	uint8_t right;
	uint16_t steer;

	current = head;
	line_count = 0;

	/* Initialize current frame */
	for (i = 0; i < MAX_LINES; i++) {
		current->lines[i].edge_left = 0;
		current->lines[i].edge_right = 0;
		current->lines[i].center = 0;
		current->lines[i].width = 0;
		current->precompound_width = 0;
		current->precompound_slope = 0;
		current->precompound_center = 0;
		current->cycles_since_precompound = 0;
		current->cycles_since_prenone = 0;
		current->central_track_index = 0;
		current->count = 0;
	}

	/* Iterate through the camera output array and find valid track(s) */
	for (i = 0; i < 128; i++) {
		if (binarized[i] == 1 && (i == 0 || binarized[i - 1] == 0)) {
			left = i;

			while (binarized[i] != 0) {
				if (i == 127) {
					right = i;
					break;
				} else if (binarized[i + 1] == 0)
					right = i;

				i++;
			}

			if (right - left >= MAX_WIDTH) {
				line_count = 0;
				i = 128;
			} else if (
				right - left < MAX_WIDTH &&
				right - left > MIN_WIDTH
			) {
				line_count++;
				current->lines[line_count - 1].edge_left =
					left;
				current->lines[line_count - 1].edge_right =
					right;
				current->lines[line_count - 1].center =
					(right + left) / 2;
				current->lines[line_count - 1].width =
					right - left + 1;
			}
		}
	}

	/* Print information of valid lines for debug */
	for (i = 0; i < line_count; i++)
		UARTprintf("  %d: %d ", i, current->lines[i].center);

	/* Update linked list */
	current->next = tail;
	head = tail->next->next->next;
	head->next = NULL;
	tail = current;

	/* Control algorithm */
	set_current_state();
	steer = set_steer();
	/* Update new servo position */
	servo_update(steer);

	/* Print variables for debug */
	UARTprintf(" state: %d ", tail->frame_state);
	UARTprintf("current: %d", current->central_track_index);
	UARTprintf(" steer: %d\n", steer);
}

uint16_t set_steer(void)
{
	int16_t p, i, d;
	uint16_t steer;
	int speed;

	switch (tail->frame_state) {
	case SINGLE:
		/* Compute PID */
		p = get_proportional(tail->lines[0].center);
		i = get_integral();
		d = get_derivative();
		/* Print PID for debug */
		UARTprintf(" p: %d i: %d d: %d ", p, i, d);
		/* Convert PID to servo pulse width */
		steer = p * Kp + i * Ki + d * Kd + 150;
		/* Speed is inversely proportional to steer */
		speed = SPEED_MAX - 1.5 * abs(steer - 150);

		/* Do not allow speed to drop too low */
		if (speed < 10)
			speed = 10;

		/* Update new motor speed */
		motor_update(speed, true);
		break;
	case COMPOUND: /* Potentially overlapping lines */
		/* Slow down */
		motor_update(SPEED_LOW, true);
		/* Compute PID */
		p = get_proportional(tail->lines[0].center);
		i = get_integral();
		d = get_derivative();
		steer = p * Kp + i * Ki + d * Kd + 150;
		UARTprintf(" p: %d i: %d d: %d ", p, i, d);
		break;
	case MULTIPLE: /* See more than one track */
		if (tail->precompound_width != 0) {
			tail->precompound_center = 0;
			tail->precompound_slope = 0;
			tail->precompound_width = 0;
			p =
				get_proportional(
					tail->
					lines[tail->central_track_index].center
				);
		} else
			p =
				get_proportional(
					tail->
					lines[tail->central_track_index].center
				);

		steer = (p) / 1.5 + 150;
		motor_update(SPEED_MAX, true);
		break;
	case NONE: /* Lose sight of the track */
		if (tail->prenone_center < 64)
			p = 50;
		else if (tail->prenone_center > 64)
			p = -50;

		steer = p + 150;

		if (tail->prev_state == COMPOUND)
			motor_update(SPEED_MAX, true);
		else if (tail->count < 7) /* Break for first few cycles */
			motor_update(SPEED_MAX, false);
		else
			motor_update(SPEED_LOW, true);

		break;
	}

	return steer;
}

int8_t get_proportional(uint8_t center)
{
	return (64 - (int8_t)center);
}

int8_t get_derivative(void)
{
	int8_t prev =
		64 -
		(int8_t)
		tail->next->lines[tail->next->central_track_index].center;
	int8_t curr =
		64 - (int8_t)tail->lines[tail->central_track_index].center;

	return curr - prev;
}

int16_t get_integral(void)
{
	int i;
	int16_t integral = 0;
	frame_t *current = tail;

	/* Iterate through linked list */
	for (i = 0; i < BUFFER_COUNT; i++) {
		integral +=
			(64 -
			(int8_t)
			current->lines[tail->central_track_index].center);
		current = current->next;
	}

	return integral;
}

void gray2bw(void)
{
	int threshold;
	int i;
	unsigned int min = 256, max = 0;

	/* Find min and max */
	for (i = 0; i < 128; i++) {
		if (min > buffer->data[i]) {
			min = buffer->data[i];
		}

		if (max < buffer->data[i]) {
			max = buffer->data[i];
		}
	}

	/* Compute threshold */
	threshold = (min + max) / 2;

	/* If sees nothing */
	if (threshold < 14)
		threshold = 30;

	for (i = 0; i < 128; i++) {
		binarized[i] = (buffer->data[i] > threshold) ? 1 : 0;
		UARTprintf("%d", binarized[i]);
	}

	UARTprintf(" %d ", threshold);
}

void set_current_state(void)
{
	int i;
	int8_t center_dev;
	tail->prev_state = tail->next->prev_state;

	switch (tail->next->frame_state) {
	case INITIAL:
		tail->frame_state = SINGLE;
		tail->prev_state = INITIAL;
		tail->count = 1;
		break;
	case SINGLE:
		if (line_count > 1) {
			tail->frame_state = MULTIPLE;
			tail->count = 1;
			tail->prev_state = SINGLE;

			for (i = 0; i < MAX_LINES; i++) {
				tail->central_track_index = 0;

				if (i == 0)
					center_dev = abs((int)tail->lines[i].center - (int)tail->next->lines[0].center);

				if (abs((int)tail->lines[i].center - (int)tail->next->lines[0].center) < center_dev) {
					center_dev = abs((int)tail->lines[i].center - (int)tail->next->lines[0].center);
					tail->central_track_index = i;
				}
			}
		} else if (line_count == 1) {
			if (tail->next->next->frame_state == INITIAL) {
				tail->frame_state = SINGLE;
				tail->count = tail->next->count + 1;
			} else if ((int8_t)tail->lines[0].width - (int8_t)tail->next->lines[0].width > 7) {
				tail->frame_state = COMPOUND;
				tail->count = 1;
				tail->prev_state = SINGLE;
				tail->precompound_width = tail->next->lines[0].width;
				tail->precompound_center = tail->next->lines[0].center;
				tail->precompound_slope = tail->next->lines[0].width - tail->next->next->lines[0].width;
				tail->cycles_since_precompound = 1;
			} else {
				tail->frame_state = SINGLE;
				tail->count = tail->next->count + 1;
			}
		} else {
			tail->frame_state = NONE;
			tail->count = 1;
			tail->prev_state = SINGLE;
			tail->prenone_center = tail->next->lines[0].center;
			tail->prenone_width = tail->next->lines[0].width;
			tail->prenone_slope = tail->next->lines[0].width - tail->next->next->lines[0].width;
			tail->cycles_since_prenone = 1;
		}

		break;
	case COMPOUND:
		if (line_count > 1) {
			tail->frame_state = MULTIPLE;
			tail->count = 1;
			tail->prev_state = COMPOUND;
			tail->cycles_since_precompound = tail->next->cycles_since_precompound + 1;
			tail->precompound_width = tail->next->precompound_width;
			tail->precompound_center = tail->next->precompound_center;
			tail->precompound_slope = tail->next->precompound_slope;

			for (i = 0; i < MAX_LINES; i++) {
				tail->central_track_index = 0;

				if (i == 0)
					center_dev = abs((int)tail->lines[i].center - (int)(tail->precompound_center + tail->precompound_slope * tail->cycles_since_precompound));
				if (abs((int)tail->lines[i].center - (int)(tail->precompound_center + tail->precompound_slope * tail->cycles_since_precompound)) < center_dev) {
					center_dev = abs((int)tail->lines[i].center - (int)(tail->precompound_center + tail->precompound_slope * tail->cycles_since_precompound));
					tail->central_track_index = i;
				}
			}
		} else if (line_count == 1) {
			if (tail->lines[0].width < tail->next->precompound_width + 5) {
				tail->cycles_since_precompound = tail->next->cycles_since_precompound + 1;
				tail->frame_state = SINGLE;
				tail->count = 1;
				tail->prev_state = COMPOUND;
			} else {
				tail->cycles_since_precompound = tail->next->cycles_since_precompound + 1;
				tail->precompound_width = tail->next->precompound_width;
				tail->precompound_center = tail->next->precompound_center;
				tail->precompound_slope = tail->next->precompound_slope;
				tail->frame_state = COMPOUND;
				tail->count = tail->next->count + 1;
			}
		} else {
			tail->frame_state = NONE;
			tail->count = 1;
			tail->prev_state = COMPOUND;
			tail->prenone_compound_center = tail->next->lines[0].center;
			tail->prenone_center = tail->next->precompound_center;
			tail->prenone_width = tail->next->precompound_width;
			tail->prenone_slope = tail->next->precompound_slope;
			tail->cycles_since_prenone = 1;
		}

		break;
	case MULTIPLE:
		if (line_count > 1) {
			tail->frame_state = MULTIPLE;
			tail->count = tail->next->count + 1;

			for (i = 0; i < MAX_LINES; i++) {
				tail->central_track_index = 0;

				if (i == 0)
					center_dev = abs((int)tail->lines[i].center - (int)(tail->next->lines[tail->next->central_track_index].center));

				if (abs((int)tail->lines[i].center - (int)(tail->next->lines[tail->next->central_track_index].center)) < center_dev) {
					center_dev = abs((int)tail->lines[i].center - (int)(tail->next->lines[tail->next->central_track_index].center));
					tail->central_track_index = i;
				}
			}
		} else if (line_count == 1) {
			if (tail->lines[0].width > tail->next->lines[tail->next->central_track_index].width + 2) {
				tail->frame_state = COMPOUND;
				tail->count = 1;
				tail->prev_state = MULTIPLE;
				tail->precompound_width = tail->next->lines[tail->next->central_track_index].width;
				tail->precompound_center = tail->next->lines[tail->next->central_track_index].center;
				tail->precompound_slope = tail->next->lines[tail->next->central_track_index].center - tail->next->next->lines[tail->next->next->central_track_index].center;
				tail->cycles_since_precompound = 1;
			} else {
				tail->frame_state = SINGLE;
				tail->count = 1;
				tail->prev_state = MULTIPLE;
			}
		} else {
			tail->frame_state = NONE;
			tail->count = 1;
			tail->prev_state = MULTIPLE;
			tail->prenone_center = tail->next->lines[tail->next->central_track_index].center;
			tail->prenone_width = tail->next->lines[tail->next->central_track_index].center;
			tail->prenone_slope = tail->next->lines[tail->next->central_track_index].center - tail->next->next->lines[tail->next->next->central_track_index].center;
			tail->cycles_since_prenone = 1;
		}

		break;
	default:
		if (line_count == 1) {
			/* NONE -> COMPOUND */
			if (tail->lines[0].width > tail->next->prenone_width + 2) {
				tail->frame_state = COMPOUND;
				tail->count = 1;
				tail->prev_state = NONE;
				tail->precompound_width = tail->prenone_width;
				tail->precompound_center = tail->prenone_center;
				tail->precompound_slope = tail->prenone_slope;
				tail->cycles_since_precompound = tail->cycles_since_prenone + 1;
			} else { /* NONE -> SINGLE */
				tail->frame_state = SINGLE;
				tail->prev_state = NONE;
				tail->count = 1;
			}
		} else { /* NONE -> NONE */
			tail->frame_state = NONE;
			tail->count = tail->next->count + 1;
			tail->prenone_compound_center = tail->next->prenone_compound_center;
			tail->prenone_center = tail->next->prenone_center;
			tail->prenone_width = tail->next->prenone_width;
			tail->prenone_slope = tail->next->prenone_slope;
			tail->cycles_since_prenone += 1;
		}

		break;
	}
}

void UART0_IntHandler(void)
{
	uint32_t ui32Status;
	char c;

	ui32Status = UARTIntStatus(UART0_BASE, true);
	UARTIntClear(UART0_BASE, ui32Status);

	while (UARTCharsAvail(UART0_BASE)) {
		c = (char)UARTCharGetNonBlocking(UART0_BASE);
		UARTCharPutNonBlocking(UART0_BASE, c);
	}
}

void BT_UART_IntHandler(void)
{
	uint32_t ui32Status;
	char c;

	ui32Status = UARTIntStatus(BT_UART_BASE, true);
	UARTIntClear(BT_UART_BASE, ui32Status);

	while (UARTCharsAvail(BT_UART_BASE)) {
		c = (char)UARTCharGetNonBlocking(BT_UART_BASE);
		UARTCharPutNonBlocking(BT_UART_BASE, c);
	}
}
