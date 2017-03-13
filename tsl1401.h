#ifndef TSL1401_H
#define TSL1401_H

/* Timer 0 */
#define TSL1401_TIMER_PERIPH	SYSCTL_PERIPH_TIMER0
#define TSL1401_TIMER_BASE	TIMER0_BASE
#define TSL1401_TIMER_FREQ	100
#define TSL1401_TIMER_INT	INT_TIMER0A

/* AOUT */
#define AOUT_GPIO_PERIPH	SYSCTL_PERIPH_GPIOE
#define AOUT_ADC_PERIPH		SYSCTL_PERIPH_ADC0
#define AOUT_BASE		GPIO_PORTE_BASE
#define AOUT_PIN		GPIO_PIN_3
#define AOUT_ADC_BASE		ADC0_BASE

/* CLK */
#define CLK_GPIO_PERIPH		SYSCTL_PERIPH_GPIOE
#define CLK_BASE		GPIO_PORTE_BASE
#define CLK_PIN			GPIO_PIN_2

/* SI */
#define SI_GPIO_PERIPH		SYSCTL_PERIPH_GPIOE
#define SI_BASE			GPIO_PORTE_BASE
#define SI_PIN			GPIO_PIN_1

/* For image processing */
#define MAX_LINES		2
#define MAX_WIDTH		60
#define MIN_WIDTH		5
#define BUFFER_COUNT		5

typedef enum state {
	SINGLE,
	COMPOUND,
	MULTIPLE,
	NONE,
	INITIAL
} State;

typedef struct buffer {
	uint8_t data[128];
	uint8_t histogram[256];
	bool isDone;
} buffer_t;

typedef struct line {
	uint8_t edge_left;
	uint8_t edge_right;
	uint8_t center;
	uint8_t width;
} line_t;

typedef struct frame {
	line_t lines[MAX_LINES];
	struct frame *next;
	uint8_t central_track_index;
	/* For saving information when losing sight of target */
	uint8_t prenone_center;
	uint8_t prenone_compound_center;
	uint8_t prenone_width;
	int8_t prenone_slope;
	uint8_t cycles_since_prenone;
	/* For saving information when entering a complex state */
	uint8_t precompound_width;
	uint8_t precompound_center;
	int8_t precompound_slope;
	uint8_t cycles_since_precompound;
	State frame_state;
	State prev_state; /* Previous state before change */
	uint32_t count;
} frame_t;

frame_t frame0;
frame_t frame1;
frame_t frame2;
frame_t frame3;
frame_t frame4;
frame_t *head;
frame_t *tail;
frame_t *current;

extern bool isBuffer1;
extern uint8_t current_buffer_count;
extern buffer_t buffer1;
extern buffer_t buffer2;
buffer_t *current_buffer;

void TSL1401Timer_IntHandler(void);
void TSL1401ADC_IntHandler(void);

void TSL1401Config(void);
void ReadFrame(void);
void TriggerADC(void);

#endif
