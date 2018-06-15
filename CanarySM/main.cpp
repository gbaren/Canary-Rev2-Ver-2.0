//#define TEST

#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "canary.h"

enum input {flash, timeout, none};
input in = none;

unsigned int wdt_counter = 0;
unsigned int prescaler_frequency = 8000;
unsigned char timeout_state = RUNNING;

void delay_ms(unsigned long ms)
{
	while(ms--)
	_delay_ms(1);
}

void led_flash(unsigned char color, unsigned char times, unsigned int delay) {
	led_green_off();
	led_red_off();
	for (int i=0; i<times; i++) {
		switch (color) {
			case LED_GREEN :
				led_green_on();
				break;
			case LED_RED :
				led_red_on();
				break;
			case LED_ORANGE :
				led_green_on();
				led_red_on();
				break;
		}
		delay_ms(delay);
		led_green_off();
		led_red_off();
		delay_ms(delay);
	}
}

unsigned int get_timeout() {
	volatile unsigned int multiplier = 0;
	volatile unsigned int timeout = 0;
	volatile unsigned char cfg_timeout = !readbit(PINB,SW0) + 1;
	
	#ifdef TEST
		multiplier = 60000;		// 1 minute for testing
	#else
		multiplier = TIMEOUT_MULTIPLIER;
	#endif

	if (timeout_state == RUNNING) {
		timeout = (cfg_timeout * multiplier) / prescaler_frequency + 1;
	} else {
		timeout = (MOBO_ON_WAIT_TIME / prescaler_frequency) + 1;
	}
	
	return (timeout);
}

ISR(PCINT0_vect) {
	in = flash;
	timeout_state = RUNNING;
}

ISR(WDT_vect) {
	setbit(WDTCR, WDIE);	// this keeps us from resetting the micro
	wdt_counter++;
	if (wdt_counter > get_timeout()) {
		in = timeout;
	} else {
		in = none;
	}
}

void state_entry(void) {
	DDRB =  0b00001011;		// set PB0, PB1 and PB3 for output
	PORTB = 0b00101111;		// enable pull-ups for all pins but the HDLED (PB4)

	clrbit(ADCSRA,ADEN);	// disable ADC (default is enabled in all sleep modes)
	setbit(GIMSK, PCIE);	// enable pin change interrupts
	setbit(PCMSK, PCINT4);	// setup to interrupt on pin change of PB4 (HDLED)

	WDTCR |= bitval(WDE) | bitval(WDIE) | WDT_TIMEOUT_8S;
	prescaler_frequency = 8000;
	
	led_flash(LED_RED,1,FLASH_DELAY_LONG_MS);
	led_flash(LED_ORANGE,1,FLASH_DELAY_LONG_MS);
	led_flash(LED_GREEN,1,FLASH_DELAY_LONG_MS);
}

void state_run(void) {
	int timeout_fifths = (int)((float)wdt_counter / get_timeout() / .2) + 1;
	led_flash(LED_ORANGE,timeout_fifths,FLASH_DELAY_SHORT_MS);
	timeout_state = RUNNING;
}

void state_led_flashed() {
	led_flash(LED_GREEN,1,FLASH_DELAY_SHORT_MS);
	wdt_counter = 0;
}

void state_mobo_off(void) {
	led_flash(LED_RED,3,FLASH_DELAY_LONG_MS);
	mobo_reset_on();
	delay_ms(POWER_CYCLE_HOLD_MS);
	mobo_reset_off();
	wdt_counter = 0;
	timeout_state = RESTARTING;
}

void state_reset_wait(void) {
	int timeout_fifths = (int)((float)wdt_counter / get_timeout() / .2) + 1;
	led_flash(LED_RED,timeout_fifths,FLASH_DELAY_SHORT_MS);
}

void state_mobo_on(void) {
	led_flash(LED_GREEN,2,FLASH_DELAY_SHORT_MS);
	mobo_reset_on();
	delay_ms(POWER_CYCLE_ON_MS);
	mobo_reset_off();
	wdt_counter = 0;
	timeout_state = RUNNING;
}

enum input wait_for_interrupt() {
	MCUCR |= SLEEP_MODE_PWR_DOWN;	// set sleep mode
	setbit(MCUCR,SE);				// sleep enable bit
	sei();							// enable interrupts
	sleep_cpu();					// sleep
	clrbit(MCUCR,SE);				// sleep disable
	return (in);
}

void (* state[])(void) = {state_entry,state_run,state_led_flashed,state_mobo_off,state_reset_wait,state_mobo_on};
enum state_code {entry,run,led_flashed,mobo_off,reset_wait,mobo_on};

struct transition {
	enum state_code state;
	enum input		input;
	enum state_code new_state;
};

struct transition state_transitions[] = {
	{entry,			flash,		run},
	{entry,			timeout,	run},
	{entry,			none,		run},
	{run,			flash,		led_flashed},
	{run,			timeout,	mobo_off},
	{run,			none,		run},
	{led_flashed,	flash,		run},
	{led_flashed,	timeout,	run},
	{led_flashed,	none,		run},
	{mobo_off,		flash,		reset_wait},
	{mobo_off,		timeout,	reset_wait},
	{mobo_off,		none,		reset_wait},
	{reset_wait,	flash,		run},
	{reset_wait,	timeout,	mobo_on},
	{reset_wait,	none,		reset_wait},
	{mobo_on,		flash,		run},
	{mobo_on,		timeout,	run},
	{mobo_on,		none,		run}
};


enum state_code lookup_transition(enum state_code state_code, enum input input) {
	enum state_code to_state = state_code;
	unsigned char transition_index = 0;
	unsigned int num_transitions = sizeof(state_transitions) / sizeof(transition);

	for (transition_index=0; transition_index < num_transitions; transition_index++) {
		if ((state_transitions[transition_index].state == state_code) && (state_transitions[transition_index].input == input)) break;
	}
	
	if (transition_index > num_transitions) {
		to_state = state_code;
	} else {
		to_state = state_transitions[transition_index].new_state;
	}

	return (to_state);
};

int main() {
	enum state_code cur_state = entry;
	enum input input;
	void (* state_func)(void);

	while (1) {
		state_func = state[cur_state];
		state_func();
		input = wait_for_interrupt();
		cur_state = lookup_transition(cur_state,input);
	}
}    



