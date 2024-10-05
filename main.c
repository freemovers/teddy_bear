/*
* Bear.c
*
* Created: 12/14/2020 8:45:53 PM
* Author : svandeBor
* LED's on PA1, PA3, PA6, PA7
*
*  PA6     PA7
*		PA1		(TCA - WO1)
*		PA3
*
* Button on PA2
* 
* Patterns:
* 1. Single heart beat		PA6 0x40
* 2. Single breath			PA7 0x80
* 3. Single flash			PA3 0x08
* 4. Full heart beat		PA6 + PA1 0x42
* 5. Full breath			PA7 + PA1 0x82
* 6. Outer flash			PA3 + PA1 0x0A
* 7. Pulse					PA1 0x02
*/

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include "main.h"

#define NO_LEDS		0x00
#define INNER_LED	0x02
#define OUTER_LEDS	0xC8
#define ALL_LEDS	0xCA

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
#define USERROW_OFFSET	(0x1400)

int8_t pwmChange = 2;
int8_t ledson = NO_LEDS;
int8_t ledsoff = ALL_LEDS;

int8_t buttonpressed = 0;
int8_t buttonStillPressed = 0;
uint8_t buttonTimePressed = 0;
int8_t enterSleepMode = 0;
uint8_t flashCount = 0;

int8_t LEDS[] = {0x40, 0x80, 0x08, 0x42, 0x82, 0x0A, 0x02};
int8_t RANDOMLEDS[] = {0x40, 0x00, 0x00, 0x80, 0x00, 0x00, 0x08, 0x00, 0x00, 0x02, 0x00, 0x00};

typedef enum st_led_state {
	PR_INTERVAL,
	QRS_UP,
	QRS_DOWN,
	ST_SEGMENT,
	TWAVE_UP,
	TWAVE_DOWN,
} led_state_t;

volatile led_state_t led_state = PR_INTERVAL;



typedef void (*SimplePatternList[])();		// List of patterns to cycle through.  Each is defined as a separate function below.
SimplePatternList gPatterns = { singlePulse_pattern, singleBreath_pattern, singleFlash_pattern, fullPulse_pattern, fullBreath_pattern, fullFlash_pattern, sparkle_pattern };
uint8_t gCurrentPatternNumber = 0;			// Index number of which pattern is current

void RTC_init(void)
{
	RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;		// 32.768kHz Internal Crystal Oscillator (INT32K)
	while (RTC.STATUS > 0);					// Wait for all register to be synchronized
	RTC.PER = 0x7F;							// Max for overflow (128 Hz, 7.78ms)
	RTC.CMP = 0x06;							// Compare at 244us
	RTC.CNT = 0x0;
	RTC.INTCTRL |= RTC_OVF_bm;				// Enable overflow Interrupt which will trigger ISR
	RTC.INTCTRL |= RTC_CMP_bm;				// Enable compare Interrupt which will trigger ISR
	RTC.CTRLA = RTC_PRESCALER_DIV1_gc		// 32768 / 1 = 32768 (Hz)
	| RTC_RTCEN_bm							// Enable: enabled
	| RTC_RUNSTDBY_bm;						// Run In Standby: enabled

	RTC.PITINTCTRL = RTC_PI_bm;				// Periodic Interrupt: enabled
	RTC.PITCTRLA = RTC_PERIOD_CYC512_gc		// 32768 / 512 = 64Hz, 15.6ms
	| RTC_PITEN_bm;							// Enable: enabled
}

ISR(RTC_CNT_vect)
{
	if (RTC.INTFLAGS & RTC_CMP_bm)
	{
		RTC.INTFLAGS = RTC_CMP_bm;          // Clear flag by writing '1':
		PORTA.OUTCLR = ledsoff;
	}
	if (RTC.INTFLAGS & RTC_OVF_bm)
	{
		RTC.INTFLAGS = RTC_OVF_bm;          // Clear flag by writing '1':
		PORTA.OUTSET = ledson;
	}
}

ISR(RTC_PIT_vect)
{
	RTC.PITINTFLAGS = RTC_PI_bm;			// Clear flag by writing '1'
	
	if(enterSleepMode > 15) sleepPattern();
	else if(buttonStillPressed) showPattern();
	else gPatterns[gCurrentPatternNumber]();
	
	if(!(PORTA.IN & PIN2_bm)) push_button_action();
	else {
		if(enterSleepMode > 15) {			// enter sleep mode
			enterSleepMode++;				// Debounce the release of the button
			if(enterSleepMode > 20) {
				gCurrentPatternNumber = ((gCurrentPatternNumber - 1) + ARRAY_SIZE( gPatterns)) % ARRAY_SIZE( gPatterns);
				sleep_device();
			}
		}
		else if(enterSleepMode > 1) {
			enterSleepMode++;				// Debounce the release of the button
			if(enterSleepMode > 8) {
				wake_device();
			}
		}
		else {
			if(buttonStillPressed != 0) {
				ledson = NO_LEDS;
				ledsoff = ALL_LEDS;
			}
			buttonStillPressed = 0;
		}
	}
}
void push_button_action(void)
{
	if(!buttonStillPressed)	{
		buttonpressed++;
		buttonTimePressed = 0;
		// add one to the current pattern number, and wrap around at the end
		gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
		RTC.CMP = 0x08;						// Reset to avoid overflow on some patterns
	}
	buttonStillPressed = 1;
	buttonTimePressed++;
	if(buttonTimePressed > 128) enterSleepMode = 16;
}

void sleep_device(void)
{
	RTC.PITINTCTRL &= ~(RTC_PITEN_bm);		// stop the PIT in sleep mode to conserve energy
	while (RTC.PITSTATUS);				// Wait for all registers to be synchronized
	enterSleepMode = 2;
	buttonTimePressed = 0;
	PORTA.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t*)(&USERROW.USERROW0 - USERROW_OFFSET), 0x01);
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t*)(&USERROW.USERROW1 - USERROW_OFFSET), gCurrentPatternNumber);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	ledson = NO_LEDS;
	ledsoff = ALL_LEDS;
	PORTA.OUTCLR = ALL_LEDS;
}

void wake_device(void)
{
	enterSleepMode = 0;
	PORTA.DIRSET |= PIN1_bm  | PIN3_bm | PIN6_bm | PIN7_bm;
	PORTA.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
}

// Wake up routine
ISR(PORTA_PORT_vect)
{
	PORTA.INTFLAGS = PIN2_bm;				// clear interrupt flag
	PORTA.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
	RTC.PITINTCTRL = RTC_PI_bm;
	set_sleep_mode(SLEEP_MODE_STANDBY);		// Set sleep mode to IDLE mode
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t*)(&USERROW.USERROW0 - USERROW_OFFSET), 0x00);
}

void PORT_init(void)
{
	PORTA.PIN0CTRL = PORT_PULLUPEN_bm | PORT_ISC_INPUT_DISABLE_gc; // Disable input buffer and enable the internal pull-up on PAx pins to conserve energy
	PORTA.DIRSET = ALL_LEDS;
	PORTA.PIN2CTRL &= ~(PORT_ISC_INPUT_DISABLE_gc);
}

int main(void)
{
	RTC_init();
	PORT_init();
	set_sleep_mode(SLEEP_MODE_STANDBY);		// Set sleep mode to STANDBY mode
	sleep_enable();
	sei();
	eeprom_busy_wait();
	gCurrentPatternNumber = eeprom_read_byte((uint8_t*)(&USERROW.USERROW1 - USERROW_OFFSET));
	if(gCurrentPatternNumber > ARRAY_SIZE( gPatterns)) gCurrentPatternNumber = 0;
	RTC_init();
	eeprom_busy_wait();
	if(eeprom_read_byte((uint8_t*)(&USERROW.USERROW0 - USERROW_OFFSET))) sleep_device();
	else wake_device();

	while (1) {
		sleep_cpu();						// Nothing to do here
	}
}

void singlePulse_pattern(void)
{
	switch(led_state)
	{		
		case PR_INTERVAL:
		RTC.CMP += pwmChange;
		if(RTC.CMP >= 0x50) {
			led_state = QRS_UP;
			ledson = INNER_LED;
			RTC.CMP = 0x06;
		}
		break;
		case QRS_UP:
		RTC.CNT = 0x0;
		RTC.CMP += pwmChange * 2;
		if(RTC.CMP >= 0x70) {
			led_state = QRS_DOWN;
			ledson = INNER_LED;
		}
		break;
		case QRS_DOWN:
		RTC.CMP -= pwmChange * 2;
		if(RTC.CMP <= 0x06) {
			led_state = ST_SEGMENT;
			ledson = NO_LEDS;
		}
		break;
		case ST_SEGMENT:
		RTC.CMP += pwmChange;
		if(RTC.CMP >= 0x10) {
			led_state = TWAVE_UP;
			ledson = INNER_LED;
			RTC.CMP = 0x06;
		}
		break;
		case TWAVE_UP:
		RTC.CMP += pwmChange;
		if(RTC.CMP >= 0x30) {
			led_state = TWAVE_DOWN;
			ledson = INNER_LED;
		}
		break;
		case TWAVE_DOWN:
		RTC.CMP -= pwmChange;
		if(RTC.CMP <= 0x06) {
			led_state = PR_INTERVAL;
			ledson = NO_LEDS;
		}
	}
}

void fullPulse_pattern(void)
{
	switch(led_state)
	{
		case PR_INTERVAL:
		RTC.CMP += pwmChange;
		if(RTC.CMP >= 0x50) {
			led_state = QRS_UP;
			ledson = INNER_LED;
			RTC.CMP = 0x06;
		}
		break;
		case QRS_UP:
		RTC.CNT = 0x0;
		RTC.CMP += pwmChange * 2;
		if(RTC.CMP >= 0x70) {
			led_state = QRS_DOWN;
			ledson = INNER_LED;
		}
		break;
		case QRS_DOWN:
		RTC.CMP -= pwmChange * 2;
		if(RTC.CMP <= 0x06) {
			led_state = ST_SEGMENT;
			ledson = NO_LEDS;
		}
		break;
		case ST_SEGMENT:
		RTC.CMP += pwmChange;
		if(RTC.CMP >= 0x10) {
			led_state = TWAVE_UP;
			ledson = INNER_LED;
			RTC.CMP = 0x06;
		}
		break;
		case TWAVE_UP:
		RTC.CMP += pwmChange;
		if(RTC.CMP >= 0x16) {
			ledson = ALL_LEDS;
			ledsoff = OUTER_LEDS;
		}
		if(RTC.CMP >= 0x30) {
			led_state = TWAVE_DOWN;
			ledson = ALL_LEDS;
		}
		break;
		case TWAVE_DOWN:
		RTC.CMP -= pwmChange;
		if(RTC.CMP <= 0x20) {
			ledsoff = ALL_LEDS;
		}
		if(RTC.CMP <= 0x06) {
			led_state = PR_INTERVAL;
			ledson = NO_LEDS;
		}
	}
}

void singleBreath_pattern(void)
{
	ledson = INNER_LED;
	ledsoff = ALL_LEDS;
	switch(led_state)
	{
		case QRS_UP:
		RTC.CNT = 0x0;
		RTC.CMP += pwmChange / 2;
		if(RTC.CMP >= 0x70) {
			led_state = QRS_DOWN;

		}
		break;
		case QRS_DOWN:
		RTC.CMP -= pwmChange / 2;
		if(RTC.CMP <= 0x06) {
			led_state = QRS_UP;
		}
		break;
		default :
		led_state = QRS_UP;
	}
}

void fullBreath_pattern(void)
{
	ledson = OUTER_LEDS;
	ledsoff = ALL_LEDS;
	switch(led_state)
	{
		case QRS_UP:
		RTC.CNT = 0x0;
		RTC.CMP += pwmChange / 2;
		if(RTC.CMP >= 0x70) {
			led_state = QRS_DOWN;

		}
		break;
		case QRS_DOWN:
		RTC.CMP -= pwmChange / 2;
		if(RTC.CMP <= 0x06) {
			led_state = QRS_UP;
		}
		break;
		default :
		led_state = QRS_UP;
	}
}

void singleFlash_pattern(void)
{
	RTC.CMP = 0x20;
	if(flashCount++>20) {
		ledsoff = ALL_LEDS;
		ledson = INNER_LED;
	}
	if(flashCount>25) {
		ledsoff = ALL_LEDS;
		ledson = NO_LEDS;
		flashCount = 0;
	}
}

void fullFlash_pattern(void)
{
	RTC.CMP = 0x20;
	if(flashCount++>20) {
		ledsoff = ALL_LEDS;
		ledson = OUTER_LEDS;
	}
	if(flashCount>25) {
		ledsoff = ALL_LEDS;
		ledson = NO_LEDS;
		flashCount = 0;
	}
}

void sparkle_pattern(void)
{
	ledsoff = ALL_LEDS;
	switch(led_state)
	{
		case PR_INTERVAL:
		ledson = RANDOMLEDS[rand() % 0x0C];
		led_state = QRS_UP;
		RTC.CNT = 0x0;
		break;
		
		case QRS_UP:
		RTC.CMP += pwmChange;
		if(RTC.CMP >= 0x50) {
			led_state = QRS_DOWN;

		}
		break;
		
		case QRS_DOWN:
		RTC.CMP -= pwmChange;
		if(RTC.CMP <= 0x06) {
			led_state = PR_INTERVAL;
		}
		break;
		
		default :
		led_state = PR_INTERVAL;
	}
}

void sleepPattern(void)
{
	RTC.CMP = 0x10;
	if(flashCount++>5) {
		ledsoff = ALL_LEDS;
		ledson = LEDS[gCurrentPatternNumber];
	}
	if(flashCount>10) {
		ledsoff = ALL_LEDS;
		ledson = NO_LEDS;
		flashCount = 0;
	}
}

void showPattern(void)
{
	RTC.CMP = 0x10;
	ledsoff = ALL_LEDS;
	ledson = LEDS[gCurrentPatternNumber];
}
