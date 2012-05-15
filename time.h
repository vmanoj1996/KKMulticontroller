/*  time.h
 *  Total ripoff from Arduino libraries (wiring.c)
 */

#ifndef TIME_H
#define TIME_H

#include "config.h"

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
#define microsecondsToClockCycles(a) ( ((a) * (F_CPU / 1000L)) / 1000L )

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile uint64_t timer0_overflow_count = 0;
volatile uint64_t timer0_millis = 0;
static uint8_t timer0_fract = 0;

uint64_t TIME_Millis(void);
void TIME_Setup(void);


#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
SIGNAL(TIM0_OVF_vect)
#else
SIGNAL(TIMER0_OVF_vect)
#endif
{
   // copy these to local variables so they can be stored in registers
   // (volatile variables must be read from memory on every access)
   uint64_t m = timer0_millis;
   uint8_t f = timer0_fract;

   m += MILLIS_INC;
   f += FRACT_INC;
   if (f >= FRACT_MAX) {
      f -= FRACT_MAX;
      m += 1;
   }

   timer0_fract = f;
   timer0_millis = m;
   timer0_overflow_count++;
}

uint64_t TIME_Millis()
{
   uint64_t m;
   uint8_t oldSREG = SREG;

   // disable interrupts while we read timer0_millis or we might get an
   // inconsistent value (e.g. in the middle of a write to timer0_millis)
   cli();
   m = timer0_millis;
   SREG = oldSREG;

   return m;
}

void TIME_Setup()
{
	// this needs to be called before setup() or some functions won't
	// work there
	sei();

	// on the ATmega168, timer 0 is also used for fast hardware pwm
	// (using phase-correct PWM would mean that timer 0 overflowed half as often
	// resulting in different millis() behavior on the ATmega8 and ATmega168)
#if defined(TCCR0A) && defined(WGM01)
	TCCR0A |= _BV(WGM01);
	TCCR0A |= _BV(WGM00);
#endif

	// set timer 0 prescale factor to 64
#if defined(__AVR_ATmega128__)
	// CPU specific: different values for the ATmega128
	TCCR0 |= _BV(CS02);
#elif defined(TCCR0) && defined(CS01) && defined(CS00)
	// this combination is for the standard atmega8
	TCCR0 |= _BV(CS01);
	TCCR0 |= _BV(CS00);
#elif defined(TCCR0B) && defined(CS01) && defined(CS00)
	// this combination is for the standard 168/328/1280/2560
	TCCR0B |= _BV(CS01);
	TCCR0B |= _BV(CS00);
#elif defined(TCCR0A) && defined(CS01) && defined(CS00)
	// this combination is for the __AVR_ATmega645__ series
	TCCR0A |= _BV(CS01);
	TCCR0A |= _BV(CS00);
#else
	#error Timer 0 prescale factor 64 not set correctly
#endif

	// enable timer 0 overflow interrupt
#if defined(TIMSK) && defined(TOIE0)
	TIMSK |= _BV(TOIE0);
#elif defined(TIMSK0) && defined(TOIE0)
	TIMSK0 |= _BV(TOIE0);
#else
	#error	Timer 0 overflow interrupt not set correctly
#endif

	// timers 1 and 2 are used for phase-correct hardware pwm
	// this is better for motors as it ensures an even waveform
	// note, however, that fast pwm mode can achieve a frequency of up
	// 8 MHz (with a 16 MHz clock) at 50% duty cycle

#if defined(TCCR1B) && defined(CS11) && defined(CS10)
	TCCR1B = 0;

	// set timer 1 prescale factor to 64
	TCCR1B |= _BV(CS11);
#if F_CPU >= 8000000L
	TCCR1B |= _BV(CS10);
#endif
#elif defined(TCCR1) && defined(CS11) && defined(CS10)
	TCCR1 |= _BV(CS11);
#if F_CPU >= 8000000L
	TCCR1 |= _BV(CS10);
#endif
#endif
	// put timer 1 in 8-bit phase correct pwm mode
#if defined(TCCR1A) && defined(WGM10)
	TCCR1A |= _BV(WGM10);
#elif defined(TCCR1)
	#warning this needs to be finished
#endif

	// set timer 2 prescale factor to 64
#if defined(TCCR2) && defined(CS22)
	TCCR2 |= _BV(CS22);
#elif defined(TCCR2B) && defined(CS22)
	TCCR2B |= _BV(CS22);
#else
	#warning Timer 2 not finished (may not be present on this CPU)
#endif

	// configure timer 2 for phase correct pwm (8-bit)
#if defined(TCCR2) && defined(WGM20)
	TCCR2 |= _BV(WGM20);
#elif defined(TCCR2A) && defined(WGM20)
	TCCR2A |= _BV(WGM20);
#else
	#warning Timer 2 not finished (may not be present on this CPU)
#endif

#if defined(TCCR3B) && defined(CS31) && defined(WGM30)
	TCCR3B |= _BV(CS31);		// set timer 3 prescale factor to 64
	TCCR3B |= _BV(CS30);
	TCCR3A |= _BV(WGM30);		// put timer 3 in 8-bit phase correct pwm mode
#endif

#if defined(TCCR4B) && defined(CS41) && defined(WGM40)
	TCCR4B |= _BV(CS41);		// set timer 4 prescale factor to 64
	TCCR4B |= _BV(CS40);
	TCCR4A |= _BV(WGM40);		// put timer 4 in 8-bit phase correct pwm mode
#endif

#if defined(TCCR5B) && defined(CS51) && defined(WGM50)
	TCCR5B |= _BV(CS51);		// set timer 5 prescale factor to 64
	TCCR5B |= _BV(CS50);
	TCCR5A |= _BV(WGM50);		// put timer 5 in 8-bit phase correct pwm mode
#endif

#if defined(ADCSRA)
	// set a2d prescale factor to 128
	// 16 MHz / 128 = 125 KHz, inside the desired 50-200 KHz range.
	// XXX: this will not work properly for other clock speeds, and
	// this code should use F_CPU to determine the prescale factor.
	ADCSRA |= _BV(ADPS2);
	ADCSRA |= _BV(ADPS1);
	ADCSRA |= _BV(ADPS0);

	// enable a2d conversions
	ADCSRA |= _BV(ADEN);
#endif

	// the bootloader connects pins 0 and 1 to the USART; disconnect them
	// here so they can be used as normal digital i/o; they will be
	// reconnected in Serial.begin()
#if defined(UCSRB)
	UCSRB = 0;
#elif defined(UCSR0B)
	UCSR0B = 0;
#endif
}

#endif
