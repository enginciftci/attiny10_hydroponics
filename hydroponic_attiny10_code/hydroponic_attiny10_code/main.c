/*
 * main.c
 *
 * Created: 02.06.2021 13:14:15
 * Author : Engin CIFTCI
 * Description: Timer code for hydroponic electronic
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdint.h>
//#define PRG_DEBUG // Enable for debugging

// Wake up by WDT interrupt.
// Don't need to do anything here but (auto-) clearing the interrupt flag.
EMPTY_INTERRUPT(WDT_vect)

EMPTY_INTERRUPT(TIM0_OVF_vect)

EMPTY_INTERRUPT(ADC_vect)

/*
  Delay in powerdown mode. Wake up by watchdog interrupt.
 */
void delay_power_down_wdt(uint8_t wdto)
{
    wdt_reset();
    wdt_enable(wdto);
    WDTCSR |= (1<<WDIE);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    // Make sure interrupts are enabled and the I flag is restored
    NONATOMIC_BLOCK(NONATOMIC_RESTORESTATE)
    {
		sleep_cpu();
		wdt_disable();
    }
    sleep_disable();
}

void drive_motor(uint8_t tm)
{
    // Set port
	PORTB |= 1<<PINB2; // SET HIGH
	for (uint8_t i=1; i <= tm; i++)
	{
		_delay_ms(100);
	}
	PORTB &= ~(1<<PINB2); // SET LOW
}

uint16_t light_sensor(void)
{
	uint16_t adc4;
	uint8_t i;
	// Take four ADC samples, add them in adc4
	for (i = 0, adc4 = 0; i < 4; i++)
	{
		// Start a conversion
		ADCSRA |= (1<<ADSC);
		// wait until it's finished
		while (ADCSRA & (1<<ADSC))
		; // Nothing
		adc4 += ADCL;
	}
	adc4 = adc4 >> 2; // divide by 4
	return adc4;
}

void beep(uint16_t tm, uint16_t tone)
{
	OCR0A = tone;
	TCCR0A = 1<<COM0B0 | 0<<WGM00; // Toggle OC0B, CTC mode
	TCCR0B = 1<<WGM02 | 3<<CS00;   // CTC mode, use OCR0A; /64
	TCNT0 = 0; // Clear timer
	for (uint16_t i=1; i <= tm; i++)
	{
		_delay_ms(1);
	}
    TCCR0B &= ~(1<<CS02 | 1<<CS01 | 1<<CS00 );  // Disable
	TCCR0A = 0; // clear
	PORTB &= ~(1<<PINB1); // SET LOW
}

int main(void)
{
	const uint8_t offset= 11;
	const uint16_t toneL = 208; // Low Freq
	const uint16_t toneM = 142; // Mid Freq
	const uint16_t toneH = 78;  // High Freq
	#ifdef PRG_DEBUG
		const uint8_t motor_drive_time = 20;
	#else
		const uint8_t motor_drive_time = 100;
	#endif // DEBUG

	uint8_t count = 75 ; // clear
	uint8_t hour_counter = 18 ; // clear
	uint8_t repeat_flood = 0; // clear
	uint8_t day = 0; // clear (1 day, 0 night)

	// Clock Setting
	CCP    = 0xD8; // Protected I/O register
	CLKMSR = 0x00; // Calibrated Internal 8 MHzOscillator
	CCP    = 0xD8; // Protected I/O register
	CLKPSR = 0X00; // Clock Prescale Register
	
	// Motor Control
    DDRB = 1<<PINB2; // Set as output PB2
	//PORTB |= 1<<PINB2; // SET HIGH
	PORTB &= ~(1<<PINB2); // SET LOW
	
	// Light Sensor
	ADMUX = 0; // ADC channel 0
	DIDR0 = (1<<ADC0D); // Disable digital input on PB0
	ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0); // Enable ADC, presc 1:8 for 125kHz ADC-clock
	uint16_t adc4;
	
	// Beeper Output
	DDRB |=1<<PINB1; // Set as output PB1

	sei();
	
    while(1)
    {
		
		if (count >= 75) // 10 minutes
		{ 
            beep(70,toneH);
			adc4 = light_sensor(); // Measure Light sensor
			
			// NIGHT
			if (adc4 < 5) 
			{
				if (hour_counter >=  18) // every 3 hour
				{
					hour_counter = 1; //clear
					repeat_flood = 2; // repeat
					drive_motor(motor_drive_time); // 100 x100ms = 10sec power on Motor
					beep(200,toneM);
					beep(200,toneL);
					day = 0; // night
				}
				else
				{
					hour_counter ++;
				}
			}
			// DAY
			else
			{
				if (hour_counter >= 6) // every 1 hour
				{
					hour_counter = 1; //clear
					repeat_flood = 2; // repeat
					drive_motor(motor_drive_time); // 100 x100ms = 10sec power on Motor
					beep(200,toneL);
					beep(200,toneM);
					day = 1; // day
				}
				else
				{
					hour_counter++;
				}
			}
			count = offset; // clear (offset)
		}/*
		else if ((count % 7) == 0 ) // 1 minutes
		{
			beep(70,toneC);
			count+=1;
		}*/
		else if ((count % 7) & (repeat_flood > 0 ))
		{
			repeat_flood--;
			drive_motor(motor_drive_time); // 100 x100ms = 10sec power on Motor
			if (day > 0)
			{   
				// DAY
				beep(200,toneL);
				beep(200,toneM);
			}
			else
			{
				// NIGHT
				beep(200,toneM);
				beep(200,toneL);
			}
			count++;
		}
		else
		{
			count++;
		}
		delay_power_down_wdt(WDTO_8S);
		//delay_power_down_wdt(WDTO_15MS); // DEBUG !
		//delay_power_down_wdt(WDTO_30MS); // DEBUG !
    }
}