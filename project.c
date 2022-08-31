/********************************************
 *
 *  Name: Mitchell Lin
 *  Email: mtlin@usc.edu
 *  Section: Wed 2pm
 *  Assignment: EE 109 Final Project
 *
 ********************************************/

#include "project.h"
#include "lcd.h"
#include "ds18b20.h"         
#include "encoder.h"
#include "serial.h"

#define FOSC 16000000 // Clock frequency
#define BAUD 9600 // Baud rate used
#define MYUBRR (FOSC/16/BAUD-1) // Value for UBRR0

/* QUESTIONS */
// Rx/Tx DDR and PORT manipulation?
// 

int main(void) {
    // Initialize...
	// DDR bits
	DDRD &= ~((1 << 2) | (1 << 3)); // Rotary encoder set as inputs
	DDRB |= (1 << 3) | (1 << 4) | (1 << 5); // LED + servo output
	DDRC |= (1 << 5); // Buzzer output
	DDRD |= (1 << 4); // Buffer output
	DDRC &= ~((1 << 1) | (1 << 2)); // Buttons input
	DDRC |= (1 << 4);

	// PORT bits (pull-up resistors)
	PORTD |= (1 << PD2) | (1 << PD3); // Rotary encoder
	PORTD &= ~(1 << PD4); // Buffer
	PORTB &= ~((1 << PB4) | (1 << PB5)); // LEDs
	PORTC |= (1 << PC1) | (1 << PC2); // Buttons
	PORTD |= (1 << PD0) | (1 << PD1); // Serial PORT

	// Serial initialization
	UBRR0 = MYUBRR;
	UCSR0B |= (1 << TXEN0 | 1 << RXEN0 | 1 << RXCIE0); // Enable RX and TX
	UCSR0C = (3 << UCSZ00); // Async., no parity,
							// 1 stop bit, 8 data bits


	unsigned char t[2]; // Array that temperature is read into

	lcd_init(); // LCD

	if (ds_init() == 0) {    // Initialize the DS18B20 thermometer
		lcd_moveto(0,0);
        lcd_stringout("Sensor missing");
		while(1) {}
    }

	// TIMERS
	timer0_init();
	timer1_init();
	timer2_init();
	char oldComp = 0;

	// Local interrupts
	PCICR |= (1 << PCIE2);
	PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
	PCICR |= (1 << PCIE1);
	PCMSK1 |= (1 << PCINT9) | (1 << PCINT10);

    // Write a spash screen to the LCD
    lcd_writecommand(1);
    lcd_moveto(0,0);
    lcd_stringout("Mitchell Lin");
    lcd_moveto(1, 0);
    lcd_stringout("EE109 Project");
    _delay_ms(800);
	_delay_ms(800);
	lcd_writecommand(1);

    // Read the A and B inputs to determine the intial state.
    // In the state number, B is the MSB and A is the LSB.
    // Warning: Do NOT read A and B separately.  You should read BOTH inputs
    // at the same time, then determine the A and B values from that value.    
	
    
	if (!b && !a) old_state = 0;
    else if (!b && a) old_state = 1;
    else if (b && !a) old_state = 2;
    else old_state = 3;
    new_state = old_state;

	// Check if EEPROM is valid (read)
	threshold = eeprom_read_byte((void *) 100);
	if (threshold > 90 || threshold < 50){
		threshold = 70;
	}

	sei(); // enable interrupts
	ds_convert(); // begin first conversion
	interruptCount = 0;
	int tempComp = 0;

	receiving = 0;
	received = 0;
	remote = 0;

    while (1) {		// Loop forever

		// Declare variables that isr can modify

		// Print if local or remote.
		lcd_moveto(1, 10);
		if (!remote) lcd_stringout("Local ");
		else if (remote) lcd_stringout("Remote");

		if (ds_temp(t)) {    // True if conversion complete
            /*
              Process the values returned in t[0]
              and t[1] to find the temperature.
            */

			/* Convert t[] values to Celsius then to Fahrenheit */
			localTempOnes = t[1];
			localTempOnes = localTempOnes << 8;
			localTempOnes |= t[0];
			localTempOnes *= 9;
			localTempOnes /= 8;
			localTempOnes += 320;
			localTempTenths = localTempOnes % 10;
			localTempOnes /= 10;
			
			int ocr = localTempOnes;
			ocr *= 23;
			ocr /= -60;
			ocr += 50;
			OCR2A = ocr;

			/* Transmit using serial */
			send[0] = '@';
			send[1] = '+';
			send[2] = (localTempOnes / 10) + 48;
			send[3] = (localTempOnes % 10) + 48;
			send[4] = '#';
			for (int i = 0; i < 5; i++) {
				while ((UCSR0A & (1 << UDRE0)) == 0) { }
				UDR0 = send[i];
			}

			// Print temperature
			char str[5];
			lcd_moveto(0,0);
			lcd_stringout("    ");
			if (!remote) {
				lcd_moveto(0,0);
				snprintf(str, 5, "%02d.%d", localTempOnes, localTempTenths);
				lcd_stringout(str);
			}
			else if (remote && received) {
				lcd_moveto(0,0);
				remoteTempTens = buf[2];
				remoteTempTens = buf[3];
				snprintf(str, 5, "%d%d", remoteTempTens, remoteTempOnes);
				received = 0;
				lcd_stringout(str);
				lcd_moveto(0,2);
				lcd_stringout("  ");
			}

            ds_convert();   // Start next conversion
        }

		if (changed) { 	// Did state change?
			changed = 0;   // Reset changed flag
			lcd_moveto(1,0);
			lcd_stringout("          ");
			if (!remote) {
				if (localTempOnes < threshold) {
					tempComp = 0;
				}
				else if ((localTempOnes == threshold) && (localTempTenths != 0)) {
					tempComp = 1;
				}
				else if (localTempOnes - threshold < 3) {
					tempComp = 1;
				}
				else if (localTempOnes - threshold > 3) {
					tempComp = 2;
				}
			}
			else if (remote) {
				if (remoteTempTens < threshold) {
					tempComp = 0;
				}
				else if ((remoteTempTens == threshold) && (remoteTempOnes != 0)) {
					tempComp = 1;
				}
				else if (remoteTempTens - threshold < 3) {
					tempComp = 1;
				}
				else if (remoteTempTens - threshold > 3) {
					tempComp = 2;
				}
			}

			eeprom_update_byte((void *) 100, threshold);

			lcd_moveto(0,14);
			char str2[3];
			snprintf(str2, 3, "%d", threshold);
			lcd_stringout(str2);

			lcd_moveto(1, 0);
			if (tempComp == 0) {
				oldComp = 0;
				lcd_stringout("COOL");
				// Green LED on
				PORTB |= (1 << PB5);
				// timers off
				TCCR1B &= ~(0b111 << CS10);
				TCCR0B &= ~(0b111 << CS00);
				PORTB &= ~(1 << PB4);
			}
			else if (tempComp == 1) {
				oldComp = 1;
				lcd_stringout("WARM");
				// TIMER0 off
				TCCR0B &= ~(0b111 << CS00);
				// Green LED off
				PORTB &= ~(1 << PB5);
				// Red LED blinking using TIMER1 (1 Hz)
				// How do we send this to the timer? By setting the prescalar.
				PORTB |= (1 << PB4);
				TCCR1B |= (0b100 << CS10);
			}
			else if (tempComp == 2) {
				lcd_stringout("HOT");
				// Red LED on
				PORTB &= ~(1 << PB5);
				PORTB |= (1 << PB4);
				// Unset timer here.
				TCCR1B &= ~(0b111 << CS10);
				// User TIMER0 to generate tone on buzzer
				if (oldComp != 2) {
					TCCR0B |= (0b100 << CS00); // Prescalar = 256
				}
				oldComp = 2;
			}
		}
	}
}

/* 8-bit Timer 0 */ 
ISR(TIMER0_COMPA_vect) { 
	PORTC ^= (1 << PC5);
	interruptCount++;
	if (interruptCount == 800) {
		TCCR0B &= ~(0b111 << CS00); // Stop timer
		interruptCount = 0;
	}
} 

/* 16-bit Timer 1 */ 
ISR(TIMER1_COMPA_vect) {
	PORTB ^= (1 << PB4);
}

/* 8-bit Timer 2 */
ISR(TIMER2_COMPA_vect) {}

/* Pin Change Interrupt to detect button press */
ISR(PCINT1_vect) {
	char x = PINC;
	if ((x & (1 << 1)) == 0) {
		// Local (BLUE)
		remote = 0;
		// changed = 1;
	}
	else if ((x & (1 << 2)) == 0) {
		// Remote (YELLOW)
		remote = 1;    
		// changed = 1; 
	}
}

/* TIMER INITIALIZATION FUNCTIONS */
void timer2_init(void)
{
    TCCR2A |= (0b11 << WGM00);  // Fast PWM mode, modulus = 256
    TCCR2A |= (0b10 << COM0A0); // Turn D11 on at 0x00 and off at OCR2A
	TIMSK2 |= (1 << OCIE2A);
    OCR2A = 25;                // Initial pulse duty cycle of 50%
    TCCR2B |= (0b111 << CS20);  // Prescaler = 1024 for 16ms period
}
void timer1_init(void)
{
	// // Set the Waveform Generation Mode bits (WGM13 - WGM10) to 1111
	TCCR1A |= (0b11 << WGM10);
	TCCR1B |= (0b11 << WGM12);
	
	TCCR1A |= (0b10 << COM1B0); // make the OC1B output line serve as the pulse output
	TIMSK1 |= (1 << OCIE1A);

    OCR1A = 31250; // Initial pulse duty cycle of 50% as per above

	/* Set the Clock Source bits (CS12, CS11, CS10) to 
	the correct values for the prescaler you selected above*/
    TCCR1B |= (0b100 << CS10);  // Prescaler = 256
}
void timer0_init(void) {
	TCCR0A |= (0b10 << WGM00);
	TIMSK0 |= (1 << OCIE0A);
	OCR0A = 78;
	TCCR0B |= (0b100 << CS00); // Prescalar = 256
}
