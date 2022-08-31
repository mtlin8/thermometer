#include "encoder.h"

volatile unsigned char new_state, old_state;
volatile unsigned char changed = 0;  // Flag for state change
volatile int threshold = 70;
volatile unsigned char a, b; // Rotary encoder variables

ISR(PCINT2_vect) {
	char x = PIND;
	if (x & (1 << 2)) {
		a = 1;
	}
	else a = 0;

	if (x & (1 << 3)) {
		b = 1;
	}
	else b = 0;

	// For each state, examine the two input bits to see if state
	// has changed, and if so set "new_state" to the new state,
	// and adjust the threshold value.
	if (old_state == 0) {

		// Handle A and B inputs for state 0
		if (a) {
			new_state = 1;
			threshold++;
		}
		else if (b) {
			new_state = 2;
			threshold--;
		}
	}
	else if (old_state == 1) { // 01

		// Handle A and B inputs for state 1
		if (!a) {
			new_state = 0;
			threshold--;
		}
		else if (b) {
			new_state = 3;
			threshold++;
		}
	}
	else if (old_state == 2) { // 10

		// Handle A and B inputs for state 2			
		if (a) {
			new_state = 3;
			threshold--;
		}
		else if (!b) {
			new_state = 0;
			threshold++;
		}
	}
	else {   // old_state = 3 // 11

		// Handle A and B inputs for state 3
		if (!a) {
			new_state = 2;
			threshold++;
		}
		else if (!b) {
			new_state = 1;
			threshold--;
		}
	}

	// If state changed, update the value of old_state,
	// and set a flag that the state has changed.
	if (new_state != old_state) {
		changed = 1;
		old_state = new_state;
		if (threshold > 90) {
			threshold = 90;
		}
		else if (threshold < 50) {
			threshold = 50;
		}
	}
}

