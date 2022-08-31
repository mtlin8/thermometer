#include "project.h"

volatile extern unsigned char new_state, old_state;
volatile extern unsigned char changed;  // Flag for state change
volatile extern int threshold;
volatile extern unsigned char a, b; // Rotary encoder variables