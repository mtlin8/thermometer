#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

void timer2_init(void);
void timer1_init(void);
void timer0_init(void);

volatile unsigned char new_state, old_state; // extern
volatile unsigned char tempComp;
volatile unsigned char changed;  // Flag for state change extern
volatile int threshold; // extern
volatile unsigned char a, b; // Rotary encoder variables extern
volatile unsigned char remote; // 0 = local, 1 = remote
volatile int interruptCount;
volatile int localTempOnes; 
volatile int localTempTenths; 
volatile int remoteTempTens;
volatile int remoteTempOnes;
volatile char send[6];
volatile unsigned char receiving;
volatile unsigned char received;
volatile char count;
volatile char buf[6];

