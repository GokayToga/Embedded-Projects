#include <avr/io.h>
#include <util/delay.h>
#define F_CPU 16000000UL

#define ENA_PORT PORTB
#define ENB_PORT PORTB

#define IN1_PORT PORTC
#define IN2_PORT PORTC
#define IN3_PORT PORTC
#define IN4_PORT PORTC

#define IN1_PIN PC0
#define IN2_PIN PC1
#define IN3_PIN PC2
#define IN4_PIN PC3

#define ENA_PIN PB1
#define ENB_PIN PB2

#define BUTTON_FORWARD_PIN PD6
#define BUTTON_REVERSE_PIN PD7
#define BUTTON_STOP_PIN PB0

void initializePorts() {
	// Configure motor control pins on Port C as outputs
	DDRC |= (1 << IN1_PIN) | (1 << IN2_PIN) | (1 << IN3_PIN) | (1 << IN4_PIN);
	
	// Configure motor enable pins on Port B as outputs
	DDRB |= (1 << ENA_PIN) | (1 << ENB_PIN);
	
	
	// Configure button pins as inputs with pull-up resistors
	DDRD &= ~((1 << BUTTON_FORWARD_PIN) | (1 << BUTTON_REVERSE_PIN));
	PORTD |= (1 << BUTTON_FORWARD_PIN) | (1 << BUTTON_REVERSE_PIN);
	DDRB &= ~(1 << BUTTON_STOP_PIN);
	PORTB |= (1 << BUTTON_STOP_PIN);
}

void motorForward() {
	IN1_PORT |= (1 << IN1_PIN);
	IN2_PORT &= ~(1 << IN2_PIN);
	IN3_PORT &= ~(1 << IN3_PIN);
	IN4_PORT |= (1 << IN4_PIN);
	ENA_PORT |= (1 << ENA_PIN);
	ENB_PORT |= (1 << ENB_PIN);
}

void motorReverse() {
	IN1_PORT &= ~(1 << IN1_PIN);
	IN2_PORT |= (1 << IN2_PIN);
	IN3_PORT |= (1 << IN3_PIN);
	IN4_PORT &= ~(1 << IN4_PIN);
	ENA_PORT |= (1 << ENA_PIN);
	ENB_PORT |= (1 << ENB_PIN);
}

void motorStop() {
	ENA_PORT &= ~(1 << ENA_PIN);
	ENB_PORT &= ~(1 << ENB_PIN);
}

int main() {
	initializePorts();

	while (1) {
		if (!(PIND & (1 << BUTTON_FORWARD_PIN))) {
			motorForward();
			_delay_ms(100);  // delay
			} else if (!(PIND & (1 << BUTTON_REVERSE_PIN))) {
			motorReverse();
			_delay_ms(100);  // delay
			} else if (!(PINB & (1 << BUTTON_STOP_PIN))) {
			motorStop();
			_delay_ms(100);  // delay
		}
	}

	return 0;
}