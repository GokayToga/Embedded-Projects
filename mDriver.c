#include <avr/io.h>
#include <util/delay.h>

#define IN1_PORT PORTD  // Change to the appropriate port
#define IN2_PORT PORTD  // Change to the appropriate port
#define IN3_PORT PORTD  // Change to the appropriate port
#define IN4_PORT PORTD  // Change to the appropriate port
#define ENA_PORT PORTB  // Change to the appropriate port
#define ENB_PORT PORTB  // Change to the appropriate port

#define IN1_PIN PD2     // Change to the appropriate pin
#define IN2_PIN PD3     // Change to the appropriate pin
#define IN3_PIN PD4     // Change to the appropriate pin
#define IN4_PIN PD5     // Change to the appropriate pin
#define ENA_PIN PB1     // Change to the appropriate pin
#define ENB_PIN PB2     // Change to the appropriate pin

#define BUTTON_FORWARD_PIN PD6  // Change to the appropriate pin
#define BUTTON_REVERSE_PIN PD7  // Change to the appropriate pin
#define BUTTON_STOP_PIN PB0     // Change to the appropriate pin

void initializePorts() {
    // Configure motor control pins as outputs
    DDRD |= (1 << IN1_PIN) | (1 << IN2_PIN) | (1 << IN3_PIN) | (1 << IN4_PIN);
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
            _delay_ms(100);  // Debounce delay
        }
        else if (!(PIND & (1 << BUTTON_REVERSE_PIN))) {
            motorReverse();
            _delay_ms(100);  // Debounce delay
        }
        else if (!(PINB & (1 << BUTTON_STOP_PIN))) {
            motorStop();
            _delay_ms(100);  // Debounce delay
        }
    }

    return 0;
}