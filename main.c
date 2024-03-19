/*
 * main.c
 *
 * Created: 9/19/2023 4:14:33 PM
 *  Author: GRBSVM-STJ04PC
 */ 
#define F_CPU 16000000UL
#define sw (PINB&(1<<0))
#include <avr/interrupt.h>
#include <xc.h>
#include <avr/io.h>
#include <util/delay.h>
unsigned char i;

int main(void)
{
	DDRB &= ~(1<<0);
	PORTB |= (1<<0);
	DDRC = 0xFF;
	i = 64;
	OCR0 = i;
	TCCR0 = 0x61;
	
	DDRB |= (1<<PB3);
	DDRD &= ~(1<<PD2);
	PORTD |= (1<<PD2);
	GICR = (1<<INT0);
	MCUCR = 0X02;
	sei();
	
    while(1)
    {
        //TODO:: Please write your application code
		if(sw==0)
			PORTC = 0x01 ;
		else
			PORTC = 0x02;
			
			OCR0 = i;
    }
	
	return 0;
}

ISR (INT0_vect){
	i = i + 10;
}
ISR(INT2_vect){
	i = i - 10;
}