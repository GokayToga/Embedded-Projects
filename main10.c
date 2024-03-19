/*
 * ATmega32_SPI_Slave.c
 * http://www.electronicwings.com
 * https://www.electronicwings.com/avr-atmega/atmega1632-spi
 */ 


#define F_CPU 8000000UL			/* Define CPU Frequency 8MHz */
#include <avr/io.h>			/* Include AVR std. library file */
#include <util/delay.h>			/* Include Delay header file */
#include <stdio.h>			/* Include std. i/p o/p file */
#include <string.h>			/* Include string header file */
//#include "LCD_16x2_H_file.h"		/* Include LCD header file */
//#include "SPI_Slave_H_file.h"		/* Include SPI slave header file */

#include <avr/io.h>
#include <util/delay.h>

#define SS_Enable   (PORTB |= (1<<PB4))
#define MOSI        PB5
#define MISO        PB6
#define SCK         PB7
#define SS          PB4

// Define LCD control pins and data port
#define LCD_RS 0   // Register select pin
#define LCD_RW 1   // Read/write pin (set to write mode)
#define LCD_EN 2   // Enable pin
#define LCD_D4 4   // Data bit 4
#define LCD_D5 5   // Data bit 5
#define LCD_D6 6   // Data bit 6
#define LCD_D7 7   // Data bit 7
#define LCD_PORT PORTB  // Change to the appropriate port
// Function to send a command to the LCD
void LCD_Command(unsigned char cmd) {
	LCD_PORT = (LCD_PORT & 0x0F) | (cmd & 0xF0);  // Send high nibble
	LCD_PORT &= ~(1 << LCD_RS);  // RS = 0 for command
	LCD_PORT |= (1 << LCD_EN);   // Enable pulse
	_delay_us(1);                // Small delay
	LCD_PORT &= ~(1 << LCD_EN);  // Disable pulse
	
	_delay_us(200);  // Delay after command

	LCD_PORT = (LCD_PORT & 0x0F) | ((cmd << 4) & 0xF0);  // Send low nibble
	LCD_PORT |= (1 << LCD_EN);   // Enable pulse
	_delay_us(1);                // Small delay
	LCD_PORT &= ~(1 << LCD_EN);  // Disable pulse

	_delay_ms(2);  // Delay after command
}

// Function to send a data byte to the LCD
void LCD_Data(unsigned char data) {
	LCD_PORT = (LCD_PORT & 0x0F) | (data & 0xF0);  // Send high nibble
	LCD_PORT |= (1 << LCD_RS);   // RS = 1 for data
	LCD_PORT |= (1 << LCD_EN);   // Enable pulse
	_delay_us(1);                // Small delay
	LCD_PORT &= ~(1 << LCD_EN);  // Disable pulse
	
	_delay_us(200);  // Delay after data

	LCD_PORT = (LCD_PORT & 0x0F) | ((data << 4) & 0xF0);  // Send low nibble
	LCD_PORT |= (1 << LCD_RS);   // RS = 1 for data
	LCD_PORT |= (1 << LCD_EN);   // Enable pulse
	_delay_us(1);                // Small delay
	LCD_PORT &= ~(1 << LCD_EN);  // Disable pulse

	_delay_us(100);  // Delay after data
}

// Function to initialize the LCD
void LCD_Init() {
	// Configure LCD control pins and data port as outputs
	DDRB |= (1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_EN) | (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);

	// Initialize LCD in 4-bit mode
	_delay_ms(20);            // Wait for LCD to power up
	LCD_Command(0x02);        // Set 4-bit mode
	LCD_Command(0x28);        // 2-line, 5x8 font size
	LCD_Command(0x0C);        // Display ON, cursor OFF
	LCD_Command(0x06);        // Entry mode: Increment cursor
	LCD_Clear();               // Clear the screen
}

// Function to clear the LCD screen
void LCD_Clear() {
	LCD_Command(0x01);  // Clear display
	_delay_ms(2);       // Delay after clearing
}

// Function to move the cursor to a specific position (0-based)
void LCD_SetCursor(uint8_t row, uint8_t col) {
	uint8_t position = row * 0x40 + col;
	LCD_Command(0x80 | position);  // Set DDRAM address
}

// Function to display a string at the current cursor position
void LCD_DisplayString(const char *str) {
	while (*str) {
		LCD_Data(*str++);
	}
}

void LCD_String_xy(uint8_t row, uint8_t col, const char *str) {
	// Calculate the DDRAM address based on row and column
	uint8_t position = row * 0x40 + col;
	LCD_Command(0x80 | position);  // Set DDRAM address
	
	// Display the string
	while (*str) {
		LCD_Data(*str++);
	}
}

void SPI_Init()					/* SPI Initialize function */
{
	DDRB &= ~((1<<MOSI)|(1<<SCK)|(1<<SS));  /* Make MOSI, SCK, SS as
 						input pins */
	DDRB |= (1<<MISO);			/* Make MISO pin as 
						output pin */
	SPCR = (1<<SPE);			/* Enable SPI in slave mode */
}
char SPI_Receive()			/* SPI Receive data function */
{
	while(!(SPSR & (1<<SPIF)));	/* Wait till reception complete */
	return(SPDR);			/* Return received data */
}

int main(void)
{
	uint8_t count;
	char buffer[5];
	
	LCD_Init();
	SPI_Init();
	
	LCD_String_xy(1, 0, "Slave Device");
	LCD_String_xy(2, 0, "Receive Data:    ");
	while (1)			/* Receive count continuous */
	{
		count = SPI_Receive();
		sprintf(buffer, "%d   ", count);
		LCD_String_xy(2, 13, buffer);
	}

}