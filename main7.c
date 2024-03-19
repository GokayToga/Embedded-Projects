/*
 * ATmega16_Master.c
 * http://www.electronicwings.com
 * https://www.electronicwings.com/avr-atmega/atmega1632-i2c
 */ 
/* Define bit rate */
#define SCL_CLK 100000UL  // 100 kHz
#define BITRATE(TWSR)	((F_CPU/SCL_CLK)-16)/(2*pow(4,(TWSR&((1<<TWPS0)|(1<<TWPS1)))))
#define F_CPU 8000000UL		/* Define CPU clock Frequency 8MHz */
#include <avr/io.h>		/* Include AVR std. library file */
#include <util/delay.h>		/* Include inbuilt defined Delay header file */
#include <stdio.h>		/* Include standard I/O header file */
#include <string.h>		/* Include string header file */
//#include <I2C_Master_H_file.h>	/* Include I2C header file */
//#include <LCD_16x2_H_file.h>	/* Include LCD header file */
//#include "lcd.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
//#include "LCD_16x2_H_file.h"

#include <avr/io.h>
#include <util/delay.h>

// Define LCD control pins and data port
#define LCD_RS 0   // Register select pin
#define LCD_RW 1   // Read/write pin (set to write mode)
#define LCD_EN 2   // Enable pin
#define LCD_D4 4   // Data bit 4
#define LCD_D5 5   // Data bit 5
#define LCD_D6 6   // Data bit 6
#define LCD_D7 7   // Data bit 7
#define LCD_PORT PORTB  // Change to the appropriate port


#define Slave_Write_Address		0x20
#define Slave_Read_Address		0x21
#define	count				10

void I2C_Stop()			/* I2C stop function */
{
	TWCR=(1<<TWSTO)|(1<<TWINT)|(1<<TWEN);/* Enable TWI, generate stop */
	while(TWCR&(1<<TWSTO));	/* Wait until stop condition execution */
}
char I2C_Read_Nack()		/* I2C read  function */
{
	TWCR=(1<<TWEN)|(1<<TWINT);	/* Enable TWI and clear interrupt flag */
	while(!(TWCR&(1<<TWINT)));	/* Wait until TWI finish its current job */
	return TWDR;		/* Return received data */
}
char I2C_Read_Ack()		/* I2C read  function */
{
	TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWEA); /* Enable TWI, generation of  */
	while(!(TWCR&(1<<TWINT)));	/* Wait until TWI finish its current job */
	return TWDR;			/* Return received data */
}
uint8_t I2C_Write(char data)	/* I2C write function */
{
	uint8_t status;		/* Declare variable */
	TWDR=data;			/* Copy data in TWI data register */
	TWCR=(1<<TWEN)|(1<<TWINT);	/* Enable TWI and clear interrupt flag */
	while(!(TWCR&(1<<TWINT)));	/* Wait until TWI finish its current job */
	status=TWSR&0xF8;		/* Read TWI status register */
	if(status==0x28)		/* Check for data transmitted & received */
	return 0;			/* Return 0 to indicate  received */
	if(status==0x30)		/* Check for data transmitted & received */
	return 1;			/* Return 1 to indicate  received */
	else
	return 2;			/* Else return 2 for data transmission failure */
}
uint8_t I2C_Repeated_Start(char read_address) /* I2C repeated start function */
{
	uint8_t status;		/* Declare variable */
	TWCR=(1<<TWSTA)|(1<<TWEN)|(1<<TWINT);/* Enable TWI, generate start */
	while(!(TWCR&(1<<TWINT)));	/* Wait until TWI finish its current job */
	status=TWSR&0xF8;		/* Read TWI status register */
	if(status!=0x10)		/* Check for repeated start transmitted */
	return 0;			/* Return 0 for repeated start condition fail */
	TWDR=read_address;		/* Write SLA+R in TWI data register */
	TWCR=(1<<TWEN)|(1<<TWINT);	/* Enable TWI and clear interrupt flag */
	while(!(TWCR&(1<<TWINT)));	/* Wait until TWI finish its current job */
	status=TWSR&0xF8;		/* Read TWI status register */
	if(status==0x40)		/* Check for SLA+R transmitted & received */
	return 1;			/* Return 1 to indicate  received */
	if(status==0x48)		/* Check for SLA+R transmitted & received */
	return 2;			/* Return 2 to indicate  received */
	else
	return 3;			/* Else return 3 to indicate SLA+W failed */
}
uint8_t I2C_Start(char write_address)/* I2C start function */
{
	uint8_t status;		/* Declare variable */
	TWCR=(1<<TWSTA)|(1<<TWEN)|(1<<TWINT); /* Enable TWI, generate START */
	while(!(TWCR&(1<<TWINT)));	/* Wait until TWI finish its current job */
	status=TWSR&0xF8;		/* Read TWI status register */
	if(status!=0x08)		/* Check weather START transmitted or not? */
	return 0;			/* Return 0 to indicate start condition fail */
	TWDR=write_address;		/* Write SLA+W in TWI data register */
	TWCR=(1<<TWEN)|(1<<TWINT);	/* Enable TWI & clear interrupt flag */
	while(!(TWCR&(1<<TWINT)));	/* Wait until TWI finish its current job */
	status=TWSR&0xF8;		/* Read TWI status register */
	if(status==0x18)		/* Check for SLA+W transmitted & received */
	return 1;			/* Return 1 to indicate  received */
	if(status==0x20)		/* Check for SLA+W transmitted & received */
	return 2;			/* Return 2 to indicate  received */
	else
	return 3;			/* Else return 3 to indicate SLA+W failed */
}
void I2C_Init()			/* I2C initialize function */
{
	TWBR = BITRATE(TWSR=0x00);	/* Get bit rate register value by formula */
}


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

int main() {
	char buffer[10];
	
	LCD_Init(); // Initialize the LCD
	I2C_Init(); // Initialize I2C
	
	LCD_String_xy(1, 0, "Master Device");
	
	while (1) {
		LCD_String_xy(2, 0, "Sending :       ");
		I2C_Start(Slave_Write_Address);
		_delay_ms(5);
		for (uint8_t i = 0; i < count ; i++) {
			sprintf(buffer, "%d    ", i);
			LCD_String_xy(2, 13, buffer);
			I2C_Write(i);
			_delay_ms(500);
		}
		LCD_String_xy(2, 0, "Receiving :       ");
		I2C_Repeated_Start(Slave_Read_Address);
		_delay_ms(5);
		for (uint8_t i = 0; i < count; i++) {
			if (i < count - 1)
			sprintf(buffer, "%d    ", I2C_Read_Ack());
			else
			sprintf(buffer, "%d    ", I2C_Read_Nack());
			LCD_String_xy(2, 13, buffer);
			_delay_ms(500);
		}
		I2C_Stop();
	}
}