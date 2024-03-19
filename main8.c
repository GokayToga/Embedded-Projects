/*
 * ATmega32_Slave.c
 * http://www.electronicwings.com
 *
 */ 


#define F_CPU 8000000UL		/* Define CPU clock Frequency 8MHz */
#include <avr/io.h>			/* Include AVR std. library file */
#include <util/delay.h>		/* Include inbuilt defined Delay header file */
#include <stdio.h>			/* Include standard I/O header file */
#include <string.h>			/* Include string header file */

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
#define Slave_Address		0x20

#define F_CPU 8000000UL		/* Define CPU clock Frequency 8MHz */
#include <avr/io.h>		/* Include AVR std. library file */
#include <util/delay.h>		/* Include inbuilt defined Delay header file */
#include <stdio.h>		/* Include standard I/O header file */
#include <string.h>		/* Include string header file */

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>



void I2C_Slave_Init(uint8_t slave_address)
{
	TWAR=slave_address;		/* Assign Address in TWI address register */
	TWCR=(1<<TWEN)|(1<<TWEA)|(1<<TWINT);/* Enable TWI, Enable ack generation */
}

int8_t I2C_Slave_Listen()
{
	while(1)
	{
		uint8_t status;			/* Declare variable */
		while(!(TWCR&(1<<TWINT)));	/* Wait to be addressed */
		status=TWSR&0xF8;		/* Read TWI status register */
		if(status==0x60||status==0x68)	/* Own SLA+W received & returned */
		return 0;			/* Return 0 to indicate  returned */
		if(status==0xA8||status==0xB0)	/* Own SLA+R received & returned */
		return 1;			/* Return 0 to indicate  returned */
		if(status==0x70||status==0x78)	/* General call received & returned */
		return 2;			/* Return 1 to indicate  returned */
		else
		continue;			/* Else continue */
	}
}

int8_t I2C_Slave_Transmit(char data)
{
	uint8_t status;
	TWDR=data;			/* Write data to TWDR to be transmitted */
	TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWEA);/* Enable TWI & clear interrupt flag */
	while(!(TWCR&(1<<TWINT)));	/* Wait until TWI finish its current job */
	status=TWSR&0xF8;		/* Read TWI status register */
	if(status==0xA0)		/* Check for STOP/REPEATED START received */
	{
		TWCR|=(1<<TWINT);	/* Clear interrupt flag & return -1 */
		return -1;
	}
	if(status==0xB8)		/* Check for data transmitted & received */
	return 0;			/* If yes then return 0 */
	if(status==0xC0)		/* Check for data transmitted & received */
	{
		TWCR|=(1<<TWINT);	/* Clear interrupt flag & return -2 */
		return -2;
	}
	if(status==0xC8)		/* Last byte transmitted with  received */
	return -3;			/* If yes then return -3 */
	else			/* else return -4 */
	return -4;
}

char I2C_Slave_Receive()
{
	uint8_t status;		/* Declare variable */
	TWCR=(1<<TWEN)|(1<<TWEA)|(1<<TWINT);/* Enable TWI & generation of  */
	while(!(TWCR&(1<<TWINT)));	/* Wait until TWI finish its current job */
	status=TWSR&0xF8;		/* Read TWI status register */
	if(status==0x80||status==0x90)/* Check for data received & returned */
	return TWDR;		/* If yes then return received data */

	/* Check for data received,  returned & switched to not addressed slave mode */
	if(status==0x88||status==0x98)
	return TWDR;		/* If yes then return received data */
	if(status==0xA0)		/* Check  STOP/REPEATED START */
	{
		TWCR|=(1<<TWINT);	/* Clear interrupt flag & return -1 */
		return -1;
	}
	else
	return -2;			/* Else return -2 */
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



int main(void)
{
    char buffer[10];
    int8_t count = 0;
	
    LCD_Init();
    I2C_Slave_Init(Slave_Address);
	
    LCD_String_xy(1, 0, "Slave Device");
	
    while (1)
     {
	switch(I2C_Slave_Listen())				/* Check for SLA+W or SLA+R */
	 {
	    case 0:
		{
		  LCD_String_xy(2, 0, "Receiving :       ");
		  do
		  {
		    sprintf(buffer, "%d    ", count);
		    LCD_String_xy(2, 13, buffer);
		    count = I2C_Slave_Receive();	/* Receive data byte*/
		  } while (count != -1);			/* Receive until STOP/REPEATED START */
		  count = 0;
		  break;
		}
	    case 1:
		{
		  int8_t Ack_status;
		  LCD_String_xy(2, 0, "Sending :       ");
		  do
		  {
		    Ack_status = I2C_Slave_Transmit(count);/* Send data byte */
		    sprintf(buffer, "%d    ",count);
		    LCD_String_xy(2, 13, buffer);
		    count++;
		  } while (Ack_status == 0);				/* Send until Ack is receive */
		  break;
		}
	    default:
		break;
	 }
     }
}