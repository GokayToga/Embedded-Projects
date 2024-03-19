#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/twi.h>

#define SLAVE_ADDRESS 0x50 // Set the slave address

void i2c_init_slave(uint8_t address) {
	TWAR = (address << 1);
	TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
}

int main() {
	// Initialize I2C as slave
	i2c_init_slave(SLAVE_ADDRESS);

	sei(); // Enable global interrupts

	while (1) {
		// Your code to handle received data goes here
	}

	return 0;
}

// I2C receive complete ISR
ISR(TWI_vect) {
	uint8_t received_data;
	printf("Received data: %x\n", received_data); // Print received data


	// Check the status register for the received data
	if ((TWSR & 0xF8) == TW_SR_SLA_ACK) {
		// Slave address received with acknowledgment, ready to receive data
		TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
		} else if ((TWSR & 0xF8) == TW_SR_DATA_ACK) {
		// Data received with acknowledgment
		received_data = TWDR; // Read received data
		// Process received data as needed
		// ...

		// Ready to receive the next byte
		TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
		} else {
		// Unexpected condition, handle error if needed
		// ...
	}
}