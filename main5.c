#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define F_CPU 8000000UL // Set your MCU's clock frequency
#define BAUD_RATE 9600  // Set the desired baud rate
#define MY_TWBR ((F_CPU / (BAUD_RATE * 100UL)) - 16) / 2
#define SLAVE_ADDRESS 0x50

void i2c_init() {
	TWSR = 0;
	TWBR = (uint8_t)MY_TWBR;
	TWCR = (1 << TWEN);
}

void i2c_start() {
	TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
}

void i2c_stop() {
	TWCR = (1 << TWSTO) | (1 << TWINT) | (1 << TWEN);
	while (TWCR & (1 << TWSTO));
}

void i2c_write(uint8_t data) {
	printf("Sending data...\n");
	TWDR = data;
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
}

int main() {
	// Initialize I2C
	i2c_init();

	while (1) {
		// Start condition
		i2c_start();

		// Send slave address (replace SLAVE_ADDRESS with the actual address)
		i2c_write(SLAVE_ADDRESS << 1); // << 1 appends the R/W bit (0 for write, 1 for read)

		// Send data to the slave
		i2c_write(0x55); // Example data to send

		// Stop condition
		i2c_stop();

		// Delay before next transmission
		_delay_ms(1000); // Adjust as needed
	}

	return 0;
}