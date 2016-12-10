#include "mhz19.h"

uint8_t getCheckSum(uint8_t *data) {
	uint8_t checksum = 0;

	for (size_t i = 1; i < PACKET_LENGTH - 1; i++) {
		checksum += data[i];
	}

	return (0xff - checksum) + 1;
}

uint8_t mhz19_get(uint16_t *co2_ppm) {
	uint8_t command[PACKET_LENGTH] = {};

	command[0] = 0xff;
	command[1] = 0x01;
	command[2] = GAS_CONCENTRAYION;
	command[PACKET_LENGTH - 1] = getCheckSum(command);

	if (HAL_UART_Transmit(&MHZ19_UART, command, PACKET_LENGTH, 500) != HAL_OK) {
		return 1;
	}

	uint8_t data[PACKET_LENGTH] = {};
	if (HAL_UART_Receive(&MHZ19_UART, data, PACKET_LENGTH, 1000) != HAL_OK) {
		return 1;
	}

	if (getCheckSum(data) != data[8]) {
		return 1;
	}

	*co2_ppm = ((uint16_t)((uint8_t) data[2]) * 256) + (uint8_t) data[3];

	return 0;
}
