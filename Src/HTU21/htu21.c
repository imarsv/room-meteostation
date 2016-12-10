#include "htu21.h"

uint8_t htu21_status(uint8_t *status) {
	uint8_t data[] = { HTU21DF_READREG };

	if (HAL_I2C_Master_Transmit(&HTU21DF_I2C, HTU21D_ADDRESS, data, 1, 200) != HAL_OK) {
		return 1;
	}

	data[0] = 0x00;
	if (HAL_I2C_Master_Receive(&HTU21DF_I2C, HTU21D_ADDRESS, data, 1, 500) != HAL_OK) {
		return 1;
	}

	*status = data[0];

	return 0;
}

uint8_t htu21_reset() {
	uint8_t data[] = { HTU21DF_RESET };

	if (HAL_I2C_Master_Transmit(&HTU21DF_I2C, HTU21D_ADDRESS, data, 1, 200) != HAL_OK) {
		return 1;
	}

	return 0;
}

uint8_t htu21_temperature(float *temperature) {
	uint8_t data[] = { HTU21DF_READTEMP, 0x00 };

	if (HAL_I2C_Master_Transmit(&HTU21DF_I2C, HTU21D_ADDRESS, data, 1, 200) != HAL_OK) {
		return 1;
	}

	data[0] = 0x00;
	data[1] = 0x00;
	if (HAL_I2C_Master_Receive(&HTU21DF_I2C, HTU21D_ADDRESS, data, 2, 500) != HAL_OK) {
		return 1;
	}

	uint16_t t = (((uint16_t) data[0] << 8) | ((uint16_t) data[1])) & 0xFFFC;

//	float temp = t;
//	temp *= 175.72;
//	temp /= 65536;
//	temp -= 46.85;
//
//	*temperature = temp;

	*temperature = ((float) t / 0x10000) * 175.72 - 46.85;

	return 0;
}

uint8_t htu21_humidity(float *humidity) {
	uint8_t data[] = { HTU21DF_READHUM, 0x00 };

	if (HAL_I2C_Master_Transmit(&HTU21DF_I2C, HTU21D_ADDRESS, data, 1, 200) != HAL_OK) {
		return 1;
	}

	data[0] = 0x00;
	data[1] = 0x00;
	if (HAL_I2C_Master_Receive(&HTU21DF_I2C, HTU21D_ADDRESS, data, 2, 500) != HAL_OK) {
		return 1;
	}

	uint16_t h = (((uint16_t) data[0] << 8) | ((uint16_t) data[1])) & 0xFFFC;

//	float hum = h;
//	hum *= 125;
//	hum /= 65536;
//	hum -= 6;
//
//	*humidity = hum;

	*humidity = ((float) h / 0x10000) * 125 - 6;

	return 0;
}

