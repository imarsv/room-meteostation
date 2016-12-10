/*
 * htu21.h
 *
 *  Created on: 21 лист. 2016 р.
 *      Author: MarS
 */

#ifndef HTU21_HTU21_H_
#define HTU21_HTU21_H_

#define HTU21D_ADDRESS        (0x80)

#define HTU21DF_READTEMP      (0xE3)
#define HTU21DF_READHUM       (0xE5)
#define HTU21DF_WRITEREG      (0xE6)
#define HTU21DF_READREG       (0xE7)
#define HTU21DF_RESET         (0xFE)

/**
 * Alert
 */
#include <i2c.h>
#define HTU21DF_I2C            hi2c2

uint8_t htu21_status(uint8_t *status);
uint8_t htu21_reset();
uint8_t htu21_temperature(float *temperature);
uint8_t htu21_humidity(float *humidity);

#endif /* HTU21_HTU21_H_ */
