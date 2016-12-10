/*
 * mh_z19.h
 *
 *  Created on: 22 лист. 2016 р.
 *      Author: MarS
 */

#ifndef MHZ19_MH_Z19_H_
#define MHZ19_MH_Z19_H_

/**
 * Alert
 */
#include <usart.h>
#define MHZ19_UART				huart1

#define PACKET_LENGTH           (9)
#define GAS_CONCENTRAYION		(0x86)

uint8_t mhz19_get(uint16_t *co2_ppm);


#endif /* MHZ19_MH_Z19_H_ */
