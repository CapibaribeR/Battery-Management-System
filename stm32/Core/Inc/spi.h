/*
 * spi.h
 *
 *  Created on: Dec 4, 2025
 *      Author: vtaua
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#ifdef __cplusplus
extern "C" {

#endif

#include "main.h"

extern SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void);

#ifdef __cplusplus
}
#endif


#endif /* INC_SPI_H_ */
