#ifndef SCCB_H
#define SCCB_H

#include "main.h"

void sccb_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data);
uint8_t sccb_read_reg(uint8_t dev_addr, uint8_t reg);

#endif
