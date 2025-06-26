#ifndef __I2C_SLAVE_H__
#define __I2C_SLAVE_H__

#include <stddef.h>
#include <string.h>

#include "i2c.h"
#include "registers.h"

#define RX_SIZE 6
#define TX_SIZE 32
#define I2C_TIMEOUT_MS 10

typedef enum {
  I2C_STATE_IDLE = 0,
  I2C_STATE_WAIT_REG_ADDR,
  I2C_STATE_WAIT_WRITE_DATA,
  I2C_STATE_WAIT_READ_DATA,
} I2CState_t;

HAL_StatusTypeDef I2C_begin(void);

#endif
