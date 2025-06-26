#ifndef __STATUS_H__
#define __STATUS_H__

#include <stdint.h>

#include "main.h"

#define STATUS_MAX_TIME (0xffff)
#define STATUS_WRITE(val) HAL_GPIO_WritePin(STATUS_GPIO_Port, STATUS_Pin, val)

typedef enum {
  LED_STATUS_CUSTOM,
  LED_STATUS_BLINK,
} LEDStatusMode_t;

typedef struct {
  uint16_t on_ms;
  uint16_t off_ms;

  uint8_t status;
  uint32_t last_tick;

  uint8_t count;  // blink count
  LEDStatusMode_t mode;
} LED_Status_t;

extern LED_Status_t _led_status;

void set_status_all(LEDStatusMode_t mode, uint16_t on_ms, uint16_t off_ms,
                    uint8_t count);
void set_status(uint16_t on_ms, uint16_t off_ms);
void set_status_blink(uint16_t on_ms, uint16_t off_ms, uint8_t count);
void set_default_status(void);

void update_status(void);

#endif
