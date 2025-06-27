#include "status.h"

LED_Status_t _led_status = {0};

void set_status_all(LEDStatusMode_t mode, uint16_t on_ms, uint16_t off_ms,
                    uint8_t count) {
  if (_led_status.on_ms == on_ms && _led_status.off_ms == off_ms &&
      _led_status.mode == mode) {
    _led_status.count = count;
    return;
  }

  _led_status.on_ms = on_ms;
  _led_status.off_ms = off_ms;
  _led_status.count = count;
  _led_status.status = 1;
  _led_status.mode = mode;
  _led_status.last_tick = HAL_GetTick();
  STATUS_WRITE(GPIO_PIN_SET);
}

void set_status(uint16_t on_ms, uint16_t off_ms) {
  set_status_all(LED_STATUS_CUSTOM, on_ms, off_ms, 0);
}

void set_status_blink(uint16_t on_ms, uint16_t off_ms, uint8_t count) {
  set_status_all(LED_STATUS_BLINK, on_ms, off_ms, count);
}

__weak void set_default_status() {
  set_status(50, 500);  // don't set to 0
}

void update_status() {
  if (_led_status.on_ms == 0 && _led_status.off_ms == 0) {
    set_default_status();
    return;
  }

  if (_led_status.on_ms == 0) {
    STATUS_WRITE(GPIO_PIN_SET);
    return;
  }

  if (_led_status.off_ms == 0) {
    STATUS_WRITE(GPIO_PIN_RESET);
    return;
  }

  uint32_t now = HAL_GetTick();
  uint16_t interval =
      _led_status.status ? _led_status.on_ms : _led_status.off_ms;

  if (_led_status.mode == LED_STATUS_BLINK && _led_status.count == 0) {
    set_default_status();
    return;
  }

  if (now - _led_status.last_tick >= interval) {
    _led_status.status ^= 1;
    _led_status.last_tick = now;

    STATUS_WRITE(_led_status.status);

    if (_led_status.mode == LED_STATUS_BLINK && !_led_status.status &&
        _led_status.count > 0) {
      _led_status.count--;

      if (_led_status.count == 0) {
        STATUS_WRITE(GPIO_PIN_RESET);
        set_default_status();
      }
    }
  }
}
