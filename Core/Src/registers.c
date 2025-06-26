#include "registers.h"

volatile reg_t g_i2c_reg_data[] = {
    [REG_LED]                 = {.addr      = REG_LED_ADDR,
                                 .type      = POINTER_CALLBACK,
                                 .reg_type  = FULL_ACCESS,
                                 .value.ptr = (void *)led_i2c_callback},
    [REG_LED_MODE_MASK]       = {.addr      = REG_LED_MODE_MASK_ADDR,
                                 .type      = POINTER_UINT16,
                                 .reg_type  = FULL_ACCESS,
                                 .value.ptr = (uint16_t *)&led_mode_mask},
    [REG_READ_THRESHOLD_ONE]  = {.addr      = REG_READ_THRESHOLD_ONE_ADDR,
                                 .type      = POINTER_CALLBACK,
                                 .reg_type  = READ_ONLY,
                                 .value.ptr = (void *)threshold_i2c_callback},
    [REG_WRITE_THRESHOLD_ONE] = {.addr      = REG_WRITE_THRESHOLD_ONE_ADDR,
                                 .type      = POINTER_CALLBACK,
                                 .reg_type  = WRITE_ONLY,
                                 .value.ptr = (void *)threshold_i2c_callback},
    [REG_THRESHOLD_ALL]       = {.addr      = REG_THRESHOLD_ALL_ADDR,
                                 .type      = POINTER_CALLBACK,
                                 .reg_type  = FULL_ACCESS,
                                 .value.ptr = (void *)threshold_i2c_callback},
    [REG_THRESHOLD_FLAG]      = {.addr      = REG_THRESHOLD_FLAG_ADDR,
                                 .type      = POINTER_UINT16,
                                 .reg_type  = FULL_ACCESS,
                                 .value.ptr = (uint16_t *)&threshold_flag},

    [REG_ADC_DATA]   = {.addr      = REG_ADC_DATA_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)mux_data,
                        .ptr_size  = sizeof(mux_data) / sizeof(mux_data[0])},
    [REG_ADC_DATA00] = {.addr      = REG_ADC_DATA00_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[0]},
    [REG_ADC_DATA01] = {.addr      = REG_ADC_DATA01_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[1]},
    [REG_ADC_DATA02] = {.addr      = REG_ADC_DATA02_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[2]},
    [REG_ADC_DATA03] = {.addr      = REG_ADC_DATA03_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[3]},
    [REG_ADC_DATA04] = {.addr      = REG_ADC_DATA04_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[4]},
    [REG_ADC_DATA05] = {.addr      = REG_ADC_DATA05_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[5]},
    [REG_ADC_DATA06] = {.addr      = REG_ADC_DATA06_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[6]},
    [REG_ADC_DATA07] = {.addr      = REG_ADC_DATA07_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[7]},
    [REG_ADC_DATA08] = {.addr      = REG_ADC_DATA08_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[8]},
    [REG_ADC_DATA09] = {.addr      = REG_ADC_DATA09_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[9]},
    [REG_ADC_DATA10] = {.addr      = REG_ADC_DATA10_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[10]},
    [REG_ADC_DATA11] = {.addr      = REG_ADC_DATA11_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[11]},
    [REG_ADC_DATA12] = {.addr      = REG_ADC_DATA12_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[12]},
    [REG_ADC_DATA13] = {.addr      = REG_ADC_DATA13_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[13]},
    [REG_ADC_DATA14] = {.addr      = REG_ADC_DATA14_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[14]},
    [REG_ADC_DATA15] = {.addr      = REG_ADC_DATA15_ADDR,
                        .type      = POINTER_UINT16,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (uint16_t *)&mux_data[15]},

    [REG_VERSION]    = {.addr = REG_VERSION_ADDR, .type = UINT16, .reg_type = READ_ONLY, .value.u16 = VERSION},
    [REG_LAST_ERROR] = {.addr      = REG_LAST_ERROR_ADDR,
                        .type      = POINTER_CALLBACK,
                        .reg_type  = READ_ONLY,
                        .value.ptr = (void *)last_error_i2c_callback}};

const uint8_t g_i2c_reg_data_size = sizeof(g_i2c_reg_data) / sizeof(reg_t);

// handler REG_LED
int led_i2c_callback(reg_t *reg, RegisterType_t mode, uint8_t *buffer, uint8_t *length) {
  if (mode == WRITE_ONLY) {
    if (buffer == NULL) *length = 2;
    else {
      uint16_t value = buffer[0] | (buffer[1] << 8);
      led_mode_mask |= value;
      custom_leds_data = value;
    }
  } else {
    *length   = 2;
    buffer[0] = leds_data & 0xFF;
    buffer[1] = leds_data >> 8;
  }

  return 0;
}

// handler REG_WRITE_THRESHOLD_ONE, REG_READ_THRESHOLD_ONE, REG_THRESHOLD_ALL
int threshold_i2c_callback(reg_t *reg, RegisterType_t mode, uint8_t *buffer, uint8_t *length) {
  switch (reg->addr) {
    case REG_WRITE_THRESHOLD_ONE_ADDR: {
      if (mode == WRITE_ONLY) {
        if (buffer == NULL) *length = 3;
        else {
          uint8_t  index = buffer[0];
          uint16_t value = buffer[1] | (buffer[2] << 8);

          if (value > 0xFFF) value = 0xFFF;
          if (index < 16) thresholds[index] = value;
        }
      }
    } break;

    case REG_READ_THRESHOLD_ONE_ADDR: {
      if (mode == READ_ONLY) {
        *length        = 3;
        uint8_t  index = buffer[0];
        uint16_t value = thresholds[index];
        buffer[1]      = value & 0xFF;
        buffer[2]      = value >> 8;
      }
    } break;

    case REG_THRESHOLD_ALL_ADDR: {
      if (mode == WRITE_ONLY) {
        if (buffer == NULL) *length = 32;
        else {
          for (int i = 0; i < 16; i++) {
            uint16_t value = buffer[2 * i] | (buffer[2 * i + 1] << 8);
            if (value > 0xFFF) value = 0xFFF;
            thresholds[i] = value;
          }
        }
      } else {
        *length = 32;
        for (int i = 0; i < 16; i++) {
          uint16_t value    = thresholds[i];
          buffer[2 * i]     = value & 0xFF;
          buffer[2 * i + 1] = value >> 8;
        }
      }
    } break;
  }

  return 0;
}

int last_error_i2c_callback(reg_t *reg, RegisterType_t mode, uint8_t *buffer, uint8_t *length) {
  if (mode == READ_ONLY) {
    *length    = 1;
    buffer[0]  = last_error;
    last_error = 0;
  }
  return 0;
}

void register_init() {
  for (int i = 0; i < g_i2c_reg_data_size; i++) {
    volatile reg_t *reg = &g_i2c_reg_data[i];

    switch (reg->type) {
      case UINT8:
        reg->_size = 1;
        break;
      case POINTER_UINT8:
        reg->_size = 1 * (reg->ptr_size ? reg->ptr_size : 1);
        break;

      case UINT16:
        reg->_size = 2;
        break;
      case POINTER_UINT16:
        reg->_size = 2 * (reg->ptr_size ? reg->ptr_size : 1);
        break;
      default:
        reg->_size = 0;
        break;
    }
  }
}

volatile reg_t *get_register_from_addr(uint8_t addr) {
  for (int i = 0; i < g_i2c_reg_data_size; i++) {
    volatile reg_t *reg = &g_i2c_reg_data[i];
    if (reg->addr == addr) {
      return reg;
    }
  }
  return NULL;
}

volatile reg_t *get_register_from_idx(reg_idx_t idx) {
  if (idx < g_i2c_reg_data_size) {
    return &g_i2c_reg_data[idx];
  }
  return NULL;
}
