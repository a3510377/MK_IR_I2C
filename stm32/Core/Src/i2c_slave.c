#include "i2c_slave.h"

static volatile reg_t *i2c_current_reg = NULL;
static volatile uint8_t i2c_rx_length = 0, i2c_current_reg_addr = 0;
static volatile uint8_t i2c_rx_buffer[RX_SIZE] = {0};
static volatile I2CState_t i2c_state = I2C_STATE_IDLE;

HAL_StatusTypeDef I2C_begin() {
  i2c_state = I2C_STATE_IDLE;
  i2c_rx_length = 0;
  i2c_current_reg = NULL;
  i2c_current_reg_addr = 0;

  register_init();

  return HAL_I2C_EnableListen_IT(&hi2c1);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (i2c_state == I2C_STATE_WAIT_WRITE_DATA && i2c_current_reg &&
      i2c_current_reg->type == POINTER_CALLBACK) {
    CustomI2CReadWriteCallback_t callback = i2c_current_reg->value.ptr;
    callback((reg_t *)i2c_current_reg, WRITE_ONLY, (uint8_t *)i2c_rx_buffer,
             (uint8_t *)&i2c_rx_length);
  }

  i2c_state = I2C_STATE_IDLE;
  i2c_current_reg = NULL;
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
                          uint16_t AddrMatchCode) {
  UNUSED(AddrMatchCode);

  if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
    i2c_state = I2C_STATE_WAIT_REG_ADDR;
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, (uint8_t *)&i2c_current_reg_addr, 1,
                                 I2C_NEXT_FRAME);
    return;
  }

  // echo
  if (i2c_current_reg_addr == 0) return;

  i2c_current_reg = get_register_from_addr(i2c_current_reg_addr);
  if (!i2c_current_reg || i2c_current_reg->reg_type == WRITE_ONLY) return;

  i2c_state = I2C_STATE_WAIT_READ_DATA;
  if (i2c_current_reg->type == POINTER_CALLBACK) {
    CustomI2CReadWriteCallback_t callback = i2c_current_reg->value.ptr;
    static uint8_t tmp_buffer[TX_SIZE];
    static uint8_t length = TX_SIZE;

    callback((reg_t *)i2c_current_reg, READ_ONLY, tmp_buffer, &length);
    if (length > TX_SIZE) length = TX_SIZE;

    if (length == 0) return;
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, tmp_buffer, length, I2C_LAST_FRAME);
  } else if (i2c_current_reg->type == POINTER_UINT8 ||
             i2c_current_reg->type == POINTER_UINT16) {
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t *)i2c_current_reg->value.ptr,
                                  i2c_current_reg->_size, I2C_LAST_FRAME);
  } else {
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t *)&i2c_current_reg->value.u16,
                                  i2c_current_reg->_size, I2C_LAST_FRAME);
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  switch (i2c_state) {
    case I2C_STATE_WAIT_REG_ADDR: {
      i2c_current_reg = get_register_from_addr(i2c_current_reg_addr);
      if (!i2c_current_reg || i2c_current_reg->reg_type == READ_ONLY) {
        i2c_state = I2C_STATE_IDLE;
        return;
      }

      if (i2c_current_reg->type == POINTER_CALLBACK) {
        CustomI2CReadWriteCallback_t callback = i2c_current_reg->value.ptr;

        i2c_rx_length = RX_SIZE;
        callback((reg_t *)i2c_current_reg, WRITE_ONLY, NULL,
                 (uint8_t *)&i2c_rx_length);
        if (i2c_rx_length > RX_SIZE) i2c_rx_length = RX_SIZE;

        i2c_state = I2C_STATE_WAIT_WRITE_DATA;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, (uint8_t *)i2c_rx_buffer,
                                     i2c_rx_length, I2C_FIRST_FRAME);
      } else if (i2c_current_reg->type == POINTER_UINT8 ||
                 i2c_current_reg->type == POINTER_UINT16) {
        i2c_state = I2C_STATE_WAIT_WRITE_DATA;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, i2c_current_reg->value.ptr,
                                     i2c_current_reg->_size, I2C_FIRST_FRAME);
      } else {
        i2c_state = I2C_STATE_WAIT_WRITE_DATA;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c,
                                     (uint8_t *)&i2c_current_reg->value.u16,
                                     i2c_current_reg->_size, I2C_FIRST_FRAME);
      }
      break;
    }

    default: {
      i2c_state = I2C_STATE_IDLE;
      i2c_current_reg = NULL;
    } break;
  }

  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  i2c_state = I2C_STATE_IDLE;
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  HAL_I2C_EnableListen_IT(hi2c);
}
