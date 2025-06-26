#ifndef __REGISTERS_H__
#define __REGISTERS_H__

#include <stdint.h>
#include <stdlib.h>

extern volatile uint8_t  last_error;
extern volatile uint16_t leds_data;
extern volatile uint16_t led_mode_mask;
extern volatile uint16_t custom_leds_data;
extern volatile uint16_t threshold_flag;
extern volatile uint16_t thresholds[16];
extern volatile uint16_t mux_data[16];

// 0x1_2_03 => v1.2.3
// v0.1.0
#define VERSION ((uint16_t)0x0100)

#define I2C_RX_BUSY_CNTR 150

#define REG_LED_ADDR 0x01
#define REG_LED_MODE_MASK_ADDR 0x02
#define REG_READ_THRESHOLD_ONE_ADDR 0x13
#define REG_WRITE_THRESHOLD_ONE_ADDR 0x14
#define REG_THRESHOLD_ALL_ADDR 0x15
#define REG_THRESHOLD_FLAG_ADDR 0x16
#define REG_ADC_DATA_ADDR 0x29
#define REG_ADC_DATA00_ADDR 0x30
#define REG_ADC_DATA01_ADDR 0x31
#define REG_ADC_DATA02_ADDR 0x32
#define REG_ADC_DATA03_ADDR 0x33
#define REG_ADC_DATA04_ADDR 0x34
#define REG_ADC_DATA05_ADDR 0x35
#define REG_ADC_DATA06_ADDR 0x36
#define REG_ADC_DATA07_ADDR 0x37
#define REG_ADC_DATA08_ADDR 0x38
#define REG_ADC_DATA09_ADDR 0x39
#define REG_ADC_DATA10_ADDR 0x3A
#define REG_ADC_DATA11_ADDR 0x3B
#define REG_ADC_DATA12_ADDR 0x3C
#define REG_ADC_DATA13_ADDR 0x3D
#define REG_ADC_DATA14_ADDR 0x3E
#define REG_ADC_DATA15_ADDR 0x3F
#define REG_VERSION_ADDR 0xFE
#define REG_LAST_ERROR_ADDR 0xFF

typedef enum reg_idx_t {
  REG_LED,                  // 自訂 LED 狀態 16-bit（寫入與讀取）
                            // > 1 表示該 LED 亮起，0 表示關閉
                            // > 若寫入 1，對應 LED 會自動設定為「自訂」模式
                            // > 讀取為讀取當前真實 LED 狀態，並非自訂狀態
                            //
  REG_LED_MODE_MASK,        // LED 模式遮罩 16-bit（寫入/讀取）
                            // > 1 表示該 LED 使用自訂狀態控制
                            // > 0 表示該 LED 使用 threshold flag 控制
                            //
  REG_READ_THRESHOLD_ONE,   // threshold 數值陣列（單筆讀）
  REG_WRITE_THRESHOLD_ONE,  // threshold 數值陣列（單筆寫）
                            //
  REG_THRESHOLD_ALL,        // threshold 數值陣列（全部讀寫）
                            //
  REG_THRESHOLD_FLAG,       // threshold 比較結果旗標 16-bit（讀取）
                            // > 每個 bit 表示該通道是否超過閾值
                            // > 1 表示超過閾值, 0 表示未超過閾值
                            //
  REG_ADC_DATA,             // ADC 資料陣列（全部讀取）
                            // > 16 * 2Byte => 32Byte
                            /* ADC 資料陣列 [以下 2Byte] */
  REG_ADC_DATA00,           // ADC 資料陣列（單筆讀取 - 00）
  REG_ADC_DATA01,           // ADC 資料陣列（單筆讀取 - 01）
  REG_ADC_DATA02,           // ADC 資料陣列（單筆讀取 - 02）
  REG_ADC_DATA03,           // ADC 資料陣列（單筆讀取 - 03）
  REG_ADC_DATA04,           // ADC 資料陣列（單筆讀取 - 04）
  REG_ADC_DATA05,           // ADC 資料陣列（單筆讀取 - 05）
  REG_ADC_DATA06,           // ADC 資料陣列（單筆讀取 - 06）
  REG_ADC_DATA07,           // ADC 資料陣列（單筆讀取 - 07）
  REG_ADC_DATA08,           // ADC 資料陣列（單筆讀取 - 08）
  REG_ADC_DATA09,           // ADC 資料陣列（單筆讀取 - 09）
  REG_ADC_DATA10,           // ADC 資料陣列（單筆讀取 - 10）
  REG_ADC_DATA11,           // ADC 資料陣列（單筆讀取 - 11）
  REG_ADC_DATA12,           // ADC 資料陣列（單筆讀取 - 12）
  REG_ADC_DATA13,           // ADC 資料陣列（單筆讀取 - 13）
  REG_ADC_DATA14,           // ADC 資料陣列（單筆讀取 - 14）
  REG_ADC_DATA15,           // ADC 資料陣列（單筆讀取 - 15）
                            //
  REG_VERSION,              // 讀取版本
  REG_LAST_ERROR,           // 讀取最後錯誤
} reg_idx_t;

typedef enum ValueType { UINT8, UINT16, POINTER_UINT8, POINTER_UINT16, POINTER_CALLBACK } ValueType_t;

typedef enum RegisterType { READ_ONLY, WRITE_ONLY, FULL_ACCESS } RegisterType_t;

typedef union {
  uint8_t  u8;
  uint16_t u16;
  void*    ptr;
} var_t;

typedef struct reg_t {
  uint8_t        addr;
  ValueType_t    type;
  RegisterType_t reg_type;
  var_t          value;

  uint8_t ptr_size;
  uint8_t _size;
} reg_t;

/**
 * @brief Custom I2C register read/write callback
 * @param reg   Pointer to the register definition
 * @param mode  Access mode: READ_ONLY or WRITE_ONLY
 * @param buffer If WRITE, input data; if READ, output buffer
 * @param length If WRITE, input length; if READ, set output length
 * @return 0 on success, negative on error
 */
typedef int (*CustomI2CReadWriteCallback_t)(reg_t* reg, RegisterType_t mode, uint8_t* buffer, uint8_t* length);

int led_i2c_callback(reg_t* reg, RegisterType_t mode, uint8_t* buffer, uint8_t* length);
int threshold_i2c_callback(reg_t* reg, RegisterType_t mode, uint8_t* buffer, uint8_t* length);
int last_error_i2c_callback(reg_t* reg, RegisterType_t mode, uint8_t* buffer, uint8_t* length);

void register_init(void);

volatile reg_t* get_register_from_addr(uint8_t addr);

volatile reg_t* get_register_from_idx(reg_idx_t idx);

#endif
