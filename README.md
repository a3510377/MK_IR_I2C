# MK_IR_I2C

基於 STM32F030F4P6 所設計的紅外線感測器陣列模組，具備 I2C 通訊能力，可快速整合至各類型機器人平台與自動化設備中。模組透過 CD74HC4067 多工器讀取 QRE1113GR 感測器，並將資料經 I2C 匯流排傳送至主控板（如 Arduino 或其他 MCU）。

## 🛠️ 硬體架構

- **MCU**：STM32F030F4P6
- **感測器**：16 顆 QRE1113GR 反射式紅外線感測器
- **類比多工器**：CD74HC4067
- **I²C 介面**：Slave 模式，支援標準與高速模式（100kHz / 400kHz）
- **輸出資料**：多通道反射值，單位為 8-bit 整數
- **擴充性**：
  - 支援多模組串接（透過不同 I²C 位址）
  - 每個感測器可單獨設定門檻值與使能狀態

## 📦 軟體功能

- I²C slave 實作，可接收主機命令與回傳感測資料
- 支援動態設定感測器參數（使能/閾值）
- 感測資料自動掃描（DMA + ADC）
- 內建 LED 狀態指示

## 📋 I²C 寄存器對照表（Register Map）

| 名稱                      | 地址 | 權限     | 大小     | 說明                                                  |
| ------------------------- | ---- | -------- | -------- | ----------------------------------------------------- |
| `REG_LED`                 | 0x01 | R/W (CB) | 2 bytes  | 控制自訂 LED 狀態（16-bit），自動啟用自訂控制         |
| `REG_LED_MODE_MASK`       | 0x02 | R/W      | 2 bytes  | LED 控制遮罩，1=自訂控制，0=由閾值自動控制            |
| `REG_READ_THRESHOLD_ONE`  | 0x13 | R (CB)   | 3 bytes  | 讀取單一通道閾值，格式：`[index] → [index][LSB][MSB]` |
| `REG_WRITE_THRESHOLD_ONE` | 0x14 | W (CB)   | 3 bytes  | 寫入單一通道閾值，格式：`[index][LSB][MSB]`           |
| `REG_THRESHOLD_ALL`       | 0x15 | R/W (CB) | 32 bytes | 一次讀/寫全部 16 通道閾值，每通道 2 bytes             |
| `REG_THRESHOLD_FLAG`      | 0x16 | R        | 2 bytes  | 閾值比較旗標（bitmask），1=超過，0=未超過             |
| `REG_ADC_DATA`            | 0x29 | R        | 32 bytes | 讀取所有通道 ADC 資料（16 通道 × 2 bytes）            |
| `REG_ADC_DATA00`          | 0x30 | R        | 2 bytes  | 通道 0 的 ADC 數值                                    |
| `REG_ADC_DATA01`          | 0x31 | R        | 2 bytes  | 通道 1 的 ADC 數值                                    |
| `REG_ADC_DATA02`          | 0x32 | R        | 2 bytes  | 通道 2 的 ADC 數值                                    |
| `REG_ADC_DATA03`          | 0x33 | R        | 2 bytes  | 通道 3 的 ADC 數值                                    |
| `REG_ADC_DATA04`          | 0x34 | R        | 2 bytes  | 通道 4 的 ADC 數值                                    |
| `REG_ADC_DATA05`          | 0x35 | R        | 2 bytes  | 通道 5 的 ADC 數值                                    |
| `REG_ADC_DATA06`          | 0x36 | R        | 2 bytes  | 通道 6 的 ADC 數值                                    |
| `REG_ADC_DATA07`          | 0x37 | R        | 2 bytes  | 通道 7 的 ADC 數值                                    |
| `REG_ADC_DATA08`          | 0x38 | R        | 2 bytes  | 通道 8 的 ADC 數值                                    |
| `REG_ADC_DATA09`          | 0x39 | R        | 2 bytes  | 通道 9 的 ADC 數值                                    |
| `REG_ADC_DATA10`          | 0x3A | R        | 2 bytes  | 通道 10 的 ADC 數值                                   |
| `REG_ADC_DATA11`          | 0x3B | R        | 2 bytes  | 通道 11 的 ADC 數值                                   |
| `REG_ADC_DATA12`          | 0x3C | R        | 2 bytes  | 通道 12 的 ADC 數值                                   |
| `REG_ADC_DATA13`          | 0x3D | R        | 2 bytes  | 通道 13 的 ADC 數值                                   |
| `REG_ADC_DATA14`          | 0x3E | R        | 2 bytes  | 通道 14 的 ADC 數值                                   |
| `REG_ADC_DATA15`          | 0x3F | R        | 2 bytes  | 通道 15 的 ADC 數值                                   |
| `REG_VERSION`             | 0xFE | R        | 2 bytes  | 固件版本號（例如 `0x0102` 表示 v1.0.2）               |
| `REG_LAST_ERROR`          | 0xFF | R        | 1 byte   | 最後錯誤代碼（讀取後自動清除）                        |

> 💡 **說明**：
>
> - 權限欄位：`R` = Read、`W` = Write、`CB` = 透過 callback 處理邏輯。
> - 所有資料皆為小端序（Little Endian）。
> - 欲設定閾值或控制 LED，需配合格式撰寫資料結構，詳見各節控制說明。
