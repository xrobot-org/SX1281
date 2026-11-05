#pragma once

// SX1280/SX1281 的命令、寄存器和中断位。引用的章节均指数据手册 DS_SX1280-1 rev 3.3。
// Commands, registers and interrupt bits of the SX1280/SX1281. Sections refer to the
// datasheet DS_SX1280-1 rev 3.3.

#include <cstddef>
#include <cstdint>

namespace SX1281Regs
{
inline constexpr uint32_t XTAL_HZ = 52000000UL;
inline constexpr size_t MAX_PAYLOAD_SIZE = 255;

inline constexpr uint8_t CMD_WRITE_REGISTER = 0x18;
inline constexpr uint8_t CMD_READ_REGISTER = 0x19;
inline constexpr uint8_t CMD_WRITE_BUFFER = 0x1A;
inline constexpr uint8_t CMD_READ_BUFFER = 0x1B;
inline constexpr uint8_t CMD_GET_IRQ_STATUS = 0x15;
inline constexpr uint8_t CMD_GET_RX_BUFFER_STATUS = 0x17;
inline constexpr uint8_t CMD_GET_PACKET_STATUS = 0x1D;
inline constexpr uint8_t CMD_GET_RSSI_INST = 0x1F;
inline constexpr uint8_t CMD_SET_STANDBY = 0x80;
inline constexpr uint8_t CMD_SET_RX = 0x82;
inline constexpr uint8_t CMD_SET_TX = 0x83;
inline constexpr uint8_t CMD_SET_RF_FREQUENCY = 0x86;
inline constexpr uint8_t CMD_SET_PACKET_TYPE = 0x8A;
inline constexpr uint8_t CMD_SET_MODULATION_PARAMS = 0x8B;
inline constexpr uint8_t CMD_SET_PACKET_PARAMS = 0x8C;
inline constexpr uint8_t CMD_SET_DIO_IRQ_PARAMS = 0x8D;
inline constexpr uint8_t CMD_SET_TX_PARAMS = 0x8E;
inline constexpr uint8_t CMD_SET_BUFFER_BASE_ADDRESS = 0x8F;
inline constexpr uint8_t CMD_SET_REGULATOR_MODE = 0x96;
inline constexpr uint8_t CMD_CLEAR_IRQ_STATUS = 0x97;
inline constexpr uint8_t CMD_SET_AUTO_FS = 0x9E;
inline constexpr uint8_t CMD_SET_FS = 0xC1;

inline constexpr uint16_t REG_CRC_SEED = 0x09C8;
inline constexpr uint16_t REG_SYNC_WORD_1 = 0x09CE;
inline constexpr uint16_t REG_XOSC_CAP_BANK_A = 0x0A0E;
inline constexpr uint16_t REG_XOSC_CAP_BANK_B = 0x0A0F;

inline constexpr uint8_t STANDBY_RC = 0x00;
inline constexpr uint8_t STANDBY_XOSC = 0x01;
inline constexpr uint8_t REGULATOR_LDO = 0x00;
inline constexpr uint8_t REGULATOR_DCDC = 0x01;
inline constexpr uint8_t PACKET_TYPE_GFSK = 0x00;
inline constexpr uint8_t PACKET_TYPE_LORA = 0x01;
inline constexpr uint8_t PACKET_TYPE_FLRC = 0x03;
// PA 爬升时间 8 us（表 11-49）。 / PA ramp time 8 us (table 11-49).
inline constexpr uint8_t RAMP_08_US = 0x60;
// SetTx 不设超时，SetRx 连续接收（11.6.4、11.6.5）。
// SetTx without a timeout, SetRx continuous (11.6.4, 11.6.5).
inline constexpr uint8_t TX_NO_TIMEOUT[3] = {0x00, 0x00, 0x00};
inline constexpr uint8_t RX_CONTINUOUS[3] = {0x00, 0xFF, 0xFF};
// 收发都从缓冲区地址 0 开始，各自可用整个 256 字节（第 8 节）。
// Both directions use the whole 256-byte buffer from address 0 (section 8).
inline constexpr uint8_t TX_BASE = 0x00;
inline constexpr uint8_t RX_BASE = 0x00;

// IRQ 位（表 11-73）。DIO1 只接 TxDone、RxDone、CrcError；其余只锁存在状态里。
// IRQ bits (table 11-73). Only TxDone, RxDone and CrcError reach DIO1; the others are
// latched in the status only.
inline constexpr uint16_t IRQ_TX_DONE = 0x0001;
inline constexpr uint16_t IRQ_RX_DONE = 0x0002;
inline constexpr uint16_t IRQ_SYNC_WORD_VALID = 0x0004;
inline constexpr uint16_t IRQ_HEADER_VALID = 0x0010;
inline constexpr uint16_t IRQ_HEADER_ERROR = 0x0020;
inline constexpr uint16_t IRQ_CRC_ERROR = 0x0040;
inline constexpr uint16_t IRQ_PREAMBLE_DETECTED = 0x8000;
inline constexpr uint16_t IRQ_DIO1 = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_CRC_ERROR;
inline constexpr uint16_t IRQ_LATCHED = IRQ_DIO1 | IRQ_SYNC_WORD_VALID |
                                        IRQ_HEADER_VALID | IRQ_HEADER_ERROR |
                                        IRQ_PREAMBLE_DETECTED;
// 这些位锁存着而还没有 RxDone：有包正在到达。 / These latched without an RxDone: a
// packet is arriving.
inline constexpr uint16_t IRQ_ARRIVING =
    IRQ_SYNC_WORD_VALID | IRQ_HEADER_VALID | IRQ_PREAMBLE_DETECTED;
// GetPacketStatus 的错误字节（表 11-68）：LengthError、CrcError、AbortError。
// Error byte of GetPacketStatus (table 11-68): LengthError, CrcError, AbortError.
inline constexpr uint8_t PACKET_ERRORS = 0x20 | 0x10 | 0x08;
}  // namespace SX1281Regs
