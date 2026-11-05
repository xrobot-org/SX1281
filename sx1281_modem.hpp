#pragma once

// 每种调制的固定参数。引用的章节均指数据手册 DS_SX1280-1 rev 3.3。
// Fixed parameters of each modulation. Sections refer to the datasheet DS_SX1280-1
// rev 3.3.

#include <cstdint>

#include "sx1281_regs.hpp"

/**
 * @brief 一种调制的固定参数。
 *        Fixed parameters of one modulation.
 */
struct SX1281Modem
{
  /// 同步字的写法 / How the sync word is written.
  enum class SyncWord : uint8_t
  {
    NONE,  ///< 不用同步字 / no sync word
    FIVE,  ///< 0x09CE 写 0，4 字节从 0x09CF 开始 / 0 at 0x09CE, 4 bytes from 0x09CF
    FOUR   ///< 只写 0x09CF..0x09D2 / 0x09CF..0x09D2 only
  };

  uint8_t packet_type;
  uint8_t modulation[3];
  uint8_t packet[7];           ///< SetPacketParams 的参数 / SetPacketParams parameters
  uint8_t payload_size_index;  ///< packet 里载荷长度的位置 / payload size within packet
  uint8_t max_payload;         ///< 最大载荷长度 / largest payload
  uint32_t tx_timeout_us;      ///< 发送超时 / transmission timeout
  SyncWord sync_word;          ///< 同步字的写法 / how the sync word is written
  /// 坏包看 GetPacketStatus 的错误字节，否则看 IRQ（勘误 16.3）。
  /// Bad packets show in the error byte of GetPacketStatus, otherwise in the IRQs
  /// (errata 16.3).
  bool errors_in_packet_status;
  /// 连续接收可能停在前导码之后（勘误 16.1）。
  /// Continuous receive can stop after a preamble (errata 16.1).
  bool preamble_can_stall;
};

// LoRa SF8 / BW 812.5 kHz / CR 4/5；12 符号前导码，显式包头，CRC 开，IQ 正常（14.4）。
// 255 字节的包约 110 ms。包状态是 RSSI / SNR，错误看 IRQ；没有前导码停顿。
// LoRa SF8 / BW 812.5 kHz / CR 4/5; 12-symbol preamble, explicit header, CRC on, normal
// IQ (14.4). A 255-byte packet takes about 110 ms. The packet status is RSSI / SNR, and
// errors come from the IRQs; there is no preamble stall.
inline constexpr SX1281Modem SX1281_MODEM_LORA = {
    .packet_type = SX1281Regs::PACKET_TYPE_LORA,
    .modulation = {0x80, 0x18, 0x01},
    .packet = {0x0C, 0x00, 0x00, 0x20, 0x40, 0x00, 0x00},
    .payload_size_index = 2,
    .max_payload = 255,
    .tx_timeout_us = 300000,
    .sync_word = SX1281Modem::SyncWord::NONE,
    .errors_in_packet_status = false,
    .preamble_can_stall = false};
// GFSK 2 Mb/s、带宽 2.4 MHz、调制指数 0.5、BT 0.5（表 14-1..14-3）；16 位前导码，4 字节
// 同步字 1，变长，2 字节 CRC，白化开（14.1.1）。最长的包约 1.3 ms。
// GFSK 2 Mb/s, 2.4 MHz bandwidth, modulation index 0.5, BT 0.5 (tables 14-1..14-3);
// 16-bit preamble, 4-byte sync word 1, variable length, 2-byte CRC, whitening on
// (14.1.1). The longest packet takes about 1.3 ms.
inline constexpr SX1281Modem SX1281_MODEM_GFSK = {
    .packet_type = SX1281Regs::PACKET_TYPE_GFSK,
    .modulation = {0x04, 0x01, 0x20},
    .packet = {0x30, 0x06, 0x10, 0x20, 0x00, 0x20, 0x00},
    .payload_size_index = 4,
    .max_payload = 255,
    .tx_timeout_us = 5000,
    .sync_word = SX1281Modem::SyncWord::FIVE,
    .errors_in_packet_status = true,
    .preamble_can_stall = true};
// FLRC 1.3 Mb/s、带宽 1.2 MHz、BT 1.0（表 14-31..14-33），编码率 1（无 FEC）；16 位
// 前导码，32 位同步字 1，变长，2 字节 CRC，白化关（表 14-41）。没有同步字时接收机会把
// 噪声当成包。最长的包约 1.3 ms。
// FLRC 1.3 Mb/s, 1.2 MHz bandwidth, BT 1.0 (tables 14-31..14-33), coding rate 1 (no
// FEC); 16-bit preamble, 32-bit sync word 1, variable length, 2-byte CRC, whitening off
// (table 14-41). Without a sync word the receiver takes noise for packets. The longest
// packet takes about 1.3 ms.
inline constexpr SX1281Modem SX1281_MODEM_FLRC = {
    .packet_type = SX1281Regs::PACKET_TYPE_FLRC,
    .modulation = {0x45, 0x04, 0x10},
    .packet = {0x30, 0x04, 0x10, 0x20, 0x00, 0x10, 0x08},
    .payload_size_index = 4,
    .max_payload = 127,
    .tx_timeout_us = 5000,
    .sync_word = SX1281Modem::SyncWord::FOUR,
    .errors_in_packet_status = true,
    .preamble_can_stall = true};
