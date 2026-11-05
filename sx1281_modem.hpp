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
  uint8_t packet_type;
  uint8_t modulation[3];
  uint8_t packet[7];  // 第 5 字节（载荷长度）由 SetPacketParams 填 / byte 5 (payload
                      // size) is filled by SetPacketParams
};

// LoRa SF8 / BW 812.5 kHz / CR 4/5；12 符号前导码，显式包头，CRC 开，IQ 正常（14.4）。
// LoRa SF8 / BW 812.5 kHz / CR 4/5; 12-symbol preamble, explicit header, CRC on, normal
// IQ (14.4).
inline constexpr SX1281Modem SX1281_MODEM_LORA = {
    SX1281Regs::PACKET_TYPE_LORA,
    {0x80, 0x18, 0x01},
    {0x0C, 0x00, 0x00, 0x20, 0x40, 0x00, 0x00}};
// GFSK 2 Mb/s、带宽 2.4 MHz、调制指数 0.5、BT 0.5（表 14-1..14-3）；16 位前导码，4 字节
// 同步字 1，变长，2 字节 CRC，白化开（14.1.1）。
// GFSK 2 Mb/s, 2.4 MHz bandwidth, modulation index 0.5, BT 0.5 (tables 14-1..14-3);
// 16-bit preamble, 4-byte sync word 1, variable length, 2-byte CRC, whitening on
// (14.1.1).
inline constexpr SX1281Modem SX1281_MODEM_GFSK = {
    SX1281Regs::PACKET_TYPE_GFSK,
    {0x04, 0x01, 0x20},
    {0x30, 0x06, 0x10, 0x20, 0x00, 0x20, 0x00}};
// FLRC 1.3 Mb/s、带宽 1.2 MHz、BT 1.0（表 14-31..14-33），编码率 1（无 FEC）；16 位
// 前导码，32 位同步字 1，变长，2 字节 CRC，白化关（表 14-41）。没有同步字时接收机会把
// 噪声当成包。
// FLRC 1.3 Mb/s, 1.2 MHz bandwidth, BT 1.0 (tables 14-31..14-33), coding rate 1 (no
// FEC); 16-bit preamble, 32-bit sync word 1, variable length, 2-byte CRC, whitening off
// (table 14-41). Without a sync word the receiver takes noise for packets.
inline constexpr SX1281Modem SX1281_MODEM_FLRC = {
    SX1281Regs::PACKET_TYPE_FLRC,
    {0x45, 0x04, 0x10},
    {0x30, 0x04, 0x10, 0x20, 0x00, 0x10, 0x08}};
