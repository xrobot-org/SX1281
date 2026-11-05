#pragma once

// SX1281 的实现，由 SX1281.hpp 包含。写在头文件里，调用方能把它内联进收发循环。
// Implementation of SX1281, included by SX1281.hpp. It is in a header so that callers
// can inline it into their send and receive loops.

#include <algorithm>
#include <cstring>

#include "timebase.hpp"

inline SX1281::SX1281(LibXR::SPI& spi, LibXR::GPIO& nss, LibXR::GPIO& busy,
                      LibXR::GPIO& nreset, LibXR::GPIO& dio1, LibXR::GPIO& txen,
                      LibXR::GPIO& rxen, LibXR::GPIO* dcdcen)
    : bus_(spi, nss, busy),
      nreset_(nreset),
      dio1_(dio1),
      txen_(txen),
      rxen_(rxen),
      dcdcen_(dcdcen)
{
  ConfigurePins();
}

inline void SX1281::ConfigurePins()
{
  nreset_.SetConfig({.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                     .pull = LibXR::GPIO::Pull::NONE});
  nreset_.Write(false);
  dio1_.SetConfig(
      {.direction = LibXR::GPIO::Direction::INPUT, .pull = LibXR::GPIO::Pull::NONE});
  txen_.SetConfig({.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                   .pull = LibXR::GPIO::Pull::NONE});
  rxen_.SetConfig({.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                   .pull = LibXR::GPIO::Pull::NONE});
  txen_.Write(false);
  rxen_.Write(false);
  if (dcdcen_ != nullptr)
  {
    dcdcen_->SetConfig({.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                        .pull = LibXR::GPIO::Pull::NONE});
    dcdcen_->Write(true);
  }
}

inline const SX1281Modem& SX1281::Modem() const
{
  switch (config_.modulation)
  {
    case Modulation::GFSK:
      return SX1281_MODEM_GFSK;
    case Modulation::FLRC:
      return SX1281_MODEM_FLRC;
    case Modulation::LORA:
    default:
      return SX1281_MODEM_LORA;
  }
}

inline uint8_t SX1281::MaxPayload() const { return Modem().max_payload; }

// ---------------------------------------------------------------------------------------
// 公共操作 / Public operations
// ---------------------------------------------------------------------------------------

inline LibXR::ErrorCode SX1281::Start(const Config& config)
{
  config_ = config;
  return Restart() ? LibXR::ErrorCode::OK : LibXR::ErrorCode::FAILED;
}

inline void SX1281::PrepareSend()
{
  if (state_ == State::RX)
  {
    EnterFs();
  }
}

inline LibXR::ErrorCode SX1281::Send(const uint8_t* data, uint8_t size)
{
  using namespace SX1281Regs;
  if (data == nullptr || size == 0 || size > MaxPayload())
  {
    return LibXR::ErrorCode::ARG_ERR;
  }
  switch (state_)
  {
    case State::STOPPED:
    case State::IN_RESET:
    case State::WAKING:
      return LibXR::ErrorCode::FAILED;
    case State::TX:
    {
      const TxState tx = CheckTx();
      if (tx == TxState::ON_AIR)
      {
        return LibXR::ErrorCode::BUSY;
      }
      EndTx(tx);  // 发完后 AutoFs 停在 FS / after TxDone, AutoFs stays in FS
      break;
    }
    case State::RX:
      EnterFs();
      break;
    case State::PREPARED:
      break;
  }
  SetRfSwitch(true);
  const bool ok = bus_.WriteBuffer(TX_BASE, data, size) && SetPacketParams(size) &&
                  bus_.WriteCommand(CMD_SET_TX, TX_NO_TIMEOUT, sizeof(TX_NO_TIMEOUT));
  state_ = State::TX;
  tx_started_us_ = SX1281Bus::NowUs();
  if (!ok)
  {
    tx_errors_++;
    FinishSend();
    return LibXR::ErrorCode::FAILED;
  }
  return LibXR::ErrorCode::OK;
}

inline void SX1281::Poll()
{
  switch (state_)
  {
    case State::STOPPED:
      break;
    case State::IN_RESET:
    {
      const uint32_t now = SX1281Bus::NowUs();
      if (SX1281Bus::Elapsed(now, reset_us_) >= RESET_US)
      {
        nreset_.Write(true);
        state_ = State::WAKING;
        reset_us_ = now;
      }
      break;
    }
    case State::WAKING:
      if (SX1281Bus::Elapsed(SX1281Bus::NowUs(), reset_us_) >= WAKE_US)
      {
        (void)Configure();  // 失败时静默超时后再复位 / a failure resets again later
      }
      break;
    case State::TX:
    {
      const TxState tx = CheckTx();
      if (tx != TxState::ON_AIR)
      {
        EndTx(tx);
        FinishSend();
      }
      break;
    }
    case State::PREPARED:
      // 准备了发送却没有发：重新接收。 / Prepared for a send that did not come: listen
      // again.
      EnterRx();
      break;
    case State::RX:
      if (!dio1_.Read())
      {
        Recover();
      }
      break;
  }
}

inline int SX1281::Receive(uint8_t* out)
{
  using namespace SX1281Regs;
  if (state_ != State::RX || !dio1_.Read())
  {
    return 0;
  }
  watch_.heard_us = watch_.rearmed_us = SX1281Bus::NowUs();
  watch_.arriving = false;
  // 下一包可能紧跟着到，少一条命令就早一点读到缓冲。
  // The next packet may follow right behind, so every command saved reaches the buffer
  // sooner.
  const uint16_t irq = Modem().errors_in_packet_status ? 0 : bus_.GetIrqStatus();
  bus_.ClearIrq();
  if ((irq & (IRQ_CRC_ERROR | IRQ_HEADER_ERROR)) != 0 || PacketStatusBad())
  {
    rx_errors_++;
    return -1;
  }
  uint8_t status[2] = {};
  if (!bus_.ReadCommand(CMD_GET_RX_BUFFER_STATUS, status, sizeof(status)))
  {
    rx_errors_++;
    return -1;
  }
  const uint8_t size = status[0];
  const uint8_t offset = status[1];
  if (size == 0 || size > MaxPayload() || !bus_.ReadBuffer(offset, out, size))
  {
    rx_errors_++;
    return -1;
  }
  rx_packets_++;
  return size;
}

inline LibXR::ErrorCode SX1281::Tune(uint32_t frequency_hz)
{
  using namespace SX1281Regs;
  if (state_ == State::TX)
  {
    return LibXR::ErrorCode::BUSY;
  }
  config_.frequency_hz = frequency_hz;
  if (Resetting())
  {
    return LibXR::ErrorCode::OK;  // 复位完成时用新频点配置 / used when the reset ends
  }
  const uint8_t xosc = STANDBY_XOSC;
  const bool ok = bus_.WriteCommand(CMD_SET_STANDBY, &xosc, 1) &&
                  bus_.SetRfFrequency(frequency_hz);
  bus_.ClearIrq();
  EnterRx();
  return ok ? LibXR::ErrorCode::OK : LibXR::ErrorCode::FAILED;
}

inline int16_t SX1281::RssiInstant()
{
  using namespace SX1281Regs;
  uint8_t value = 0;
  if (state_ != State::RX || !bus_.ReadCommand(CMD_GET_RSSI_INST, &value, 1))
  {
    return 0;
  }
  return static_cast<int16_t>(-static_cast<int16_t>(value));
}

// ---------------------------------------------------------------------------------------
// 启动和复位 / Start-up and reset
// ---------------------------------------------------------------------------------------

// 复位芯片并配置，阻塞约 22 ms（Start()）。 / Reset and configure the chip, blocking for
// about 22 ms (Start()).
inline bool SX1281::Restart()
{
  HoldReset();
  LibXR::Timebase::DelayMicroseconds(RESET_US);
  nreset_.Write(true);
  LibXR::Timebase::DelayMicroseconds(WAKE_US);
  return Configure();
}

inline void SX1281::HoldReset()
{
  state_ = State::IN_RESET;
  SetRfSwitch(false);
  rxen_.Write(false);
  nreset_.Write(false);
}

inline bool SX1281::Configure()
{
  using namespace SX1281Regs;
  // 14.1.1 / 14.3.1 / 14.4.1：STDBY_RC，先设包类型，再设频率、缓冲区、调制和包参数、
  // 同步字与 CRC、发射参数、IRQ 路由。
  // 14.1.1 / 14.3.1 / 14.4.1: STDBY_RC, packet type first, then frequency, buffer,
  // modulation and packet parameters, sync word and CRC, TX parameters, IRQ routing.
  const SX1281Modem& modem = Modem();
  const uint8_t regulator = dcdcen_ != nullptr ? REGULATOR_DCDC : REGULATOR_LDO;
  const uint8_t standby = STANDBY_RC;
  const uint8_t buffer_base[2] = {TX_BASE, RX_BASE};
  const int8_t power = std::clamp<int8_t>(config_.tx_power_dbm, -18, 13);
  const uint8_t tx_params[2] = {static_cast<uint8_t>(power + 18), RAMP_08_US};
  const uint8_t irq_params[8] = {
      IRQ_LATCHED >> 8, IRQ_LATCHED & 0xFF, IRQ_DIO1 >> 8, IRQ_DIO1 & 0xFF, 0, 0, 0, 0};
  const uint8_t auto_fs = 0x01;

  bool ok =
      bus_.WriteCommand(CMD_SET_STANDBY, &standby, 1) &&
      bus_.WriteCommand(CMD_SET_REGULATOR_MODE, &regulator, 1) &&
      bus_.WriteCommand(CMD_SET_PACKET_TYPE, &modem.packet_type, 1) &&
      bus_.SetRfFrequency(config_.frequency_hz) &&
      bus_.WriteCommand(CMD_SET_BUFFER_BASE_ADDRESS, buffer_base, sizeof(buffer_base)) &&
      bus_.WriteCommand(CMD_SET_MODULATION_PARAMS, modem.modulation,
                        sizeof(modem.modulation)) &&
      SetPacketParams(MaxPayload());
  if (ok && modem.sync_word != SX1281Modem::SyncWord::NONE)
  {
    ok = WriteSyncWord() && bus_.WriteRegister16(REG_CRC_SEED, config_.crc_seed);
  }
  if (ok && config_.xosc_trim >= 0)
  {
    const uint8_t xosc = STANDBY_XOSC;
    const uint8_t trim = static_cast<uint8_t>(config_.xosc_trim) & 0x1F;
    ok = bus_.WriteCommand(CMD_SET_STANDBY, &xosc, 1) &&
         bus_.WriteRegisters(REG_XOSC_CAP_BANK_A, &trim, 1) &&
         bus_.WriteRegisters(REG_XOSC_CAP_BANK_B, &trim, 1);
  }
  ok = ok && bus_.WriteCommand(CMD_SET_TX_PARAMS, tx_params, sizeof(tx_params)) &&
       bus_.WriteCommand(CMD_SET_DIO_IRQ_PARAMS, irq_params, sizeof(irq_params)) &&
       bus_.WriteCommand(CMD_SET_AUTO_FS, &auto_fs, 1);
  bus_.ClearIrq();
  EnterRx();
  watch_.heard_us = SX1281Bus::NowUs();
  return ok;
}

inline bool SX1281::WriteSyncWord()
{
  using namespace SX1281Regs;
  // GFSK：同步字 1 的第 5 字节（0x09CE）为 0，4 字节同步字从 0x09CF 开始；FLRC 只用
  // 0x09CF..0x09D2。
  // GFSK: byte 5 of sync word 1 (0x09CE) is 0 and the 4-byte word starts at 0x09CF;
  // FLRC uses 0x09CF..0x09D2 only.
  const uint8_t five[5] = {0x00, config_.sync_word[0], config_.sync_word[1],
                           config_.sync_word[2], config_.sync_word[3]};
  return Modem().sync_word == SX1281Modem::SyncWord::FIVE
             ? bus_.WriteRegisters(REG_SYNC_WORD_1, five, sizeof(five))
             : bus_.WriteRegisters(REG_SYNC_WORD_1 + 1, config_.sync_word,
                                   sizeof(config_.sync_word));
}

inline bool SX1281::SetPacketParams(uint8_t payload_size)
{
  using namespace SX1281Regs;
  const SX1281Modem& modem = Modem();
  uint8_t params[sizeof(modem.packet)];
  std::memcpy(params, modem.packet, sizeof(params));
  params[modem.payload_size_index] = payload_size;
  return bus_.WriteCommand(CMD_SET_PACKET_PARAMS, params, sizeof(params));
}

// ---------------------------------------------------------------------------------------
// 收发切换 / Switching between transmit and receive
// ---------------------------------------------------------------------------------------

inline void SX1281::EnterFs()
{
  using namespace SX1281Regs;
  // 10.8：FS 到发射 27 us；11.6.4：SetTx 前清 IRQ。
  // 10.8: FS to Tx takes 27 us; 11.6.4: IRQs are cleared before SetTx.
  bus_.WriteCommand(CMD_SET_FS, nullptr, 0);
  bus_.ClearIrq();
  state_ = State::PREPARED;
}

inline void SX1281::EnterRx()
{
  using namespace SX1281Regs;
  // 11.6.5：调用方在 SetRx 前清 IRQ。 / 11.6.5: the caller clears IRQs before SetRx.
  SetRfSwitch(false);
  bus_.WriteCommand(CMD_SET_RX, RX_CONTINUOUS, sizeof(RX_CONTINUOUS));
  state_ = State::RX;
  watch_.preamble_seen = false;
  watch_.arriving = false;
  // 停顿从这里算起：发送之后应答还在路上。 / Silence counts from here: right after a
  // transmission the answer is still on its way.
  watch_.rearmed_us = SX1281Bus::NowUs();
}

// 发送中的包是否结束；结束时计数一次，调用方随后结束这次发送。
// Whether the packet on air has ended; an end is counted once, and the caller then ends
// this transmission.
inline SX1281::TxState SX1281::CheckTx()
{
  if (dio1_.Read())
  {
    tx_packets_++;
    return TxState::DONE;
  }
  if (SX1281Bus::Elapsed(SX1281Bus::NowUs(), tx_started_us_) < Modem().tx_timeout_us)
  {
    return TxState::ON_AIR;
  }
  tx_errors_++;
  return TxState::TIMED_OUT;
}

inline void SX1281::EndTx(TxState state)
{
  using namespace SX1281Regs;
  if (state == TxState::TIMED_OUT)
  {
    bus_.WriteCommand(CMD_SET_FS, nullptr, 0);  // 可能还在发射 / it may still be on air
  }
  bus_.ClearIrq();
}

inline void SX1281::FinishSend()
{
  // 接收接受不超过最大值的任意长度（变长模式滤掉更长的包）。
  // Receive accepts any size up to the maximum (variable length drops longer packets).
  SetPacketParams(MaxPayload());
  EnterRx();
}

inline void SX1281::SetRfSwitch(bool transmit)
{
  rxen_.Write(!transmit);
  txen_.Write(transmit);
}

// ---------------------------------------------------------------------------------------
// 接收停顿恢复 / Receive stall recovery
// ---------------------------------------------------------------------------------------

inline void SX1281::Recover()
{
  using namespace SX1281Regs;
  const uint32_t now = SX1281Bus::NowUs();
  if (SX1281Bus::Elapsed(now, watch_.heard_us) >= config_.rx_restart_ms * 1000U)
  {
    restarts_++;
    HoldReset();
    reset_us_ = now;
    return;
  }
  const bool rearm_due =
      SX1281Bus::Elapsed(now, watch_.rearmed_us) >= config_.rx_rearm_ms * 1000U;
  const bool check_due = Modem().preamble_can_stall &&
                         SX1281Bus::Elapsed(now, watch_.checked_us) >= STUCK_CHECK_US;
  if (!rearm_due && !check_due)
  {
    return;
  }
  const uint16_t irq = bus_.GetIrqStatus();
  if ((rearm_due && !PacketArriving(now, irq)) || (check_due && PreambleStuck(now, irq)))
  {
    rearms_++;
    const uint8_t xosc = STANDBY_XOSC;
    bus_.WriteCommand(CMD_SET_STANDBY, &xosc, 1);
    bus_.ClearIrq();
    EnterRx();
  }
}

// 重新进入接收会截断正在到达的包。到达的时间不会超过一个最长包。 / Entering receive again
// would cut a packet that is arriving, which takes at most one longest packet.
inline bool SX1281::PacketArriving(uint32_t now, uint16_t irq)
{
  using namespace SX1281Regs;
  if ((irq & IRQ_ARRIVING) == 0)
  {
    watch_.arriving = false;
    return false;
  }
  if (!watch_.arriving)
  {
    watch_.arriving = true;
    watch_.arriving_us = now;
  }
  return SX1281Bus::Elapsed(now, watch_.arriving_us) < Modem().tx_timeout_us;
}

inline bool SX1281::PreambleStuck(uint32_t now, uint16_t irq)
{
  using namespace SX1281Regs;
  watch_.checked_us = now;
  if ((irq & IRQ_PREAMBLE_DETECTED) == 0 ||
      (irq & (IRQ_SYNC_WORD_VALID | IRQ_RX_DONE)) != 0)
  {
    watch_.preamble_seen = false;
    return false;
  }
  if (!watch_.preamble_seen)
  {
    watch_.preamble_seen = true;
    watch_.preamble_us = now;
    return false;
  }
  return SX1281Bus::Elapsed(now, watch_.preamble_us) >= STUCK_US;
}

inline bool SX1281::PacketStatusBad()
{
  using namespace SX1281Regs;
  // 勘误 16.3：GFSK、FLRC 的 RxDone 不滤掉 CRC 错的包，看错误字节（packetStatus[23:16]，
  // 表 11-68）。
  // Errata 16.3: in GFSK and FLRC, RxDone does not filter packets with CRC errors; check
  // the error byte (packetStatus[23:16], table 11-68).
  if (!Modem().errors_in_packet_status)
  {
    return false;
  }
  uint8_t status[5] = {};
  return !bus_.ReadCommand(CMD_GET_PACKET_STATUS, status, sizeof(status)) ||
         (status[2] & PACKET_ERRORS) != 0;
}
