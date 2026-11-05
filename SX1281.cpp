#include "SX1281.hpp"

#include <algorithm>
#include <cstring>

#include "timebase.hpp"

using namespace SX1281Regs;

SX1281::SX1281(LibXR::SPI& spi, LibXR::GPIO& nss, LibXR::GPIO& busy, LibXR::GPIO& nreset,
               LibXR::GPIO& dio1, LibXR::GPIO& txen, LibXR::GPIO& rxen,
               LibXR::GPIO* dcdcen)
    : bus_(spi, nss, busy),
      nreset_(nreset),
      dio1_(dio1),
      txen_(txen),
      rxen_(rxen),
      dcdcen_(dcdcen)
{
  ConfigurePins();
}

LibXR::ErrorCode SX1281::Start(const Config& config)
{
  config_ = config;
  return Restart() ? LibXR::ErrorCode::OK : LibXR::ErrorCode::FAILED;
}

uint8_t SX1281::MaxPayload() const
{
  return config_.modulation == Modulation::FLRC ? FLRC_MAX_PAYLOAD : MAX_PAYLOAD_SIZE;
}

void SX1281::PrepareSend()
{
  if (sending_ || prepared_ || recovery_ != Recovery::NONE)
  {
    return;
  }
  // 10.8：FS 到发射 27 us；11.6.4：SetTx 前清 IRQ。
  // 10.8: FS to Tx takes 27 us; 11.6.4: IRQs are cleared before SetTx.
  bus_.WriteCommand(CMD_SET_FS, nullptr, 0);
  bus_.ClearIrq();
  prepared_ = true;
}

LibXR::ErrorCode SX1281::Send(const uint8_t* data, uint8_t size)
{
  if (data == nullptr || size == 0 || size > MaxPayload())
  {
    return LibXR::ErrorCode::ARG_ERR;
  }
  if (recovery_ != Recovery::NONE)
  {
    return LibXR::ErrorCode::FAILED;
  }
  if (sending_)
  {
    const TxState state = CheckTx();
    if (state == TxState::ON_AIR)
    {
      return LibXR::ErrorCode::BUSY;
    }
    // 发完后 AutoFs 停在 FS；超时的包可能还在发射，先回到 FS。
    // After TxDone, AutoFs stays in FS; a timed-out packet may still be on air, so go
    // back to FS first.
    sending_ = false;
    if (state == TxState::TIMED_OUT)
    {
      bus_.WriteCommand(CMD_SET_FS, nullptr, 0);
    }
    bus_.ClearIrq();
  }
  else
  {
    PrepareSend();
  }
  prepared_ = false;
  SetRfSwitch(true);
  const bool ok = bus_.WriteBuffer(TX_BASE, data, size) && SetPacketParams(size) &&
                  bus_.WriteCommand(CMD_SET_TX, TX_NO_TIMEOUT, sizeof(TX_NO_TIMEOUT));
  sending_ = true;
  tx_started_us_ = SX1281Bus::NowUs();
  if (!ok)
  {
    tx_errors_++;
    FinishSend();
    return LibXR::ErrorCode::FAILED;
  }
  return LibXR::ErrorCode::OK;
}

void SX1281::Poll()
{
  if (recovery_ != Recovery::NONE)
  {
    AdvanceRecovery();
    return;
  }
  if (sending_)
  {
    const TxState state = CheckTx();
    if (state == TxState::ON_AIR)
    {
      return;
    }
    if (state == TxState::TIMED_OUT)
    {
      bus_.WriteCommand(CMD_SET_FS, nullptr, 0);  // 可能还在发射 / it may still be on air
    }
    bus_.ClearIrq();
    FinishSend();
    return;
  }
  if (prepared_)
  {
    // 准备了发送却没有发：重新接收。 / Prepared for a send that did not come: listen
    // again.
    prepared_ = false;
    EnterRx();
    return;
  }
  if (!dio1_.Read())
  {
    Recover();
  }
}

int SX1281::Receive(uint8_t* out)
{
  if (sending_ || prepared_ || recovery_ != Recovery::NONE || !dio1_.Read())
  {
    return 0;
  }
  heard_us_ = rearmed_us_ = SX1281Bus::NowUs();
  arriving_ = false;
  // 下一包可能紧跟着到，少一条命令就早一点读到缓冲：GFSK、FLRC 的错误看包状态，只有
  // LoRa 读 IRQ。
  // The next packet may follow right behind, so every command saved reaches the buffer
  // sooner: GFSK and FLRC errors come from the packet status, only LoRa reads the IRQs.
  const uint16_t irq = config_.modulation == Modulation::LORA ? bus_.GetIrqStatus() : 0;
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

LibXR::ErrorCode SX1281::Tune(uint32_t frequency_hz)
{
  if (sending_)
  {
    return LibXR::ErrorCode::BUSY;
  }
  prepared_ = false;
  config_.frequency_hz = frequency_hz;
  if (recovery_ != Recovery::NONE)
  {
    return LibXR::ErrorCode::OK;  // 复位完成时用新频点配置 / used when the reset ends
  }
  const uint8_t xosc = STANDBY_XOSC;
  const bool ok =
      bus_.WriteCommand(CMD_SET_STANDBY, &xosc, 1) && bus_.SetRfFrequency(frequency_hz);
  bus_.ClearIrq();
  EnterRx();
  return ok ? LibXR::ErrorCode::OK : LibXR::ErrorCode::FAILED;
}

int16_t SX1281::RssiInstant()
{
  uint8_t value = 0;
  if (sending_ || prepared_ || recovery_ != Recovery::NONE ||
      !bus_.ReadCommand(CMD_GET_RSSI_INST, &value, 1))
  {
    return 0;
  }
  return static_cast<int16_t>(-static_cast<int16_t>(value));
}

const SX1281Modem& SX1281::Modem() const
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

uint32_t SX1281::TxTimeoutUs() const
{
  return config_.modulation == Modulation::LORA ? LORA_TX_TIMEOUT_US : TX_TIMEOUT_US;
}

void SX1281::ConfigurePins()
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

// 复位芯片并配置，阻塞约 22 ms（Start()）。 / Reset and configure the chip, blocking
// for about 22 ms (Start()).
bool SX1281::Restart()
{
  HoldReset();
  LibXR::Timebase::DelayMicroseconds(RESET_US);
  nreset_.Write(true);
  LibXR::Timebase::DelayMicroseconds(WAKE_US);
  return Configure();
}

void SX1281::HoldReset()
{
  recovery_ = Recovery::NONE;
  sending_ = false;
  prepared_ = false;
  SetRfSwitch(false);
  rxen_.Write(false);
  nreset_.Write(false);
}

// 推进不阻塞的复位：保持复位 RESET_US，等芯片醒来 WAKE_US，再配置。
// Advance a reset that does not block: hold the reset for RESET_US, wait WAKE_US for
// the chip to wake, then configure it.
void SX1281::AdvanceRecovery()
{
  if (recovery_ == Recovery::STOPPED)
  {
    return;
  }
  const uint32_t now = SX1281Bus::NowUs();
  if (recovery_ == Recovery::IN_RESET)
  {
    if (SX1281Bus::Elapsed(now, recovery_us_) >= RESET_US)
    {
      nreset_.Write(true);
      recovery_ = Recovery::WAKING;
      recovery_us_ = now;
    }
    return;
  }
  if (SX1281Bus::Elapsed(now, recovery_us_) >= WAKE_US)
  {
    recovery_ = Recovery::NONE;
    (void)Configure();  // 失败时静默超时后再复位 / a failure resets again later
  }
}

bool SX1281::Configure()
{
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
  if (ok && config_.modulation != Modulation::LORA)
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
  heard_us_ = SX1281Bus::NowUs();
  return ok;
}

// 发送中的包是否结束；结束时计数一次，调用方随后结束这次发送。
// Whether the packet on air has ended; an end is counted once, and the caller then
// ends this transmission.
SX1281::TxState SX1281::CheckTx()
{
  if (dio1_.Read())
  {
    tx_packets_++;
    return TxState::DONE;
  }
  if (SX1281Bus::Elapsed(SX1281Bus::NowUs(), tx_started_us_) < TxTimeoutUs())
  {
    return TxState::ON_AIR;
  }
  tx_errors_++;
  return TxState::TIMED_OUT;
}

void SX1281::FinishSend()
{
  sending_ = false;
  // 接收接受不超过最大值的任意长度（变长模式滤掉更长的包）。
  // Receive accepts any size up to the maximum (variable length drops longer packets).
  SetPacketParams(MaxPayload());
  EnterRx();
}

void SX1281::EnterRx()
{
  // 11.6.5：调用方在 SetRx 前清 IRQ。 / 11.6.5: the caller clears IRQs before SetRx.
  SetRfSwitch(false);
  bus_.WriteCommand(CMD_SET_RX, RX_CONTINUOUS, sizeof(RX_CONTINUOUS));
  preamble_seen_ = false;
  arriving_ = false;
  // 停顿从这里算起：发送之后应答还在路上。 / Silence counts from here: right after a
  // transmission the answer is still on its way.
  rearmed_us_ = SX1281Bus::NowUs();
}

void SX1281::Recover()
{
  const uint32_t now = SX1281Bus::NowUs();
  if (SX1281Bus::Elapsed(now, heard_us_) >= config_.rx_restart_ms * 1000U)
  {
    restarts_++;
    HoldReset();
    recovery_ = Recovery::IN_RESET;
    recovery_us_ = now;
  }
  else if ((SX1281Bus::Elapsed(now, rearmed_us_) >= config_.rx_rearm_ms * 1000U &&
            !PacketArriving(now)) ||
           PreambleStuck(now))
  {
    rearms_++;
    const uint8_t xosc = STANDBY_XOSC;
    bus_.WriteCommand(CMD_SET_STANDBY, &xosc, 1);
    bus_.ClearIrq();
    EnterRx();
  }
}

// 重新进入接收会截断正在到达的包。到达的时间不会超过一个最长包。 / Entering receive
// again would cut a packet that is arriving, which takes at most one longest packet.
bool SX1281::PacketArriving(uint32_t now)
{
  if ((bus_.GetIrqStatus() & IRQ_ARRIVING) == 0)
  {
    arriving_ = false;
    return false;
  }
  if (!arriving_)
  {
    arriving_ = true;
    arriving_us_ = now;
  }
  return SX1281Bus::Elapsed(now, arriving_us_) < TxTimeoutUs();
}

bool SX1281::PreambleStuck(uint32_t now)
{
  // LoRa 没有这种停顿；它的包也远长于 2 ms。 / LoRa has no such stall, and its packets
  // are far longer than 2 ms.
  if (config_.modulation == Modulation::LORA ||
      SX1281Bus::Elapsed(now, checked_us_) < STUCK_CHECK_US)
  {
    return false;
  }
  checked_us_ = now;
  const uint16_t irq = bus_.GetIrqStatus();
  if ((irq & IRQ_PREAMBLE_DETECTED) == 0 ||
      (irq & (IRQ_SYNC_WORD_VALID | IRQ_RX_DONE)) != 0)
  {
    preamble_seen_ = false;
    return false;
  }
  if (!preamble_seen_)
  {
    preamble_seen_ = true;
    preamble_us_ = now;
    return false;
  }
  return SX1281Bus::Elapsed(now, preamble_us_) >= STUCK_US;
}

bool SX1281::PacketStatusBad()
{
  // 勘误 16.3：GFSK、FLRC 的 RxDone 不滤掉 CRC
  // 错的包，看错误字节（packetStatus[23:16]， 表 11-68）。LoRa 的包状态是 RSSI /
  // SNR，错误看 IRQ。 Errata 16.3: in GFSK and FLRC, RxDone does not filter packets
  // with CRC errors; check the error byte (packetStatus[23:16], table 11-68). In LoRa
  // the packet status is RSSI / SNR, and errors come from the IRQs.
  if (config_.modulation == Modulation::LORA)
  {
    return false;
  }
  uint8_t status[5] = {};
  return !bus_.ReadCommand(CMD_GET_PACKET_STATUS, status, sizeof(status)) ||
         (status[2] & PACKET_ERRORS) != 0;
}

void SX1281::SetRfSwitch(bool transmit)
{
  rxen_.Write(!transmit);
  txen_.Write(transmit);
}

bool SX1281::WriteSyncWord()
{
  // GFSK：同步字 1 的第 5 字节（0x09CE）为 0，4 字节同步字从 0x09CF 开始；FLRC 只用
  // 0x09CF..0x09D2。
  // GFSK: byte 5 of sync word 1 (0x09CE) is 0 and the 4-byte word starts at 0x09CF;
  // FLRC uses 0x09CF..0x09D2 only.
  const uint8_t gfsk[5] = {0x00, config_.sync_word[0], config_.sync_word[1],
                           config_.sync_word[2], config_.sync_word[3]};
  return config_.modulation == Modulation::GFSK
             ? bus_.WriteRegisters(REG_SYNC_WORD_1, gfsk, sizeof(gfsk))
             : bus_.WriteRegisters(REG_SYNC_WORD_1 + 1, config_.sync_word,
                              sizeof(config_.sync_word));
}

bool SX1281::SetPacketParams(uint8_t payload_size)
{
  uint8_t params[7];
  std::memcpy(params, Modem().packet, sizeof(params));
  // LoRa 的载荷长度在第 3 字节，GFSK / FLRC 在第 5 字节。
  // The payload size is byte 3 in LoRa and byte 5 in GFSK / FLRC.
  params[config_.modulation == Modulation::LORA ? 2 : 4] = payload_size;
  return bus_.WriteCommand(CMD_SET_PACKET_PARAMS, params, sizeof(params));
}
