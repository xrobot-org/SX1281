#include "sx1281_bus.hpp"

#include <cstring>

#include "timebase.hpp"

using namespace SX1281Regs;

SX1281Bus::SX1281Bus(LibXR::SPI& spi, LibXR::GPIO& nss, LibXR::GPIO& busy)
    : spi_(spi), nss_(nss), busy_(busy), spi_op_(spi_status_)
{
  nss_.SetConfig({.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                  .pull = LibXR::GPIO::Pull::UP});
  nss_.Write(true);
  busy_.SetConfig(
      {.direction = LibXR::GPIO::Direction::INPUT, .pull = LibXR::GPIO::Pull::NONE});

  LibXR::SPI::Configuration spi_config;
  spi_config.clock_polarity = LibXR::SPI::ClockPolarity::LOW;
  spi_config.clock_phase = LibXR::SPI::ClockPhase::EDGE_1;
  spi_config.prescaler = spi_.CalcPrescaler(18000000UL, 1000000UL, true);
  spi_config.double_buffer = false;
  if (spi_config.prescaler == LibXR::SPI::Prescaler::UNKNOWN)
  {
    spi_config.prescaler = LibXR::SPI::Prescaler::DIV_4;
  }
  spi_.SetConfig(spi_config);
}

uint32_t SX1281Bus::NowUs()
{
  return static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
}

bool SX1281Bus::WaitNotBusy()
{
  // 9.1：BUSY 为低时才能开始命令。 / 9.1: a command may start only while BUSY is low.
  if (!busy_.Read())
  {
    return true;
  }
  const uint32_t start = NowUs();
  while (busy_.Read())
  {
    if (Elapsed(NowUs(), start) >= BUSY_TIMEOUT_US)
    {
      return false;
    }
  }
  return true;
}

bool SX1281Bus::Exchange(size_t size)
{
  if (size == 0 || size > MAX_TRANSFER_SIZE || !WaitNotBusy())
  {
    return false;
  }
  spi_status_.store(LibXR::SPI::OperationRW::OperationPollingStatus::RUNNING,
                    std::memory_order_relaxed);
  nss_.Write(false);
  const auto ans = spi_.ReadAndWrite(LibXR::RawData(rx_, size),
                                     LibXR::ConstRawData(tx_, size), spi_op_);
  bool ok = ans == LibXR::ErrorCode::OK;
  if (ok)
  {
    // 轮询传输返回时已经完成，这时不读时间。 / A polled transfer is done when it
    // returns; the time is read only otherwise.
    auto status = spi_status_.load(std::memory_order_acquire);
    if (status == LibXR::SPI::OperationRW::OperationPollingStatus::RUNNING)
    {
      const uint32_t start = NowUs();
      while (status == LibXR::SPI::OperationRW::OperationPollingStatus::RUNNING &&
             Elapsed(NowUs(), start) < SPI_TIMEOUT_US)
      {
        status = spi_status_.load(std::memory_order_acquire);
      }
    }
    ok = status == LibXR::SPI::OperationRW::OperationPollingStatus::DONE;
  }
  nss_.Write(true);
  return ok;
}

bool SX1281Bus::WriteCommand(uint8_t command, const uint8_t* params, size_t size)
{
  tx_[0] = command;
  if (size > 0)
  {
    std::memcpy(&tx_[1], params, size);
  }
  return Exchange(size + 1);
}

bool SX1281Bus::ReadCommand(uint8_t command, uint8_t* out, size_t size)
{
  tx_[0] = command;
  std::memset(&tx_[1], 0, size + 1);
  if (!Exchange(size + 2))
  {
    return false;
  }
  std::memcpy(out, &rx_[2], size);
  return true;
}

bool SX1281Bus::WriteRegisters(uint16_t address, const uint8_t* data, size_t size)
{
  tx_[0] = CMD_WRITE_REGISTER;
  tx_[1] = static_cast<uint8_t>(address >> 8);
  tx_[2] = static_cast<uint8_t>(address);
  std::memcpy(&tx_[3], data, size);
  return Exchange(size + 3);
}

bool SX1281Bus::WriteRegister16(uint16_t address, uint16_t value)
{
  const uint8_t data[2] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
  return WriteRegisters(address, data, sizeof(data));
}

bool SX1281Bus::WriteBuffer(uint8_t offset, const uint8_t* data, uint8_t size)
{
  tx_[0] = CMD_WRITE_BUFFER;
  tx_[1] = offset;
  std::memcpy(&tx_[2], data, size);
  return Exchange(static_cast<size_t>(size) + 2);
}

bool SX1281Bus::ReadBuffer(uint8_t offset, uint8_t* out, uint8_t size)
{
  tx_[0] = CMD_READ_BUFFER;
  tx_[1] = offset;
  std::memset(&tx_[2], 0, static_cast<size_t>(size) + 1);
  if (!Exchange(static_cast<size_t>(size) + 3))
  {
    return false;
  }
  std::memcpy(out, &rx_[3], size);
  return true;
}

bool SX1281Bus::SetRfFrequency(uint32_t frequency_hz)
{
  const uint32_t reg =
      static_cast<uint32_t>((static_cast<uint64_t>(frequency_hz) << 18) / XTAL_HZ);
  const uint8_t params[3] = {static_cast<uint8_t>(reg >> 16),
                             static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg)};
  return WriteCommand(CMD_SET_RF_FREQUENCY, params, sizeof(params));
}

uint16_t SX1281Bus::GetIrqStatus()
{
  uint8_t status[2] = {};
  if (!ReadCommand(CMD_GET_IRQ_STATUS, status, sizeof(status)))
  {
    return 0;
  }
  return static_cast<uint16_t>((static_cast<uint16_t>(status[0]) << 8) | status[1]);
}

void SX1281Bus::ClearIrq()
{
  const uint8_t all[2] = {0xFF, 0xFF};
  WriteCommand(CMD_CLEAR_IRQ_STATUS, all, sizeof(all));
}
