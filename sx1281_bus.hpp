#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "gpio.hpp"
#include "spi.hpp"
#include "sx1281_regs.hpp"

/**
 * @brief SX1281 的命令总线：NSS、BUSY 和 SPI 上的命令、寄存器与缓冲区访问。
 *        Command bus of the SX1281: commands, registers and the buffer over NSS, BUSY and
 *        SPI.
 *
 * 等待 BUSY 时忙等，最长几毫秒。返回 `false` 表示 SPI 或 BUSY 超时。
 * Waits for BUSY spin, for a few milliseconds at most. `false` means an SPI or BUSY
 * timeout.
 */
class SX1281Bus
{
 public:
  /// 配置 NSS、BUSY 和 SPI / Configure NSS, BUSY and the SPI.
  SX1281Bus(LibXR::SPI& spi, LibXR::GPIO& nss, LibXR::GPIO& busy);

  bool WriteCommand(uint8_t command, const uint8_t* params, size_t size);
  bool ReadCommand(uint8_t command, uint8_t* out, size_t size);
  bool WriteRegisters(uint16_t address, const uint8_t* data, size_t size);
  bool WriteRegister16(uint16_t address, uint16_t value);
  bool WriteBuffer(uint8_t offset, const uint8_t* data, uint8_t size);
  bool ReadBuffer(uint8_t offset, uint8_t* out, uint8_t size);
  bool SetRfFrequency(uint32_t frequency_hz);

  /// 中断状态；超时时为 0 / Interrupt status; 0 on a timeout.
  uint16_t GetIrqStatus();
  void ClearIrq();

  static uint32_t NowUs();
  static uint32_t Elapsed(uint32_t now, uint32_t then) { return now - then; }

 private:
  static constexpr uint32_t BUSY_TIMEOUT_US = 20000;
  static constexpr uint32_t SPI_TIMEOUT_US = 20000;
  static constexpr size_t MAX_TRANSFER_SIZE = 4 + SX1281Regs::MAX_PAYLOAD_SIZE;

  bool WaitNotBusy();
  bool Exchange(size_t size);

  LibXR::SPI& spi_;
  LibXR::GPIO& nss_;
  LibXR::GPIO& busy_;
  std::atomic<LibXR::SPI::OperationRW::OperationPollingStatus> spi_status_{
      LibXR::SPI::OperationRW::OperationPollingStatus::READY};
  LibXR::SPI::OperationRW spi_op_;
  uint8_t tx_[MAX_TRANSFER_SIZE] = {};
  uint8_t rx_[MAX_TRANSFER_SIZE] = {};
};
