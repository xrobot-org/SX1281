#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Semtech SX1281 / SX1280 2.4 GHz LoRa 收发器驱动模块（SPI） / Driver module for the Semtech SX1281 / SX1280 2.4 GHz LoRa transceiver over SPI
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>

#include "gpio.hpp"
#include "libxr_def.hpp"
#include "mpmc_queue.hpp"
#include "semaphore.hpp"
#include "spi.hpp"
#include "thread.hpp"

/**
 * @brief SX1281 / SX1280 2.4 GHz LoRa 收发器驱动，提供基于固定包池的分组收发接口。
 *        Driver for the SX1281 / SX1280 2.4 GHz LoRa transceiver, with a packet
 *        send/receive API backed by a fixed packet pool.
 */
class SX1281
{
 public:
  static constexpr uint8_t MAX_PAYLOAD_SIZE = 255;   ///< 最大载荷长度 Max payload size
  static constexpr size_t PACKET_POOL_CAPACITY = 8;  ///< 包池容量 Pool capacity
  /// 默认包池大小 Default packet pool size
  static constexpr size_t DEFAULT_PACKET_POOL_SIZE = PACKET_POOL_CAPACITY;

  /**
   * @brief 收发的数据包，取自包池。
   *        Data packet taken from the packet pool.
   */
  struct Packet
  {
    uint8_t length = 0;                   ///< 载荷长度，字节 Payload length, bytes
    uint8_t data[MAX_PAYLOAD_SIZE] = {};  ///< 载荷数据 Payload data
  };

  /**
   * @brief 构造配置。
   *        Construction configuration.
   */
  struct Config
  {
    uint32_t frequency_hz = 2404000000UL;  ///< 射频频率，Hz RF frequency, Hz
    int8_t tx_power_dbm = 13;              ///< 发射功率，限制在 -18 到 13 dBm
    ///< TX power, clamped to -18..13 dBm
    uint16_t rx_timeout_ms = 1000;  ///< 接收超时，1 ms 步长
    ///< RX timeout in 1 ms steps
    uint16_t tx_timeout_ms = 3000;  ///< 发送超时，1 ms 步长
    ///< TX timeout in 1 ms steps
    uint16_t auto_tx_period_ms = 500;  ///< 自动发送周期，ms Auto TX period, ms
    bool auto_tx_enabled = true;       ///< 是否周期发送 ashining 信标
    ///< Whether the ashining beacon is sent periodically
    size_t irq_task_stack_depth = 2048;  ///< 工作线程栈深 Worker thread stack depth
    size_t packet_pool_size = DEFAULT_PACKET_POOL_SIZE;  ///< 包池包数，1 到 8
    ///< Packets in the pool, 1..8
    size_t tx_queue_length = 4;  ///< 发送队列长度，大于 0
    ///< TX queue length, greater than 0
    size_t rx_queue_length = 4;  ///< 接收队列长度，大于 0
    ///< RX queue length, greater than 0
  };

  /**
   * @brief 构造 SX1281：配置引脚与 SPI，复位并初始化收发器进入接收，创建 IRQ 工作线程。
   *        Construct SX1281: configure the pins and the SPI, reset and initialize the
   *        radio into RX, and create the IRQ worker thread.
   *
   * @param spi 连接收发器的 SPI 总线。
   *            SPI bus connected to the radio.
   * @param nss SPI 片选 GPIO，输出，低电平有效。
   *            SPI chip-select GPIO, output, active low.
   * @param dio1 DIO1 中断线，配置为上升沿中断。
   *             DIO1 IRQ line, configured as a rising-edge interrupt.
   * @param dio2 DIO2，配置为输入。
   *             DIO2, configured as an input.
   * @param dio3 DIO3，配置为输入。
   *             DIO3, configured as an input.
   * @param paen 射频前端 PA 使能 GPIO。
   *             PA enable GPIO of the RF front end.
   * @param lnaen 射频前端 LNA 使能 GPIO。
   *              LNA enable GPIO of the RF front end.
   * @param dcdcen DC-DC 使能 GPIO，启动时置高。
   *               DC-DC enable GPIO, driven high at start-up.
   * @param busy 收发器 BUSY 输入。
   *             BUSY input of the radio.
   * @param nreset 收发器 NRESET 输出。
   *               NRESET output of the radio.
   * @param config 构造配置。
   *               Construction configuration.
   */
  SX1281(
      LibXR::SPI& spi,
      LibXR::GPIO& nss,
      LibXR::GPIO& dio1,
      LibXR::GPIO& dio2,
      LibXR::GPIO& dio3,
      LibXR::GPIO& paen,
      LibXR::GPIO& lnaen,
      LibXR::GPIO& dcdcen,
      LibXR::GPIO& busy,
      LibXR::GPIO& nreset,
      Config config = {.frequency_hz = 2404000000, .tx_power_dbm = 13, .rx_timeout_ms = 1000, .tx_timeout_ms = 3000, .auto_tx_period_ms = 500, .auto_tx_enabled = true, .irq_task_stack_depth = 2048, .packet_pool_size = 8, .tx_queue_length = 4, .rx_queue_length = 4})
      : spi_(std::addressof(spi)),
        nss_(std::addressof(nss)),
        dio1_(std::addressof(dio1)),
        dio2_(std::addressof(dio2)),
        dio3_(std::addressof(dio3)),
        paen_(std::addressof(paen)),
        lnaen_(std::addressof(lnaen)),
        dcdcen_(std::addressof(dcdcen)),
        busy_(std::addressof(busy)),
        nreset_(std::addressof(nreset)),
        spi_op_(spi_sem_, SPI_TIMEOUT_MS),
        free_queue_(SanitizePoolSize(config.packet_pool_size)),
        tx_queue_(SanitizeQueueLength(config.tx_queue_length)),
        rx_queue_(SanitizeQueueLength(config.rx_queue_length)),
        config_(config)
  {
    ASSERT(config_.packet_pool_size > 0 &&
           config_.packet_pool_size <= PACKET_POOL_CAPACITY);
    ASSERT(config_.tx_queue_length > 0);
    ASSERT(config_.rx_queue_length > 0);

    config_.packet_pool_size = SanitizePoolSize(config_.packet_pool_size);
    config_.tx_queue_length = SanitizeQueueLength(config_.tx_queue_length);
    config_.rx_queue_length = SanitizeQueueLength(config_.rx_queue_length);

    PrimePacketPool();

    ConfigurePins();
    ConfigureDio1Interrupt();

    state_ = State::STANDBY;
    SetRfSwitch(State::STANDBY);
    Reset();

    uint8_t id[2] = {};
    ASSERT(ReadRegisters(REG_CHIP_ID, id, sizeof(id)));
    chip_id_ = static_cast<uint16_t>((static_cast<uint16_t>(id[0]) << 8) | id[1]);
    ASSERT(chip_id_ != 0x0000 && chip_id_ != 0xFFFF);

    const uint8_t regulator = REGULATOR_DCDC;
    ASSERT(WriteCommand(CMD_SET_REGULATOR_MODE, &regulator, 1));
    ASSERT(SetStandby(STANDBY_RC));
    ASSERT(SetPacketType(PACKET_TYPE_LORA));
    ASSERT(SetModulationParams());
    ASSERT(SetPacketParams(MAX_PAYLOAD_SIZE));
    ASSERT(SetXoscCap(0x11));
    ASSERT(SetRfFrequency(config_.frequency_hz));
    ASSERT(SetBufferBaseAddress(0, 0));
    ASSERT(SetTxParams(config_.tx_power_dbm, RAMP_02_US));
    ASSERT(ClearIrqStatus(IRQ_RADIO_ALL));
    ASSERT(EnterRx());

    ready_ = true;
    last_auto_tx_ms_ = LibXR::Thread::GetTime();
    irq_thread_.Create(this, ThreadFunc, "sx1281_irq", config_.irq_task_stack_depth,
                       LibXR::Thread::Priority::HIGH);
  }

  /**
   * @brief 从包池取出一个空包，包池为空时立即返回 nullptr。
   *        Take an empty packet from the pool; returns nullptr immediately when the
   *        pool is empty.
   *
   * @param timeout_ms 保留参数，目前未使用。
   *                   Reserved, currently unused.
   * @return 取得的包，所有权属于调用方；包池为空时为 nullptr。
   *         The packet, owned by the caller; nullptr when the pool is empty.
   */
  Packet* AllocatePacket(uint32_t timeout_ms = 0)
  {
    UNUSED(timeout_ms);
    Packet* packet = nullptr;
    if (free_queue_.Pop(packet) != LibXR::ErrorCode::OK)
    {
      return nullptr;
    }

    if (packet == nullptr || !PacketFromPool(packet) ||
        !PacketOwnerIs(packet, PacketOwner::FREE))
    {
      ASSERT(false);
      return nullptr;
    }

    SetPacketOwner(packet, PacketOwner::USER);
    packet->length = 0;
    return packet;
  }

  /**
   * @brief 把包加入发送队列，并唤醒工作线程。
   *        Queue a packet for transmission and wake the worker thread.
   *
   * @param packet 取自 AllocatePacket() 的包，length 须在 1 到 255 之间。
   *               Packet obtained from AllocatePacket(), with length from 1 to 255.
   * @return 入队成功为 true，此时包的所有权转移给驱动；length 无效或发送队列已满时为
   *         false，包仍属于调用方。
   *         true when queued, in which case ownership moves to the driver; false for
   *         an invalid length or a full TX queue, in which case the caller keeps the
   *         packet.
   */
  bool Send(Packet* packet)
  {
    if (packet == nullptr || packet->length == 0 || packet->length > MAX_PAYLOAD_SIZE)
    {
      return false;
    }

    if (!PacketFromPool(packet) || !PacketOwnerIs(packet, PacketOwner::USER))
    {
      ASSERT(false);
      return false;
    }

    SetPacketOwner(packet, PacketOwner::TX_QUEUE);
    if (tx_queue_.Push(packet) != LibXR::ErrorCode::OK)
    {
      SetPacketOwner(packet, PacketOwner::USER);
      tx_dropped_++;
      return false;
    }

    irq_sem_.Post();
    return true;
  }

  /**
   * @brief 取出一个已收到的包，没有排队的包时立即返回 nullptr。
   *        Take a received packet; returns nullptr immediately when none is queued.
   *
   * @param timeout_ms 保留参数，目前未使用。
   *                   Reserved, currently unused.
   * @return 收到的包，处理后须调用 Release() 归还；没有包时为 nullptr。
   *         The received packet, to be returned with Release() after processing;
   *         nullptr when none is available.
   */
  Packet* Receive(uint32_t timeout_ms = 0)
  {
    UNUSED(timeout_ms);
    Packet* packet = nullptr;
    if (rx_queue_.Pop(packet) != LibXR::ErrorCode::OK)
    {
      return nullptr;
    }

    if (packet == nullptr || !PacketFromPool(packet) ||
        !PacketOwnerIs(packet, PacketOwner::RX_QUEUE))
    {
      ASSERT(false);
      return nullptr;
    }

    SetPacketOwner(packet, PacketOwner::USER);
    return packet;
  }

  /**
   * @brief 把 AllocatePacket() 或 Receive() 得到的包归还包池。
   *        Return a packet obtained from AllocatePacket() or Receive() to the pool.
   *
   * @param packet 待归还的包。
   *               Packet to return.
   */
  void Release(Packet* packet)
  {
    if (packet == nullptr)
    {
      return;
    }

    if (!PacketFromPool(packet) || !PacketOwnerIs(packet, PacketOwner::USER))
    {
      ASSERT(false);
      return;
    }

    ReturnToPool(packet);
  }

  /// 芯片 ID Chip ID
  uint16_t ChipId() const { return chip_id_; }
  /// 初始化已完成 Initialization finished
  bool Ready() const { return ready_; }
  /// 成功接收的包数 Packets received successfully
  uint32_t RxPackets() const { return rx_packets_; }
  /// 发送完成的包数 Packets transmitted
  uint32_t TxPackets() const { return tx_packets_; }
  /// 因包池为空或接收队列已满而丢弃的接收帧数
  /// Received frames dropped because the pool was empty or the RX queue was full
  uint32_t RxDropped() const { return rx_dropped_; }
  /// 因包池为空或发送队列已满而丢弃的发送数
  /// Transmissions dropped because the pool was empty or the TX queue was full
  uint32_t TxDropped() const { return tx_dropped_; }
  /// 发送失败或超时的包数 Packets that failed or timed out in TX
  uint32_t TxErrors() const { return tx_errors_; }
  /// 接收队列中的包数 Packets in the RX queue
  size_t RxQueued() { return rx_queue_.Size(); }
  /// 发送队列中的包数 Packets in the TX queue
  size_t TxQueued() { return tx_queue_.Size(); }
  /// 包池中的空闲包数 Free packets in the pool
  size_t FreePackets() { return free_queue_.Size(); }
  /// 包池大小 Packet pool size
  size_t PacketPoolSize() const { return config_.packet_pool_size; }

 private:
  enum class State : uint8_t
  {
    STANDBY,
    RX,
    TX
  };

  enum class PacketOwner : uint8_t
  {
    FREE,
    USER,
    TX_QUEUE,
    TX_ACTIVE,
    RX_QUEUE
  };

  static constexpr uint32_t XTAL_HZ = 52000000UL;
  static constexpr uint16_t SPI_TIMEOUT_MS = 50;
  static constexpr uint16_t BUSY_TIMEOUT_MS = 100;
  static constexpr size_t MAX_TRANSFER_SIZE = 260;

  static constexpr uint8_t CMD_WRITE_REGISTER = 0x18;
  static constexpr uint8_t CMD_READ_REGISTER = 0x19;
  static constexpr uint8_t CMD_WRITE_BUFFER = 0x1A;
  static constexpr uint8_t CMD_READ_BUFFER = 0x1B;
  static constexpr uint8_t CMD_SET_STANDBY = 0x80;
  static constexpr uint8_t CMD_SET_TX = 0x83;
  static constexpr uint8_t CMD_SET_RX = 0x82;
  static constexpr uint8_t CMD_SET_PACKET_TYPE = 0x8A;
  static constexpr uint8_t CMD_SET_RF_FREQUENCY = 0x86;
  static constexpr uint8_t CMD_SET_TX_PARAMS = 0x8E;
  static constexpr uint8_t CMD_SET_BUFFER_BASE_ADDRESS = 0x8F;
  static constexpr uint8_t CMD_SET_MODULATION_PARAMS = 0x8B;
  static constexpr uint8_t CMD_SET_PACKET_PARAMS = 0x8C;
  static constexpr uint8_t CMD_GET_RX_BUFFER_STATUS = 0x17;
  static constexpr uint8_t CMD_SET_DIO_IRQ_PARAMS = 0x8D;
  static constexpr uint8_t CMD_GET_IRQ_STATUS = 0x15;
  static constexpr uint8_t CMD_CLEAR_IRQ_STATUS = 0x97;
  static constexpr uint8_t CMD_SET_REGULATOR_MODE = 0x96;

  static constexpr uint16_t REG_CHIP_ID = 0x0944;
  static constexpr uint16_t REG_XOSC_CAP_BANK_A = 0x0A0E;
  static constexpr uint16_t REG_XOSC_CAP_BANK_B = 0x0A0F;

  static constexpr uint8_t STANDBY_RC = 0x00;
  static constexpr uint8_t STANDBY_XOSC = 0x01;
  static constexpr uint8_t REGULATOR_DCDC = 0x01;
  static constexpr uint8_t PACKET_TYPE_LORA = 0x01;
  static constexpr uint8_t LORA_SF8 = 0x80;
  static constexpr uint8_t LORA_BW0800 = 0x18;
  static constexpr uint8_t LORA_CR45 = 0x01;
  static constexpr uint8_t LORA_VARIABLE_LENGTH = 0x00;
  static constexpr uint8_t LORA_CRC_ON = 0x20;
  static constexpr uint8_t LORA_IQ_NORMAL = 0x40;
  static constexpr uint8_t RAMP_02_US = 0x00;
  static constexpr uint8_t TICK_1000_US = 0x02;

  static constexpr uint16_t IRQ_RADIO_ALL = 0xFFFF;
  static constexpr uint16_t IRQ_TX_DONE = 0x0001;
  static constexpr uint16_t IRQ_RX_DONE = 0x0002;
  static constexpr uint16_t IRQ_HEADER_ERROR = 0x0020;
  static constexpr uint16_t IRQ_CRC_ERROR = 0x0040;
  static constexpr uint16_t IRQ_RX_TX_TIMEOUT = 0x4000;

  static uint32_t Elapsed(uint32_t now, uint32_t then)
  {
    return static_cast<uint32_t>(now - then);
  }

  static size_t SanitizeQueueLength(size_t length) { return length == 0 ? 1 : length; }

  static size_t SanitizePoolSize(size_t size)
  {
    if (size == 0)
    {
      return 1;
    }

    return std::min(size, PACKET_POOL_CAPACITY);
  }

  static uint16_t MsToTickSteps(uint16_t ms) { return ms == 0 ? 0 : ms; }

  static void ThreadFunc(SX1281* radio) { radio->WorkerLoop(); }

  void PrimePacketPool()
  {
    for (size_t i = 0; i < config_.packet_pool_size; i++)
    {
      packet_pool_[i].length = 0;
      packet_owner_[i] = PacketOwner::FREE;
      ASSERT(free_queue_.Push(&packet_pool_[i]) == LibXR::ErrorCode::OK);
    }
  }

  bool PacketFromPool(const Packet* packet) const
  {
    const auto base = reinterpret_cast<uintptr_t>(&packet_pool_[0]);
    const auto end = reinterpret_cast<uintptr_t>(&packet_pool_[config_.packet_pool_size]);
    const auto ptr = reinterpret_cast<uintptr_t>(packet);

    return ptr >= base && ptr < end && ((ptr - base) % sizeof(Packet)) == 0;
  }

  size_t PacketIndex(const Packet* packet) const
  {
    ASSERT(PacketFromPool(packet));
    const auto base = reinterpret_cast<uintptr_t>(&packet_pool_[0]);
    const auto ptr = reinterpret_cast<uintptr_t>(packet);
    return static_cast<size_t>((ptr - base) / sizeof(Packet));
  }

  bool PacketOwnerIs(const Packet* packet, PacketOwner owner) const
  {
    return packet_owner_[PacketIndex(packet)] == owner;
  }

  void SetPacketOwner(const Packet* packet, PacketOwner owner)
  {
    packet_owner_[PacketIndex(packet)] = owner;
  }

  void ReturnToPool(Packet* packet)
  {
    packet->length = 0;
    SetPacketOwner(packet, PacketOwner::FREE);
    ASSERT(free_queue_.Push(packet) == LibXR::ErrorCode::OK);
  }

  void ConfigurePins()
  {
    nss_->SetConfig({LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::UP});
    nss_->Write(true);

    nreset_->SetConfig(
        {LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});
    busy_->SetConfig({LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::NONE});
    dio1_->SetConfig({LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::NONE});
    dio2_->SetConfig({LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::NONE});
    dio3_->SetConfig({LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::NONE});

    paen_->SetConfig({LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});
    lnaen_->SetConfig(
        {LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});
    dcdcen_->SetConfig(
        {LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});

    paen_->Write(false);
    lnaen_->Write(false);
    dcdcen_->Write(true);

    LibXR::SPI::Configuration spi_config;
    spi_config.clock_polarity = LibXR::SPI::ClockPolarity::LOW;
    spi_config.clock_phase = LibXR::SPI::ClockPhase::EDGE_1;
    spi_config.prescaler = spi_->CalcPrescaler(18000000UL, 1000000UL, true);
    spi_config.double_buffer = false;
    if (spi_config.prescaler == LibXR::SPI::Prescaler::UNKNOWN)
    {
      spi_config.prescaler = LibXR::SPI::Prescaler::DIV_4;
    }
    spi_->SetConfig(spi_config);
  }

  void ConfigureDio1Interrupt()
  {
    dio1_->DisableInterrupt();
    dio1_->SetConfig({LibXR::GPIO::Direction::RISING_INTERRUPT, LibXR::GPIO::Pull::NONE});

    auto irq_cb = LibXR::GPIO::Callback::Create(
        [](bool in_isr, SX1281* radio) { radio->irq_sem_.PostFromCallback(in_isr); },
        this);

    dio1_->RegisterCallback(irq_cb);
    dio1_->EnableInterrupt();
  }

  void WorkerLoop()
  {
    while (true)
    {
      if (irq_sem_.Wait(GetWorkerWaitMs()) == LibXR::ErrorCode::OK)
      {
        HandleRadioIrq();
      }

      PeriodicWork();
    }
  }

  uint32_t GetWorkerWaitMs()
  {
    ASSERT(ready_);

    if (state_ != State::TX && tx_queue_.Size() > 0)
    {
      return 0;
    }

    if (!config_.auto_tx_enabled || state_ != State::RX)
    {
      return UINT32_MAX;
    }

    const uint32_t now = LibXR::Thread::GetTime();
    const uint32_t elapsed = Elapsed(now, last_auto_tx_ms_);
    if (elapsed >= config_.auto_tx_period_ms)
    {
      return 0;
    }

    return config_.auto_tx_period_ms - elapsed;
  }

  void PeriodicWork()
  {
    const uint32_t now = LibXR::Thread::GetTime();

    ASSERT(ready_);

    TryStartQueuedTx();

    if (config_.auto_tx_enabled && state_ == State::RX &&
        Elapsed(now, last_auto_tx_ms_) >= config_.auto_tx_period_ms)
    {
      static constexpr uint8_t PING[] = {'a', 's', 'h', 'i', 'n', 'i', 'n', 'g'};
      Packet* packet = AllocatePacket();
      if (packet != nullptr)
      {
        packet->length = sizeof(PING);
        std::memcpy(packet->data, PING, sizeof(PING));
        if (Send(packet))
        {
          last_auto_tx_ms_ = now;
        }
        else
        {
          Release(packet);
        }
      }
      else
      {
        tx_dropped_++;
      }
    }
  }

  void HandleRadioIrq()
  {
    ASSERT(ready_);

    const uint16_t irq = GetIrqStatus();
    if (irq == 0)
    {
      return;
    }

    ClearIrqStatus(IRQ_RADIO_ALL);

    if ((irq & (IRQ_CRC_ERROR | IRQ_HEADER_ERROR)) != 0)
    {
      rx_errors_++;
      ContinueRxOrTx();
    }
    else if ((irq & IRQ_RX_DONE) != 0)
    {
      if (ReadAndQueuePayload())
      {
        rx_packets_++;
      }
      else
      {
        rx_errors_++;
      }
      ContinueRxOrTx();
    }
    else if ((irq & IRQ_TX_DONE) != 0)
    {
      tx_packets_++;
      ReleaseActiveTxPacket();
      ContinueRxOrTx();
    }
    else if ((irq & IRQ_RX_TX_TIMEOUT) != 0)
    {
      if (state_ == State::TX)
      {
        tx_errors_++;
        ReleaseActiveTxPacket();
      }
      ContinueRxOrTx();
    }
  }

  bool TryStartQueuedTx()
  {
    ASSERT(ready_);

    if (state_ == State::TX)
    {
      return false;
    }

    ASSERT(active_tx_packet_ == nullptr);

    Packet* packet = nullptr;
    if (tx_queue_.Pop(packet) != LibXR::ErrorCode::OK)
    {
      return false;
    }

    ASSERT(packet != nullptr);
    ASSERT(PacketFromPool(packet));
    ASSERT(PacketOwnerIs(packet, PacketOwner::TX_QUEUE));

    active_tx_packet_ = packet;
    SetPacketOwner(active_tx_packet_, PacketOwner::TX_ACTIVE);
    if (!StartTx(*packet))
    {
      ReleaseActiveTxPacket();
      tx_errors_++;
      ASSERT(EnterRx());
      ready_ = true;
      return true;
    }

    return true;
  }

  void ContinueRxOrTx()
  {
    state_ = State::STANDBY;
    if (!TryStartQueuedTx())
    {
      ASSERT(EnterRx());
      ready_ = true;
    }
  }

  bool StartTx(const Packet& packet)
  {
    if (packet.length == 0 || packet.length > MAX_PAYLOAD_SIZE)
    {
      return false;
    }

    SetRfSwitch(State::TX);
    if (!SetPacketParams(packet.length))
    {
      return false;
    }
    if (!SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                         0, 0))
    {
      return false;
    }
    if (!WriteBuffer(0, packet.data, packet.length))
    {
      return false;
    }
    if (!SetTx(config_.tx_timeout_ms))
    {
      return false;
    }

    state_ = State::TX;
    return true;
  }

  void Reset()
  {
    nss_->Write(true);
    nreset_->Write(true);
    LibXR::Thread::Sleep(20);
    nreset_->Write(false);
    LibXR::Thread::Sleep(50);
    nreset_->Write(true);
    LibXR::Thread::Sleep(20);
    WaitOnBusy(BUSY_TIMEOUT_MS);
  }

  bool WaitOnBusy(uint16_t timeout_ms)
  {
    const uint32_t start = LibXR::Thread::GetTime();
    while (busy_->Read())
    {
      if (Elapsed(LibXR::Thread::GetTime(), start) >= timeout_ms)
      {
        return false;
      }
      LibXR::Thread::Sleep(1);
    }
    return true;
  }

  void Select() { nss_->Write(false); }
  void Deselect() { nss_->Write(true); }

  bool Exchange(size_t size)
  {
    if (size == 0 || size > MAX_TRANSFER_SIZE)
    {
      return false;
    }
    std::memset(rx_, 0, size);
    Select();
    const auto ans = spi_->ReadAndWrite(LibXR::RawData(rx_, size),
                                        LibXR::ConstRawData(tx_, size), spi_op_);
    Deselect();
    return ans == LibXR::ErrorCode::OK;
  }

  bool WriteCommand(uint8_t command, const uint8_t* buffer, uint16_t size)
  {
    if (!WaitOnBusy(BUSY_TIMEOUT_MS) || static_cast<size_t>(size) + 1 > MAX_TRANSFER_SIZE)
    {
      return false;
    }

    tx_[0] = command;
    if (size > 0)
    {
      std::memcpy(&tx_[1], buffer, size);
    }

    if (!Exchange(static_cast<size_t>(size) + 1))
    {
      return false;
    }
    return WaitOnBusy(BUSY_TIMEOUT_MS);
  }

  bool ReadCommand(uint8_t command, uint8_t* buffer, uint16_t size)
  {
    if (buffer == nullptr || !WaitOnBusy(BUSY_TIMEOUT_MS) ||
        static_cast<size_t>(size) + 2 > MAX_TRANSFER_SIZE)
    {
      return false;
    }

    tx_[0] = command;
    tx_[1] = 0x00;
    std::memset(&tx_[2], 0, size);

    if (!Exchange(static_cast<size_t>(size) + 2))
    {
      return false;
    }
    std::memcpy(buffer, &rx_[2], size);
    return WaitOnBusy(BUSY_TIMEOUT_MS);
  }

  bool WriteRegisters(uint16_t address, const uint8_t* buffer, uint16_t size)
  {
    if (buffer == nullptr || !WaitOnBusy(BUSY_TIMEOUT_MS) ||
        static_cast<size_t>(size) + 3 > MAX_TRANSFER_SIZE)
    {
      return false;
    }

    tx_[0] = CMD_WRITE_REGISTER;
    tx_[1] = static_cast<uint8_t>(address >> 8);
    tx_[2] = static_cast<uint8_t>(address);
    std::memcpy(&tx_[3], buffer, size);

    if (!Exchange(static_cast<size_t>(size) + 3))
    {
      return false;
    }
    return WaitOnBusy(BUSY_TIMEOUT_MS);
  }

  bool ReadRegisters(uint16_t address, uint8_t* buffer, uint16_t size)
  {
    if (buffer == nullptr || !WaitOnBusy(BUSY_TIMEOUT_MS) ||
        static_cast<size_t>(size) + 4 > MAX_TRANSFER_SIZE)
    {
      return false;
    }

    tx_[0] = CMD_READ_REGISTER;
    tx_[1] = static_cast<uint8_t>(address >> 8);
    tx_[2] = static_cast<uint8_t>(address);
    tx_[3] = 0x00;
    std::memset(&tx_[4], 0, size);

    if (!Exchange(static_cast<size_t>(size) + 4))
    {
      return false;
    }
    std::memcpy(buffer, &rx_[4], size);
    return WaitOnBusy(BUSY_TIMEOUT_MS);
  }

  bool WriteRegister(uint16_t address, uint8_t value)
  {
    return WriteRegisters(address, &value, 1);
  }

  bool WriteBuffer(uint8_t offset, const uint8_t* buffer, uint8_t size)
  {
    if (buffer == nullptr || !WaitOnBusy(BUSY_TIMEOUT_MS) ||
        static_cast<size_t>(size) + 2 > MAX_TRANSFER_SIZE)
    {
      return false;
    }

    tx_[0] = CMD_WRITE_BUFFER;
    tx_[1] = offset;
    std::memcpy(&tx_[2], buffer, size);

    if (!Exchange(static_cast<size_t>(size) + 2))
    {
      return false;
    }
    return WaitOnBusy(BUSY_TIMEOUT_MS);
  }

  bool ReadBuffer(uint8_t offset, uint8_t* buffer, uint8_t size)
  {
    if (buffer == nullptr || !WaitOnBusy(BUSY_TIMEOUT_MS) ||
        static_cast<size_t>(size) + 3 > MAX_TRANSFER_SIZE)
    {
      return false;
    }

    tx_[0] = CMD_READ_BUFFER;
    tx_[1] = offset;
    tx_[2] = 0x00;
    std::memset(&tx_[3], 0, size);

    if (!Exchange(static_cast<size_t>(size) + 3))
    {
      return false;
    }
    std::memcpy(buffer, &rx_[3], size);
    return WaitOnBusy(BUSY_TIMEOUT_MS);
  }

  bool SetStandby(uint8_t mode) { return WriteCommand(CMD_SET_STANDBY, &mode, 1); }

  bool SetPacketType(uint8_t packet_type)
  {
    packet_type_ = packet_type;
    return WriteCommand(CMD_SET_PACKET_TYPE, &packet_type, 1);
  }

  bool SetModulationParams()
  {
    const uint8_t params[3] = {LORA_SF8, LORA_BW0800, LORA_CR45};
    return WriteCommand(CMD_SET_MODULATION_PARAMS, params, sizeof(params));
  }

  bool SetPacketParams(uint8_t payload_length)
  {
    const uint8_t params[7] = {
        12, LORA_VARIABLE_LENGTH, payload_length, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0};
    return WriteCommand(CMD_SET_PACKET_PARAMS, params, sizeof(params));
  }

  bool SetRfFrequency(uint32_t frequency_hz)
  {
    const uint32_t reg =
        static_cast<uint32_t>((static_cast<uint64_t>(frequency_hz) << 18) / XTAL_HZ);
    const uint8_t params[3] = {static_cast<uint8_t>(reg >> 16),
                               static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg)};
    return WriteCommand(CMD_SET_RF_FREQUENCY, params, sizeof(params));
  }

  bool SetTxParams(int8_t power_dbm, uint8_t ramp_time)
  {
    power_dbm = std::clamp<int8_t>(power_dbm, -18, 13);
    const uint8_t params[2] = {static_cast<uint8_t>(power_dbm + 18), ramp_time};
    return WriteCommand(CMD_SET_TX_PARAMS, params, sizeof(params));
  }

  bool SetBufferBaseAddress(uint8_t tx_base, uint8_t rx_base)
  {
    const uint8_t params[2] = {tx_base, rx_base};
    return WriteCommand(CMD_SET_BUFFER_BASE_ADDRESS, params, sizeof(params));
  }

  bool SetDioIrqParams(uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask,
                       uint16_t dio3_mask)
  {
    const uint8_t params[8] = {
        static_cast<uint8_t>(irq_mask >> 8),  static_cast<uint8_t>(irq_mask),
        static_cast<uint8_t>(dio1_mask >> 8), static_cast<uint8_t>(dio1_mask),
        static_cast<uint8_t>(dio2_mask >> 8), static_cast<uint8_t>(dio2_mask),
        static_cast<uint8_t>(dio3_mask >> 8), static_cast<uint8_t>(dio3_mask)};
    return WriteCommand(CMD_SET_DIO_IRQ_PARAMS, params, sizeof(params));
  }

  bool SetTx(uint16_t timeout_ms)
  {
    const uint16_t steps = MsToTickSteps(timeout_ms);
    const uint8_t params[3] = {TICK_1000_US, static_cast<uint8_t>(steps >> 8),
                               static_cast<uint8_t>(steps)};
    return WriteCommand(CMD_SET_TX, params, sizeof(params));
  }

  bool SetRx(uint16_t timeout_ms)
  {
    const uint16_t steps = MsToTickSteps(timeout_ms);
    const uint8_t params[3] = {TICK_1000_US, static_cast<uint8_t>(steps >> 8),
                               static_cast<uint8_t>(steps)};
    return WriteCommand(CMD_SET_RX, params, sizeof(params));
  }

  bool SetXoscCap(uint8_t cap_value)
  {
    cap_value &= 0x1F;
    return SetStandby(STANDBY_XOSC) && WriteRegister(REG_XOSC_CAP_BANK_A, cap_value) &&
           WriteRegister(REG_XOSC_CAP_BANK_B, cap_value);
  }

  uint16_t GetIrqStatus()
  {
    uint8_t status[2] = {};
    if (!ReadCommand(CMD_GET_IRQ_STATUS, status, sizeof(status)))
    {
      return 0;
    }
    return static_cast<uint16_t>((static_cast<uint16_t>(status[0]) << 8) | status[1]);
  }

  bool ClearIrqStatus(uint16_t irq)
  {
    const uint8_t params[2] = {static_cast<uint8_t>(irq >> 8), static_cast<uint8_t>(irq)};
    return WriteCommand(CMD_CLEAR_IRQ_STATUS, params, sizeof(params));
  }

  bool EnterRx()
  {
    SetRfSwitch(State::RX);
    if (!SetPacketParams(MAX_PAYLOAD_SIZE))
    {
      return false;
    }
    if (!SetDioIrqParams(
            IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
            IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR, 0, 0))
    {
      return false;
    }
    if (!SetRx(config_.rx_timeout_ms))
    {
      return false;
    }
    state_ = State::RX;
    return true;
  }

  bool ReadAndQueuePayload()
  {
    uint8_t size = 0;
    uint8_t offset = 0;
    if (!ReadPayloadStatus(size, offset))
    {
      return false;
    }

    if (size == 0)
    {
      return true;
    }

    Packet* packet = AllocatePacket();
    if (packet == nullptr)
    {
      rx_dropped_++;
      return true;
    }

    if (!ReadPayload(*packet, offset, size))
    {
      Release(packet);
      return false;
    }

    SetPacketOwner(packet, PacketOwner::RX_QUEUE);
    if (rx_queue_.Push(packet) != LibXR::ErrorCode::OK)
    {
      SetPacketOwner(packet, PacketOwner::USER);
      rx_dropped_++;
      ReturnToPool(packet);
    }

    return true;
  }

  bool ReadPayloadStatus(uint8_t& size, uint8_t& offset)
  {
    uint8_t status[2] = {};
    if (!ReadCommand(CMD_GET_RX_BUFFER_STATUS, status, sizeof(status)))
    {
      return false;
    }

    size = status[0];
    offset = status[1];
    if (size > MAX_PAYLOAD_SIZE)
    {
      size = MAX_PAYLOAD_SIZE;
    }

    return true;
  }

  bool ReadPayload(Packet& packet, uint8_t offset, uint8_t size)
  {
    packet = {};

    if (size == 0)
    {
      return true;
    }

    if (!ReadBuffer(offset, packet.data, size))
    {
      return false;
    }

    packet.length = size;
    return true;
  }

  void ReleaseActiveTxPacket()
  {
    if (active_tx_packet_ == nullptr)
    {
      return;
    }

    ASSERT(PacketOwnerIs(active_tx_packet_, PacketOwner::TX_ACTIVE));
    ReturnToPool(active_tx_packet_);
    active_tx_packet_ = nullptr;
  }

  void SetRfSwitch(State mode)
  {
    switch (mode)
    {
      case State::TX:
        paen_->Write(true);
        lnaen_->Write(false);
        break;
      case State::RX:
        paen_->Write(false);
        lnaen_->Write(true);
        break;
      case State::STANDBY:
      default:
        paen_->Write(false);
        lnaen_->Write(false);
        break;
    }
  }

  LibXR::SPI* spi_;
  LibXR::GPIO* nss_;
  LibXR::GPIO* dio1_;
  LibXR::GPIO* dio2_;
  LibXR::GPIO* dio3_;
  LibXR::GPIO* paen_;
  LibXR::GPIO* lnaen_;
  LibXR::GPIO* dcdcen_;
  LibXR::GPIO* busy_;
  LibXR::GPIO* nreset_;

  LibXR::Semaphore spi_sem_;
  LibXR::Semaphore irq_sem_;
  LibXR::SPI::OperationRW spi_op_;
  Packet packet_pool_[PACKET_POOL_CAPACITY] = {};
  PacketOwner packet_owner_[PACKET_POOL_CAPACITY] = {};
  LibXR::MPMCQueue<Packet*> free_queue_;
  LibXR::MPMCQueue<Packet*> tx_queue_;
  LibXR::MPMCQueue<Packet*> rx_queue_;
  LibXR::Thread irq_thread_;
  Config config_;

  uint8_t tx_[MAX_TRANSFER_SIZE] = {};
  uint8_t rx_[MAX_TRANSFER_SIZE] = {};

  bool ready_ = false;
  uint8_t packet_type_ = PACKET_TYPE_LORA;
  State state_ = State::STANDBY;
  Packet* active_tx_packet_ = nullptr;
  uint16_t chip_id_ = 0;
  uint32_t last_auto_tx_ms_ = 0;
  uint32_t rx_packets_ = 0;
  uint32_t tx_packets_ = 0;
  uint32_t rx_errors_ = 0;
  uint32_t tx_errors_ = 0;
  uint32_t rx_dropped_ = 0;
  uint32_t tx_dropped_ = 0;
};
