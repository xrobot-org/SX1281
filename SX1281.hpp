#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Semtech SX1281、SX1280 2.4 GHz 收发器驱动模块（SPI，LoRa / GFSK / FLRC） / Driver Module for the Semtech SX1281 and SX1280 2.4 GHz transceivers over SPI (LoRa / GFSK / FLRC)
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "gpio.hpp"
#include "libxr_def.hpp"
#include "spi.hpp"
#include "timebase.hpp"

/**
 * @brief SX1281 / SX1280 2.4 GHz 收发器驱动，LoRa、GFSK 或 FLRC 分组模式。
 *        Driver for the SX1281 / SX1280 2.4 GHz transceiver in LoRa, GFSK or FLRC
 *        packet mode.
 *
 * 驱动不带执行上下文：没有线程、信号量和中断回调。所有成员由同一个调用方上下文调用；
 * `Poll()` 由调用方周期调用，推进发送并在接收停顿时恢复。等待 BUSY 时忙等，最长几毫秒；
 * `Send()` 开始发射就返回，不等发送完成。引用的章节均指 SX1280/SX1281 数据手册
 * DS_SX1280-1 rev 3.3。
 *
 * 射频连续接收，收发之间停在 FS（SetAutoFs，11.6.12）。DIO1 只接 TxDone、RxDone 和
 * CrcError，所以接收时 DIO1 为高即有包，发送时 DIO1 为高即发完；GFSK、FLRC 下坏包由
 * GetPacketStatus 的错误字节判断（勘误 16.3）。连续接收可能停在找到前导码、找不到同步字
 * 之后，什么都收不到，直到重新进入接收（勘误 16.1），所以 PreambleDetected 和
 * SyncWordValid 也锁存在中断状态里，`Poll()` 每 1 ms 查一次，前导码出现 2 ms 没有同步字
 * 就重新进入接收。另外 `rx_rearm_ms` 没收到包且没有包正在到达时重新进入接收，
 * `rx_restart_ms` 没收到包时复位并重新配置；这次复位不阻塞，在之后的 `Poll()` 里分步
 * 完成（约 22 ms，期间不收不发）。
 *
 * The driver has no execution context of its own: no thread, semaphore or interrupt
 * callback. One caller context calls every member; the caller calls `Poll()`
 * periodically, which advances a transmission and recovers a stalled receiver. Waits for
 * BUSY spin, for a few milliseconds at most; `Send()` returns once the packet is on air
 * and does not wait for TX done. Sections refer to the SX1280/SX1281 datasheet
 * DS_SX1280-1 rev 3.3.
 *
 * The radio receives continuously and stays in FS between transmit and receive
 * (SetAutoFs, 11.6.12). Only TxDone, RxDone and CrcError reach DIO1, so a high DIO1 means
 * a packet while receiving and the end of the packet while transmitting; in GFSK and FLRC
 * the error byte of GetPacketStatus tells a bad packet (errata 16.3). Continuous receive
 * can stop after a preamble without a sync word and then hear nothing until receive is
 * entered again (errata 16.1), so PreambleDetected and SyncWordValid are latched in the
 * IRQ status too; `Poll()` looks every 1 ms and enters receive again when a preamble has
 * had no sync word for 2 ms. It also enters receive again after `rx_rearm_ms` without a
 * packet unless one is arriving, and resets and reconfigures the radio after
 * `rx_restart_ms`; that reset does not block and completes over the following `Poll()`
 * calls (about 22 ms, during which nothing is sent or received).
 */
class SX1281
{
 public:
  static constexpr uint8_t MAX_PAYLOAD_SIZE = 255;  ///< 最大载荷长度 Max payload size

  /**
   * @brief 调制方式。
   *        Modulation.
   */
  enum class Modulation : uint8_t
  {
    LORA,  ///< LoRa SF8，BW 812.5 kHz，CR 4/5，最大 255 字节 / up to 255 bytes
    GFSK,  ///< GFSK 2 Mb/s，BT 0.5，调制指数 0.5，最大 255 字节 / up to 255 bytes
    FLRC   ///< FLRC 1.3 Mb/s，BT 1.0，无 FEC，最大 127 字节 / no FEC, up to 127 bytes
  };

  /**
   * @brief 射频配置，由 `Start()` 写入。
   *        Radio configuration, applied by `Start()`.
   */
  struct Config
  {
    Modulation modulation = Modulation::LORA;  ///< 调制方式 Modulation
    uint32_t frequency_hz = 2404000000UL;      ///< 射频频率，Hz RF frequency, Hz
    /// 发射功率，限制在 -18 到 13 dBm / TX power, clamped to -18..13 dBm
    int8_t tx_power_dbm = 13;
    /// GFSK、FLRC 的 32 位同步字；FLRC 应避开勘误 16.4 列出的值。LoRa 不用。
    /// 32-bit sync word for GFSK and FLRC; FLRC should avoid the values listed in errata
    /// 16.4. Unused in LoRa.
    uint8_t sync_word[4] = {'X', 'R', 'S', 'W'};
    uint16_t crc_seed = 0xFFFF;  ///< GFSK、FLRC 的 CRC 初值 CRC seed for GFSK and FLRC
    /// XOSC 负载电容微调值 0..31，负数表示保持芯片默认值。
    /// XOSC load capacitor trim 0..31; negative keeps the chip default.
    int8_t xosc_trim = -1;
    /// 无包多久重新进入接收，ms / Re-enter receive after this long without a packet, ms
    uint16_t rx_rearm_ms = 100;
    /// 无包多久复位射频，ms / Reset the radio after this long without a packet, ms
    uint16_t rx_restart_ms = 2000;
  };

  /**
   * @brief 构造 SX1281：配置引脚与 SPI，射频保持复位直到 `Start()`。
   *        Construct SX1281: configure the pins and the SPI; the radio stays in reset
   *        until `Start()`.
   *
   * @param spi 连接收发器的 SPI 总线，设为 CPOL 低、第一边沿采样、最高 18 MHz。
   *            SPI bus connected to the radio, set to CPOL low, first-edge sampling, up
   *            to 18 MHz.
   * @param nss SPI 片选 GPIO，输出，低电平有效。
   *            SPI chip-select GPIO, output, active low.
   * @param busy 收发器 BUSY 输入。
   *             BUSY input of the radio.
   * @param nreset 收发器 NRESET 输出。
   *               NRESET output of the radio.
   * @param dio1 DIO1 输入，按电平查询。
   *             DIO1 input, read by level.
   * @param txen 射频前端发射使能（PA）GPIO。
   *             Transmit enable (PA) GPIO of the RF front end.
   * @param rxen 射频前端接收使能（LNA）GPIO。
   *             Receive enable (LNA) GPIO of the RF front end.
   * @param dcdcen 可选的 DC-DC 使能 GPIO；给出时置高并选用 DC-DC 稳压器，否则用 LDO。
   *               Optional DC-DC enable GPIO; when given it is driven high and the DC-DC
   *               regulator is selected, otherwise the LDO.
   */
  SX1281(LibXR::SPI& spi, LibXR::GPIO& nss, LibXR::GPIO& busy, LibXR::GPIO& nreset,
         LibXR::GPIO& dio1, LibXR::GPIO& txen, LibXR::GPIO& rxen,
         LibXR::GPIO* dcdcen = nullptr)
      : spi_(spi),
        nss_(nss),
        busy_(busy),
        nreset_(nreset),
        dio1_(dio1),
        txen_(txen),
        rxen_(rxen),
        dcdcen_(dcdcen),
        spi_op_(spi_status_)
  {
    ConfigurePins();
  }

  /**
   * @brief 复位并配置射频，结束时处于接收。
   *        Reset and configure the radio; ends in receive.
   *
   * @param config 射频配置。
   *               Radio configuration.
   * @return 所有配置命令都被接受时为 `OK`，否则为 `FAILED`；失败后 `Poll()` 在
   *         `rx_restart_ms` 无包后会再次复位。
   *         `OK` when every configuration command was accepted, otherwise `FAILED`;
   *         after a failure `Poll()` resets the radio again after `rx_restart_ms`
   *         without a packet.
   * @note 复位脉冲与启动共阻塞约 22 ms。
   *       Blocks about 22 ms for the reset pulse and start-up.
   */
  LibXR::ErrorCode Start(const Config& config)
  {
    config_ = config;
    return Restart() ? LibXR::ErrorCode::OK : LibXR::ErrorCode::FAILED;
  }

  /**
   * @brief 当前调制下的最大载荷长度。
   *        Largest payload in the current modulation.
   */
  uint8_t MaxPayload() const
  {
    return config_.modulation == Modulation::FLRC ? FLRC_MAX_PAYLOAD : MAX_PAYLOAD_SIZE;
  }

  /**
   * @brief 提前离开接收进入 FS，让收发切换与调用方的其他工作重叠；之后不发送而调用
   *        `Poll()` 时重新进入接收。
   *        Leave receive for FS ahead of a transmission, so that the switch overlaps the
   *        caller's other work; a `Poll()` without a `Send()` in between listens again.
   */
  void PrepareSend()
  {
    if (sending_ || prepared_ || recovery_ != Recovery::NONE)
    {
      return;
    }
    // 10.8：FS 到发射 27 us；11.6.4：SetTx 前清 IRQ。
    // 10.8: FS to Tx takes 27 us; 11.6.4: IRQs are cleared before SetTx.
    WriteCommand(CMD_SET_FS, nullptr, 0);
    ClearIrq();
    prepared_ = true;
  }

  /**
   * @brief 发送一包：写入载荷并开始发射就返回，发完由 `Poll()` 转回接收。上一包已经
   *        发完而 `Poll()` 还没转回接收时，这一包直接从 FS 发出，连续的包之间不进入接收。
   *        Send one packet: load the payload and start the transmission, then return;
   *        `Poll()` returns to receive once it is done. When the previous packet is done
   *        and no `Poll()` has returned to receive yet, this one goes out straight from
   *        FS, so back-to-back packets do not enter receive in between.
   *
   * @return `OK` 表示已开始发射；上一包还在发射时为 `BUSY`；长度为 0 或超过
   *         `MaxPayload()` 时为 `ARG_ERR`；SPI 或 BUSY 超时、复位恢复进行中时为
   *         `FAILED`。
   *         `OK` once the packet is on air; `BUSY` while the previous one is still on
   *         air; `ARG_ERR` for size 0 or above `MaxPayload()`; `FAILED` on an SPI or BUSY
   *         timeout, or while a recovery reset is in progress.
   */
  LibXR::ErrorCode Send(const uint8_t* data, uint8_t size)
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
        WriteCommand(CMD_SET_FS, nullptr, 0);
      }
      ClearIrq();
    }
    else
    {
      PrepareSend();
    }
    prepared_ = false;
    SetRfSwitch(true);
    const bool ok = WriteBuffer(TX_BASE, data, size) && SetPacketParams(size) &&
                    WriteCommand(CMD_SET_TX, TX_NO_TIMEOUT, sizeof(TX_NO_TIMEOUT));
    sending_ = true;
    tx_started_us_ = NowUs();
    if (!ok)
    {
      tx_errors_++;
      FinishSend();
      return LibXR::ErrorCode::FAILED;
    }
    return LibXR::ErrorCode::OK;
  }

  /**
   * @brief 是否有包正在发送。
   *        Whether a packet is being sent.
   */
  bool Sending() const { return sending_; }

  /**
   * @brief 推进收发：发送中等到 TxDone（或超时）就转回接收；接收中没有待取的包时执行
   *        停顿恢复。
   *        Advance the radio: while sending, return to receive on TxDone (or a
   *        timeout); while receiving with no packet waiting, run the stall recovery.
   */
  void Poll()
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
        WriteCommand(CMD_SET_FS, nullptr, 0);  // 可能还在发射 / it may still be on air
      }
      ClearIrq();
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

  /**
   * @brief 取一个收到的包。
   *        Fetch one received packet.
   *
   * @param out 至少 `MaxPayload()` 字节。
   *            At least `MaxPayload()` bytes.
   * @return 包长；没有包时为 0；坏包（CRC、包头、长度）时为 -1。
   *         The packet size; 0 when none arrived; -1 for a bad packet (CRC, header,
   *         length).
   */
  int Receive(uint8_t* out)
  {
    if (sending_ || prepared_ || recovery_ != Recovery::NONE || !dio1_.Read())
    {
      return 0;
    }
    heard_us_ = rearmed_us_ = NowUs();
    arriving_ = false;
    // 下一包可能紧跟着到，少一条命令就早一点读到缓冲：GFSK、FLRC 的错误看包状态，只有
    // LoRa 读 IRQ。
    // The next packet may follow right behind, so every command saved reaches the buffer
    // sooner: GFSK and FLRC errors come from the packet status, only LoRa reads the IRQs.
    const uint16_t irq = config_.modulation == Modulation::LORA ? GetIrqStatus() : 0;
    ClearIrq();
    if ((irq & (IRQ_CRC_ERROR | IRQ_HEADER_ERROR)) != 0 || PacketStatusBad())
    {
      rx_errors_++;
      return -1;
    }
    uint8_t status[2] = {};
    if (!ReadCommand(CMD_GET_RX_BUFFER_STATUS, status, sizeof(status)))
    {
      rx_errors_++;
      return -1;
    }
    const uint8_t size = status[0];
    const uint8_t offset = status[1];
    if (size == 0 || size > MaxPayload() || !ReadBuffer(offset, out, size))
    {
      rx_errors_++;
      return -1;
    }
    rx_packets_++;
    return size;
  }

  /**
   * @brief 换频点：离开接收，设新频率，回到连续接收；调制、同步字等其余配置不变。之后的
   *        复位恢复也用这个频点。还没取走的包丢掉。
   *        Change the frequency: leave receive, set the new frequency and listen again;
   *        the modulation, sync word and the rest of the configuration stay. Later
   *        resets of the stall recovery use this frequency too. A packet not fetched yet
   *        is dropped.
   *
   * @return `OK`；发送中为 `BUSY`；SPI 或 BUSY 超时时为 `FAILED`。
   *         `OK`; `BUSY` while sending; `FAILED` on an SPI or BUSY timeout.
   */
  LibXR::ErrorCode Tune(uint32_t frequency_hz)
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
        WriteCommand(CMD_SET_STANDBY, &xosc, 1) && SetRfFrequency(frequency_hz);
    ClearIrq();
    EnterRx();
    return ok ? LibXR::ErrorCode::OK : LibXR::ErrorCode::FAILED;
  }

  /**
   * @brief 当前信道上的瞬时 RSSI（GetRssiInst），接收中有效。进入接收后约 300 us 内的
   *        读数不可靠，调用方丢掉。
   *        Instantaneous RSSI on the current channel (GetRssiInst), valid while
   *        receiving. Readings within about 300 us of entering receive are not
   *        reliable; the caller drops them.
   *
   * @return 单位 0.5 dBm，例如 -170 为 -85 dBm；不在接收中、SPI 或 BUSY 超时时为 0。
   *         In units of 0.5 dBm, e.g. -170 for -85 dBm; 0 when not receiving and on an
   *         SPI or BUSY timeout.
   */
  int16_t RssiInstant()
  {
    uint8_t value = 0;
    if (sending_ || prepared_ || recovery_ != Recovery::NONE ||
        !ReadCommand(CMD_GET_RSSI_INST, &value, 1))
    {
      return 0;
    }
    return static_cast<int16_t>(-static_cast<int16_t>(value));
  }

  uint32_t TxPackets() const { return tx_packets_; }  ///< 发出的包 Packets sent
  /// 发送失败（超时、SPI）的包 Packets whose transmission failed (timeout, SPI)
  uint32_t TxErrors() const { return tx_errors_; }
  uint32_t RxPackets() const { return rx_packets_; }  ///< 收到的好包 Good packets
  uint32_t RxErrors() const { return rx_errors_; }    ///< 坏包 Bad packets
  /// 停顿恢复重新进入接收的次数 Times the stall recovery entered receive again
  uint32_t Rearms() const { return rearms_; }
  /// 复位重新配置的次数（不含第一次 `Start()`） Resets, not counting the first `Start()`
  uint32_t Restarts() const { return restarts_; }

 private:
  static constexpr uint32_t XTAL_HZ = 52000000UL;
  static constexpr uint32_t BUSY_TIMEOUT_US = 20000;
  static constexpr uint32_t SPI_TIMEOUT_US = 20000;
  static constexpr size_t MAX_TRANSFER_SIZE = 4 + MAX_PAYLOAD_SIZE;
  static constexpr uint8_t FLRC_MAX_PAYLOAD = 127;
  // 最长的 GFSK / FLRC 包约 1.3 ms；LoRa SF8 255 字节约 110 ms。
  // The longest GFSK / FLRC packet takes about 1.3 ms; LoRa SF8 with 255 bytes about
  // 110 ms.
  static constexpr uint32_t TX_TIMEOUT_US = 5000;
  static constexpr uint32_t RESET_US = 2000;  // 复位保持 / reset held
  static constexpr uint32_t WAKE_US = 20000;  // 复位后到能接命令 / reset to ready
  static constexpr uint32_t LORA_TX_TIMEOUT_US = 300000;
  static constexpr uint32_t STUCK_CHECK_US = 1000;
  static constexpr uint32_t STUCK_US = 2000;

  static constexpr uint8_t CMD_WRITE_REGISTER = 0x18;
  static constexpr uint8_t CMD_READ_REGISTER = 0x19;
  static constexpr uint8_t CMD_WRITE_BUFFER = 0x1A;
  static constexpr uint8_t CMD_READ_BUFFER = 0x1B;
  static constexpr uint8_t CMD_GET_IRQ_STATUS = 0x15;
  static constexpr uint8_t CMD_GET_RX_BUFFER_STATUS = 0x17;
  static constexpr uint8_t CMD_GET_PACKET_STATUS = 0x1D;
  static constexpr uint8_t CMD_SET_STANDBY = 0x80;
  static constexpr uint8_t CMD_SET_RX = 0x82;
  static constexpr uint8_t CMD_SET_TX = 0x83;
  static constexpr uint8_t CMD_SET_RF_FREQUENCY = 0x86;
  static constexpr uint8_t CMD_SET_PACKET_TYPE = 0x8A;
  static constexpr uint8_t CMD_SET_MODULATION_PARAMS = 0x8B;
  static constexpr uint8_t CMD_SET_PACKET_PARAMS = 0x8C;
  static constexpr uint8_t CMD_SET_DIO_IRQ_PARAMS = 0x8D;
  static constexpr uint8_t CMD_SET_TX_PARAMS = 0x8E;
  static constexpr uint8_t CMD_SET_BUFFER_BASE_ADDRESS = 0x8F;
  static constexpr uint8_t CMD_SET_REGULATOR_MODE = 0x96;
  static constexpr uint8_t CMD_CLEAR_IRQ_STATUS = 0x97;
  static constexpr uint8_t CMD_SET_AUTO_FS = 0x9E;
  static constexpr uint8_t CMD_SET_FS = 0xC1;
  static constexpr uint8_t CMD_GET_RSSI_INST = 0x1F;

  static constexpr uint16_t REG_CRC_SEED = 0x09C8;
  static constexpr uint16_t REG_SYNC_WORD_1 = 0x09CE;
  static constexpr uint16_t REG_XOSC_CAP_BANK_A = 0x0A0E;
  static constexpr uint16_t REG_XOSC_CAP_BANK_B = 0x0A0F;

  static constexpr uint8_t STANDBY_RC = 0x00;
  static constexpr uint8_t STANDBY_XOSC = 0x01;
  static constexpr uint8_t REGULATOR_LDO = 0x00;
  static constexpr uint8_t REGULATOR_DCDC = 0x01;
  static constexpr uint8_t PACKET_TYPE_GFSK = 0x00;
  static constexpr uint8_t PACKET_TYPE_LORA = 0x01;
  static constexpr uint8_t PACKET_TYPE_FLRC = 0x03;
  // PA 爬升时间 8 us（表 11-49）。 / PA ramp time 8 us (table 11-49).
  static constexpr uint8_t RAMP_08_US = 0x60;
  // SetTx 不设超时，SetRx 连续接收（11.6.4、11.6.5）。
  // SetTx without a timeout, SetRx continuous (11.6.4, 11.6.5).
  static constexpr uint8_t TX_NO_TIMEOUT[3] = {0x00, 0x00, 0x00};
  static constexpr uint8_t RX_CONTINUOUS[3] = {0x00, 0xFF, 0xFF};
  // 收发都从缓冲区地址 0 开始，各自可用整个 256 字节（第 8 节）。
  // Both directions use the whole 256-byte buffer from address 0 (section 8).
  static constexpr uint8_t TX_BASE = 0x00;
  static constexpr uint8_t RX_BASE = 0x00;

  // IRQ 位（表 11-73）。DIO1 只接 TxDone、RxDone、CrcError；其余只锁存在状态里。
  // IRQ bits (table 11-73). Only TxDone, RxDone and CrcError reach DIO1; the others are
  // latched in the status only.
  static constexpr uint16_t IRQ_TX_DONE = 0x0001;
  static constexpr uint16_t IRQ_RX_DONE = 0x0002;
  static constexpr uint16_t IRQ_SYNC_WORD_VALID = 0x0004;
  static constexpr uint16_t IRQ_HEADER_VALID = 0x0010;
  static constexpr uint16_t IRQ_HEADER_ERROR = 0x0020;
  static constexpr uint16_t IRQ_CRC_ERROR = 0x0040;
  static constexpr uint16_t IRQ_PREAMBLE_DETECTED = 0x8000;
  static constexpr uint16_t IRQ_DIO1 = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_CRC_ERROR;
  static constexpr uint16_t IRQ_LATCHED = IRQ_DIO1 | IRQ_SYNC_WORD_VALID |
                                          IRQ_HEADER_VALID | IRQ_HEADER_ERROR |
                                          IRQ_PREAMBLE_DETECTED;
  // 这些位锁存着而还没有 RxDone：有包正在到达。 / These latched without an RxDone: a
  // packet is arriving.
  static constexpr uint16_t IRQ_ARRIVING =
      IRQ_SYNC_WORD_VALID | IRQ_HEADER_VALID | IRQ_PREAMBLE_DETECTED;
  // GetPacketStatus 的错误字节（表 11-68）：LengthError、CrcError、AbortError。
  // Error byte of GetPacketStatus (table 11-68): LengthError, CrcError, AbortError.
  static constexpr uint8_t PACKET_ERRORS = 0x20 | 0x10 | 0x08;

  /**
   * @brief 一种调制的固定参数。
   *        Fixed parameters of one modulation.
   */
  struct ModemParams
  {
    uint8_t packet_type;
    uint8_t modulation[3];
    uint8_t packet[7];  // 第 5 字节（载荷长度）由 SetPacketParams 填 / byte 5 (payload
                        // size) is filled by SetPacketParams
  };

  // LoRa SF8 / BW 812.5 kHz / CR 4/5；12 符号前导码，显式包头，CRC 开，IQ 正常（14.4）。
  // LoRa SF8 / BW 812.5 kHz / CR 4/5; 12-symbol preamble, explicit header, CRC on, normal
  // IQ (14.4).
  static constexpr ModemParams LORA_PARAMS = {
      PACKET_TYPE_LORA, {0x80, 0x18, 0x01}, {0x0C, 0x00, 0x00, 0x20, 0x40, 0x00, 0x00}};
  // GFSK 2 Mb/s、带宽 2.4 MHz、调制指数 0.5、BT 0.5（表 14-1..14-3）；16 位前导码，4 字节
  // 同步字 1，变长，2 字节 CRC，白化开（14.1.1）。
  // GFSK 2 Mb/s, 2.4 MHz bandwidth, modulation index 0.5, BT 0.5 (tables 14-1..14-3);
  // 16-bit preamble, 4-byte sync word 1, variable length, 2-byte CRC, whitening on
  // (14.1.1).
  static constexpr ModemParams GFSK_PARAMS = {
      PACKET_TYPE_GFSK, {0x04, 0x01, 0x20}, {0x30, 0x06, 0x10, 0x20, 0x00, 0x20, 0x00}};
  // FLRC 1.3 Mb/s、带宽 1.2 MHz、BT 1.0（表 14-31..14-33），编码率 1（无 FEC）；16 位
  // 前导码，32 位同步字 1，变长，2 字节 CRC，白化关（表 14-41）。没有同步字时接收机会把
  // 噪声当成包。
  // FLRC 1.3 Mb/s, 1.2 MHz bandwidth, BT 1.0 (tables 14-31..14-33), coding rate 1 (no
  // FEC); 16-bit preamble, 32-bit sync word 1, variable length, 2-byte CRC, whitening off
  // (table 14-41). Without a sync word the receiver takes noise for packets.
  static constexpr ModemParams FLRC_PARAMS = {
      PACKET_TYPE_FLRC, {0x45, 0x04, 0x10}, {0x30, 0x04, 0x10, 0x20, 0x00, 0x10, 0x08}};

  const ModemParams& Modem() const
  {
    switch (config_.modulation)
    {
      case Modulation::GFSK:
        return GFSK_PARAMS;
      case Modulation::FLRC:
        return FLRC_PARAMS;
      case Modulation::LORA:
      default:
        return LORA_PARAMS;
    }
  }

  uint32_t TxTimeoutUs() const
  {
    return config_.modulation == Modulation::LORA ? LORA_TX_TIMEOUT_US : TX_TIMEOUT_US;
  }

  static uint32_t NowUs()
  {
    return static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
  }

  static uint32_t Elapsed(uint32_t now, uint32_t then) { return now - then; }

  void ConfigurePins()
  {
    nss_.SetConfig({.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                    .pull = LibXR::GPIO::Pull::UP});
    nss_.Write(true);
    nreset_.SetConfig({.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                       .pull = LibXR::GPIO::Pull::NONE});
    nreset_.Write(false);
    busy_.SetConfig(
        {.direction = LibXR::GPIO::Direction::INPUT, .pull = LibXR::GPIO::Pull::NONE});
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

  // 复位芯片并配置，阻塞约 22 ms（Start()）。 / Reset and configure the chip, blocking
  // for about 22 ms (Start()).
  bool Restart()
  {
    HoldReset();
    LibXR::Timebase::DelayMicroseconds(RESET_US);
    nreset_.Write(true);
    LibXR::Timebase::DelayMicroseconds(WAKE_US);
    return Configure();
  }

  void HoldReset()
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
  void AdvanceRecovery()
  {
    if (recovery_ == Recovery::STOPPED)
    {
      return;
    }
    const uint32_t now = NowUs();
    if (recovery_ == Recovery::IN_RESET)
    {
      if (Elapsed(now, recovery_us_) >= RESET_US)
      {
        nreset_.Write(true);
        recovery_ = Recovery::WAKING;
        recovery_us_ = now;
      }
      return;
    }
    if (Elapsed(now, recovery_us_) >= WAKE_US)
    {
      recovery_ = Recovery::NONE;
      (void)Configure();  // 失败时静默超时后再复位 / a failure resets again later
    }
  }

  bool Configure()
  {
    // 14.1.1 / 14.3.1 / 14.4.1：STDBY_RC，先设包类型，再设频率、缓冲区、调制和包参数、
    // 同步字与 CRC、发射参数、IRQ 路由。
    // 14.1.1 / 14.3.1 / 14.4.1: STDBY_RC, packet type first, then frequency, buffer,
    // modulation and packet parameters, sync word and CRC, TX parameters, IRQ routing.
    const ModemParams& modem = Modem();
    const uint8_t regulator = dcdcen_ != nullptr ? REGULATOR_DCDC : REGULATOR_LDO;
    const uint8_t standby = STANDBY_RC;
    const uint8_t buffer_base[2] = {TX_BASE, RX_BASE};
    const int8_t power = std::clamp<int8_t>(config_.tx_power_dbm, -18, 13);
    const uint8_t tx_params[2] = {static_cast<uint8_t>(power + 18), RAMP_08_US};
    const uint8_t irq_params[8] = {
        IRQ_LATCHED >> 8, IRQ_LATCHED & 0xFF, IRQ_DIO1 >> 8, IRQ_DIO1 & 0xFF, 0, 0, 0, 0};
    const uint8_t auto_fs = 0x01;

    bool ok =
        WriteCommand(CMD_SET_STANDBY, &standby, 1) &&
        WriteCommand(CMD_SET_REGULATOR_MODE, &regulator, 1) &&
        WriteCommand(CMD_SET_PACKET_TYPE, &modem.packet_type, 1) &&
        SetRfFrequency(config_.frequency_hz) &&
        WriteCommand(CMD_SET_BUFFER_BASE_ADDRESS, buffer_base, sizeof(buffer_base)) &&
        WriteCommand(CMD_SET_MODULATION_PARAMS, modem.modulation,
                     sizeof(modem.modulation)) &&
        SetPacketParams(MaxPayload());
    if (ok && config_.modulation != Modulation::LORA)
    {
      ok = WriteSyncWord() && WriteRegister16(REG_CRC_SEED, config_.crc_seed);
    }
    if (ok && config_.xosc_trim >= 0)
    {
      const uint8_t xosc = STANDBY_XOSC;
      const uint8_t trim = static_cast<uint8_t>(config_.xosc_trim) & 0x1F;
      ok = WriteCommand(CMD_SET_STANDBY, &xosc, 1) &&
           WriteRegisters(REG_XOSC_CAP_BANK_A, &trim, 1) &&
           WriteRegisters(REG_XOSC_CAP_BANK_B, &trim, 1);
    }
    ok = ok && WriteCommand(CMD_SET_TX_PARAMS, tx_params, sizeof(tx_params)) &&
         WriteCommand(CMD_SET_DIO_IRQ_PARAMS, irq_params, sizeof(irq_params)) &&
         WriteCommand(CMD_SET_AUTO_FS, &auto_fs, 1);
    ClearIrq();
    EnterRx();
    heard_us_ = NowUs();
    return ok;
  }

  enum class TxState : uint8_t
  {
    ON_AIR,
    DONE,
    TIMED_OUT
  };

  // 发送中的包是否结束；结束时计数一次，调用方随后结束这次发送。
  // Whether the packet on air has ended; an end is counted once, and the caller then
  // ends this transmission.
  TxState CheckTx()
  {
    if (dio1_.Read())
    {
      tx_packets_++;
      return TxState::DONE;
    }
    if (Elapsed(NowUs(), tx_started_us_) < TxTimeoutUs())
    {
      return TxState::ON_AIR;
    }
    tx_errors_++;
    return TxState::TIMED_OUT;
  }

  void FinishSend()
  {
    sending_ = false;
    // 接收接受不超过最大值的任意长度（变长模式滤掉更长的包）。
    // Receive accepts any size up to the maximum (variable length drops longer packets).
    SetPacketParams(MaxPayload());
    EnterRx();
  }

  void EnterRx()
  {
    // 11.6.5：调用方在 SetRx 前清 IRQ。 / 11.6.5: the caller clears IRQs before SetRx.
    SetRfSwitch(false);
    WriteCommand(CMD_SET_RX, RX_CONTINUOUS, sizeof(RX_CONTINUOUS));
    preamble_seen_ = false;
    arriving_ = false;
    // 停顿从这里算起：发送之后应答还在路上。 / Silence counts from here: right after a
    // transmission the answer is still on its way.
    rearmed_us_ = NowUs();
  }

  void Recover()
  {
    const uint32_t now = NowUs();
    if (Elapsed(now, heard_us_) >= config_.rx_restart_ms * 1000U)
    {
      restarts_++;
      HoldReset();
      recovery_ = Recovery::IN_RESET;
      recovery_us_ = now;
    }
    else if ((Elapsed(now, rearmed_us_) >= config_.rx_rearm_ms * 1000U &&
              !PacketArriving(now)) ||
             PreambleStuck(now))
    {
      rearms_++;
      const uint8_t xosc = STANDBY_XOSC;
      WriteCommand(CMD_SET_STANDBY, &xosc, 1);
      ClearIrq();
      EnterRx();
    }
  }

  // 重新进入接收会截断正在到达的包。到达的时间不会超过一个最长包。 / Entering receive
  // again would cut a packet that is arriving, which takes at most one longest packet.
  bool PacketArriving(uint32_t now)
  {
    if ((GetIrqStatus() & IRQ_ARRIVING) == 0)
    {
      arriving_ = false;
      return false;
    }
    if (!arriving_)
    {
      arriving_ = true;
      arriving_us_ = now;
    }
    return Elapsed(now, arriving_us_) < TxTimeoutUs();
  }

  bool PreambleStuck(uint32_t now)
  {
    // LoRa 没有这种停顿；它的包也远长于 2 ms。 / LoRa has no such stall, and its packets
    // are far longer than 2 ms.
    if (config_.modulation == Modulation::LORA ||
        Elapsed(now, checked_us_) < STUCK_CHECK_US)
    {
      return false;
    }
    checked_us_ = now;
    const uint16_t irq = GetIrqStatus();
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
    return Elapsed(now, preamble_us_) >= STUCK_US;
  }

  bool PacketStatusBad()
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
    return !ReadCommand(CMD_GET_PACKET_STATUS, status, sizeof(status)) ||
           (status[2] & PACKET_ERRORS) != 0;
  }

  void SetRfSwitch(bool transmit)
  {
    rxen_.Write(!transmit);
    txen_.Write(transmit);
  }

  bool WaitNotBusy()
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

  bool Exchange(size_t size)
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

  bool WriteCommand(uint8_t command, const uint8_t* params, size_t size)
  {
    tx_[0] = command;
    if (size > 0)
    {
      std::memcpy(&tx_[1], params, size);
    }
    return Exchange(size + 1);
  }

  bool ReadCommand(uint8_t command, uint8_t* out, size_t size)
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

  bool WriteRegisters(uint16_t address, const uint8_t* data, size_t size)
  {
    tx_[0] = CMD_WRITE_REGISTER;
    tx_[1] = static_cast<uint8_t>(address >> 8);
    tx_[2] = static_cast<uint8_t>(address);
    std::memcpy(&tx_[3], data, size);
    return Exchange(size + 3);
  }

  bool WriteRegister16(uint16_t address, uint16_t value)
  {
    const uint8_t data[2] = {static_cast<uint8_t>(value >> 8),
                             static_cast<uint8_t>(value)};
    return WriteRegisters(address, data, sizeof(data));
  }

  bool WriteSyncWord()
  {
    // GFSK：同步字 1 的第 5 字节（0x09CE）为 0，4 字节同步字从 0x09CF 开始；FLRC 只用
    // 0x09CF..0x09D2。
    // GFSK: byte 5 of sync word 1 (0x09CE) is 0 and the 4-byte word starts at 0x09CF;
    // FLRC uses 0x09CF..0x09D2 only.
    const uint8_t gfsk[5] = {0x00, config_.sync_word[0], config_.sync_word[1],
                             config_.sync_word[2], config_.sync_word[3]};
    return config_.modulation == Modulation::GFSK
               ? WriteRegisters(REG_SYNC_WORD_1, gfsk, sizeof(gfsk))
               : WriteRegisters(REG_SYNC_WORD_1 + 1, config_.sync_word,
                                sizeof(config_.sync_word));
  }

  bool WriteBuffer(uint8_t offset, const uint8_t* data, uint8_t size)
  {
    tx_[0] = CMD_WRITE_BUFFER;
    tx_[1] = offset;
    std::memcpy(&tx_[2], data, size);
    return Exchange(static_cast<size_t>(size) + 2);
  }

  bool ReadBuffer(uint8_t offset, uint8_t* out, uint8_t size)
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

  bool SetPacketParams(uint8_t payload_size)
  {
    uint8_t params[7];
    std::memcpy(params, Modem().packet, sizeof(params));
    // LoRa 的载荷长度在第 3 字节，GFSK / FLRC 在第 5 字节。
    // The payload size is byte 3 in LoRa and byte 5 in GFSK / FLRC.
    params[config_.modulation == Modulation::LORA ? 2 : 4] = payload_size;
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

  uint16_t GetIrqStatus()
  {
    uint8_t status[2] = {};
    if (!ReadCommand(CMD_GET_IRQ_STATUS, status, sizeof(status)))
    {
      return 0;
    }
    return static_cast<uint16_t>((static_cast<uint16_t>(status[0]) << 8) | status[1]);
  }

  void ClearIrq()
  {
    const uint8_t all[2] = {0xFF, 0xFF};
    WriteCommand(CMD_CLEAR_IRQ_STATUS, all, sizeof(all));
  }

  LibXR::SPI& spi_;
  LibXR::GPIO& nss_;
  LibXR::GPIO& busy_;
  LibXR::GPIO& nreset_;
  LibXR::GPIO& dio1_;
  LibXR::GPIO& txen_;
  LibXR::GPIO& rxen_;
  LibXR::GPIO* dcdcen_;

  std::atomic<LibXR::SPI::OperationRW::OperationPollingStatus> spi_status_{
      LibXR::SPI::OperationRW::OperationPollingStatus::READY};
  LibXR::SPI::OperationRW spi_op_;
  Config config_;

  uint8_t tx_[MAX_TRANSFER_SIZE] = {};
  uint8_t rx_[MAX_TRANSFER_SIZE] = {};

  enum class Recovery : uint8_t
  {
    STOPPED,  // Start() 之前，芯片在复位中 / before Start(), the chip is held in reset
    NONE,
    IN_RESET,
    WAKING
  };

  bool sending_ = false;
  bool prepared_ = false;
  Recovery recovery_ = Recovery::STOPPED;
  uint32_t recovery_us_ = 0;
  bool arriving_ = false;
  uint32_t arriving_us_ = 0;
  bool preamble_seen_ = false;
  uint32_t tx_started_us_ = 0;
  uint32_t heard_us_ = 0;     // 上一个包或启动的时间 / last packet, or start
  uint32_t rearmed_us_ = 0;   // 上次进入接收的时间 / last time receive was entered
  uint32_t checked_us_ = 0;   // 上次查前导码停顿的时间 / last stuck-preamble check
  uint32_t preamble_us_ = 0;  // 第一次看到无同步字前导码的时间 / first preamble without
                              // a sync word

  uint32_t tx_packets_ = 0;
  uint32_t tx_errors_ = 0;
  uint32_t rx_packets_ = 0;
  uint32_t rx_errors_ = 0;
  uint32_t rearms_ = 0;
  uint32_t restarts_ = 0;
};
