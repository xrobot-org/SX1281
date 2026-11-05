#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Semtech SX1281、SX1280 2.4 GHz 收发器驱动模块（SPI，LoRa / GFSK / FLRC） / Driver Module for the Semtech SX1281 and SX1280 2.4 GHz transceivers over SPI (LoRa / GFSK / FLRC)
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstddef>
#include <cstdint>

#include "gpio.hpp"
#include "libxr_def.hpp"
#include "spi.hpp"
#include "sx1281_bus.hpp"
#include "sx1281_modem.hpp"


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
  static constexpr uint8_t MAX_PAYLOAD_SIZE = SX1281Regs::MAX_PAYLOAD_SIZE;  ///< 最大载荷长度 Max payload size

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
         LibXR::GPIO* dcdcen = nullptr);

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
  LibXR::ErrorCode Start(const Config& config);

  /**
   * @brief 当前调制下的最大载荷长度。
   *        Largest payload in the current modulation.
   */
  uint8_t MaxPayload() const;

  /**
   * @brief 提前离开接收进入 FS，让收发切换与调用方的其他工作重叠；之后不发送而调用
   *        `Poll()` 时重新进入接收。
   *        Leave receive for FS ahead of a transmission, so that the switch overlaps the
   *        caller's other work; a `Poll()` without a `Send()` in between listens again.
   */
  void PrepareSend();

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
  LibXR::ErrorCode Send(const uint8_t* data, uint8_t size);

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
  void Poll();

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
  int Receive(uint8_t* out);

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
  LibXR::ErrorCode Tune(uint32_t frequency_hz);

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
  int16_t RssiInstant();

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
  static constexpr uint32_t RESET_US = 2000;  // 复位保持 / reset held
  static constexpr uint32_t WAKE_US = 20000;  // 复位后到能接命令 / reset to ready
  static constexpr uint32_t STUCK_CHECK_US = 1000;
  static constexpr uint32_t STUCK_US = 2000;

  enum class TxState : uint8_t
  {
    ON_AIR,
    DONE,
    TIMED_OUT
  };

  enum class Recovery : uint8_t
  {
    STOPPED,  // Start() 之前，芯片在复位中 / before Start(), the chip is held in reset
    NONE,
    IN_RESET,
    WAKING
  };

  const SX1281Modem& Modem() const;
  void ConfigurePins();
  bool Restart();
  void HoldReset();
  void AdvanceRecovery();
  bool Configure();
  TxState CheckTx();
  void FinishSend();
  void EnterRx();
  void Recover();
  bool PacketArriving(uint32_t now);
  bool PreambleStuck(uint32_t now);
  bool PacketStatusBad();
  void SetRfSwitch(bool transmit);
  bool WriteSyncWord();
  bool SetPacketParams(uint8_t payload_size);

  SX1281Bus bus_;
  LibXR::GPIO& nreset_;
  LibXR::GPIO& dio1_;
  LibXR::GPIO& txen_;
  LibXR::GPIO& rxen_;
  LibXR::GPIO* dcdcen_;
  Config config_;

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
