# SX1281

Semtech SX1281、SX1280 2.4 GHz 收发器驱动模块（SPI，LoRa / GFSK / FLRC） / Driver Module for the Semtech SX1281 and SX1280 2.4 GHz transceivers over SPI (LoRa / GFSK / FLRC)

## 1. 模块作用 / Purpose

SX1281 使收发器工作在 LoRa、GFSK 或 FLRC 分组模式，驱动 NSS、NRESET、BUSY、DIO1 和射频开关各 GPIO。模块不带执行上下文：没有线程、信号量和中断回调，所有成员由同一个调用方上下文调用。应用调用 `Start()` 配置射频，用 `Send()` 发包，用 `Receive()` 取包，并周期调用 `Poll()`。

- 构造时配置引脚（NSS、NRESET、TXEN、RXEN 为输出，BUSY、DIO1 为输入，给出 DCDCEN 时置高），把 SPI 总线设为 CPOL 低、第一边沿采样、最高 18 MHz，射频保持复位。
- `Start()` 复位芯片，写入调制、频率、包参数、同步字、CRC 初值和发射功率，然后进入连续接收。三种调制的参数固定：LoRa SF8 / BW 812.5 kHz / CR 4/5，最大 255 字节；GFSK 2 Mb/s，最大 255 字节；FLRC 1.3 Mb/s、无 FEC，最大 127 字节。
- 射频在收发之间停在 FS。DIO1 只接 TxDone、RxDone 和 CrcError，模块按电平查询 DIO1。
- `Poll()` 在发送完成或超时后转回接收；接收中没有待取的包时执行停顿恢复：GFSK、FLRC 下前导码出现 2 ms 仍没有同步字时重新进入接收（数据手册勘误 16.1），`rx_rearm_ms` 没收到包时重新进入接收，`rx_restart_ms` 没收到包时复位并重新配置。
- 射频开关跟随收发器状态：发送时 TXEN 为高、RXEN 为低，接收时 TXEN 为低、RXEN 为高，复位期间两者均为低。

SX1281 brings the radio up in LoRa, GFSK or FLRC packet mode and drives the NSS, NRESET, BUSY, DIO1 and RF switch GPIOs. The Module has no execution context of its own: no thread, semaphore or interrupt callback, and one caller context calls every member. The application configures the radio with `Start()`, sends with `Send()`, fetches packets with `Receive()` and calls `Poll()` periodically.

- Construction configures the pins (NSS, NRESET, TXEN and RXEN outputs, BUSY and DIO1 inputs, DCDCEN driven high when given), sets the SPI bus to CPOL low and first-edge sampling at up to 18 MHz, and keeps the radio in reset.
- `Start()` resets the chip, writes the modulation, frequency, packet parameters, sync word, CRC seed and TX power, and enters continuous receive. The parameters of the three modulations are fixed: LoRa SF8 / BW 812.5 kHz / CR 4/5, up to 255 bytes; GFSK 2 Mb/s, up to 255 bytes; FLRC 1.3 Mb/s without FEC, up to 127 bytes.
- The radio stays in FS between transmit and receive. Only TxDone, RxDone and CrcError reach DIO1, which the Module reads by level.
- `Poll()` returns to receive once a transmission is done or has timed out; while receiving with no packet waiting it runs the stall recovery: in GFSK and FLRC it enters receive again when a preamble has had no sync word for 2 ms (datasheet errata 16.1), it enters receive again after `rx_rearm_ms` without a packet, and it resets and reconfigures the radio after `rx_restart_ms`.
- The RF switch follows the radio state: TX has TXEN high and RXEN low, RX has TXEN low and RXEN high, and both are low during reset.

## 2. 分组接口 / Packet API

- `ErrorCode Start(const Config& config)`：复位并配置射频，结束在接收，阻塞约 22 ms。所有配置命令被接受时返回 `OK`，否则返回 `FAILED`。
- `uint8_t MaxPayload()`：当前调制下的最大载荷长度。
- `ErrorCode Send(const uint8_t* data, uint8_t size)`：写入载荷并开始发射就返回，不等发送完成。上一包已经发完而还没调用 `Poll()` 时直接从 FS 发出，连续的包之间不进入接收。上一包还在发射时返回 `BUSY`，长度为 0 或超过 `MaxPayload()` 时返回 `ARG_ERR`，SPI 或 BUSY 超时时返回 `FAILED`。
- `bool Sending()`：是否有包正在发送。
- `void PrepareSend()`：提前离开接收进入 FS，让收发切换与应用的其他工作重叠；之后不发送而调用 `Poll()` 时重新进入接收。
- `void Poll()`：推进发送与停顿恢复。
- `int Receive(uint8_t* out)`：取一个收到的包，`out` 至少 `MaxPayload()` 字节。返回包长；没有包时返回 0；CRC、包头或长度错误的坏包返回 -1。
- `ErrorCode Tune(uint32_t frequency_hz)`：换频点并回到连续接收，其余配置不变，之后的复位恢复也用这个频点。发送中返回 `BUSY`。
- `int16_t RssiInstant()`：当前信道的瞬时 RSSI，单位 0.5 dBm（-170 为 -85 dBm），接收中有效；进入接收后约 300 µs 内的读数不可靠。

计数：`TxPackets()`、`TxErrors()`、`RxPackets()`、`RxErrors()`、`Rearms()`、`Restarts()`。

其他 Module 把 `SX1281&` 作为构造依赖，并在配置中引用本实例的 id，即可使用该收发器。

- `ErrorCode Start(const Config& config)`: reset and configure the radio, ending in receive; blocks about 22 ms. Returns `OK` when every configuration command was accepted, otherwise `FAILED`.
- `uint8_t MaxPayload()`: the largest payload in the current modulation.
- `ErrorCode Send(const uint8_t* data, uint8_t size)`: load the payload and start the transmission, then return without waiting for it to finish. When the previous packet is done and `Poll()` has not run since, the packet goes out straight from FS, so back-to-back packets do not enter receive in between. Returns `BUSY` while the previous packet is still on air, `ARG_ERR` for size 0 or above `MaxPayload()`, and `FAILED` on an SPI or BUSY timeout.
- `bool Sending()`: whether a packet is being sent.
- `void PrepareSend()`: leave receive for FS ahead of a transmission, so that the switch overlaps the application's other work; a `Poll()` without a `Send()` in between listens again.
- `void Poll()`: advance a transmission and the stall recovery.
- `int Receive(uint8_t* out)`: fetch one received packet into `out`, which holds at least `MaxPayload()` bytes. Returns the packet size, 0 when none arrived, and -1 for a bad packet (CRC, header or length error).
- `ErrorCode Tune(uint32_t frequency_hz)`: change the frequency and listen again, keeping the rest of the configuration; later resets of the stall recovery use this frequency too. Returns `BUSY` while sending.
- `int16_t RssiInstant()`: the instantaneous RSSI on the current channel in units of 0.5 dBm (-170 is -85 dBm), valid while receiving; readings within about 300 µs of entering receive are not reliable.

Counters: `TxPackets()`, `TxErrors()`, `RxPackets()`, `RxErrors()`, `Rearms()`, `Restarts()`.

Another Module uses the radio by taking `SX1281&` as a constructor dependency and referencing this instance by its id in the configuration.

## 3. 构造接口 / Constructor

```cpp
SX1281(LibXR::SPI& spi,
       LibXR::GPIO& nss,
       LibXR::GPIO& busy,
       LibXR::GPIO& nreset,
       LibXR::GPIO& dio1,
       LibXR::GPIO& txen,
       LibXR::GPIO& rxen,
       LibXR::GPIO* dcdcen = nullptr);
```

依赖：

- `spi`：连接收发器的 `LibXR::SPI` 总线，片选由 `nss` 驱动。传输按轮询方式等待完成，DMA 与非 DMA 的 SPI 驱动都可以使用。
- `nss`：SPI 片选 GPIO，输出，低电平有效。
- `busy`：收发器的 BUSY 输入。
- `nreset`：收发器的 NRESET 输出。
- `dio1`：DIO1 输入。
- `txen`：射频前端的发射使能（PA）GPIO。
- `rxen`：射频前端的接收使能（LNA）GPIO。
- `dcdcen`：可选的 DC-DC 使能 GPIO。给出时置高并选用芯片的 DC-DC 稳压器，否则使用 LDO。

`Start()` 的配置参数（`Config` 字段）：

- `modulation`：`LORA`、`GFSK` 或 `FLRC`，默认 `LORA`。
- `frequency_hz`：射频频率，单位 Hz，默认 `2404000000`。
- `tx_power_dbm`：发射功率，单位 dBm，限制在 -18 到 13，默认 `13`。
- `sync_word`：GFSK、FLRC 的 32 位同步字，默认 `"XRSW"`。FLRC 应避开数据手册勘误 16.4 列出的值。
- `crc_seed`：GFSK、FLRC 的 CRC 初值，默认 `0xFFFF`。
- `xosc_trim`：XOSC 负载电容微调值 0 到 31，负数保持芯片默认值，默认 `-1`。
- `rx_rearm_ms`：无包多久重新进入接收，默认 `100`。
- `rx_restart_ms`：无包多久复位射频，默认 `2000`。

Dependencies:

- `spi`: the `LibXR::SPI` bus connected to the radio; chip select is driven by `nss`. Transfers are waited for by polling, so SPI drivers with or without DMA both work.
- `nss`: the SPI chip-select GPIO, output, active low.
- `busy`: the BUSY input of the radio.
- `nreset`: the NRESET output of the radio.
- `dio1`: the DIO1 input.
- `txen`: the transmit enable (PA) GPIO of the RF front end.
- `rxen`: the receive enable (LNA) GPIO of the RF front end.
- `dcdcen`: the optional DC-DC enable GPIO. When given it is driven high and the chip's DC-DC regulator is selected; otherwise the LDO is used.

Configuration parameters of `Start()` (`Config` fields):

- `modulation`: `LORA`, `GFSK` or `FLRC`, default `LORA`.
- `frequency_hz`: RF frequency in Hz, default `2404000000`.
- `tx_power_dbm`: TX power in dBm, clamped to -18 to 13, default `13`.
- `sync_word`: the 32-bit sync word for GFSK and FLRC, default `"XRSW"`. FLRC should avoid the values listed in datasheet errata 16.4.
- `crc_seed`: the CRC seed for GFSK and FLRC, default `0xFFFF`.
- `xosc_trim`: the XOSC load capacitor trim 0 to 31; a negative value keeps the chip default; default `-1`.
- `rx_rearm_ms`: re-enter receive after this long without a packet, default `100`.
- `rx_restart_ms`: reset the radio after this long without a packet, default `2000`.

## 4. Topic

无 / None

## 5. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/SX1281` 写入的实例，依赖填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/SX1281`, with the dependencies set to names registered by the BSP with `XR_REGISTER` (Registration):

```yaml
modules:
  - module: xrobot-org/SX1281
    id: sx1281_0
    args:
      - spi: spi1
      - nss: radio_nss
      - busy: radio_busy
      - nreset: radio_nreset
      - dio1: radio_dio1
      - txen: radio_txen
      - rxen: radio_rxen
```

使用它的应用或 Module 调用 `Start()` 并在自己的循环里调用 `Poll()`：

The application or Module that uses it calls `Start()` and calls `Poll()` from its own loop:

```cpp
SX1281::Config config;
config.modulation = SX1281::Modulation::GFSK;
radio.Start(config);

uint8_t packet[SX1281::MAX_PAYLOAD_SIZE];
for (;;)
{
  radio.Poll();
  const int size = radio.Receive(packet);
  if (size > 0)
  {
    radio.Send(packet, static_cast<uint8_t>(size));
  }
}
```

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：一片 SX1281 或 SX1280 收发器，通过 SPI 连接，另有 NSS、NRESET、BUSY、DIO1、TXEN、RXEN 各一个 GPIO，以及可选的 DCDCEN。

Dependencies: LibXR.

Hardware: one SX1281 or SX1280 transceiver on SPI, with one GPIO each for NSS, NRESET, BUSY, DIO1, TXEN and RXEN, and an optional DCDCEN.
