# SX1281

Semtech SX1281 / SX1280 2.4 GHz LoRa 收发器驱动模块（SPI） / Driver module for the Semtech SX1281 / SX1280 2.4 GHz LoRa transceiver over SPI

## 1. 模块作用 / Purpose

SX1281 使收发器工作在 LoRa 分组模式，驱动 NSS、NRESET、BUSY、DIO、PA、LNA 与 DC-DC 各 GPIO，由 IRQ 驱动的工作线程处理收发，并提供基于固定包池和指针的分组接口（申请、发送、接收、释放）。

- 构造时配置引脚（NSS 与 NRESET 为输出，BUSY 与 DIO1 到 DIO3 为输入，PAEN、LNAEN、DCDCEN 为输出且 DCDCEN 置高），把 SPI 总线设为 CPOL 低、第一边沿采样、最高 18 MHz，将 DIO1 设为上升沿中断，复位芯片并读取芯片 ID。随后依次选择 DC-DC 稳压器、LoRa 分组类型、SF8 / BW 812.5 kHz / CR 4/5、12 符号前导码、可变长度包头、CRC 开启、IQ 正常、XOSC 微调 `0x11`、`frequency_hz` 与 `tx_power_dbm`，并进入接收。每一步都用 `ASSERT` 检查。
- 工作线程 `sx1281_irq`（`HIGH` 优先级，栈深 `irq_task_stack_depth`）由 DIO1 和 `Send()` 唤醒。它读取 IRQ 状态，把收到的载荷入队，统计 CRC 与包头错误，在发送完成或发送超时时释放已发送的包，然后启动下一个排队的发送，或回到接收。
- 射频开关跟随收发器状态：发送时 PAEN 为高、LNAEN 为低，接收时 PAEN 为低、LNAEN 为高，待机时两者均为低。
- 自动发送：`auto_tx_enabled` 为 `true` 且收发器处于接收状态时，工作线程每隔 `auto_tx_period_ms` 发送 8 字节载荷 `ashining`。默认开启，应用自行发送数据时把 `auto_tx_enabled` 设为 `false`。

SX1281 brings the radio up in LoRa packet mode, drives the NSS, NRESET, BUSY, DIO, PA, LNA and DC-DC GPIOs, services the radio from an IRQ-driven worker thread, and provides a fixed-pool, pointer-based packet API (allocate, send, receive, release).

- Construction configures the pins (NSS and NRESET outputs, BUSY and DIO1 to DIO3 inputs, PAEN, LNAEN and DCDCEN outputs with DCDCEN driven high), sets the SPI bus to CPOL low and first-edge sampling at up to 18 MHz, arms DIO1 as a rising-edge interrupt, resets the chip and reads its ID. It then selects the DC-DC regulator, the LoRa packet type, SF8 / BW 812.5 kHz / CR 4/5, a 12-symbol preamble, the variable-length header, CRC on, normal IQ, XOSC trim `0x11`, `frequency_hz` and `tx_power_dbm`, and enters RX. Every step is checked with `ASSERT`.
- The worker thread `sx1281_irq` (`HIGH` priority, stack depth `irq_task_stack_depth`) is woken by DIO1 and by `Send()`. It reads the IRQ status, queues received payloads, counts CRC and header errors, releases the transmitted packet on TX done or TX timeout, and then starts the next queued TX or returns to RX.
- The RF switch follows the radio state: TX has PAEN high and LNAEN low, RX has PAEN low and LNAEN high, standby has both low.
- Auto TX: while `auto_tx_enabled` is `true` and the radio is in RX, the worker thread sends the 8-byte payload `ashining` every `auto_tx_period_ms`. It is enabled by default; `auto_tx_enabled` is set to `false` when the application sends its own traffic.

## 2. 分组接口 / Packet API

数据通路是异步的，收发和自动发送共用一个固定大小的包池。`SX1281::Packet` 包含 `length` 与 `data[255]`（`MAX_PAYLOAD_SIZE`）。

- `Packet* AllocatePacket()`：从包池取出一个包，包池为空时返回 `nullptr`。
- `bool Send(Packet* packet)`：把包加入发送队列。只有返回 `true` 时包的所有权才转移给驱动；`length` 为 0 或发送队列已满时返回 `false`，包仍属于调用方。
- `Packet* Receive()`：取出一个已收到的包，没有排队的包时返回 `nullptr`。
- `void Release(Packet* packet)`：把 `AllocatePacket()` 或 `Receive()` 得到的包归还包池。

包池为空或接收队列已满时，收到的帧被丢弃并计入 `RxDropped()`。状态读取函数：`ChipId()`、`Ready()`、`RxPackets()`、`TxPackets()`、`RxDropped()`、`TxDropped()`、`TxErrors()`、`RxQueued()`、`TxQueued()`、`FreePackets()`、`PacketPoolSize()`。

其他 Module 把 `SX1281&` 作为构造依赖，并在配置中引用本实例的 id，即可使用该收发器。

The data path is asynchronous and uses a fixed packet pool shared by TX, RX and auto TX. `SX1281::Packet` holds `length` and `data[255]` (`MAX_PAYLOAD_SIZE`).

- `Packet* AllocatePacket()`: take a packet from the pool; returns `nullptr` when the pool is empty.
- `bool Send(Packet* packet)`: queue a packet for transmission. Ownership moves to the driver only when it returns `true`; it returns `false` for `length` 0 or when the TX queue is full, and the packet stays with the caller.
- `Packet* Receive()`: take a received packet, or `nullptr` when none is queued.
- `void Release(Packet* packet)`: return a packet obtained from `AllocatePacket()` or `Receive()` to the pool.

Received frames are dropped and counted in `RxDropped()` when the pool is empty or the RX queue is full. Status getters: `ChipId()`, `Ready()`, `RxPackets()`, `TxPackets()`, `RxDropped()`, `TxDropped()`, `TxErrors()`, `RxQueued()`, `TxQueued()`, `FreePackets()`, `PacketPoolSize()`.

Another Module uses the radio by taking `SX1281&` as a constructor dependency and referencing this instance by its id in the configuration.

## 3. 构造接口 / Constructor

```cpp
SX1281(LibXR::SPI& spi,
       LibXR::GPIO& nss,
       LibXR::GPIO& dio1,
       LibXR::GPIO& dio2,
       LibXR::GPIO& dio3,
       LibXR::GPIO& paen,
       LibXR::GPIO& lnaen,
       LibXR::GPIO& dcdcen,
       LibXR::GPIO& busy,
       LibXR::GPIO& nreset,
       Config config = {...});  // 节选 / excerpt
```

依赖：

- `spi`：连接收发器的 `LibXR::SPI` 总线，片选由 `nss` 驱动。
- `nss`：SPI 片选 GPIO，输出，低电平有效。
- `dio1`：DIO1 中断线，配置为上升沿中断。
- `dio2`、`dio3`：DIO2 与 DIO3，配置为输入。
- `paen`：射频前端的 PA 使能 GPIO。
- `lnaen`：射频前端的 LNA 使能 GPIO。
- `dcdcen`：DC-DC 使能 GPIO，启动时置高。
- `busy`：收发器的 BUSY 输入。
- `nreset`：收发器的 NRESET 输出。

配置参数（`Config` 字段，括号内为默认值）：

- `frequency_hz`：射频频率，单位 Hz（`2404000000`）。
- `tx_power_dbm`：发射功率，单位 dBm，限制在 -18 到 13（`13`）。
- `rx_timeout_ms`：传给收发器的接收超时，以 1 ms 为步长（`1000`），超时后模块重新进入接收。
- `tx_timeout_ms`：发送超时，以 1 ms 为步长（`3000`），超时的包计入 `TxErrors()`。
- `auto_tx_period_ms`：自动发送周期，单位 ms（`500`）。
- `auto_tx_enabled`：是否周期发送 `ashining` 信标（`true`）。
- `irq_task_stack_depth`：工作线程栈深（`2048`）。
- `packet_pool_size`：包池中的包数，范围 1 到 8（`8`）。
- `tx_queue_length`：发送队列长度，大于 0（`4`）。
- `rx_queue_length`：接收队列长度，大于 0（`4`）。

Dependencies:

- `spi`: the `LibXR::SPI` bus connected to the radio; chip select is driven by `nss`.
- `nss`: the SPI chip-select GPIO, output, active low.
- `dio1`: the DIO1 IRQ line, configured as a rising-edge interrupt.
- `dio2`, `dio3`: DIO2 and DIO3, configured as inputs.
- `paen`: the PA enable GPIO of the RF front end.
- `lnaen`: the LNA enable GPIO of the RF front end.
- `dcdcen`: the DC-DC enable GPIO, driven high at start-up.
- `busy`: the BUSY input of the radio.
- `nreset`: the NRESET output of the radio.

Configuration parameters (`Config` fields, defaults in parentheses):

- `frequency_hz`: RF frequency in Hz (`2404000000`).
- `tx_power_dbm`: TX power in dBm, clamped to -18 to 13 (`13`).
- `rx_timeout_ms`: RX timeout passed to the radio in 1 ms steps (`1000`); the Module re-enters RX after a timeout.
- `tx_timeout_ms`: TX timeout in 1 ms steps (`3000`); a timed-out packet counts in `TxErrors()`.
- `auto_tx_period_ms`: auto TX period in ms (`500`).
- `auto_tx_enabled`: whether the `ashining` beacon is sent periodically (`true`).
- `irq_task_stack_depth`: stack depth of the worker thread (`2048`).
- `packet_pool_size`: number of packets in the pool, 1 to 8 (`8`).
- `tx_queue_length`: TX queue length, greater than 0 (`4`).
- `rx_queue_length`: RX queue length, greater than 0 (`4`).

## 4. Topic

无 / None

## 5. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/SX1281` 写入的实例，依赖填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/SX1281`, with the dependencies set to names registered by the BSP's `XR_REGISTER` (Registration):

```yaml
modules:
  - module: xrobot-org/SX1281
    id: sx1281_0
    args:
      - spi: spi1
      - nss: lora_nss
      - dio1: lora_dio1
      - dio2: lora_dio2
      - dio3: lora_dio3
      - paen: lora_paen
      - lnaen: lora_lnaen
      - dcdcen: lora_dcdcen
      - busy: lora_busy
      - nreset: lora_nreset
      - config:
          frequency_hz: 2404000000
          tx_power_dbm: 13
          rx_timeout_ms: 1000
          tx_timeout_ms: 3000
          auto_tx_period_ms: 500
          auto_tx_enabled: true
          irq_task_stack_depth: 2048
          packet_pool_size: 8
          tx_queue_length: 4
          rx_queue_length: 4
```

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：一片 SX1281 或 SX1280 收发器，通过 SPI 连接，另有 NSS、NRESET、BUSY、DIO1 到 DIO3、PAEN、LNAEN、DCDCEN 各一个 GPIO，DIO1 接到可触发中断的 GPIO。

Dependencies: LibXR.

Hardware: one SX1281 or SX1280 transceiver on SPI, with one GPIO each for NSS, NRESET, BUSY, DIO1 to DIO3, PAEN, LNAEN and DCDCEN, and DIO1 wired to a GPIO that can raise interrupts.
