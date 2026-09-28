# SX1281

XRobot Module for the Semtech SX1281 / SX1280 2.4 GHz LoRa transceiver over SPI.

The Module brings the radio up in LoRa packet mode, drives the NSS, NRESET, BUSY,
DIO, PA, LNA and DC-DC GPIOs, services the radio from an IRQ-driven worker thread,
and exposes a fixed-pool, pointer-based packet API (allocate, send, receive, release).

## Behaviour

- Construction configures the pins (NSS and NRESET outputs, BUSY and DIO1-3 inputs,
  PAEN / LNAEN / DCDCEN outputs with DCDCEN driven high), sets the SPI bus to
  CPOL low / first-edge sampling at up to 18 MHz, arms DIO1 as a rising-edge
  interrupt, resets the chip and reads its ID. It then selects the DC-DC regulator,
  LoRa packet type, SF8 / BW 812.5 kHz / CR 4/5, 12-symbol preamble, variable-length
  header, CRC on, normal IQ, XOSC trim `0x11`, `frequency_hz` and `tx_power_dbm`,
  and enters RX. Every step is checked with `ASSERT`.
- A worker thread `sx1281_irq` (priority HIGH, stack `irq_task_stack_depth`) is woken
  by DIO1 and by `Send()`. It reads the IRQ status, queues received payloads,
  counts CRC / header errors, releases the transmitted packet on TX done or TX
  timeout, then starts the next queued TX or returns to RX.
- The RF switch follows the radio state: TX = PAEN high / LNAEN low, RX = PAEN low /
  LNAEN high, standby = both low.
- Auto TX: while `auto_tx_enabled` is true and the radio is in RX, the worker sends
  the 8-byte payload `ashining` every `auto_tx_period_ms`. It is enabled by default;
  set `auto_tx_enabled` to `false` when the application sends its own traffic.

## Packet API

The data path is asynchronous and uses a fixed packet pool shared by TX, RX and
auto TX. `SX1281::Packet` holds `length` and `data[255]` (`MAX_PAYLOAD_SIZE`).

- `Packet* AllocatePacket(uint32_t timeout_ms = 0)`: take a packet from the pool;
  returns `nullptr` when the pool is empty. It does not block (`timeout_ms` is unused).
- `bool Send(Packet* packet)`: queue a packet for transmission. Ownership moves to the
  driver only when it returns `true`; it returns `false` for `length` 0 or when the
  TX queue is full, and the caller keeps the packet.
- `Packet* Receive(uint32_t timeout_ms = 0)`: take a received packet, or `nullptr`
  when none is queued. It does not block (`timeout_ms` is unused).
- `void Release(Packet* packet)`: return a packet obtained from `AllocatePacket()` or
  `Receive()` to the pool.

Received frames are dropped (counted in `RxDropped()`) when the pool is empty or the
RX queue is full. Status getters: `ChipId()`, `Ready()`, `RxPackets()`,
`TxPackets()`, `RxDropped()`, `TxDropped()`, `TxErrors()`, `RxQueued()`,
`TxQueued()`, `FreePackets()`, `PacketPoolSize()`.

Another Module uses the radio by taking `SX1281&` as a constructor dependency and
referencing this instance by its id.

## Dependencies

No other Modules; LibXR only.

## Constructor

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
       Config config = {...});
```

Dependencies:

- `spi`: the `LibXR::SPI` bus connected to the radio; chip select is driven by `nss`.
- `nss`: SPI chip-select GPIO (output, active low).
- `dio1`: DIO1 IRQ line, configured as a rising-edge interrupt.
- `dio2`, `dio3`: DIO2 / DIO3, configured as inputs (not otherwise used).
- `paen`: PA enable GPIO of the RF front end.
- `lnaen`: LNA enable GPIO of the RF front end.
- `dcdcen`: DC-DC enable GPIO, driven high at start-up.
- `busy`: BUSY input of the radio.
- `nreset`: NRESET output of the radio.

Configuration (`Config` fields, defaults in brackets):

- `frequency_hz`: RF frequency in Hz (`2404000000`).
- `tx_power_dbm`: TX power in dBm, clamped to -18..13 (`13`).
- `rx_timeout_ms`: RX timeout passed to the radio in 1 ms steps (`1000`); the Module
  re-enters RX after a timeout.
- `tx_timeout_ms`: TX timeout in 1 ms steps (`3000`); a timed-out packet counts in
  `TxErrors()`.
- `auto_tx_period_ms`: auto TX period in ms (`500`).
- `auto_tx_enabled`: send the `ashining` beacon periodically (`true`).
- `irq_task_stack_depth`: worker thread stack depth (`2048`).
- `packet_pool_size`: number of packets in the pool, 1..8 (`8`).
- `tx_queue_length`: TX queue length, > 0 (`4`).
- `rx_queue_length`: RX queue length, > 0 (`4`).

## Use

```sh
xrobot module add xrobot-org/SX1281
xrobot setup
xrobot instance add xrobot-org/SX1281
```

`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; fill the dependencies with the names of the
objects the BSP registers with `XR_REGISTER`:

```yaml
modules:
  - module: xrobot-org/SX1281
    id: sx1281_0
    args:
      - spi: spi2
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
          frequency_hz: '2404000000'
          tx_power_dbm: '13'
          rx_timeout_ms: '1000'
          tx_timeout_ms: '3000'
          auto_tx_period_ms: '500'
          auto_tx_enabled: 'true'
          irq_task_stack_depth: '2048'
          packet_pool_size: '8'
          tx_queue_length: '4'
          rx_queue_length: '4'
```

BSP side:

```cpp
XR_REGISTER(spi2, LibXR::SPI);
XR_REGISTER(lora_nss, LibXR::GPIO);
XR_REGISTER(lora_dio1, LibXR::GPIO);
XR_REGISTER(lora_dio2, LibXR::GPIO);
XR_REGISTER(lora_dio3, LibXR::GPIO);
XR_REGISTER(lora_paen, LibXR::GPIO);
XR_REGISTER(lora_lnaen, LibXR::GPIO);
XR_REGISTER(lora_dcdcen, LibXR::GPIO);
XR_REGISTER(lora_busy, LibXR::GPIO);
XR_REGISTER(lora_nreset, LibXR::GPIO);
```

Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/SX1281` in a BSP, prints the manifest and
the current constructor.
