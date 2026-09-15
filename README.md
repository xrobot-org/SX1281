# SX1281

## Static assembly source line

This source line uses explicit C++ constructor dependencies and ordered instance
arguments. Inspect the current primary header with `xrobot_mod_parser --path .`;
its declarations, not old manifest/config examples, define the interface.
Historical HardwareContainer/ApplicationManager examples below apply only to the
older dynamic source tags. Device/protocol descriptions remain relevant.
See the XRobot [migration guide](https://github.com/xrobot-org/XRobot/blob/dev/MIGRATION.md).
Compilation is not hardware validation; retain version-specific board evidence.


Semtech SX1281 / SX1280 SPI 2.4 GHz LoRa transceiver module for XRobot.

This module initializes the radio over SPI, configures LoRa packet mode, manages
NSS, reset, busy, DIO, PA, LNA, and DC-DC GPIOs, handles IRQ-driven receive /
transmit state in a worker thread, and provides pointer-based packet allocation,
send, receive, and release APIs.

The data path is asynchronous and fixed-pool based. Call `AllocatePacket()` to
get a packet from the pool, fill `packet->data` and `packet->length`, then call
`Send(packet)`. Ownership moves to the driver only when `Send()` returns `true`.
`Receive()` returns a queued packet pointer; the caller must call `Release()` on
the packet after consuming it.

## Required Hardware

- `sx1281_spi`
- `sx1281_nss`
- `sx1281_dio1`
- `sx1281_dio2`
- `sx1281_dio3`
- `sx1281_paen`
- `sx1281_lnaen`
- `sx1281_dcdcen`
- `sx1281_busy`
- `sx1281_nreset`

## Constructor Arguments

- `config.frequency_hz`: default `2404000000`
- `config.tx_power_dbm`: default `13`
- `config.rx_timeout_ms`: default `1000`
- `config.tx_timeout_ms`: default `3000`
- `config.auto_tx_period_ms`: default `500`
- `config.auto_tx_enabled`: default `true`
- `config.irq_task_stack_depth`: default `2048`
- `config.packet_pool_size`: default `8`
- `config.tx_queue_length`: default `4`
- `config.rx_queue_length`: default `4`

## Packet API

- `AllocatePacket(timeout_ms)`: return a packet from the fixed pool
- `Send(packet, timeout_ms)`: enqueue a packet for transmission and transfer ownership to the driver
- `Receive(timeout_ms)`: dequeue a received packet pointer
- `Release(packet)`: return a packet to the fixed pool

## XRobot Configuration Example

```yaml
- id: radio
  name: SX1281
  constructor_args:
    config:
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
