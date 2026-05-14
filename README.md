# SX1281

XRobot Header Only Module for SEMTECH SX1281 LoRa Connect™ Long Range Low Power LoRa® 2.4GHz RF Transceiver Without Ranging.

## Required Hardware

`spi_sx1281`/`spi1`/`SPI1`, `sx1281_nss`/`sx1281_cs`, `dio1`, `dio2`,
`dio3`, `paen`, `lnaen`, `dcdcen`, `busy`, `nreset`.

## Constructor Arguments

`SX1281::Config` with RF frequency, TX power, RX/TX timeouts, and optional
periodic test transmit settings plus fixed packet-pool and TX/RX queue depths.
`dio1` must be an interrupt-capable GPIO.

The data path is asynchronous and pointer-based. Call `AllocatePacket()` to get
a packet from the fixed pool, fill `packet->data` and `packet->length`, then
call `Send(packet)`. Ownership moves to the driver only when `Send()` returns
true. `Receive()` returns a queued packet pointer; the caller must call
`Release(packet)` after consuming it. The TX/RX queues only store packet
pointers, so payload buffers stay in the fixed pool and are not copied through
FreeRTOS queues.

## Template Arguments

None

## Depends

None
