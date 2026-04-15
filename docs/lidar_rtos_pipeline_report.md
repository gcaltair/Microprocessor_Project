# LiDAR FreeRTOS Pipeline Report

## 1) System architecture

Implemented pipeline:

`USART6 + DMA (circular ping-pong) -> DMA ISR callback -> Queue1 (descriptor) -> LiDAR Parser Task -> Queue2 (parsed summary) -> Output Task`

Main implementation files:

- `Core/Src/lidar.c`, `Core/Src/lidar.h`
- `Core/Src/usart.c`, `Core/Inc/usart.h`
- `Core/Src/dma.c`
- `Core/Src/stm32f4xx_it.c`, `Core/Inc/stm32f4xx_it.h`
- `Core/Src/main.c`

## 2) DMA ping-pong strategy

- USART6 RX is configured as `DMA_CIRCULAR` with one buffer `LIDAR_DMA_BUFFER_SIZE = 512`.
- Buffer is split into two fixed halves (`LIDAR_DMA_HALF_SIZE = 256`).
- Half-transfer callback means first half is ready (`offset=0`, `length=256`).
- Transfer-complete callback means second half is ready (`offset=256`, `length=256`).
- Reception remains continuous; no restart per block.

## 3) Why ISR stays short

ISR/callback does only:

- identify which half completed
- fill a tiny descriptor (`offset`, `length`, `block_id`, `event_counter`)
- enqueue descriptor to Queue1 using non-blocking call

No parsing and no raw-byte copy is done in ISR.

## 4) Why Queue1 carries only small descriptor

Queue1 message type is `LidarDmaBlockDesc_t`:

- `offset`
- `length`
- `block_id`
- `event_counter`

Raw LiDAR bytes are never copied into Queue1. Parser task reads bytes directly from DMA memory.

## 5) Parser state across continuous blocks

Parser keeps persistent stream state in `LidarStreamParserState_t`:

- partial 5-byte standard node buffer
- current byte index

If a node starts in one DMA half and ends in the next, parsing continues correctly because state is retained between queue events.

## 6) Queue2 payload

Queue2 message type is `LidarParseResult_t` and contains per-block processed output:

- block/event identifiers
- valid node count
- invalid node count
- start-node count
- minimum distance (mm)
- maximum distance (mm)
- parser resync count

## 7) Task priorities and scheduling behavior

- `LidarParser` task priority: `osPriorityAboveNormal` (higher)
- `LidarOutput` task priority: `osPriorityNormal` (lower)

Rationale:

- parsing keeps up with incoming stream and should run before printing
- printing is less time-critical and can be deferred

Task state behavior:

- Parser task is usually **Blocked** on Queue1, becomes **Ready/Running** when DMA descriptor arrives, then returns to **Blocked**.
- Output task is usually **Blocked** on Queue2, becomes **Ready/Running** when parser posts a result, then returns to **Blocked**.

## 8) DMA block size choice

Chosen half-block size: `256 bytes`.

Justification:

- large enough to reduce interrupt frequency
- small enough to keep parser latency low
- at 460800 bps this gives frequent but manageable processing slices for demonstration

## 9) Demo-friendly output format

Output task wakes on every Queue2 result, but prints a throttled summary every 25 blocks
(to match UART5 9600 baud bandwidth):

`[LDR] evt=<...> blk=<...> blocks=<...> valid=<...> min=<...>mm max=<...>mm q1drop=<...> q2drop=<...>`

This clearly demonstrates:

- interrupt-driven block events
- Queue1 -> parser -> Queue2 flow
- parser throughput and data quality in real time
