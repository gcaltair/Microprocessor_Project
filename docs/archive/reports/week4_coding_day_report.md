# Week 4 Coding Day Report

## FreeRTOS LiDAR Data Pipeline for Continuous UART-DMA Reception

### 1. Introduction

The objective of this coding day was to extend an existing RPLidar SCAN-mode receiver into a correct FreeRTOS software pipeline. The required design was not simply to read bytes from UART, but to demonstrate a real-time architecture in which DMA receives a continuous byte stream, the interrupt service routine (ISR) stays short, task-level parsing is performed outside interrupt context, and the processed result is forwarded to a separate downstream stage for communication and debugging.

Our implementation follows the required architecture and embeds it into the wider robot software stack:

```text
USART6 + circular DMA
-> DMA half/full event
-> Queue 1: DMA block descriptor
-> LiDAR Parser Task
-> Queue 2: processed scan message
-> downstream localisation / mapping
-> communication task for Bluetooth debug output
```

The key idea is that the DMA buffer stores raw LiDAR bytes continuously, while FreeRTOS queues only carry lightweight metadata or processed results. This avoids unnecessary copying, keeps the ISR short, and cleanly separates sensing, parsing, and presentation.

### 2. DMA Block Strategy and Justification

The LiDAR receiver uses `USART6` with circular DMA. The DMA target buffer is `4096` bytes and is split into two equal halves of `2048` bytes each. The half-buffer size is therefore:

```text
LIDAR_DMA_RX_BUFFER_SIZE = 4096 bytes
LIDAR_DMA_HALF_BUFFER_SIZE = 2048 bytes
```

This ping-pong arrangement is appropriate for a continuous byte stream because one half can be processed while DMA is filling the other half. When the DMA half-complete callback fires, the first half is guaranteed to be ready. When the DMA complete callback fires, the second half is guaranteed to be ready and DMA wraps back to the beginning of the buffer.

The chosen block size is a practical compromise:

- It is large enough to reduce interrupt frequency and ISR overhead.
- It is small enough to keep parsing latency bounded.
- At `460800` baud, a `2048`-byte half-buffer fills in roughly `44 ms`, which is short enough for responsive processing but long enough to avoid excessive interrupt load.

This design also supports safe handover between interrupt context and task context. The parser task only reads from a half-buffer after DMA has declared it complete, so the task never parses bytes that are still being written by DMA.

### 3. ISR to Queue 1 Design

The ISR is intentionally minimal. The DMA-related UART callbacks do not parse LiDAR packets and do not copy large amounts of data. Instead, they only identify which half of the DMA buffer has become ready and push a small descriptor to Queue 1.

In our implementation, Queue 1 carries the following structure:

```c
typedef struct {
    uint16_t offset;
    uint16_t length;
    uint32_t sequence;
} LidarDmaBlockMsg_t;
```

This message contains exactly the information needed by the parser task:

- `offset`: where the completed block starts in the DMA buffer
- `length`: the valid block length
- `sequence`: a monotonically increasing event counter for debugging and overflow tracking

This is a better design than copying raw bytes into the queue, because the queue remains small and deterministic while the raw data stays in the DMA memory buffer. If Queue 1 ever becomes full, the firmware records an overflow flag and increments an overflow counter for diagnosis. This makes the failure mode observable instead of silent.

### 4. LiDAR Parser Task

The LiDAR Parser Task blocks on Queue 1 using `osMessageQueueGet(..., osWaitForever)`. It only runs when a completed DMA block becomes available. This demonstrates the required FreeRTOS blocked-to-ready wake-up behaviour.

Once woken, the parser task:

1. Reads the block descriptor from Queue 1.
2. Accesses the corresponding bytes directly from the DMA buffer using the received `offset` and `length`.
3. Parses the continuous LiDAR byte stream one byte at a time.
4. Builds a scan-level result in a reusable scan buffer.
5. Sends a processed result message to Queue 2 when one full scan has been assembled.

The LiDAR protocol in this implementation uses standard SCAN frames of `5` bytes per measurement. Because DMA block boundaries do not align with LiDAR packet boundaries, the parser keeps an internal `packet_buffer` and `packet_idx` so that partial packets can continue across DMA blocks. This is essential: a streaming parser must not assume that each DMA block starts on a packet boundary.

The parser also handles scan boundaries correctly. When it detects the first sample of a new scan while the current scan buffer already contains points, it marks the current scan as complete, saves the first packet of the next scan in `pending_packet`, and emits the completed scan first. On the next activation, the pending packet is consumed before new bytes are parsed. This prevents the first measurement of the next scan from being lost.

This mechanism is the main reason why task-level parsing is required. The continuous LiDAR stream has no guarantee that block boundaries and logical scan boundaries will coincide.

### 5. Processed Result and Queue 2 Design

The coding-day brief allows each team to choose a meaningful processed output. We selected a scan-level result rather than a fixed number of samples per DMA block, because the number of valid points contained in a block is inherently variable.

Queue 2 carries the following processed result structure:

```c
typedef struct {
    uint8_t scan_index;
    uint16_t point_count;
    uint32_t scan_sequence;
    SlamPose2D_t pose_snapshot;
    SlamPose2D_t corrected_pose;
    LidarScanQuality_t quality;
    float localization_fitness_m;
    uint16_t localization_inliers;
    uint8_t localization_mode;
} LidarScanMsg_t;
```

This design is useful for two reasons.

First, it provides a meaningful summary of the parsed scan:

- raw point count
- usable point count
- rejected points due to range limits
- rejected points due to low quality
- minimum and maximum valid distance

Second, it avoids copying the entire point cloud through the queue. The raw points remain in one of four reusable scan buffers, and Queue 2 passes only the `scan_index` plus metadata. This again keeps queue traffic compact and deterministic.

In the integrated system, the Queue 2 message is first consumed by downstream localisation and mapping tasks, and the final communication/debug stage sends the buffered scan through `UART5` over Bluetooth as a binary packet. This extends the required coding-day pipeline into the larger SLAM system without changing the core DMA-to-task handover principle.

### 6. Output / Debug Separation

The output/debug path is separated from parsing. The communication task remains blocked until a scan is ready for transmission, then packages and transmits the scan over `UART5` using DMA-based transmission. This means that LiDAR parsing is not delayed by terminal or Bluetooth output latency.

This separation is important because output is relatively slow compared with memory access and parsing. If printing or binary transmission were performed in the parser task or ISR, the LiDAR pipeline would become much more vulnerable to dropped blocks and timing jitter.

The parser task is also assigned a higher priority than the communication task. This ensures that when both tasks are ready, parsing the incoming sensor data is preferred over non-critical output.

### 7. FreeRTOS Scheduling Behaviour

The required task-state behaviour can be explained as follows:

- The parser task is normally in the `BLOCKED` state waiting on Queue 1.
- A DMA half/full interrupt posts a descriptor, so the parser task becomes `READY`.
- The scheduler runs the parser task, which moves to `RUNNING`.
- After processing the block, the parser task posts a processed message to Queue 2 and returns to the blocked state.
- The downstream consumer or communication task wakes from its own blocked state and handles the processed result.

This is the core FreeRTOS demonstration requested in the brief: interrupt-driven event notification, queue-based communication, and task scheduling with clear functional separation.

### 8. FreeRTOS Memory Objects and Runtime Resources

The implementation creates the main RTOS memory objects expected by the task:

- message queues for DMA block descriptors, processed scan messages, free scan buffers, communication, and commands
- task stacks for each created thread
- task control blocks managed by the kernel
- mutexes for shared odometry, localisation, mapping, PID, and control data
- a semaphore for the periodic control loop

In addition to kernel objects, the application uses several important static buffers:

- one `4096`-byte DMA receive buffer for LiDAR raw data
- four reusable LiDAR scan buffers
- two `8192`-byte transmit buffers for double-buffered Bluetooth output

Using static buffers is appropriate for this embedded target because memory use is explicit and predictable.

### 9. Why the ISR Must Remain Short

Keeping the ISR short is a fundamental design rule in this application. The robot is not running LiDAR alone; it also has control, odometry, localisation, mapping, and communication tasks. If the ISR performed packet parsing or formatting work, interrupt latency would increase and could interfere with other time-sensitive functions.

Our ISR therefore performs only event notification. This reduces worst-case interrupt time, improves system responsiveness, and makes the software architecture easier to reason about. All computationally heavier work is deferred to the parser task, where blocking, prioritisation, and debugging are much easier to manage.

### 10. Conclusion

This implementation demonstrates the intended FreeRTOS LiDAR pipeline correctly. Circular DMA receives the continuous LiDAR byte stream into alternating half-buffers, the ISR announces completed blocks using a small Queue 1 descriptor, the LiDAR Parser Task reconstructs packets and complete scans in task context, and Queue 2 forwards a processed scan result to the downstream system and debug output path.

The most important engineering outcome is not only that LiDAR data is received, but that it is handled using a defensible real-time architecture: short ISR execution, queue-based task wake-up, safe DMA-buffer handover, scan-aware stream parsing, and clean separation between processing and communication. This is exactly the architecture required for scaling from a simple coding-day exercise to a larger embedded SLAM robot system.
