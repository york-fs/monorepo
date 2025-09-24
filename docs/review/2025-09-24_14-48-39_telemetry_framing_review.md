# Telemetry Framing Implementation Review - 2025-09-24

## Project Overview

This review covers the implementation of robust messaging framing for the STM32 firmware logging board telemetry functionality in the York Formula Student monorepo.

## Current System State

### Hardware Platform
- **MCU**: STM32F103C8T6 (Blue Pill development board)
- **Communication**: UART (USART2) for telemetry output
- **Protocol**: Protocol Buffers (nanopb) for message serialization
- **Baud Rate**: 57600 (based on BRR = 484 calculation)

### Existing Implementation
The current logging board implementation (`src/loggingboard.cc`) has:
- Basic UART setup and configuration
- Protocol Buffers message encoding using a simple `Test` message
- Raw data transmission without framing
- LED indicator for successful encoding

```cpp
// Current problematic implementation:
Test example = {42};
std::array<uint8_t, Test_size> buffer{};
pb_ostream_t stream = pb_ostream_from_buffer(buffer.data(), buffer.size());
pb_encode(&stream, Test_fields, &example);
UART_send_bytes(buffer);  // Raw data with potential 0x00 bytes
```

## Key Discussions and Decisions

### 1. Message Framing Requirements
**Problem Identified**: Raw binary data transmission over UART can be unreliable due to:
- Zero bytes (0x00) being misinterpreted as message delimiters
- No error detection capabilities
- No message type identification
- No length validation

**Solution Adopted**: COBS (Consistent Overhead Byte Stuffing) + CRC framing protocol

### 2. COBS Encoding Analysis
**Detailed Discussion**: Comprehensive explanation of COBS algorithm including:
- **Purpose**: Eliminates 0x00 bytes from data stream for reliable UART communication
- **Algorithm**: Block-based encoding with code bytes indicating data length
- **Performance**: O(n) time complexity with minimal overhead
- **Reliability**: Deterministic encoding/decoding with perfect data reconstruction

**Key Concepts Covered**:
- Block-based processing with 254-byte maximum per block
- Code byte mechanism (1-255 range)
- Zero-byte boundary handling
- Edge cases (empty data, all zeros, long non-zero runs)

### 3. Frame Structure Design
**Proposed Frame Format**:
```
[START_DELIMITER] [MSG_TYPE] [LENGTH] [PAYLOAD] [CRC32] [END_DELIMITER]
    0xAA             1 byte    2 bytes   Variable   4 bytes    0xBB
```

**Components**:
- **Delimiters**: 0xAA (start) and 0xBB (end) for frame boundary detection
- **Message Type**: 8-bit identifier for different telemetry message types
- **Length**: 16-bit payload length field
- **Payload**: COBS-encoded Protocol Buffers data
- **CRC32**: Error detection using existing HAL CRC functionality

### 4. Integration Strategy
**Implementation Plan**:
1. Create `telemetry_framing.cc/hh` with COBS codec functions
2. Implement frame creation and parsing utilities
3. Update logging board to use framed transmission
4. Add message type definitions
5. Implement receiver-side parsing logic

**Key Functions to Implement**:
- `cobs_encode()` / `cobs_decode()`: Core COBS algorithm
- `create_frame()`: Frame construction with CRC calculation
- `parse_frame()`: Frame validation and payload extraction
- `send_telemetry_message()`: High-level transmission function

## Technical Implementation Details

### COBS Algorithm Walkthrough
**Encoding Example**: Data `[0x01, 0x02, 0x00, 0x03, 0x04]`
1. Block 1: `[0x01, 0x02]` → Code 0x03 → Output: `[0x03, 0x01, 0x02]`
2. Block 2: `[0x03, 0x04]` → Code 0x03 → Output: `[0x03, 0x01, 0x02, 0x03, 0x03, 0x04]`
3. Final zero addition → Output: `[0x03, 0x01, 0x02, 0x03, 0x03, 0x04, 0x00]`

**Decoding Process**: Reverse operation with code bytes directing reconstruction

### CRC Integration
**Existing Infrastructure**: Hardware CRC peripheral (STM32 CRC unit) already available in HAL:
```cpp
std::uint32_t crc_compute(std::span<const std::uint8_t> data);
```

**Usage**: CRC calculation over message type + length + payload for error detection

### Memory Considerations
- **STM32F103C8T6**: 20KB Flash, 64KB RAM
- **COBS Overhead**: ~1-2 bytes per 254 bytes of data
- **Frame Overhead**: 8 bytes (delimiters + type + length + CRC)
- **Total Impact**: Minimal for typical telemetry messages

## MAVLink Inspiration
**Reference Study**: MAVLink protocol analysis for potential future enhancements:
- Message ID system (256 possible message types)
- Sequence numbers for message ordering
- Component ID and system ID fields
- Dual CRC system for enhanced reliability
- Message signing capabilities

## Next Steps and Recommendations

### Immediate Actions
1. **Implement COBS codec** in `src/telemetry_framing.cc`
2. **Create message type definitions** for telemetry data
3. **Update logging board** to use framed transmission
4. **Test with simple receiver** implementation

### Future Enhancements
1. **Sequence numbering** for message ordering and loss detection
2. **Timestamp integration** for telemetry timing
3. **Message signing** for authenticity verification
4. **Variable payload sizes** beyond current fixed buffers
5. **Multi-message type support** for comprehensive telemetry

### Testing Strategy
1. **Unit testing** of COBS encode/decode functions
2. **Integration testing** with actual UART hardware
3. **Error injection testing** to verify CRC detection
4. **Performance testing** under load conditions

## Conclusion

The proposed COBS + CRC framing implementation provides a robust, lightweight solution for reliable telemetry communication over UART. The approach balances simplicity with reliability, making it well-suited for the resource-constrained STM32 platform while providing the error detection and message integrity required for automotive telemetry applications.

**Estimated Implementation Time**: 2-3 development sessions
**Complexity Level**: Moderate
**Risk Level**: Low (well-understood algorithms, existing CRC infrastructure)

---

*Review conducted on: 2025-09-24 14:48:39*
*Focus: Telemetry framing implementation planning and COBS algorithm analysis*
