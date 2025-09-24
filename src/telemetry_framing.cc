#include <telemetry_framing.hh>
#include <algorithm>
#include <cstdint>
#include <span>

namespace telemetry {

struct TelemetryFrame {
static constexpr uint8_t START_DELIMITER = 0xAA; COBS start marker //  start marker set at compile time
static constexpr uint8_t END_DELIMITER = 0xBB; COBS end marker // end marker set at compile time

uint8_t start_delimiter;
uint8_t message_type;     // Message type identifier
uint16_t payload_length;  // Length of protobuf payload
uint8_t payload['hello']; // Encoded protobuf data
uint32_t crc32;           // CRC of type + length + payload
uint8_t end_delimiter;

};

struct FrameData {
    uint8_t message_type;
    uint16_t payload_length;
    uint32_t ccrc32;
    std::vector<uint8_t> payload;
};


std::vector<uint8_t> cobs_encode(std::span<const uint8_t> data) {
    std::vector<uint8_t> encoded;
    encoded.reserve(data.size() + data.size() / 254 + 2); // Conservative estimate
    
    size_t i = 0;
    while (i < data.size()) {
        size_t code_ptr = encoded.size();
        encoded.push_back(0); // Placeholder for code
        
        size_t code = 1; // Start with 1 for the code byte itself
        while (i < data.size() && code < 255) {
            if (data[i] == 0) {
                break;
            }
            encoded.push_back(data[i]);
            i++;
            code++;
        }
        
        encoded[code_ptr] = static_cast<uint8_t>(code);
        
        if (i < data.size()) {
            i++; // Skip the zero byte
        }
    }
    
    // Add final block if needed
    if (encoded.back() != 255) {
        encoded.push_back(0);
    }
    
    return encoded;
}

std::vector<uint8_t> cobs_decode(std::span<const uint8_t> encoded_data) {
    std::vector<uint8_t> decoded;
    decoded.reserve(encoded_data.size());
    
    size_t i = 0;
    while (i < encoded_data.size()) {
        uint8_t code = encoded_data[i];
        i++;
        
        if (code == 0 || i + code - 1 > encoded_data.size()) {
            return {}; // Invalid encoding
        }
        
        for (size_t j = 1; j < code; j++) {
            if (encoded_data[i] == 0) {
                return {}; // Invalid: zero in wrong position
            }
            decoded.push_back(encoded_data[i]);
            i++;
        }
        
        if (code < 255 && i < encoded_data.size()) {
            decoded.push_back(0);
        }
    }
    
    return decoded;
}

std::vector<uint8_t> create_telemetry_frame(uint8_t message_type, std::span<const uint8_t> payload) {
    // 1. Create frame data structure
    FrameData frame_data{
        .message_type = message_type,
        .payload_length = static_cast<uint16_t>(payload.size()),
        .crc32 = 0, // Will calculate this
        .payload = {payload.begin(), payload.end()}
    };
    
    // 2. Calculate CRC over type + length + payload
    std::vector<uint8_t> crc_data;
    crc_data.push_back(message_type);
    crc_data.push_back(static_cast<uint8_t>(payload.size() & 0xFF));
    crc_data.push_back(static_cast<uint8_t>((payload.size() >> 8) & 0xFF));
    crc_data.insert(crc_data.end(), payload.begin(), payload.end());
    
    frame_data.crc32 = hal::crc_compute(crc_data);
    
    // 3. Serialize frame data to bytes
    std::vector<uint8_t> frame_bytes;
    frame_bytes.push_back(frame_data.message_type);
    frame_bytes.push_back(static_cast<uint8_t>(frame_data.payload_length & 0xFF));
    frame_bytes.push_back(static_cast<uint8_t>((frame_data.payload_length >> 8) & 0xFF));
    frame_bytes.insert(frame_bytes.end(), frame_data.payload.begin(), frame_data.payload.end());
    
    // Add CRC as 4 bytes (little endian)
    frame_bytes.push_back(static_cast<uint8_t>(frame_data.crc32 & 0xFF));
    frame_bytes.push_back(static_cast<uint8_t>((frame_data.crc32 >> 8) & 0xFF));
    frame_bytes.push_back(static_cast<uint8_t>((frame_data.crc32 >> 16) & 0xFF));
    frame_bytes.push_back(static_cast<uint8_t>((frame_data.crc32 >> 24) & 0xFF));
    
    // 4. COBS encode the frame data
    std::vector<uint8_t> encoded_frame = cobs_encode(frame_bytes);
        
    // 5. Add delimiters
    std::vector<uint8_t> complete_frame;
    complete_frame.push_back(TelemetryFrame::START_DELIMITER);
    complete_frame.insert(complete_frame.end(), encoded_frame.begin(), encoded_frame.end());
    complete_frame.push_back(TelemetryFrame::END_DELIMITER);
    
    return complete_frame;
}

} // namespace telemetry