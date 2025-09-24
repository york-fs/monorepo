#include <telemetry_framing.hh>
#include <algorithm>
#include <cstdint>
#include <span>

namespace telemetry {

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

} // namespace telemetry