#pragma once

#include <cstdint>

namespace bms {

enum class SegmentMode : std::uint8_t {
    Normal = 0x55,
    ReducedSampleRate = 0xaa,
};

} // namespace bms
